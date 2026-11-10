//! Multi-file binary `DAF/SPK` catalog implementation backed by [`navigate_astrodynamics::spk::SpkEphemeris`].

use glam::DVec3;
use navigate_astrodynamics::spk::{SpkDirectedLink, SpkEphemeris as NavSpkEphemeris};

use crate::{
    daf::{SpkFile, SpkSegment},
    hash::{ContentHash, DomainHasher},
    ids::NaifId,
    time::{CoverageSet, TdbEpoch, TdbSpanNs},
};

use super::{
    DerivativeOrder, EphemerisCatalog, EphemerisError, KinematicStateIcrf, MarsCenterApproximation,
};

/// Multi-file binary `DAF/SPK` catalog with last-loaded segment precedence and shortest-chain LCA resolution.
///
/// Shares the underlying [`navigate_astrodynamics::spk::SpkEphemeris`] segment graph and
/// Chebyshev evaluator with `Navigate`.
#[derive(Clone, Debug)]
pub struct SpkEphemerisCatalog {
    hash: ContentHash,
    mars_policy: MarsCenterApproximation,
    inner: NavSpkEphemeris,
    segments: Vec<SpkSegment>,
}

impl SpkEphemerisCatalog {
    /// Builds a catalog from loaded `SpkFile`s in load order (later files & later segments take precedence).
    #[must_use]
    pub fn from_files(files: Vec<SpkFile>, mars_policy: MarsCenterApproximation) -> Self {
        let mut h = DomainHasher::new(b"sim-orbital:spk-catalog:v1");
        h.write_u64(files.len() as u64);
        for f in &files {
            h.write_hash(f.content_hash());
        }
        match mars_policy {
            MarsCenterApproximation::StrictRequire499 => h.write_i32(0),
            MarsCenterApproximation::UseBarycenter4 { documented_bound_m } => {
                h.write_i32(1);
                h.write_f64_canonical(documented_bound_m);
            }
        }

        let mut segments = Vec::new();
        let mut nav_files = Vec::with_capacity(files.len());
        for f in files {
            segments.extend_from_slice(f.segments());
            nav_files.push(f.into_nav());
        }

        Self {
            hash: h.finish(),
            mars_policy,
            inner: NavSpkEphemeris::from_files(nav_files),
            segments,
        }
    }

    /// Parses one or more raw `.bsp` byte slices in precedence order.
    pub fn from_bsp_slices(
        slices: &[&[u8]],
        mars_policy: MarsCenterApproximation,
    ) -> Result<Self, EphemerisError> {
        let mut files = Vec::with_capacity(slices.len());
        for &raw in slices {
            files.push(SpkFile::from_bytes(raw)?);
        }
        Ok(Self::from_files(files, mars_policy))
    }

    /// Configured Mars `499` center resolution policy.
    #[must_use]
    pub const fn mars_policy(&self) -> MarsCenterApproximation {
        self.mars_policy
    }

    /// Underlying [`navigate_astrodynamics::spk::SpkEphemeris`] instance shared with `Navigate`.
    #[must_use]
    pub const fn as_navigate_spk(&self) -> &NavSpkEphemeris {
        &self.inner
    }

    fn remap_body(&self, id: NaifId) -> Result<NaifId, EphemerisError> {
        let known = self.inner.known_bodies();
        if known.contains(&id.0) {
            return Ok(id);
        }
        if id == NaifId::MARS && known.contains(&NaifId::MARS_BARYCENTER.0) {
            return match self.mars_policy {
                MarsCenterApproximation::StrictRequire499 => {
                    Err(EphemerisError::MissingBodyCenter {
                        requested: NaifId::MARS,
                        available_barycenter: NaifId::MARS_BARYCENTER,
                    })
                }
                MarsCenterApproximation::UseBarycenter4 { .. } => Ok(NaifId::MARS_BARYCENTER),
            };
        }
        if let Some(bary) = id.parent_planetary_barycenter()
            && id.0 % 100 == 99
            && known.contains(&bary.0)
        {
            return Err(EphemerisError::MissingBodyCenter {
                requested: id,
                available_barycenter: bary,
            });
        }
        Ok(id)
    }

    fn resolve_chain(
        &self,
        target: NaifId,
        center: NaifId,
    ) -> Result<Vec<SpkDirectedLink>, EphemerisError> {
        self.inner
            .resolve_chain_links(target.0, center.0)
            .map_err(|_| EphemerisError::UnconnectedBodies { target, center })
    }

    fn eval_link(
        &self,
        link: SpkDirectedLink,
        epoch: TdbEpoch,
    ) -> Result<Option<KinematicStateIcrf>, EphemerisError> {
        for &(seg_idx, is_lo_to_hi) in self.inner.link_candidates(link) {
            let Some(seg) = self.segments.get(seg_idx) else {
                continue;
            };
            if seg.contains(epoch) {
                let (t_whole_s, t_sub_s) = epoch.split_j2000_seconds();
                let (p, v, a) = seg
                    .inner()
                    .evaluate_pva_split_j2000(t_whole_s, t_sub_s)
                    .map_err(crate::daf::DafError::from)?;
                let raw = KinematicStateIcrf {
                    pos_m: DVec3::from_array(p),
                    vel_m_s: DVec3::from_array(v),
                    acc_m_s2: DVec3::from_array(a),
                };
                return Ok(Some(if link.forward == is_lo_to_hi {
                    raw
                } else {
                    raw.negated()
                }));
            }
        }
        Ok(None)
    }

    fn link_coverage(&self, link: SpkDirectedLink) -> CoverageSet {
        let candidates = self.inner.link_candidates(link);
        let mut spans = Vec::with_capacity(candidates.len());
        for &(seg_idx, _) in candidates {
            if let Some(seg) = self.segments.get(seg_idx) {
                spans.push(seg.span_ns());
            }
        }
        CoverageSet::from_spans(spans)
    }
}

impl EphemerisCatalog for SpkEphemerisCatalog {
    fn content_hash(&self) -> ContentHash {
        self.hash
    }

    fn catalog_model_id(&self) -> &str {
        "naif-spk-chebyshev"
    }

    fn is_analytical_approximation(&self) -> bool {
        false
    }

    fn coverage(&self, target: NaifId, center: NaifId, _order: DerivativeOrder) -> Vec<TdbSpanNs> {
        let (Ok(t_eff), Ok(c_eff)) = (self.remap_body(target), self.remap_body(center)) else {
            return Vec::new();
        };
        if t_eff == c_eff {
            return CoverageSet::unbounded().spans().to_vec();
        }
        let Ok(chain) = self.resolve_chain(t_eff, c_eff) else {
            return Vec::new();
        };
        let mut acc = CoverageSet::unbounded();
        for link in chain {
            acc = acc.intersect(&self.link_coverage(link));
            if acc.is_empty() {
                break;
            }
        }
        acc.spans().to_vec()
    }

    fn geometric_state_icrf(
        &self,
        target: NaifId,
        center: NaifId,
        epoch: TdbEpoch,
        order: DerivativeOrder,
    ) -> Result<KinematicStateIcrf, EphemerisError> {
        let t_eff = self.remap_body(target)?;
        let c_eff = self.remap_body(center)?;
        if t_eff == c_eff {
            return Ok(KinematicStateIcrf::ZERO);
        }
        let chain = self.resolve_chain(t_eff, c_eff)?;
        let mut total = KinematicStateIcrf::ZERO;
        for link in chain {
            let Some(step) = self.eval_link(link, epoch)? else {
                return Err(EphemerisError::OutOfCoverage {
                    target,
                    center,
                    epoch_j2000_s: epoch.as_j2000_seconds_f64(),
                });
            };
            total = total.plus(step);
        }
        Ok(match order {
            DerivativeOrder::Position => KinematicStateIcrf {
                pos_m: total.pos_m,
                vel_m_s: DVec3::ZERO,
                acc_m_s2: DVec3::ZERO,
            },
            DerivativeOrder::Velocity => KinematicStateIcrf {
                pos_m: total.pos_m,
                vel_m_s: total.vel_m_s,
                acc_m_s2: DVec3::ZERO,
            },
            DerivativeOrder::Acceleration => total,
        })
    }

    fn segment_knots_in(
        &self,
        target: NaifId,
        center: NaifId,
        start: TdbEpoch,
        end: TdbEpoch,
    ) -> Vec<TdbEpoch> {
        let (Ok(t_eff), Ok(c_eff)) = (self.remap_body(target), self.remap_body(center)) else {
            return Vec::new();
        };
        let Ok(chain) = self.resolve_chain(t_eff, c_eff) else {
            return Vec::new();
        };
        let mut knots = Vec::new();
        for link in chain {
            for &(seg_idx, _) in self.inner.link_candidates(link) {
                if let Some(seg) = self.segments.get(seg_idx) {
                    seg.collect_interior_knots_in(start, end, &mut knots);
                }
            }
        }
        knots.sort();
        knots.dedup();
        knots
    }
}
