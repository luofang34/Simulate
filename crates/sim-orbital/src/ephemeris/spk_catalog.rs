//! Multi-file binary `DAF/SPK` catalog implementation with precedence and LCA chain resolution.

use std::collections::{BTreeMap, BTreeSet, VecDeque};

use glam::DVec3;

use crate::{
    daf::{SpkFile, SpkSegment},
    hash::{ContentHash, DomainHasher},
    ids::NaifId,
    time::{CoverageSet, TdbEpoch, TdbSpanNs},
};

use super::{
    DerivativeOrder, EphemerisCatalog, EphemerisError, KinematicStateIcrf, MarsCenterApproximation,
};

#[derive(Clone, Copy, Debug)]
struct DirectedLink {
    seg_indices_high_to_low: usize,
    forward: bool,
}

#[derive(Clone, Debug)]
struct EdgeSlot {
    candidates: Vec<(usize, bool)>,
}

/// Multi-file binary `DAF/SPK` catalog with last-loaded segment precedence and shortest-chain LCA resolution.
#[derive(Clone, Debug)]
pub struct SpkEphemerisCatalog {
    hash: ContentHash,
    mars_policy: MarsCenterApproximation,
    edge_lists: Vec<EdgeSlot>,
    segments: Vec<SpkSegment>,
    adjacency: BTreeMap<NaifId, Vec<(NaifId, DirectedLink)>>,
    known_bodies: BTreeSet<NaifId>,
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
        for f in files {
            segments.extend_from_slice(f.segments());
        }

        let mut pair_map: BTreeMap<(NaifId, NaifId), Vec<(usize, bool)>> = BTreeMap::new();
        let mut known_bodies = BTreeSet::new();
        let mut idx = 0_usize;
        while let Some(seg) = segments.get(idx) {
            let t = seg.target();
            let c = seg.center();
            known_bodies.insert(t);
            known_bodies.insert(c);
            let (lo, hi, is_lo_to_hi) = if t <= c { (t, c, true) } else { (c, t, false) };
            pair_map
                .entry((lo, hi))
                .or_default()
                .push((idx, is_lo_to_hi));
            idx = idx.wrapping_add(1);
        }

        let mut edge_lists = Vec::with_capacity(pair_map.len());
        let mut adjacency: BTreeMap<NaifId, Vec<(NaifId, DirectedLink)>> = BTreeMap::new();
        for ((lo, hi), mut list) in pair_map {
            list.reverse();
            let slot = edge_lists.len();
            edge_lists.push(EdgeSlot { candidates: list });
            adjacency.entry(lo).or_default().push((
                hi,
                DirectedLink {
                    seg_indices_high_to_low: slot,
                    forward: true,
                },
            ));
            adjacency.entry(hi).or_default().push((
                lo,
                DirectedLink {
                    seg_indices_high_to_low: slot,
                    forward: false,
                },
            ));
        }

        Self {
            hash: h.finish(),
            mars_policy,
            edge_lists,
            segments,
            adjacency,
            known_bodies,
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

    fn remap_body(&self, id: NaifId) -> Result<NaifId, EphemerisError> {
        if self.known_bodies.contains(&id) {
            return Ok(id);
        }
        if id == NaifId::MARS && self.known_bodies.contains(&NaifId::MARS_BARYCENTER) {
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
            && self.known_bodies.contains(&bary)
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
    ) -> Result<Vec<DirectedLink>, EphemerisError> {
        if target == center {
            return Ok(Vec::new());
        }
        let mut visited: BTreeMap<NaifId, Option<(NaifId, DirectedLink)>> = BTreeMap::new();
        let mut queue = VecDeque::new();
        visited.insert(target, None);
        queue.push_back(target);

        while let Some(curr) = queue.pop_front() {
            if curr == center {
                let mut chain = Vec::new();
                let mut walk = center;
                while let Some(Some((prev, link))) = visited.get(&walk).copied() {
                    chain.push(link);
                    walk = prev;
                }
                chain.reverse();
                return Ok(chain);
            }
            if let Some(neighbors) = self.adjacency.get(&curr) {
                for &(next, link) in neighbors {
                    if let std::collections::btree_map::Entry::Vacant(e) = visited.entry(next) {
                        e.insert(Some((curr, link)));
                        queue.push_back(next);
                    }
                }
            }
        }
        Err(EphemerisError::UnconnectedBodies { target, center })
    }

    fn eval_link(
        &self,
        link: DirectedLink,
        epoch: TdbEpoch,
    ) -> Result<Option<KinematicStateIcrf>, EphemerisError> {
        let Some(slot) = self.edge_lists.get(link.seg_indices_high_to_low) else {
            return Ok(None);
        };
        for &(seg_idx, is_lo_to_hi) in &slot.candidates {
            let Some(seg) = self.segments.get(seg_idx) else {
                continue;
            };
            if seg.contains(epoch) {
                let (pos_m, vel_m_s, acc_m_s2) = seg.evaluate_icrf(epoch)?;
                let raw = KinematicStateIcrf {
                    pos_m,
                    vel_m_s,
                    acc_m_s2,
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

    fn link_coverage(&self, link: DirectedLink) -> CoverageSet {
        let Some(slot) = self.edge_lists.get(link.seg_indices_high_to_low) else {
            return CoverageSet::empty();
        };
        let mut spans = Vec::with_capacity(slot.candidates.len());
        for &(seg_idx, _) in &slot.candidates {
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
            if let Some(slot) = self.edge_lists.get(link.seg_indices_high_to_low) {
                for &(seg_idx, _) in &slot.candidates {
                    if let Some(seg) = self.segments.get(seg_idx) {
                        seg.collect_interior_knots_in(start, end, &mut knots);
                    }
                }
            }
        }
        knots.sort();
        knots.dedup();
        knots
    }
}
