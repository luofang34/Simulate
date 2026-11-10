//! Stacked precision binary-PCK (`ITRF93`, `MOON_PA_DE440`, `MOON_ME_DE440_ME421`)
//! and IAU text-PCK orientation catalog.

use std::collections::BTreeMap;

use glam::DMat3;
use serde::{Deserialize, Serialize};

use crate::{
    daf::{
        BpcFile, BpcSegment,
        bpc::{rot_axis_1, rot_axis_2, rot_axis_3},
    },
    hash::{ContentHash, DomainHasher},
    ids::{FrameCode, NaifId},
    kpl::KplPool,
    time::{CoverageSet, TdbEpoch, TdbSpanNs},
};

use super::{
    BodyOrientationCatalog, BodyRotationState, IauPckCatalog, OrientationError, OrientationPolicy,
    SurfaceNavGrade, coverage_for_grade,
};

/// Which lunar body-fixed frame `StackedOrientationCatalog` outputs for `NaifId::MOON (301)`.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum LunarTargetFrame {
    /// Lunar Mean-Earth / Polar-Axis (`MOON_ME = 31001`, cartographic standard for LOLA DEMs & landing coordinates).
    MeanEarth,
    /// Lunar Principal Axes (`MOON_PA = 31000`, dynamical principal-inertia frame).
    PrincipalAxes,
}

#[derive(Clone, Debug)]
struct PrecisionTrack {
    output_frame: FrameCode,
    left_offset: DMat3,
    segments_high_to_low: Vec<BpcSegment>,
    coverage: CoverageSet,
}

impl PrecisionTrack {
    fn new(output_frame: FrameCode, left_offset: DMat3, mut segments: Vec<BpcSegment>) -> Self {
        segments.reverse();
        let spans = segments.iter().map(BpcSegment::span_ns).collect();
        let coverage = CoverageSet::from_spans(spans);
        Self {
            output_frame,
            left_offset,
            segments_high_to_low: segments,
            coverage,
        }
    }

    fn evaluate(&self, epoch: TdbEpoch) -> Result<Option<BodyRotationState>, OrientationError> {
        for seg in &self.segments_high_to_low {
            if seg.contains(epoch) {
                let raw = seg.evaluate_icrf_to_body(epoch)?;
                return Ok(Some(BodyRotationState {
                    r_icrf_to_fixed: self.left_offset * raw.r_base_to_body,
                    omega_body_rad_s: self.left_offset * raw.omega_body_rad_s,
                    alpha_body_rad_s2: self.left_offset * raw.alpha_body_rad_s2,
                    frame_code: self.output_frame,
                    grade: SurfaceNavGrade::PrecisionGeodetic,
                }));
            }
        }
        Ok(None)
    }
}

/// Multi-tier orientation catalog combining binary PCK tracks (`ITRF93`, `MOON_PA_DE440` + `MOON_ME` FK)
/// with `IauPckCatalog` under explicit per-body [`OrientationPolicy`] rules.
#[derive(Clone, Debug)]
pub struct StackedOrientationCatalog {
    hash: ContentHash,
    iau: IauPckCatalog,
    precision: BTreeMap<NaifId, PrecisionTrack>,
    policies: BTreeMap<NaifId, OrientationPolicy>,
    default_policy: OrientationPolicy,
}

impl StackedOrientationCatalog {
    /// Builds a stacked catalog from an [`IauPckCatalog`], optional Earth `ITRF93` `.bpc` files,
    /// optional Lunar `MOON_PA_DE440` `.bpc` files + `moon_de440_*.tf` frame kernel, and per-body policies.
    pub fn build(
        iau: IauPckCatalog,
        earth_bpc_files: Vec<BpcFile>,
        moon_bpc_files: Vec<BpcFile>,
        moon_fk_bytes: Option<&[u8]>,
        lunar_target: LunarTargetFrame,
        policies: BTreeMap<NaifId, OrientationPolicy>,
        default_policy: OrientationPolicy,
    ) -> Result<Self, OrientationError> {
        let mut h = DomainHasher::new(b"sim-orbital:stacked-orientation:v1");
        h.write_hash(iau.content_hash());
        h.write_u64(earth_bpc_files.len() as u64);
        for f in &earth_bpc_files {
            h.write_hash(f.content_hash());
        }
        h.write_u64(moon_bpc_files.len() as u64);
        for f in &moon_bpc_files {
            h.write_hash(f.content_hash());
        }
        if let Some(fk) = moon_fk_bytes {
            h.write_bytes(fk);
        } else {
            h.write_u64(0);
        }
        h.write_i32(match lunar_target {
            LunarTargetFrame::MeanEarth => 0,
            LunarTargetFrame::PrincipalAxes => 1,
        });
        h.write_i32(policy_tag(default_policy));
        for (&body, &pol) in &policies {
            h.write_i32(body.0);
            h.write_i32(policy_tag(pol));
        }

        let mut precision = BTreeMap::new();
        let earth_segs: Vec<BpcSegment> = earth_bpc_files
            .into_iter()
            .flat_map(|f| f.segments().to_vec())
            .filter(|s| s.frame_class_id() == FrameCode::ITRF93)
            .collect();
        if !earth_segs.is_empty() {
            precision.insert(
                NaifId::EARTH,
                PrecisionTrack::new(FrameCode::ITRF93, DMat3::IDENTITY, earth_segs),
            );
        }

        let moon_segs: Vec<BpcSegment> = moon_bpc_files
            .into_iter()
            .flat_map(|f| f.segments().to_vec())
            .filter(|s| s.frame_class_id() == FrameCode::MOON_PA_DE440)
            .collect();
        if !moon_segs.is_empty() {
            let (frame_code, left_offset) = match lunar_target {
                LunarTargetFrame::PrincipalAxes => (FrameCode::MOON_PA, DMat3::IDENTITY),
                LunarTargetFrame::MeanEarth => {
                    let fk_raw = moon_fk_bytes.ok_or_else(|| {
                        OrientationError::MalformedFrameKernel(
                            "LunarTargetFrame::MeanEarth requires moon_de440 frame kernel"
                                .to_owned(),
                        )
                    })?;
                    (FrameCode::MOON_ME, parse_lunar_me_from_pa_fk(fk_raw)?)
                }
            };
            precision.insert(
                NaifId::MOON,
                PrecisionTrack::new(frame_code, left_offset, moon_segs),
            );
        }

        Ok(Self {
            hash: h.finish(),
            iau,
            precision,
            policies,
            default_policy,
        })
    }

    /// Underlying approximate IAU trigonometric catalog.
    #[must_use]
    pub const fn iau_catalog(&self) -> &IauPckCatalog {
        &self.iau
    }

    /// Effective policy configured for `body`.
    #[must_use]
    pub fn policy_for(&self, body: NaifId) -> OrientationPolicy {
        self.policies
            .get(&body)
            .copied()
            .unwrap_or(self.default_policy)
    }
}

impl BodyOrientationCatalog for StackedOrientationCatalog {
    fn content_hash(&self) -> ContentHash {
        self.hash
    }

    fn catalog_model_id(&self) -> &str {
        "naif-stacked-bpc-iau"
    }

    fn coverage(&self, body: NaifId, min_grade: SurfaceNavGrade) -> Vec<TdbSpanNs> {
        let empty = CoverageSet::empty();
        let prec_cov = self
            .precision
            .get(&body)
            .map_or(&empty, |track| &track.coverage);
        let allow_iau = self.policy_for(body) == OrientationPolicy::AllowApproximateIau
            && self.iau.contains_body(body);
        coverage_for_grade(prec_cov, allow_iau, min_grade)
    }

    fn body_rotation(
        &self,
        body: NaifId,
        epoch: TdbEpoch,
    ) -> Result<BodyRotationState, OrientationError> {
        if let Some(track) = self.precision.get(&body)
            && let Some(rot) = track.evaluate(epoch)?
        {
            return Ok(rot);
        }
        match self.policy_for(body) {
            OrientationPolicy::RequirePrecision => Err(OrientationError::OutOfPrecisionCoverage {
                body,
                epoch_j2000_s: epoch.as_j2000_seconds_f64(),
            }),
            OrientationPolicy::AllowApproximateIau => self.iau.evaluate_iau(body, epoch),
        }
    }
}

const fn policy_tag(policy: OrientationPolicy) -> i32 {
    match policy {
        OrientationPolicy::RequirePrecision => 0,
        OrientationPolicy::AllowApproximateIau => 1,
    }
}

/// Parses `TKFRAME_31009_ANGLES` and `TKFRAME_31009_AXES` from `moon_de440_*.tf`, returning
/// the constant orthogonal matrix $R_{\text{PA}\to\text{ME}}$ (`v_ME = R_pa_to_me * v_PA`).
pub fn parse_lunar_me_from_pa_fk(fk_bytes: &[u8]) -> Result<DMat3, OrientationError> {
    let pool = KplPool::parse_bytes(fk_bytes)?;
    let spec = pool.first_text("TKFRAME_31009_SPEC").unwrap_or("");
    if !spec.eq_ignore_ascii_case("ANGLES") {
        return Err(OrientationError::MalformedFrameKernel(format!(
            "TKFRAME_31009_SPEC={spec}"
        )));
    }
    let units = pool.first_text("TKFRAME_31009_UNITS").unwrap_or("RADIANS");
    let scale = if units.eq_ignore_ascii_case("ARCSECONDS") {
        core::f64::consts::PI / (180.0 * 3_600.0)
    } else if units.eq_ignore_ascii_case("DEGREES") {
        core::f64::consts::PI / 180.0
    } else {
        1.0
    };
    let angles = pool.require_floats("TKFRAME_31009_ANGLES")?;
    let axes = pool.require_floats("TKFRAME_31009_AXES")?;
    if angles.len() != 3 || axes.len() != 3 {
        return Err(OrientationError::MalformedFrameKernel(
            "expected 3 Euler angles and axes".to_owned(),
        ));
    }
    let mut m_me_to_pa = DMat3::IDENTITY;
    let mut i = 0_usize;
    while i < 3 {
        let ang_rad = angles.get(i).copied().unwrap_or(0.0) * scale;
        let axis = axes.get(i).copied().unwrap_or(0.0) as i32;
        let r_i = match axis {
            1 => rot_axis_1(ang_rad),
            2 => rot_axis_2(ang_rad),
            3 => rot_axis_3(ang_rad),
            other => {
                return Err(OrientationError::MalformedFrameKernel(format!(
                    "invalid Euler axis {other}"
                )));
            }
        };
        m_me_to_pa *= r_i;
        i = i.wrapping_add(1);
    }
    Ok(m_me_to_pa.transpose())
}
