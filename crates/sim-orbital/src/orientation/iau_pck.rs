//! NAIF Text PCK (`pck00011.tpc`) IAU rotational-element series evaluator.
//!
//! Handles quadratic nutation-precession phase polynomials (`BODY<bary>_MAX_PHASE_DEGREE = 2`,
//! required by Mars `BODY4` in N0067+ `pck00011.tpc`) and computes exact analytical first
//! and second time derivatives of $(\alpha_0, \delta_0, W)$.

use std::collections::BTreeMap;

use crate::{
    daf::bpc::{euler_313_kinematics, rot_axis_3},
    hash::ContentHash,
    ids::NaifId,
    kpl::KplPool,
    scalar_math::{mul_add_unfused, sin_cos},
    time::{TdbEpoch, TdbSpanNs},
};

use super::{
    BodyOrientationCatalog, BodyRotationState, OrientationError, SurfaceNavGrade,
    coverage_for_grade, default_iau_frame_code,
};

const DEG_TO_RAD: f64 = core::f64::consts::PI / 180.0;
const SEC_PER_DAY: f64 = 86_400.0;
const SEC_PER_JULIAN_CENTURY: f64 = 36_525.0 * SEC_PER_DAY;

#[derive(Clone, Copy, Debug)]
struct PhaseAnglePoly {
    c0_deg: f64,
    c1_deg_per_cy: f64,
    c2_deg_per_cy2: f64,
}

#[derive(Clone, Debug)]
struct IauBodySeries {
    pole_ra_deg: [f64; 3],
    pole_dec_deg: [f64; 3],
    pm_deg: [f64; 3],
    long_axis_deg: f64,
    nut_ra_deg: Vec<f64>,
    nut_dec_deg: Vec<f64>,
    nut_pm_deg: Vec<f64>,
    phases: Vec<PhaseAnglePoly>,
}

type ScalarJet2 = (f64, f64, f64);

impl IauBodySeries {
    fn evaluate(&self, body: NaifId, epoch: TdbEpoch) -> BodyRotationState {
        let (alpha, delta, pm) = self.evaluate_angles_deg(epoch);
        let half_pi = core::f64::consts::FRAC_PI_2;
        let phi = (
            alpha.0 * DEG_TO_RAD + half_pi,
            alpha.1 * DEG_TO_RAD,
            alpha.2 * DEG_TO_RAD,
        );
        let theta = (
            half_pi - delta.0 * DEG_TO_RAD,
            -delta.1 * DEG_TO_RAD,
            -delta.2 * DEG_TO_RAD,
        );
        let psi = (pm.0 * DEG_TO_RAD, pm.1 * DEG_TO_RAD, pm.2 * DEG_TO_RAD);
        let mut kin = euler_313_kinematics(phi, theta, psi);
        if self.long_axis_deg != 0.0 {
            let c_la = rot_axis_3(self.long_axis_deg * DEG_TO_RAD);
            kin.r_base_to_body = c_la * kin.r_base_to_body;
            kin.omega_body_rad_s = c_la * kin.omega_body_rad_s;
            kin.alpha_body_rad_s2 = c_la * kin.alpha_body_rad_s2;
        }
        BodyRotationState {
            r_icrf_to_fixed: kin.r_base_to_body,
            omega_body_rad_s: kin.omega_body_rad_s,
            alpha_body_rad_s2: kin.alpha_body_rad_s2,
            frame_code: default_iau_frame_code(body),
            grade: SurfaceNavGrade::ApproximateIau,
        }
    }

    fn evaluate_angles_deg(&self, epoch: TdbEpoch) -> (ScalarJet2, ScalarJet2, ScalarJet2) {
        let (whole_s, sub_s) = epoch.split_j2000_seconds();
        let t_s = whole_s + sub_s;
        let t_cy = t_s / SEC_PER_JULIAN_CENTURY;
        let inv_cy = 1.0 / SEC_PER_JULIAN_CENTURY;
        let inv_cy2 = inv_cy * inv_cy;
        let inv_day = 1.0 / SEC_PER_DAY;

        let [ra0, ra1, ra2] = self.pole_ra_deg;
        let mut alpha = (
            mul_add_unfused(t_cy, mul_add_unfused(t_cy, ra2, ra1), ra0),
            mul_add_unfused(2.0 * t_cy, ra2, ra1) * inv_cy,
            (2.0 * ra2) * inv_cy2,
        );
        let [dec0, dec1, dec2] = self.pole_dec_deg;
        let mut delta = (
            mul_add_unfused(t_cy, mul_add_unfused(t_cy, dec2, dec1), dec0),
            mul_add_unfused(2.0 * t_cy, dec2, dec1) * inv_cy,
            (2.0 * dec2) * inv_cy2,
        );
        let [w0, w1, w2] = self.pm_deg;
        let d_whole = whole_s * inv_day;
        let d_sub = sub_s * inv_day;
        let d_total = d_whole + d_sub;
        let w1_turns = (w1 * d_whole) % 360.0 + w1 * d_sub;
        let mut pm = (
            (w0 + w1_turns + w2 * d_total * d_total) % 360.0,
            mul_add_unfused(2.0 * d_total, w2, w1) * inv_day,
            (2.0 * w2) * inv_day * inv_day,
        );
        self.accumulate_nutation(t_cy, inv_cy, inv_cy2, &mut alpha, &mut delta, &mut pm);
        (alpha, delta, pm)
    }

    fn accumulate_nutation(
        &self,
        t_cy: f64,
        inv_cy: f64,
        inv_cy2: f64,
        alpha: &mut (f64, f64, f64),
        delta: &mut (f64, f64, f64),
        pm: &mut (f64, f64, f64),
    ) {
        let mut idx = 0_usize;
        while let Some(ph) = self.phases.get(idx) {
            let th_deg = mul_add_unfused(
                t_cy,
                mul_add_unfused(t_cy, ph.c2_deg_per_cy2, ph.c1_deg_per_cy),
                ph.c0_deg,
            );
            let dth = mul_add_unfused(2.0 * t_cy, ph.c2_deg_per_cy2, ph.c1_deg_per_cy)
                * inv_cy
                * DEG_TO_RAD;
            let ddth = (2.0 * ph.c2_deg_per_cy2) * inv_cy2 * DEG_TO_RAD;
            let (s_th, c_th) = sin_cos((th_deg % 360.0) * DEG_TO_RAD);

            let a_ra = self.nut_ra_deg.get(idx).copied().unwrap_or(0.0);
            if a_ra != 0.0 {
                alpha.0 += a_ra * s_th;
                alpha.1 += a_ra * c_th * dth;
                alpha.2 += a_ra * (c_th * ddth - s_th * dth * dth);
            }
            let a_dec = self.nut_dec_deg.get(idx).copied().unwrap_or(0.0);
            if a_dec != 0.0 {
                delta.0 += a_dec * c_th;
                delta.1 -= a_dec * s_th * dth;
                delta.2 -= a_dec * (s_th * ddth + c_th * dth * dth);
            }
            let a_pm = self.nut_pm_deg.get(idx).copied().unwrap_or(0.0);
            if a_pm != 0.0 {
                pm.0 += a_pm * s_th;
                pm.1 += a_pm * c_th * dth;
                pm.2 += a_pm * (c_th * ddth - s_th * dth * dth);
            }
            idx = idx.wrapping_add(1);
        }
    }
}

/// Approximate IAU trigonometric orientation catalog parsed from `pck00011.tpc`.
#[derive(Clone, Debug)]
pub struct IauPckCatalog {
    hash: ContentHash,
    models: BTreeMap<NaifId, IauBodySeries>,
}

impl IauPckCatalog {
    /// Parses all `BODY<id>_POLE_RA` / `POLE_DEC` / `PM` series from `pck00011.tpc` bytes.
    pub fn from_pck_bytes(bytes: &[u8]) -> Result<Self, OrientationError> {
        let pool = KplPool::parse_bytes(bytes)?;
        let mut models = BTreeMap::new();

        for (var_name, _) in pool.iter() {
            let Some(id) = parse_pole_ra_body(var_name) else {
                continue;
            };
            if let Some(series) = build_body_series(&pool, id) {
                models.insert(id, series);
            }
        }
        Ok(Self {
            hash: pool.content_hash(),
            models,
        })
    }

    /// Whether an IAU trigonometric model exists for `body`.
    #[must_use]
    pub fn contains_body(&self, body: NaifId) -> bool {
        self.models.contains_key(&body)
    }

    /// Evaluates the approximate IAU orientation of `body` at `epoch`.
    pub fn evaluate_iau(
        &self,
        body: NaifId,
        epoch: TdbEpoch,
    ) -> Result<BodyRotationState, OrientationError> {
        let series = self
            .models
            .get(&body)
            .ok_or(OrientationError::MissingBodyModel(body))?;
        Ok(series.evaluate(body, epoch))
    }
}

impl BodyOrientationCatalog for IauPckCatalog {
    fn content_hash(&self) -> ContentHash {
        self.hash
    }

    fn coverage(&self, body: NaifId, min_grade: SurfaceNavGrade) -> Vec<TdbSpanNs> {
        let empty = crate::time::CoverageSet::empty();
        coverage_for_grade(&empty, self.contains_body(body), min_grade)
    }

    fn body_rotation(
        &self,
        body: NaifId,
        epoch: TdbEpoch,
    ) -> Result<BodyRotationState, OrientationError> {
        self.evaluate_iau(body, epoch)
    }
}

fn parse_pole_ra_body(var_name: &str) -> Option<NaifId> {
    let inner = var_name.strip_prefix("BODY")?.strip_suffix("_POLE_RA")?;
    let code: i32 = inner.parse().ok()?;
    Some(NaifId(code))
}

fn poly3_from_slice(vals: &[f64]) -> [f64; 3] {
    [
        vals.first().copied().unwrap_or(0.0),
        vals.get(1).copied().unwrap_or(0.0),
        vals.get(2).copied().unwrap_or(0.0),
    ]
}

fn build_body_series(pool: &KplPool, id: NaifId) -> Option<IauBodySeries> {
    let prefix = format!("BODY{}", id.0);
    let pole_ra = poly3_from_slice(&pool.floats(&format!("{prefix}_POLE_RA"))?);
    let pole_dec = poly3_from_slice(&pool.floats(&format!("{prefix}_POLE_DEC"))?);
    let pm = poly3_from_slice(&pool.floats(&format!("{prefix}_PM"))?);
    let long_axis_deg = pool
        .floats(&format!("{prefix}_LONG_AXIS"))
        .and_then(|v| v.first().copied())
        .unwrap_or(0.0);

    let nut_ra_deg = pool
        .floats(&format!("{prefix}_NUT_PREC_RA"))
        .unwrap_or_default();
    let nut_dec_deg = pool
        .floats(&format!("{prefix}_NUT_PREC_DEC"))
        .unwrap_or_default();
    let nut_pm_deg = pool
        .floats(&format!("{prefix}_NUT_PREC_PM"))
        .unwrap_or_default();

    let bary_code = id.parent_planetary_barycenter().unwrap_or(id).0;
    let max_deg = pool
        .floats(&format!("BODY{bary_code}_MAX_PHASE_DEGREE"))
        .and_then(|v| v.first().copied())
        .map_or(1_usize, |d| (d as usize).clamp(1, 3));
    let stride = max_deg.wrapping_add(1);
    let raw_angles = pool
        .floats(&format!("BODY{bary_code}_NUT_PREC_ANGLES"))
        .unwrap_or_default();

    let mut phases = Vec::new();
    let mut k = 0_usize;
    while k.wrapping_add(stride) <= raw_angles.len() {
        let c0_deg = raw_angles.get(k).copied().unwrap_or(0.0);
        let c1_deg_per_cy = raw_angles.get(k.wrapping_add(1)).copied().unwrap_or(0.0);
        let c2_deg_per_cy2 = if stride >= 3 {
            raw_angles.get(k.wrapping_add(2)).copied().unwrap_or(0.0)
        } else {
            0.0
        };
        phases.push(PhaseAnglePoly {
            c0_deg,
            c1_deg_per_cy,
            c2_deg_per_cy2,
        });
        k = k.wrapping_add(stride);
    }

    Some(IauBodySeries {
        pole_ra_deg: pole_ra,
        pole_dec_deg: pole_dec,
        pm_deg: pm,
        long_axis_deg,
        nut_ra_deg,
        nut_dec_deg,
        nut_pm_deg,
        phases,
    })
}
