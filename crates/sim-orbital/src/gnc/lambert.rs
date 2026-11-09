//! Universal-variable Lambert boundary-value solver and finite-burn lead-time targeter.

use glam::DVec3;
use serde::{Deserialize, Serialize};

use crate::{
    kepler::stumpff_c2_c3,
    scalar_math::{acos_clamped, sqrt},
};

/// Error returned when a universal-variable Lambert boundary-value problem is ill-posed or fails to converge.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize, thiserror::Error)]
pub enum LambertError {
    /// Non-positive or non-finite time of flight or gravitational parameter.
    #[error("invalid Lambert inputs: tof_s = {tof_s}, mu_m3_s2 = {mu_m3_s2}")]
    InvalidInput {
        /// Requested transfer time of flight (`s`).
        tof_s: f64,
        /// Central body gravitational parameter (`m^3/s^2`).
        mu_m3_s2: f64,
    },
    /// Initial or terminal radius is zero, non-finite, or below the `1.0 m` numerical minimum.
    #[error("degenerate Lambert radius: r1_m = {r1_m}, r2_m = {r2_m}")]
    DegenerateRadius {
        /// Initial radius magnitude (`m`).
        r1_m: f64,
        /// Terminal radius magnitude (`m`).
        r2_m: f64,
    },
    /// Initial and terminal position vectors are collinear (`0` or `pi` rad), leaving the transfer plane undefined.
    #[error("collinear transfer geometry (transfer plane undefined): cos(dnu) = {cos_dnu}")]
    CollinearTransferGeometry {
        /// Cosine of the angle between `r1_m` and `r2_m`.
        cos_dnu: f64,
    },
    /// No valid single-revolution universal-variable bracket `[psi_low, psi_high]` satisfies `y(psi) > 0` for `tof_s`.
    #[error(
        "infeasible single-revolution Lambert transfer for tof_s = {tof_s} (a_geom = {a_geom})"
    )]
    InfeasibleTransfer {
        /// Requested transfer time of flight (`s`).
        tof_s: f64,
        /// Universal geometry parameter `A` (`m`).
        a_geom: f64,
    },
    /// Universal-variable root-finder did not converge within the iteration budget.
    #[error(
        "Lambert solver did not converge in {iterations} iterations (residual_tof_s = {residual_tof_s})"
    )]
    DidNotConverge {
        /// Number of iterations executed.
        iterations: u32,
        /// Remaining time-of-flight residual (`s`).
        residual_tof_s: f64,
    },
}

/// Solution to Lambert's orbital boundary-value problem between `r1_m` and `r2_m` in `tof_s`.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct LambertSolution {
    /// Required departure velocity at `r1_m` (`m/s`).
    #[serde(with = "crate::plant::attitude_math::serde_dvec3")]
    pub v1_m_s: DVec3,
    /// Resulting arrival velocity at `r2_m` (`m/s`).
    #[serde(with = "crate::plant::attitude_math::serde_dvec3")]
    pub v2_m_s: DVec3,
    /// Converged universal parameter $\psi$.
    pub psi: f64,
    /// Swept transfer angle $\Delta\nu \in (0, 2\pi)$ (`rad`).
    pub transfer_angle_rad: f64,
    /// Number of safeguarded secant-bisection iterations used.
    pub iterations: u32,
}

/// Finite-burn maneuver plan derived from an impulsive $\Delta\mathbf{v}$ and acceleration limit.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct FiniteBurnPlan {
    /// Unit burn direction in `ICRF`.
    #[serde(with = "crate::plant::attitude_math::serde_dvec3")]
    pub unit_dir_icrf: DVec3,
    /// Total $\Delta v$ magnitude (`m/s`).
    pub delta_v_m_s: f64,
    /// Commanded proper acceleration during burn (`m/s^2`, `<= 39.2266`).
    pub accel_m_s2: f64,
    /// Finite burn duration (`s`).
    pub duration_s: f64,
    /// Lead time (`s`) before nominal impulsive epoch to ignite so mid-burn aligns with impulse.
    pub ignition_lead_s: f64,
}

impl FiniteBurnPlan {
    /// Builds a finite-burn plan from an impulsive velocity change `delta_v_icrf_m_s` and `max_accel_m_s2`.
    #[must_use]
    pub fn from_impulsive(delta_v_icrf_m_s: DVec3, max_accel_m_s2: f64) -> Self {
        let dv = sqrt(delta_v_icrf_m_s.length_squared());
        let a = max_accel_m_s2.max(1.0e-3);
        let dir = if dv > 1.0e-9 {
            delta_v_icrf_m_s / dv
        } else {
            DVec3::Y
        };
        let dur = dv / a;
        Self {
            unit_dir_icrf: dir,
            delta_v_m_s: dv,
            accel_m_s2: a,
            duration_s: dur,
            ignition_lead_s: 0.5 * dur,
        }
    }
}

fn evaluate_lambert_tof(
    psi: f64,
    r1: f64,
    r2: f64,
    a_geom: f64,
    sqrt_mu: f64,
) -> Option<(f64, f64)> {
    let (c2, c3) = stumpff_c2_c3(psi);
    if c2 <= 1.0e-30 {
        return None;
    }
    let sqrt_c2 = sqrt(c2);
    let y = r1 + r2 + a_geom * (psi * c3 - 1.0) / sqrt_c2;
    if y <= 0.0 {
        return None;
    }
    let chi = sqrt(y / c2);
    let tof = (chi * chi * chi * c3 + a_geom * sqrt(y)) / sqrt_mu;
    if tof.is_finite() && tof > 0.0 {
        Some((y, tof))
    } else {
        None
    }
}

fn find_lambert_bracket(
    r1: f64,
    r2: f64,
    a_geom: f64,
    tof_s: f64,
    sqrt_mu: f64,
) -> Result<(f64, f64), LambertError> {
    let psi_asymptotic_max = 4.0 * core::f64::consts::PI * core::f64::consts::PI - 1.0e-4;
    let mut psi_low = -4.0_f64;
    let mut psi_high = 4.0_f64;

    if a_geom > 0.0 {
        while evaluate_lambert_tof(psi_low, r1, r2, a_geom, sqrt_mu).is_some() && psi_low > -1.0e12
        {
            psi_low *= 2.0;
        }
        let mut probe_hi = 0.0_f64;
        for _ in 0..64_u32 {
            let mid = 0.5 * (psi_low + probe_hi);
            if evaluate_lambert_tof(mid, r1, r2, a_geom, sqrt_mu).is_some() {
                probe_hi = mid;
            } else {
                psi_low = mid;
            }
        }
        psi_low = probe_hi;
    } else {
        for _ in 0..32_u32 {
            if let Some((_, t_low)) = evaluate_lambert_tof(psi_low, r1, r2, a_geom, sqrt_mu)
                && t_low <= tof_s
            {
                break;
            }
            psi_low *= 2.0;
        }
    }

    let Some((_, t_low)) = evaluate_lambert_tof(psi_low, r1, r2, a_geom, sqrt_mu) else {
        return Err(LambertError::InfeasibleTransfer { tof_s, a_geom });
    };
    for _ in 0..48_u32 {
        if let Some((_, t_high)) = evaluate_lambert_tof(psi_high, r1, r2, a_geom, sqrt_mu)
            && t_high >= tof_s
        {
            break;
        }
        psi_high = 0.5 * (psi_high + psi_asymptotic_max);
    }
    let Some((_, t_high)) = evaluate_lambert_tof(psi_high, r1, r2, a_geom, sqrt_mu) else {
        return Err(LambertError::InfeasibleTransfer { tof_s, a_geom });
    };
    if t_low > tof_s || t_high < tof_s {
        return Err(LambertError::InfeasibleTransfer { tof_s, a_geom });
    }
    Ok((psi_low, psi_high))
}

/// Solves the 0-revolution universal-variable Lambert problem from `r1_m` to `r2_m` across
/// `tof_s` (`s`) under `mu_m3_s2` (`m^3/s^2`).
///
/// When `long_way` is `false`, solves for the short-way transfer ($\Delta\nu \in (0, \pi)$,
/// angular momentum aligned with $+\mathbf{r}_1 \times \mathbf{r}_2$). When `long_way` is `true`,
/// solves for the long-way transfer ($\Delta\nu \in (\pi, 2\pi)$, angular momentum aligned with
/// $-\mathbf{r}_1 \times \mathbf{r}_2$).
pub fn solve_lambert_universal(
    r1_m: DVec3,
    r2_m: DVec3,
    tof_s: f64,
    mu_m3_s2: f64,
    long_way: bool,
) -> Result<LambertSolution, LambertError> {
    if !tof_s.is_finite() || !mu_m3_s2.is_finite() || tof_s <= 0.0 || mu_m3_s2 <= 0.0 {
        return Err(LambertError::InvalidInput { tof_s, mu_m3_s2 });
    }
    if !r1_m.is_finite() || !r2_m.is_finite() {
        return Err(LambertError::DegenerateRadius {
            r1_m: f64::NAN,
            r2_m: f64::NAN,
        });
    }
    let r1 = sqrt(r1_m.length_squared());
    let r2 = sqrt(r2_m.length_squared());
    if r1 <= 1.0 || r2 <= 1.0 {
        return Err(LambertError::DegenerateRadius { r1_m: r1, r2_m: r2 });
    }
    let cos_dnu = (r1_m.dot(r2_m) / (r1 * r2)).clamp(-1.0, 1.0);
    if (1.0 - cos_dnu.abs()) <= 1.0e-12 {
        return Err(LambertError::CollinearTransferGeometry { cos_dnu });
    }
    let base_angle = acos_clamped(cos_dnu);
    let transfer_angle_rad = if long_way {
        2.0 * core::f64::consts::PI - base_angle
    } else {
        base_angle
    };
    let dir_sign = if long_way { -1.0 } else { 1.0 };
    let a_geom = dir_sign * sqrt(r1 * r2 * (1.0 + cos_dnu));

    let sqrt_mu = sqrt(mu_m3_s2);
    let (mut psi_low, mut psi_high) = find_lambert_bracket(r1, r2, a_geom, tof_s, sqrt_mu)?;
    let mut psi = 0.0_f64.clamp(psi_low, psi_high);
    let tol_s = 1.0e-11 * (1.0 + tof_s);
    let mut last_err = f64::INFINITY;

    for iter in 0..128_u32 {
        let Some((y_pos, tof_eval)) = evaluate_lambert_tof(psi, r1, r2, a_geom, sqrt_mu) else {
            psi_low = psi;
            psi = 0.5 * (psi_low + psi_high);
            continue;
        };
        let err = tof_eval - tof_s;
        last_err = err;
        if err.abs() <= tol_s || (psi_high - psi_low).abs() <= 1.0e-14 * (1.0 + psi.abs()) {
            let f = 1.0 - y_pos / r1;
            let g = a_geom * sqrt(y_pos / mu_m3_s2);
            let gdot = 1.0 - y_pos / r2;
            return Ok(LambertSolution {
                v1_m_s: (r2_m - f * r1_m) / g,
                v2_m_s: (gdot * r2_m - r1_m) / g,
                psi,
                transfer_angle_rad,
                iterations: iter.wrapping_add(1),
            });
        }
        if err < 0.0 {
            psi_low = psi;
        } else {
            psi_high = psi;
        }
        psi = 0.5 * (psi_low + psi_high);
    }

    Err(LambertError::DidNotConverge {
        iterations: 128,
        residual_tof_s: last_err,
    })
}
