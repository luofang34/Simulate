//! Short-term encounter B-plane geometry and 2D Gaussian collision probability
//! evaluated via `libm`-pure adaptive Gauss–Kronrod `G7-K15` quadrature.

use glam::{DMat3, DVec3};
use serde::{Deserialize, Serialize};

use crate::variational::{StateCovariance6, VariationalError};

const INV_SQRT_2PI: f64 = 0.398_942_280_401_432_7;
const INV_SQRT_PI: f64 = 0.564_189_583_547_756_3;
const INV_SQRT_2: f64 = 0.707_106_781_186_547_5;
const GAUSS_TAIL_SIGMA: f64 = 38.0;
const MAX_ADAPTIVE_PANELS: usize = 256;

/// Non-negative 15-point Gauss–Kronrod `(node, k15_weight, g7_weight)` triples on `[-1, 1]`.
const G7_K15_TABLE: [(f64, f64, f64); 8] = [
    (9.914_553_711_208_126e-1, 2.293_532_201_052_922_4e-2, 0.0),
    (
        9.491_079_123_427_585e-1,
        6.309_209_262_997_856e-2,
        1.294_849_661_688_697e-1,
    ),
    (8.648_644_233_597_691e-1, 1.047_900_103_222_501_9e-1, 0.0),
    (
        7.415_311_855_993_945e-1,
        1.406_532_597_155_259_2e-1,
        2.797_053_914_892_766_5e-1,
    ),
    (5.860_872_354_676_911e-1, 1.690_047_266_392_679e-1, 0.0),
    (
        4.058_451_513_773_972e-1,
        1.903_505_780_647_854_2e-1,
        3.818_300_505_051_189e-1,
    ),
    (2.077_849_550_078_984_8e-1, 2.044_329_400_752_988_9e-1, 0.0),
    (0.0, 2.094_821_410_847_278_2e-1, 4.179_591_836_734_694e-1),
];

/// Positive 8-point Gauss–Legendre `(node, weight)` pairs on `[-1, 1]`.
const GL8_HALF_TABLE: [(f64, f64); 4] = [
    (9.602_898_564_975_363e-1, 1.012_285_362_903_762_6e-1),
    (7.966_664_774_136_267e-1, 2.223_810_344_533_744_8e-1),
    (5.255_324_099_163_29e-1, 3.137_066_458_778_872_7e-1),
    (1.834_346_424_956_498e-1, 3.626_837_833_783_62e-1),
];

/// Result of projecting an encounter into the B-plane and integrating collision probability $P_c$.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct BPlaneEncounterResult {
    /// Short-term encounter collision probability $P_c \in [0, 1]$.
    pub probability: f64,
    /// Squared Mahalanobis miss distance $d_M^2 = x_m^2/\sigma_x^2 + y_m^2/\sigma_y^2$ in the B-plane.
    pub mahalanobis_distance_sq: f64,
    /// B-plane miss distance $\|\mathbf{r}_b\| = \sqrt{x_m^2 + y_m^2}$ ($\text{m}$).
    pub miss_distance_m: f64,
    /// Principal-axis $1\sigma$ position standard deviations $[\sigma_x, \sigma_y]$ in the B-plane ($\text{m}$, ordered $\sigma_x \ge \sigma_y$).
    pub b_plane_sigmas_m: [f64; 2],
    /// Principal-axis miss components $[x_m, y_m]$ in the B-plane ($\text{m}$).
    pub miss_b_plane_m: [f64; 2],
}

/// Evaluates the short-term linear-encounter B-plane collision probability $P_c$ over the
/// hard-body disk of combined radius `combined_radius_m` ($R_c > 0$).
pub fn compute_bplane_collision_probability(
    primary_pos_icrf_m: DVec3,
    primary_vel_icrf_m_s: DVec3,
    primary_cov_icrf: &StateCovariance6,
    secondary_pos_icrf_m: DVec3,
    secondary_vel_icrf_m_s: DVec3,
    secondary_cov_icrf: &StateCovariance6,
    combined_radius_m: f64,
) -> Result<BPlaneEncounterResult, VariationalError> {
    if !combined_radius_m.is_finite() || combined_radius_m <= 0.0 {
        return Err(VariationalError::InvalidCombinedRadius { combined_radius_m });
    }
    if !primary_pos_icrf_m.is_finite()
        || !primary_vel_icrf_m_s.is_finite()
        || !secondary_pos_icrf_m.is_finite()
        || !secondary_vel_icrf_m_s.is_finite()
    {
        return Err(VariationalError::NonFiniteState);
    }
    if !primary_cov_icrf.is_positive_semidefinite()
        || !secondary_cov_icrf.is_positive_semidefinite()
    {
        return Err(VariationalError::NonPositiveSemidefiniteCovariance);
    }

    let r_rel = secondary_pos_icrf_m - primary_pos_icrf_m;
    let v_rel = secondary_vel_icrf_m_s - primary_vel_icrf_m_s;
    let (_, t_hat, n_hat) = bplane_triad(v_rel)?;
    let (b_t, b_n) = (r_rel.dot(t_hat), r_rel.dot(n_hat));

    let p_sum = primary_cov_icrf.position_cov_m2() + secondary_cov_icrf.position_cov_m2();
    let (sigma_x, sigma_y, cos_rot, sin_rot) = project_and_eigendecompose(p_sum, t_hat, n_hat)?;
    let x_m = b_t * cos_rot + b_n * sin_rot;
    let y_m = -b_t * sin_rot + b_n * cos_rot;
    let miss_distance_m = libm::hypot(x_m, y_m);
    let dx = x_m / sigma_x;
    let dy = y_m / sigma_y;
    let mahalanobis_distance_sq = dx * dx + dy * dy;

    let probability =
        integrate_bplane_disk_probability(x_m, y_m, sigma_x, sigma_y, combined_radius_m)
            .clamp(0.0, 1.0);

    Ok(BPlaneEncounterResult {
        probability,
        mahalanobis_distance_sq,
        miss_distance_m,
        b_plane_sigmas_m: [sigma_x, sigma_y],
        miss_b_plane_m: [x_m, y_m],
    })
}

fn bplane_triad(v_rel: DVec3) -> Result<(DVec3, DVec3, DVec3), VariationalError> {
    let v_sq = v_rel.length_squared();
    if !v_sq.is_finite() || v_sq <= 0.0 {
        return Err(VariationalError::DegenerateEncounter);
    }
    let s_hat = v_rel / libm::sqrt(v_sq);
    let ref_axis = if s_hat.z.abs() < 0.9 {
        DVec3::Z
    } else {
        DVec3::X
    };
    let t_raw = s_hat.cross(ref_axis);
    let t_hat = t_raw / libm::sqrt(t_raw.length_squared());
    let n_raw = s_hat.cross(t_hat);
    let n_hat = n_raw / libm::sqrt(n_raw.length_squared());
    Ok((s_hat, t_hat, n_hat))
}

fn project_and_eigendecompose(
    p_sum: DMat3,
    t_hat: DVec3,
    n_hat: DVec3,
) -> Result<(f64, f64, f64, f64), VariationalError> {
    let a = t_hat.dot(p_sum * t_hat);
    let b = n_hat.dot(p_sum * n_hat);
    let c = 0.5 * (t_hat.dot(p_sum * n_hat) + n_hat.dot(p_sum * t_hat));
    if !a.is_finite() || !b.is_finite() || !c.is_finite() {
        return Err(VariationalError::NonFiniteState);
    }

    let m = 0.5 * (a + b);
    let d = 0.5 * (a - b);
    let r = libm::hypot(d, c);
    let lambda_1 = m + r;
    let lambda_2 = if m - r > 0.1 * m {
        m - r
    } else if lambda_1 > 0.0 {
        (a * b - c * c) / lambda_1
    } else {
        m - r
    };

    if !lambda_1.is_finite() || !lambda_2.is_finite() || lambda_1 <= 0.0 || lambda_2 <= 0.0 {
        return Err(VariationalError::SingularBPlaneCovariance);
    }
    let (sin_rot, cos_rot) = if c.abs() <= 1.0e-300 * m.max(1.0) {
        if a >= b { (0.0, 1.0) } else { (1.0, 0.0) }
    } else {
        libm::sincos(0.5 * libm::atan2(c, d))
    };
    Ok((libm::sqrt(lambda_1), libm::sqrt(lambda_2), cos_rot, sin_rot))
}

pub(crate) fn integrate_bplane_disk_probability(
    x_m: f64,
    y_m: f64,
    sigma_x: f64,
    sigma_y: f64,
    r_c: f64,
) -> f64 {
    let x_lo = (-r_c).max(x_m - GAUSS_TAIL_SIGMA * sigma_x);
    let x_hi = r_c.min(x_m + GAUSS_TAIL_SIGMA * sigma_x);
    if x_lo >= x_hi {
        return 0.0;
    }

    let theta_lo = libm::asin((x_lo / r_c).clamp(-1.0, 1.0));
    let theta_hi = libm::asin((x_hi / r_c).clamp(-1.0, 1.0));
    if theta_lo >= theta_hi {
        return 0.0;
    }

    let breaks = collect_theta_breakpoints(theta_lo, theta_hi, x_m, y_m, sigma_x, sigma_y, r_c);
    let inv_sqrt2_sigy = INV_SQRT_2 / sigma_y;
    let integrand = |theta: f64| -> f64 {
        let (sin_t, cos_t) = libm::sincos(theta);
        let cos_pos = cos_t.max(0.0);
        let x = r_c * sin_t;
        let h = r_c * cos_pos;
        let zx = (x - x_m) / sigma_x;
        let gx = (h * INV_SQRT_2PI / sigma_x) * libm::exp(-0.5 * zx * zx);
        if gx <= 0.0 {
            return 0.0;
        }
        let a = (-h - y_m) * inv_sqrt2_sigy;
        let b = (h - y_m) * inv_sqrt2_sigy;
        gx * normal_interval_prob(a, b)
    };

    adaptive_gk15_integrate(&breaks, &integrand)
}

fn collect_theta_breakpoints(
    theta_lo: f64,
    theta_hi: f64,
    x_m: f64,
    y_m: f64,
    sigma_x: f64,
    sigma_y: f64,
    r_c: f64,
) -> Vec<f64> {
    let mut pts = Vec::with_capacity(16);
    pts.push(theta_lo);
    pts.push(theta_hi);
    if 0.0 > theta_lo && 0.0 < theta_hi {
        pts.push(0.0);
    }
    for x in [x_m, x_m - 8.0 * sigma_x, x_m + 8.0 * sigma_x] {
        if x.abs() < r_c {
            let th = libm::asin((x / r_c).clamp(-1.0, 1.0));
            if th > theta_lo && th < theta_hi {
                pts.push(th);
            }
        }
    }
    let abs_ym = y_m.abs();
    for y_crit in [abs_ym - 8.0 * sigma_y, abs_ym, abs_ym + 8.0 * sigma_y] {
        if y_crit > 0.0 && y_crit < r_c {
            let th = libm::acos((y_crit / r_c).clamp(-1.0, 1.0));
            for signed in [-th, th] {
                if signed > theta_lo && signed < theta_hi {
                    pts.push(signed);
                }
            }
        }
    }
    pts.sort_by(f64::total_cmp);
    let mut dedup: Vec<f64> = Vec::with_capacity(pts.len());
    for p in pts {
        if dedup
            .last()
            .is_none_or(|&prev| (p - prev).abs() > 1.0e-13 * (1.0 + prev.abs()))
        {
            dedup.push(p);
        } else if let Some(last) = dedup.last_mut() {
            *last = (*last).max(p);
        }
    }
    if let Some(last) = dedup.last_mut() {
        *last = theta_hi;
    }
    dedup
}

#[derive(Clone, Copy)]
struct Panel {
    a: f64,
    b: f64,
    val: f64,
    err: f64,
}

fn adaptive_gk15_integrate<F: Fn(f64) -> f64>(breaks: &[f64], f: &F) -> f64 {
    let mut panels = Vec::with_capacity(32);
    let mut total_val = 0.0_f64;
    let mut total_err = 0.0_f64;
    for pair in breaks.windows(2) {
        let [a, b] = *pair else {
            continue;
        };
        if b <= a {
            continue;
        }
        let (val, err) = eval_gk15_panel(a, b, f);
        total_val += val;
        total_err += err;
        panels.push(Panel { a, b, val, err });
    }

    while panels.len() < MAX_ADAPTIVE_PANELS && total_err > 1.0e-13 * total_val.abs() {
        let Some((idx, worst)) = panels
            .iter()
            .enumerate()
            .max_by(|(_, p1), (_, p2)| p1.err.total_cmp(&p2.err))
        else {
            break;
        };
        let (a, b) = (worst.a, worst.b);
        let mid = 0.5 * (a + b);
        if mid <= a || mid >= b || (b - a) <= 1.0e-15 * (1.0 + a.abs() + b.abs()) {
            break;
        }
        let old_val = worst.val;
        let old_err = worst.err;
        let (vl, el) = eval_gk15_panel(a, mid, f);
        let (vr, er) = eval_gk15_panel(mid, b, f);
        total_val += (vl + vr) - old_val;
        total_err += (el + er) - old_err;
        if let Some(slot) = panels.get_mut(idx) {
            *slot = Panel {
                a,
                b: mid,
                val: vl,
                err: el,
            };
        }
        panels.push(Panel {
            a: mid,
            b,
            val: vr,
            err: er,
        });
    }
    total_val
}

pub(crate) fn eval_gk15_panel<F: Fn(f64) -> f64>(a: f64, b: f64, f: &F) -> (f64, f64) {
    let mid = 0.5 * (a + b);
    let half = 0.5 * (b - a);
    let mut sum_k15 = 0.0_f64;
    let mut sum_g7 = 0.0_f64;
    for &(node, wk, wg) in &G7_K15_TABLE {
        if node == 0.0 {
            let fc = f(mid);
            sum_k15 += wk * fc;
            sum_g7 += wg * fc;
        } else {
            let dx = half * node;
            let f_sum = f(mid - dx) + f(mid + dx);
            sum_k15 += wk * f_sum;
            if wg != 0.0 {
                sum_g7 += wg * f_sum;
            }
        }
    }
    let val_k15 = half * sum_k15;
    let val_g7 = half * sum_g7;
    (val_k15, (val_k15 - val_g7).abs())
}

/// Evaluates $\frac{1}{\sqrt{\pi}} \int_a^b e^{-t^2}\,dt$ for $a \le b$ without
/// catastrophic subtraction cancellation in deep tails or narrow intervals.
pub(crate) fn normal_interval_prob(a: f64, b: f64) -> f64 {
    if a >= b {
        return 0.0;
    }
    if a < 0.0 && b > 0.0 {
        return 0.5 * (libm::erf(b) + libm::erf(-a));
    }
    let (ap, bp) = if a >= 0.0 { (a, b) } else { (-b, -a) };
    if bp - ap <= 0.05 {
        let mid = 0.5 * (ap + bp);
        let half = 0.5 * (bp - ap);
        let mut sum = 0.0_f64;
        for &(node, weight) in &GL8_HALF_TABLE {
            let t_minus = mid - half * node;
            let t_plus = mid + half * node;
            sum += weight * (libm::exp(-t_minus * t_minus) + libm::exp(-t_plus * t_plus));
        }
        return half * INV_SQRT_PI * sum;
    }
    0.5 * (libm::erfc(ap) - libm::erfc(bp)).max(0.0)
}
