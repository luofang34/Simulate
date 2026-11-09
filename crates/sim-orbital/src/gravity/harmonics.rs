//! Nonsingular Cartesian solid-harmonic potential and gravity acceleration.
//!
//! Evaluates fully normalized Stokes coefficients $(\bar{C}_{nm}, \bar{S}_{nm})$ directly
//! in body-fixed Cartesian coordinates $(x, y, z)$ via the normalized Cunningham /
//! Montenbruck & Gill $(\bar{V}_{nm}, \bar{W}_{nm})$ recurrence, avoiding spherical
//! coordinate singularities at the North and South poles ($\phi = \pm 90^\circ$).

use glam::DVec3;

use crate::{constants::HarmonicTerm, scalar_math::sqrt};

/// Evaluated gravitational potential perturbation and body-fixed acceleration.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct HarmonicFieldEval {
    /// Specific gravitational potential energy perturbation $U_{\text{harm}}$ in $\text{J}/\text{kg}$
    /// (such that $\mathbf{a}_{\text{harm}} = -\nabla U_{\text{harm}}$).
    pub potential_j_kg: f64,
    /// Nonsingular harmonic gravity acceleration in body-fixed Cartesian axes ($\text{m}/\text{s}^2$).
    pub accel_fixed_m_s2: DVec3,
}

#[inline]
fn idx_tri(n: usize, m: usize, stride: usize) -> usize {
    n.saturating_mul(stride).saturating_add(m)
}

#[inline]
fn read_cell(buf: &[f64], n: usize, m: usize, stride: usize) -> f64 {
    buf.get(idx_tri(n, m, stride)).copied().unwrap_or(0.0)
}

#[inline]
fn write_cell(buf: &mut [f64], n: usize, m: usize, stride: usize, val: f64) {
    if let Some(slot) = buf.get_mut(idx_tri(n, m, stride)) {
        *slot = val;
    }
}

/// Evaluates the spherical-harmonic perturbation potential $U_{\text{harm}}$ and acceleration
/// $\mathbf{a}_{\text{harm}} = -\nabla U_{\text{harm}}$ in body-fixed coordinates.
///
/// Only harmonic terms with degree $n \ge 1$ and $0 \le m \le n$ are accumulated; the
/// central point-mass term ($n = 0$) is handled separately so third-body differential
/// Battin formulations remain cancellation-free.
#[must_use]
pub fn evaluate_spherical_harmonics_fixed(
    pos_fixed_m: DVec3,
    mu_m3_s2: f64,
    r_eq_m: f64,
    terms: &[HarmonicTerm],
) -> HarmonicFieldEval {
    if terms.is_empty() || r_eq_m <= 0.0 || mu_m3_s2 <= 0.0 {
        return HarmonicFieldEval {
            potential_j_kg: 0.0,
            accel_fixed_m_s2: DVec3::ZERO,
        };
    }
    let r_sq = pos_fixed_m.length_squared();
    if r_sq <= 0.0 || !r_sq.is_finite() {
        return HarmonicFieldEval {
            potential_j_kg: 0.0,
            accel_fixed_m_s2: DVec3::ZERO,
        };
    }

    let n_max = terms.iter().map(|t| t.degree as usize).max().unwrap_or(0);
    if n_max == 0 {
        return HarmonicFieldEval {
            potential_j_kg: 0.0,
            accel_fixed_m_s2: DVec3::ZERO,
        };
    }

    let max_rec = n_max.wrapping_add(1);
    let stride = max_rec.wrapping_add(1);
    let table_len = stride.saturating_mul(stride);
    let mut v_bar = vec![0.0_f64; table_len];
    let mut w_bar = vec![0.0_f64; table_len];

    populate_normalized_vw(
        pos_fixed_m,
        r_sq,
        r_eq_m,
        max_rec,
        stride,
        &mut v_bar,
        &mut w_bar,
    );
    accumulate_harmonic_terms(mu_m3_s2, r_eq_m, stride, &v_bar, &w_bar, terms)
}

fn populate_normalized_vw(
    pos_fixed_m: DVec3,
    r_sq: f64,
    r_eq_m: f64,
    max_rec: usize,
    stride: usize,
    v_bar: &mut [f64],
    w_bar: &mut [f64],
) {
    let r = sqrt(r_sq);
    let inv_r2 = 1.0 / r_sq;
    let xi = pos_fixed_m.x * r_eq_m * inv_r2;
    let eta = pos_fixed_m.y * r_eq_m * inv_r2;
    let zeta = pos_fixed_m.z * r_eq_m * inv_r2;
    let rho = (r_eq_m * r_eq_m) * inv_r2;

    write_cell(v_bar, 0, 0, stride, r_eq_m / r);
    write_cell(w_bar, 0, 0, stride, 0.0);

    let mut m: usize = 0;
    while m <= max_rec {
        if m >= 1 {
            let m_f = m as f64;
            let sec_scale = if m == 1 {
                sqrt(3.0)
            } else {
                sqrt((2.0 * m_f + 1.0) / (2.0 * m_f))
            };
            let vm1 = read_cell(v_bar, m - 1, m - 1, stride);
            let wm1 = read_cell(w_bar, m - 1, m - 1, stride);
            write_cell(v_bar, m, m, stride, sec_scale * (xi * vm1 - eta * wm1));
            write_cell(w_bar, m, m, stride, sec_scale * (xi * wm1 + eta * vm1));
        }

        let mut n = m.wrapping_add(1);
        while n <= max_rec {
            let n_f = n as f64;
            let m_f = m as f64;
            let denom = (n_f + m_f) * (n_f - m_f);
            let c1 = sqrt(((2.0 * n_f + 1.0) * (2.0 * n_f - 1.0)) / denom);
            let vn1 = read_cell(v_bar, n - 1, m, stride);
            let wn1 = read_cell(w_bar, n - 1, m, stride);

            let (mut vn, mut wn) = (c1 * zeta * vn1, c1 * zeta * wn1);
            if n >= m.wrapping_add(2) {
                let c2 = sqrt(
                    ((2.0 * n_f + 1.0) * (n_f + m_f - 1.0) * (n_f - m_f - 1.0))
                        / ((2.0 * n_f - 3.0) * denom),
                );
                vn -= c2 * rho * read_cell(v_bar, n - 2, m, stride);
                wn -= c2 * rho * read_cell(w_bar, n - 2, m, stride);
            }
            write_cell(v_bar, n, m, stride, vn);
            write_cell(w_bar, n, m, stride, wn);
            n = n.wrapping_add(1);
        }
        m = m.wrapping_add(1);
    }
}

fn accumulate_harmonic_terms(
    mu_m3_s2: f64,
    r_eq_m: f64,
    stride: usize,
    v_bar: &[f64],
    w_bar: &[f64],
    terms: &[HarmonicTerm],
) -> HarmonicFieldEval {
    let pot_scale = mu_m3_s2 / r_eq_m;
    let acc_scale = mu_m3_s2 / (r_eq_m * r_eq_m);
    let mut sum_pot = 0.0_f64;
    let mut ax = 0.0_f64;
    let mut ay = 0.0_f64;
    let mut az = 0.0_f64;

    for term in terms {
        let n = term.degree as usize;
        let m = term.order as usize;
        if n == 0 || m > n {
            continue;
        }
        let c = term.c_bar;
        let s = term.s_bar;
        let v_nm = read_cell(v_bar, n, m, stride);
        let w_nm = read_cell(w_bar, n, m, stride);
        sum_pot -= pot_scale * (c * v_nm + if m == 0 { 0.0 } else { s * w_nm });

        let n_f = n as f64;
        let m_f = m as f64;
        let np1 = n.wrapping_add(1);
        if m == 0 {
            let f_xy =
                sqrt(((2.0 * n_f + 1.0) * (n_f + 1.0) * (n_f + 2.0)) / (2.0 * (2.0 * n_f + 3.0)));
            let f_z = (n_f + 1.0) * sqrt((2.0 * n_f + 1.0) / (2.0 * n_f + 3.0));
            ax -= acc_scale * f_xy * c * read_cell(v_bar, np1, 1, stride);
            ay -= acc_scale * f_xy * c * read_cell(w_bar, np1, 1, stride);
            az -= acc_scale * f_z * c * read_cell(v_bar, np1, 0, stride);
        } else {
            let f_plus = 0.5
                * sqrt(
                    ((2.0 * n_f + 1.0) * (n_f + m_f + 1.0) * (n_f + m_f + 2.0)) / (2.0 * n_f + 3.0),
                );
            let f_minus = if m == 1 {
                sqrt(((2.0 * n_f + 1.0) * n_f * (n_f + 1.0)) / (2.0 * (2.0 * n_f + 3.0)))
            } else {
                0.5 * sqrt(
                    ((2.0 * n_f + 1.0) * (n_f - m_f + 1.0) * (n_f - m_f + 2.0)) / (2.0 * n_f + 3.0),
                )
            };
            let f_z = sqrt(
                ((2.0 * n_f + 1.0) * (n_f + m_f + 1.0) * (n_f - m_f + 1.0)) / (2.0 * n_f + 3.0),
            );
            let v_p = read_cell(v_bar, np1, m.wrapping_add(1), stride);
            let w_p = read_cell(w_bar, np1, m.wrapping_add(1), stride);
            let v_m = read_cell(v_bar, np1, m - 1, stride);
            let w_m = read_cell(w_bar, np1, m - 1, stride);
            let v_0 = read_cell(v_bar, np1, m, stride);
            let w_0 = read_cell(w_bar, np1, m, stride);

            ax += acc_scale * (-f_plus * (c * v_p + s * w_p) + f_minus * (c * v_m + s * w_m));
            ay += acc_scale * (-f_plus * (c * w_p - s * v_p) - f_minus * (c * w_m - s * v_m));
            az -= acc_scale * f_z * (c * v_0 + s * w_0);
        }
    }

    HarmonicFieldEval {
        potential_j_kg: sum_pot,
        accel_fixed_m_s2: DVec3::new(ax, ay, az),
    }
}
