//! Explicit 8th-order Dormand–Prince `DOP853` propagator with 7th-order `CONTD8`
//! dense output, quaternion geodesic error norm, and Chebyshev knot alignment.

pub mod coeffs;
mod generic;
mod plant_prop;

use serde::{Deserialize, Serialize};

use crate::{
    dense_segment::DenseSegment,
    hash::{ContentHash, DomainHasher},
    scalar_math::{powf, sqrt},
};

use coeffs::{A, B, C, D, INTERPOLATOR_POWER, N_STAGES, N_STAGES_EXTENDED};

pub use generic::{dop853_fixed_steps, dop853_integrate_generic};
pub use plant_prop::{
    PlantPropagationOutput, propagate_plant_interval, propagate_plant_until_step,
};

const SAFETY: f64 = 0.9;
const MIN_FACTOR: f64 = 0.333;
const MAX_FACTOR: f64 = 6.0;

/// Step-rejection, step-limit, or RHS evaluation failure in `DOP853`.
#[derive(Clone, Debug, PartialEq, thiserror::Error)]
pub enum Dop853Error<E> {
    /// Underlying ODE right-hand side returned an error.
    #[error("ODE right-hand side failed: {0}")]
    Rhs(E),
    /// Adaptive step size shrank below `h_min_s` without meeting error tolerance.
    #[error("step size underflow at tau = {tau_s} s (h = {h_s} s < h_min = {h_min_s} s)")]
    StepSizeUnderflow {
        /// Segment-local offset where underflow occurred.
        tau_s: f64,
        /// Rejected trial step size.
        h_s: f64,
        /// Configured minimum step floor.
        h_min_s: f64,
    },
    /// Step count exceeded `max_steps` before reaching interval end.
    #[error("exceeded max_steps ({max_steps}) at tau = {tau_s} s")]
    MaxStepsExceeded {
        /// Configured step limit.
        max_steps: u32,
        /// Segment-local offset reached before aborting.
        tau_s: f64,
    },
    /// State became non-finite (`NaN` or `±Inf`).
    #[error("non-finite state encountered at tau = {tau_s} s")]
    NonFiniteState {
        /// Segment-local offset where non-finite value appeared.
        tau_s: f64,
    },
}

/// Diagnostic step and RHS evaluation counters for a `DOP853` integration span.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Dop853Stats {
    /// Number of accepted steps.
    pub accepted_steps: u32,
    /// Number of rejected trial steps.
    pub rejected_steps: u32,
    /// Total ODE right-hand-side evaluations (including rejected and dense stages).
    pub rhs_evals: u64,
    /// Recommended initial trial step size $h_{\text{next}}$ (`s`) for continuation.
    pub next_h_hint_s: f64,
}

impl Dop853Stats {
    /// Accumulates step and RHS evaluation counters from a subsequent integration span.
    pub fn accumulate(&mut self, other: Self) {
        self.accepted_steps = self.accepted_steps.wrapping_add(other.accepted_steps);
        self.rejected_steps = self.rejected_steps.wrapping_add(other.rejected_steps);
        self.rhs_evals = self.rhs_evals.wrapping_add(other.rhs_evals);
        self.next_h_hint_s = other.next_h_hint_s;
    }
}

/// Generic scalar tolerances for fixed-dimension `DOP853` ODE integration.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct Dop853GenericConfig {
    /// Relative tolerance `rtol`.
    pub rtol: f64,
    /// Absolute tolerance `atol`.
    pub atol: f64,
    /// Minimum allowed step size (`s`).
    pub h_min_s: f64,
    /// Maximum allowed step size (`s`).
    pub h_max_s: f64,
    /// Initial trial step (`s`).
    pub h_init_s: f64,
    /// Maximum accepted + rejected step iterations.
    pub max_steps: u32,
}

impl Default for Dop853GenericConfig {
    fn default() -> Self {
        Self {
            rtol: 1.0e-12,
            atol: 1.0e-12,
            h_min_s: 1.0e-11,
            h_max_s: 3_600.0,
            h_init_s: 10.0,
            max_steps: 200_000,
        }
    }
}

/// Domain-specific 6-DoF spacecraft integration tolerances and step bounds.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct Dop853Tolerances {
    /// Dimensionless relative tolerance (`1e-11` default -> `70 µm` scale in LEO).
    pub rtol: f64,
    /// Position absolute tolerance (`m`).
    pub atol_pos_m: f64,
    /// Velocity absolute tolerance (`m/s`).
    pub atol_vel_m_s: f64,
    /// Attitude geodesic-angle absolute tolerance (`rad`).
    pub atol_att_rad: f64,
    /// Body angular-rate absolute tolerance (`rad/s`).
    pub atol_omega_rad_s: f64,
    /// Propellant mass absolute tolerance (`kg`).
    pub atol_mass_kg: f64,
    /// Reaction-wheel angular-momentum absolute tolerance (`N·m·s`).
    pub atol_h_rw_nms: f64,
    /// Minimum adaptive step size (`s`).
    pub h_min_s: f64,
    /// Maximum adaptive step size (`s`).
    pub h_max_s: f64,
    /// Default initial step size (`s`) when no continuation hint is present.
    pub h_init_s: f64,
    /// Hard cap on step attempts per propagation call.
    pub max_steps: u32,
}

impl Default for Dop853Tolerances {
    fn default() -> Self {
        Self {
            rtol: 1.0e-11,
            atol_pos_m: 1.0e-6,
            atol_vel_m_s: 1.0e-9,
            atol_att_rad: 1.0e-9,
            atol_omega_rad_s: 1.0e-9,
            atol_mass_kg: 1.0e-9,
            atol_h_rw_nms: 1.0e-7,
            h_min_s: 1.0e-9,
            h_max_s: 3_600.0,
            h_init_s: 5.0,
            max_steps: 200_000,
        }
    }
}

impl Dop853Tolerances {
    /// Canonical SHA-256 content hash for [`crate::SimulationManifestDigest`].
    #[must_use]
    pub fn content_hash(&self) -> ContentHash {
        let mut h = DomainHasher::new(b"sim-orbital:dop853-policy:v1");
        h.write_f64_canonical(self.rtol);
        h.write_f64_canonical(self.atol_pos_m);
        h.write_f64_canonical(self.atol_vel_m_s);
        h.write_f64_canonical(self.atol_att_rad);
        h.write_f64_canonical(self.atol_omega_rad_s);
        h.write_f64_canonical(self.atol_mass_kg);
        h.write_f64_canonical(self.atol_h_rw_nms);
        h.write_f64_canonical(self.h_min_s);
        h.write_f64_canonical(self.h_max_s);
        h.write_f64_canonical(self.h_init_s);
        h.write_u64(u64::from(self.max_steps));
        h.finish()
    }
}

pub(crate) fn eval_main_stages<const N: usize, E, F>(
    tau_0: f64,
    h: f64,
    y_0: &[f64; N],
    k_table: &mut [[f64; N]; N_STAGES_EXTENDED],
    rhs: &mut F,
    eval_count: &mut u64,
) -> Result<([f64; N], [f64; N]), Dop853Error<E>>
where
    F: FnMut(f64, &[f64; N]) -> Result<[f64; N], E>,
{
    let mut s: usize = 1;
    while s < N_STAGES {
        let c_s = C.get(s).copied().unwrap_or(0.0);
        let a_row = A.get(s);
        let mut dy = [0.0_f64; N];
        let mut j: usize = 0;
        while j < s {
            let a_sj = a_row.and_then(|r| r.get(j)).copied().unwrap_or(0.0);
            if a_sj != 0.0
                && let Some(kj) = k_table.get(j)
            {
                for (ds, &kv) in dy.iter_mut().zip(kj.iter()) {
                    *ds += a_sj * kv;
                }
            }
            j = j.wrapping_add(1);
        }
        let mut y_stage = *y_0;
        for (ys, &ds) in y_stage.iter_mut().zip(dy.iter()) {
            *ys += h * ds;
        }
        let ks = rhs(tau_0 + c_s * h, &y_stage).map_err(Dop853Error::Rhs)?;
        *eval_count = eval_count.wrapping_add(1);
        if let Some(slot) = k_table.get_mut(s) {
            *slot = ks;
        }
        s = s.wrapping_add(1);
    }

    let mut dy_step = [0.0_f64; N];
    for (j, &b_j) in B.iter().enumerate() {
        if b_j != 0.0
            && let Some(kj) = k_table.get(j)
        {
            for (dn, &kv) in dy_step.iter_mut().zip(kj.iter()) {
                *dn += b_j * kv;
            }
        }
    }
    let mut y_new = *y_0;
    for (yn, &dn) in y_new.iter_mut().zip(dy_step.iter()) {
        *yn += h * dn;
    }
    if y_new.iter().any(|v| !v.is_finite()) {
        return Err(Dop853Error::NonFiniteState { tau_s: tau_0 + h });
    }
    let k_fsal = rhs(tau_0 + h, &y_new).map_err(Dop853Error::Rhs)?;
    *eval_count = eval_count.wrapping_add(1);
    if let Some(slot) = k_table.get_mut(N_STAGES) {
        *slot = k_fsal;
    }
    Ok((y_new, dy_step))
}

/// Knuth error-free `TwoSum` step update (`Hairer, Lubich & Wanner §VIII.5`),
/// eliminating $O(N_{\text{steps}} \varepsilon_{\text{mach}})$ secular roundoff drift
/// using strictly IEEE-754 `+`/`-` operations.
#[inline]
pub(crate) fn compensated_add(y: f64, comp: &mut f64, delta: f64) -> f64 {
    let inc = delta + *comp;
    let sum = y + inc;
    let eff = sum - y;
    *comp = (y - (sum - eff)) + (inc - eff);
    sum
}

pub(crate) fn build_dense_segment<const N: usize, E, F>(
    tau_0: f64,
    h: f64,
    y_0: &[f64; N],
    y_new: &[f64; N],
    k_table: &mut [[f64; N]; N_STAGES_EXTENDED],
    rhs: &mut F,
    eval_count: &mut u64,
) -> Result<DenseSegment<N>, Dop853Error<E>>
where
    F: FnMut(f64, &[f64; N]) -> Result<[f64; N], E>,
{
    eval_dense_extra_stages(tau_0, h, y_0, k_table, rhs, eval_count)?;
    Ok(assemble_contd8_segment(tau_0, h, y_0, y_new, k_table))
}

fn eval_dense_extra_stages<const N: usize, E, F>(
    tau_0: f64,
    h: f64,
    y_0: &[f64; N],
    k_table: &mut [[f64; N]; N_STAGES_EXTENDED],
    rhs: &mut F,
    eval_count: &mut u64,
) -> Result<(), Dop853Error<E>>
where
    F: FnMut(f64, &[f64; N]) -> Result<[f64; N], E>,
{
    let mut s = N_STAGES.wrapping_add(1);
    while s < N_STAGES_EXTENDED {
        let c_s = C.get(s).copied().unwrap_or(0.0);
        let a_row = A.get(s);
        let mut dy = [0.0_f64; N];
        let mut j: usize = 0;
        while j < s {
            let a_sj = a_row.and_then(|r| r.get(j)).copied().unwrap_or(0.0);
            if a_sj != 0.0
                && let Some(kj) = k_table.get(j)
            {
                for (ds, &kv) in dy.iter_mut().zip(kj.iter()) {
                    *ds += a_sj * kv;
                }
            }
            j = j.wrapping_add(1);
        }
        let mut y_stage = *y_0;
        for (ys, &ds) in y_stage.iter_mut().zip(dy.iter()) {
            *ys += h * ds;
        }
        let ks = rhs(tau_0 + c_s * h, &y_stage).map_err(Dop853Error::Rhs)?;
        *eval_count = eval_count.wrapping_add(1);
        if let Some(slot) = k_table.get_mut(s) {
            *slot = ks;
        }
        s = s.wrapping_add(1);
    }
    Ok(())
}

fn assemble_contd8_segment<const N: usize>(
    tau_0: f64,
    h: f64,
    y_0: &[f64; N],
    y_new: &[f64; N],
    k_table: &[[f64; N]; N_STAGES_EXTENDED],
) -> DenseSegment<N> {
    let mut f_coeffs = [[0.0_f64; N]; INTERPOLATOR_POWER];
    let k0 = k_table.first().copied().unwrap_or([0.0; N]);
    let k12 = k_table.get(N_STAGES).copied().unwrap_or([0.0; N]);
    let (mut f0, mut f1, mut f2) = ([0.0_f64; N], [0.0_f64; N], [0.0_f64; N]);
    for idx in 0..N {
        let dy = y_new.get(idx).copied().unwrap_or(0.0) - y_0.get(idx).copied().unwrap_or(0.0);
        let f_old = k0.get(idx).copied().unwrap_or(0.0);
        let f_new = k12.get(idx).copied().unwrap_or(0.0);
        if let Some(s0) = f0.get_mut(idx) {
            *s0 = dy;
        }
        if let Some(s1) = f1.get_mut(idx) {
            *s1 = h * f_old - dy;
        }
        if let Some(s2) = f2.get_mut(idx) {
            *s2 = 2.0 * dy - h * (f_new + f_old);
        }
    }
    if let Some(head) = f_coeffs.get_mut(..3) {
        head.copy_from_slice(&[f0, f1, f2]);
    }
    for (row_idx, d_row) in D.iter().enumerate() {
        let mut acc = [0.0_f64; N];
        for (stage_idx, &d_val) in d_row.iter().enumerate() {
            if d_val != 0.0
                && let Some(ks) = k_table.get(stage_idx)
            {
                for (ak, &kv) in acc.iter_mut().zip(ks.iter()) {
                    *ak += h * d_val * kv;
                }
            }
        }
        if let Some(slot) = f_coeffs.get_mut(row_idx.wrapping_add(3)) {
            *slot = acc;
        }
    }
    DenseSegment {
        start_tau_s: tau_0,
        h_s: h,
        y_old: *y_0,
        f_coeffs,
    }
}

#[inline]
pub(crate) fn combine_hairer_err(h_scale: f64, err5_sq: f64, err3_sq: f64, dof: f64) -> f64 {
    if err5_sq == 0.0 && err3_sq == 0.0 {
        return 0.0;
    }
    let denom = err5_sq + 0.01 * err3_sq;
    h_scale.abs() * err5_sq / sqrt(denom * dof)
}

#[inline]
pub(crate) fn next_step_factor(err_norm: f64) -> f64 {
    if err_norm <= 0.0 {
        MAX_FACTOR
    } else {
        (SAFETY * powf(err_norm, -0.125)).clamp(MIN_FACTOR, MAX_FACTOR)
    }
}

pub(crate) fn clamp_step_to_knot(tau: f64, h: f64, end_tau: f64, knots: &[f64]) -> (f64, bool) {
    let rem = end_tau - tau;
    let mut target_h = h.min(rem);
    let mut hit_boundary = (rem - target_h).abs() <= 1.0e-14 * rem.max(1.0);
    for &k in knots {
        if k > tau + 1.0e-12 && k < tau + target_h {
            target_h = k - tau;
            hit_boundary = true;
            break;
        }
    }
    (target_h, hit_boundary)
}
