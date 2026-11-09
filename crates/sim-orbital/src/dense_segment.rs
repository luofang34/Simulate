//! Continuous 7th-order `DOP853` (`CONTD8`) dense-output polynomial segments
//! and exact degree-7 Bernstein basis conversion.

use serde::{Deserialize, Serialize};
use sim_spacetime::TickSpan;

use crate::{
    hash::{ContentHash, DomainHasher},
    ids::NaifId,
    plant::{ContinuousPlantState, PLANT_ODE_DIM, PropagationAudit},
    time::TdbEpoch,
};

const BINOM_7: [u32; 8] = [1, 7, 21, 35, 35, 21, 7, 1];

#[inline]
const fn binom_small(n: usize, k: usize) -> u32 {
    if k > n {
        return 0;
    }
    if k == 0 || k == n {
        return 1;
    }
    let mut num: u32 = 1;
    let mut den: u32 = 1;
    let mut i: usize = 0;
    while i < k {
        num *= (n - i) as u32;
        den *= (i + 1) as u32;
        i = i.wrapping_add(1);
    }
    num / den
}

/// Degree-7 `CONTD8` dense-output polynomial segment on $\tau \in [\tau_0, \tau_0 + h]$.
///
/// Evaluates $y(\tau)$ in Hairer's alternating $x(1-x)$ basis ($x = (\tau - \tau_0)/h \in [0, 1]$),
/// guaranteeing exact endpoint values $y(0) = y_0$, $y(1) = y_1$ and exact endpoint derivatives.
#[derive(Clone, Debug, PartialEq)]
pub struct DenseSegment<const N: usize> {
    /// Segment-local start time $\tau_0$ ($\text{s}$ from trajectory anchor).
    pub start_tau_s: f64,
    /// Signed or positive step size $h$ ($\text{s}$, non-zero).
    pub h_s: f64,
    /// State vector at $\tau_0$ ($x = 0$).
    pub y_old: [f64; N],
    /// Seven `CONTD8` basis vectors `F[0..7]`.
    pub f_coeffs: [[f64; N]; 7],
}

impl<const N: usize> DenseSegment<N> {
    /// Segment-local end offset $\tau_1 = \tau_0 + h$.
    #[inline]
    #[must_use]
    pub fn end_tau_s(&self) -> f64 {
        self.start_tau_s + self.h_s
    }

    /// Evaluates $y(x)$ at normalized step fraction $x \in [0, 1]$.
    #[must_use]
    pub fn evaluate_unit(&self, x: f64) -> [f64; N] {
        let one_minus_x = 1.0 - x;
        let mut y = [0.0_f64; N];
        for (rev_idx, f_vec) in self.f_coeffs.iter().rev().enumerate() {
            let mult = if rev_idx.is_multiple_of(2) {
                x
            } else {
                one_minus_x
            };
            for (yk, &fk) in y.iter_mut().zip(f_vec.iter()) {
                *yk = (*yk + fk) * mult;
            }
        }
        for (yk, &y0) in y.iter_mut().zip(self.y_old.iter()) {
            *yk += y0;
        }
        y
    }

    /// Evaluates $y(\tau)$ at segment-local offset $\tau$.
    #[must_use]
    pub fn evaluate_tau(&self, tau_s: f64) -> [f64; N] {
        let x = (tau_s - self.start_tau_s) / self.h_s;
        self.evaluate_unit(x)
    }

    /// Evaluates $(y(\tau), \dot{y}(\tau), \ddot{y}(\tau))$ analytically via the exact product rule
    /// on Hairer's alternating $x(1-x)$ Horner chain.
    #[must_use]
    pub fn evaluate_tau_with_derivs(&self, tau_s: f64) -> ([f64; N], [f64; N], [f64; N]) {
        let x = (tau_s - self.start_tau_s) / self.h_s;
        let one_minus_x = 1.0 - x;
        let mut y = [0.0_f64; N];
        let mut dy_dx = [0.0_f64; N];
        let mut d2y_dx2 = [0.0_f64; N];

        for (rev_idx, f_vec) in self.f_coeffs.iter().rev().enumerate() {
            let even = rev_idx.is_multiple_of(2);
            for (((yk, dyk), d2yk), &fk) in y
                .iter_mut()
                .zip(dy_dx.iter_mut())
                .zip(d2y_dx2.iter_mut())
                .zip(f_vec.iter())
            {
                let u = *yk + fk;
                let du = *dyk;
                let d2u = *d2yk;
                if even {
                    *d2yk = x * d2u + 2.0 * du;
                    *dyk = x * du + u;
                    *yk = x * u;
                } else {
                    *d2yk = one_minus_x * d2u - 2.0 * du;
                    *dyk = one_minus_x * du - u;
                    *yk = one_minus_x * u;
                }
            }
        }

        let inv_h = 1.0 / self.h_s;
        let inv_h2 = inv_h * inv_h;
        for (((yk, dyk), d2yk), &y0) in y
            .iter_mut()
            .zip(dy_dx.iter_mut())
            .zip(d2y_dx2.iter_mut())
            .zip(self.y_old.iter())
        {
            *yk += y0;
            *dyk *= inv_h;
            *d2yk *= inv_h2;
        }
        (y, dy_dx, d2y_dx2)
    }

    /// Converts scalar component `comp` into its exact degree-7 Bernstein control points
    /// $[b_0, b_1, \dots, b_7]$ on $x \in [0, 1]$ using strictly positive binomial weights
    /// (zero monomial-to-Bernstein cancellation).
    #[must_use]
    pub fn component_bernstein_8(&self, comp: usize) -> [f64; 8] {
        let y0 = self.y_old.get(comp).copied().unwrap_or(0.0);
        let mut b = [y0; 8];

        for (m, f_vec) in self.f_coeffs.iter().enumerate() {
            let fm = f_vec.get(comp).copied().unwrap_or(0.0);
            if fm == 0.0 {
                continue;
            }
            let a = (m / 2).wrapping_add(1);
            let beta = m.wrapping_add(1) / 2;
            let deg = a.wrapping_add(beta);
            let rem = 7_usize.saturating_sub(deg);
            let mut j: usize = 0;
            while j <= rem {
                let idx = a.wrapping_add(j);
                let num = f64::from(binom_small(rem, j));
                let den = f64::from(BINOM_7.get(idx).copied().unwrap_or(1));
                if let Some(slot) = b.get_mut(idx) {
                    *slot += fm * (num / den);
                }
                j = j.wrapping_add(1);
            }
        }
        b
    }
}

/// Evaluates a degree-7 scalar Bernstein polynomial $[b_0, \dots, b_7]$ at $x \in [0, 1]$
/// via de Casteljau's convex-combination algorithm.
#[must_use]
pub fn evaluate_bernstein_8(ctrl: &[f64; 8], x: f64) -> f64 {
    let one_minus_x = 1.0 - x;
    let mut work = *ctrl;
    let mut len: usize = 7;
    while len >= 1 {
        let mut i: usize = 0;
        while i < len {
            let left = work.get(i).copied().unwrap_or(0.0);
            let right = work.get(i.wrapping_add(1)).copied().unwrap_or(0.0);
            if let Some(slot) = work.get_mut(i) {
                *slot = one_minus_x * left + x * right;
            }
            i = i.wrapping_add(1);
        }
        len -= 1;
    }
    work.first().copied().unwrap_or(0.0)
}

/// Serializable 17-state continuous plant trajectory segment anchored to a `TdbEpoch`.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct PlantTrajectorySegment {
    /// Exact `TDB` start epoch of this single `DOP853` step ($\tau = 0$).
    pub start_epoch: TdbEpoch,
    /// Exact integer master-clock tick duration of this step (`> 0`).
    pub duration_ticks: u128,
    /// Step duration $h > 0$ in seconds (`duration_ticks / TICKS_PER_SECOND`).
    pub duration_s: f64,
    /// Primary coordinate origin active throughout this segment.
    pub primary_body: NaifId,
    /// Initial 17-component state vector at `start_epoch`.
    pub y_old: [f64; PLANT_ODE_DIM],
    /// Seven 17-component `CONTD8` dense-output basis vectors.
    pub f_coeffs: [[f64; PLANT_ODE_DIM]; 7],
    /// Exact normalized post-step plant state at `end_epoch` for 0-ulp step resumption.
    pub end_state: ContinuousPlantState,
    /// Exact post-step trial step size $h_{\text{nom}}$ computed by the adaptive step controller.
    pub next_h_hint_s: f64,
    /// Idealization/exceedance audit flags accumulated across this step.
    pub audit: PropagationAudit,
}

impl PlantTrajectorySegment {
    /// Wraps a step-local [`DenseSegment<17>`] anchored at `start_epoch`.
    #[must_use]
    pub const fn from_dense(
        start_epoch: TdbEpoch,
        duration_ticks: u128,
        primary_body: NaifId,
        dense: &DenseSegment<PLANT_ODE_DIM>,
        end_state: ContinuousPlantState,
        next_h_hint_s: f64,
        audit: PropagationAudit,
    ) -> Self {
        Self {
            start_epoch,
            duration_ticks,
            duration_s: dense.h_s,
            primary_body,
            y_old: dense.y_old,
            f_coeffs: dense.f_coeffs,
            end_state,
            next_h_hint_s,
            audit,
        }
    }

    /// Reconstructs the step-local [`DenseSegment<17>`] ($\tau \in [0, \text{duration\_s}]$).
    #[must_use]
    pub const fn as_dense(&self) -> DenseSegment<PLANT_ODE_DIM> {
        DenseSegment {
            start_tau_s: 0.0,
            h_s: self.duration_s,
            y_old: self.y_old,
            f_coeffs: self.f_coeffs,
        }
    }

    /// Exact end epoch `start_epoch + duration_ticks`.
    #[must_use]
    pub const fn end_epoch(&self) -> TdbEpoch {
        self.start_epoch
            .add_span(TickSpan::from_ticks_u128(self.duration_ticks))
    }

    /// Samples the normalized continuous plant state and relative acceleration $\ddot{\mathbf{r}}$ at `epoch`.
    #[must_use]
    pub fn sample_plant_and_accel(&self, epoch: TdbEpoch) -> (ContinuousPlantState, glam::DVec3) {
        let tau = epoch
            .signed_offset_seconds_from(self.start_epoch)
            .clamp(0.0, self.duration_s);
        let dense = self.as_dense();
        let (y, dy, _d2y) = dense.evaluate_tau_with_derivs(tau);
        let plant = ContinuousPlantState::from_ode_array(self.primary_body, &y, true);
        let [_, _, _, ax, ay, az, ..] = dy;
        (plant, glam::DVec3::new(ax, ay, az))
    }

    /// Canonical SHA-256 content hash of this trajectory segment's polynomial basis and endpoints.
    #[must_use]
    pub fn content_hash(&self) -> ContentHash {
        let mut h = DomainHasher::new(b"sim-orbital:plant-trajectory-segment:v1");
        h.write_i64(self.start_epoch.j2000_seconds());
        h.write_u64(self.start_epoch.subsec_ticks());
        h.write_u128(self.duration_ticks);
        h.write_f64_canonical(self.duration_s);
        h.write_i32(self.primary_body.0);
        for v in self.y_old {
            h.write_f64_canonical(v);
        }
        for row in self.f_coeffs {
            for v in row {
                h.write_f64_canonical(v);
            }
        }
        h.write_i32(self.end_state.primary_body.0);
        for v in self.end_state.to_ode_array() {
            h.write_f64_canonical(v);
        }
        h.write_f64_canonical(self.next_h_hint_s);
        h.write_u8(u8::from(self.audit.mass_conservation_exempt));
        h.write_u8(u8::from(self.audit.angular_momentum_exempt));
        h.write_u8(u8::from(self.audit.proper_accel_exceedance));
        h.finish()
    }
}
