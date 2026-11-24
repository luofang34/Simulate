//! Coupled 42-state translational + state transition matrix (`STM`) variational
//! integration using `DOP853`.
//!
//! # STM Matrix Layout
//!
//! `StmPropagationResult::stm` is stored in row-major `[row][col]` order over the
//! `ICRF` state perturbation vector $\delta\mathbf{x} = [\delta\mathbf{r}, \delta\mathbf{v}]^T$:
//!
//! $$\begin{bmatrix}\delta\mathbf{r}(t_f) \\ \delta\mathbf{v}(t_f)\end{bmatrix}
//! = \Phi(t_f, t_0)\begin{bmatrix}\delta\mathbf{r}(t_0) \\ \delta\mathbf{v}(t_0)\end{bmatrix},
//! \quad (\Phi\,\delta\mathbf{x})_i = \sum_{j=0}^{5} \text{stm}[i][j]\,\delta x_j.$$

use glam::{DMat3, DVec3};
use serde::{Deserialize, Serialize};
use sim_spacetime::{TICKS_PER_SECOND, TickSpan, serde_dvec3};

use crate::{
    dop853::{Dop853GenericConfig, Dop853Stats, Dop853Tolerances, dop853_integrate_generic},
    ephemeris::EphemerisCatalog,
    gravity::{ForceModel, GravityError},
    ids::NaifId,
    orientation::BodyOrientationCatalog,
    time::TdbEpoch,
    variational::VariationalError,
};

/// Dimension of the coupled translational + STM ODE state (`6 + 36 = 42`).
const VARIATIONAL_ODE_DIM: usize = 42;

/// Output of integrating the translational state and 6x6 state transition matrix $\Phi(t_f, t_0)$.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct StmPropagationResult {
    /// Initial propagation epoch $t_0$ (`TDB`).
    pub initial_epoch: TdbEpoch,
    /// Final propagation epoch $t_f$ (`TDB`).
    pub final_epoch: TdbEpoch,
    /// Final position relative to `primary_body` in non-rotating `ICRF` axes ($\text{m}$).
    #[serde(with = "serde_dvec3")]
    pub pos_rel_icrf_m: DVec3,
    /// Final velocity relative to `primary_body` in non-rotating `ICRF` axes ($\text{m}/\text{s}$).
    #[serde(with = "serde_dvec3")]
    pub vel_rel_icrf_m_s: DVec3,
    /// Row-major `6x6` state transition matrix $\Phi(t_f, t_0)$ mapping initial
    /// $(\delta\mathbf{r}_{\text{ICRF}}, \delta\mathbf{v}_{\text{ICRF}})$ at `initial_epoch`
    /// to `final_epoch` (blocks `[dimensionless, s; 1/s, dimensionless]`).
    pub stm: [[f64; 6]; 6],
    /// Adaptive `DOP853` step counts, RHS evaluation count, and continuation step hint.
    pub stats: Dop853Stats,
}

/// Integrates the 6-D translational state `(pos_rel_icrf_m, vel_rel_icrf_m_s)` and its
/// 36-element `6x6` state transition matrix $\Phi(t_f, t_0)$ across `[initial_epoch, final_epoch]`
/// under [`ForceModel`] using `dop853_integrate_generic::<42, _>`.
#[allow(clippy::too_many_arguments)]
pub fn propagate_state_and_stm(
    primary_body: NaifId,
    pos_rel_icrf_m: DVec3,
    vel_rel_icrf_m_s: DVec3,
    initial_epoch: TdbEpoch,
    final_epoch: TdbEpoch,
    tol: &Dop853Tolerances,
    force_model: &ForceModel,
    ephemeris: &dyn EphemerisCatalog,
    orientation: &dyn BodyOrientationCatalog,
) -> Result<StmPropagationResult, VariationalError> {
    if !pos_rel_icrf_m.is_finite() || !vel_rel_icrf_m_s.is_finite() {
        return Err(VariationalError::NonFiniteState);
    }
    validate_tolerances(tol)?;
    if final_epoch < initial_epoch {
        return Err(VariationalError::InvalidEpochInterval {
            initial_epoch,
            final_epoch,
        });
    }
    let total_ticks = final_epoch.ticks_since(initial_epoch);
    if total_ticks == 0 {
        return Ok(StmPropagationResult {
            initial_epoch,
            final_epoch,
            pos_rel_icrf_m,
            vel_rel_icrf_m_s,
            stm: identity_6x6(),
            stats: Dop853Stats::default(),
        });
    }

    let duration_s = final_epoch.signed_offset_seconds_from(initial_epoch);
    let scale = StateScale::new(
        primary_body,
        pos_rel_icrf_m,
        vel_rel_icrf_m_s,
        tol,
        force_model,
    );
    let cfg = scale.generic_config(tol);
    let knots = force_model.interior_knots_in(primary_body, initial_epoch, final_epoch, ephemeris);
    let knot_taus: Vec<f64> = knots
        .iter()
        .map(|k| k.signed_offset_seconds_from(initial_epoch))
        .filter(|&tau| tau > 0.0 && tau < duration_s)
        .collect();

    let y_init = scale.pack_initial(pos_rel_icrf_m, vel_rel_icrf_m_s);
    let (y_final, _segments, stats) = dop853_integrate_generic::<
        VARIATIONAL_ODE_DIM,
        GravityError,
        _,
    >(y_init, duration_s, &cfg, &knot_taus, |tau_s, y| {
        let ticks = ((tau_s.max(0.0) * (TICKS_PER_SECOND as f64)).round() as u128).min(total_ticks);
        let epoch = initial_epoch.add_span(TickSpan::from_ticks_u128(ticks));
        scale.eval_rhs(primary_body, epoch, y, force_model, ephemeris, orientation)
    })?;

    let (pos_final, vel_final) = scale.unpack_pv(&y_final);
    Ok(StmPropagationResult {
        initial_epoch,
        final_epoch,
        pos_rel_icrf_m: pos_final,
        vel_rel_icrf_m_s: vel_final,
        stm: scale.unpack_stm(&y_final),
        stats,
    })
}

/// Scales translational components to honor `atol_pos_m` / `atol_vel_m_s` and balances the
/// $[\text{s}, \text{s}^{-1}]$ STM off-diagonal blocks by characteristic time $T_c$ so every
/// entry of the scaled 42-vector sees uniform relative error control inside `dop853_integrate_generic`.
struct StateScale {
    pos_m: f64,
    vel_m_s: f64,
    time_s: f64,
    base_atol: f64,
}

impl StateScale {
    fn new(
        primary_body: NaifId,
        pos0: DVec3,
        vel0: DVec3,
        tol: &Dop853Tolerances,
        force_model: &ForceModel,
    ) -> Self {
        let base_atol = if tol.rtol > 0.0 { tol.rtol } else { 1.0e-13 };
        let r_norm = libm::sqrt(pos0.length_squared());
        let v_norm = libm::sqrt(vel0.length_squared());
        let time_s = if r_norm > 1.0 && v_norm > 1.0e-6 {
            (r_norm / v_norm).clamp(1.0, 1.0e9)
        } else if r_norm > 1.0
            && let Ok(mu) = force_model.constants().require_mu(primary_body)
        {
            libm::sqrt(r_norm * r_norm * r_norm / mu).clamp(1.0, 1.0e9)
        } else {
            1_000.0
        };
        Self {
            pos_m: tol.atol_pos_m / base_atol,
            vel_m_s: tol.atol_vel_m_s / base_atol,
            time_s,
            base_atol,
        }
    }

    fn generic_config(&self, tol: &Dop853Tolerances) -> Dop853GenericConfig {
        Dop853GenericConfig {
            rtol: tol.rtol,
            atol: self.base_atol,
            h_min_s: tol.h_min_s,
            h_max_s: tol.h_max_s,
            h_init_s: tol.h_init_s,
            max_steps: tol.max_steps,
        }
    }

    fn pack_initial(&self, pos: DVec3, vel: DVec3) -> [f64; VARIATIONAL_ODE_DIM] {
        let mut y = [0.0_f64; VARIATIONAL_ODE_DIM];
        let p = pos / self.pos_m;
        let v = vel / self.vel_m_s;
        if let Some(head) = y.get_mut(..6) {
            head.copy_from_slice(&[p.x, p.y, p.z, v.x, v.y, v.z]);
        }
        let mut diag: usize = 0;
        while diag < 6 {
            if let Some(slot) = y.get_mut(6 + 7 * diag) {
                *slot = 1.0;
            }
            diag = diag.wrapping_add(1);
        }
        y
    }

    fn unpack_pv(&self, y: &[f64; VARIATIONAL_ODE_DIM]) -> (DVec3, DVec3) {
        let [px, py, pz, vx, vy, vz, ..] = *y;
        (
            DVec3::new(px, py, pz) * self.pos_m,
            DVec3::new(vx, vy, vz) * self.vel_m_s,
        )
    }

    fn unpack_stm(&self, y: &[f64; VARIATIONAL_ODE_DIM]) -> [[f64; 6]; 6] {
        let inv_t = 1.0 / self.time_s;
        core::array::from_fn(|row| {
            core::array::from_fn(|col| {
                let raw = y.get(6 + 6 * row + col).copied().unwrap_or(0.0);
                match (row < 3, col < 3) {
                    (true, true) | (false, false) => raw,
                    (true, false) => raw * self.time_s,
                    (false, true) => raw * inv_t,
                }
            })
        })
    }

    fn eval_rhs(
        &self,
        primary_body: NaifId,
        epoch: TdbEpoch,
        y: &[f64; VARIATIONAL_ODE_DIM],
        force_model: &ForceModel,
        ephemeris: &dyn EphemerisCatalog,
        orientation: &dyn BodyOrientationCatalog,
    ) -> Result<[f64; VARIATIONAL_ODE_DIM], GravityError> {
        let (pos, vel) = self.unpack_pv(y);
        let (acc, grad) = force_model.relative_gravitational_accel_and_gradient_icrf(
            primary_body,
            pos,
            epoch,
            ephemeris,
            orientation,
        )?;
        let mut dy = [0.0_f64; VARIATIONAL_ODE_DIM];
        let dp = vel / self.pos_m;
        let dv = acc / self.vel_m_s;
        if let Some(head) = dy.get_mut(..6) {
            head.copy_from_slice(&[dp.x, dp.y, dp.z, dv.x, dv.y, dv.z]);
        }
        write_scaled_stm_rhs(y, &mut dy, grad, self.time_s);
        Ok(dy)
    }
}

fn write_scaled_stm_rhs(
    y: &[f64; VARIATIONAL_ODE_DIM],
    dy: &mut [f64; VARIATIONAL_ODE_DIM],
    grad: DMat3,
    time_s: f64,
) {
    let inv_t = 1.0 / time_s;
    let [[g00, g01, g02], [g10, g11, g12], [g20, g21, g22]] = [
        [grad.x_axis.x, grad.y_axis.x, grad.z_axis.x],
        [grad.x_axis.y, grad.y_axis.y, grad.z_axis.y],
        [grad.x_axis.z, grad.y_axis.z, grad.z_axis.z],
    ];
    let mut col: usize = 0;
    while col < 6 {
        let p0 = y.get(6 + col).copied().unwrap_or(0.0);
        let p1 = y.get(12 + col).copied().unwrap_or(0.0);
        let p2 = y.get(18 + col).copied().unwrap_or(0.0);
        let v0 = y.get(24 + col).copied().unwrap_or(0.0);
        let v1 = y.get(30 + col).copied().unwrap_or(0.0);
        let v2 = y.get(36 + col).copied().unwrap_or(0.0);
        if let Some(s) = dy.get_mut(6 + col) {
            *s = inv_t * v0;
        }
        if let Some(s) = dy.get_mut(12 + col) {
            *s = inv_t * v1;
        }
        if let Some(s) = dy.get_mut(18 + col) {
            *s = inv_t * v2;
        }
        if let Some(s) = dy.get_mut(24 + col) {
            *s = time_s * (g00 * p0 + g01 * p1 + g02 * p2);
        }
        if let Some(s) = dy.get_mut(30 + col) {
            *s = time_s * (g10 * p0 + g11 * p1 + g12 * p2);
        }
        if let Some(s) = dy.get_mut(36 + col) {
            *s = time_s * (g20 * p0 + g21 * p1 + g22 * p2);
        }
        col = col.wrapping_add(1);
    }
}

fn validate_tolerances(tol: &Dop853Tolerances) -> Result<(), VariationalError> {
    let ok = tol.rtol.is_finite()
        && tol.rtol >= 0.0
        && tol.atol_pos_m.is_finite()
        && tol.atol_pos_m > 0.0
        && tol.atol_vel_m_s.is_finite()
        && tol.atol_vel_m_s > 0.0
        && tol.h_min_s.is_finite()
        && tol.h_min_s > 0.0
        && tol.h_max_s.is_finite()
        && tol.h_max_s >= tol.h_min_s
        && tol.h_init_s.is_finite()
        && tol.h_init_s > 0.0
        && tol.max_steps > 0;
    if ok {
        Ok(())
    } else {
        Err(VariationalError::InvalidTolerances)
    }
}

const fn identity_6x6() -> [[f64; 6]; 6] {
    [
        [1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
    ]
}
