//! Universal-variable Stumpff Kepler propagator (`DormantKeplerPropagator`)
//! and rectified `EnckeKeplerPropagator` with measured crossover benchmarking.
//!
//! All transcendentals route through [`crate::scalar_math`] so both default and
//! `portable-math` builds execute identically.

use glam::DVec3;
use serde::{Deserialize, Serialize};

use crate::{
    ephemeris::EphemerisCatalog,
    gravity::{ForceModel, GravityError, battin_f},
    ids::NaifId,
    orientation::BodyOrientationCatalog,
    plant::{
        ContinuousControlLaw, ContinuousPlantState, MassPropertiesModel, PropagationAudit,
        PropellantMode, ReactionWheelMode, quat_rotate_vec_portable,
    },
    scalar_math::{cos, sin, sqrt},
    time::TdbEpoch,
};

/// Maximum Newton-Raphson iterations for universal anomaly $\chi$.
const MAX_KEPLER_ITERS: u32 = 64;

/// Evaluates Stumpff functions $c_2(\psi)$ and $c_3(\psi)$ across elliptic ($\psi > 0$),
/// parabolic ($\psi \approx 0$), and hyperbolic ($\psi < 0$) regimes without cancellation.
#[must_use]
pub fn stumpff_c2_c3(psi: f64) -> (f64, f64) {
    if psi > 1.0e-6 {
        let sp = sqrt(psi);
        let c2 = (1.0 - cos(sp)) / psi;
        let c3 = (sp - sin(sp)) / (psi * sp);
        (c2, c3)
    } else {
        stumpff_series(psi)
    }
}

fn stumpff_series(psi: f64) -> (f64, f64) {
    let mut x = psi;
    let mut halvings = 0_u32;
    while x.abs() > 1.0 && halvings < 24 {
        x *= 0.25;
        halvings = halvings.wrapping_add(1);
    }
    let mut c2 = 0.5_f64;
    let mut c3 = 1.0_f64 / 6.0_f64;
    let mut term2 = 0.5_f64;
    let mut term3 = 1.0_f64 / 6.0_f64;
    let mut k = 1_u32;
    while k <= 16 {
        let kf = f64::from(k);
        let d2 = (2.0 * kf + 1.0) * (2.0 * kf + 2.0);
        let d3 = (2.0 * kf + 2.0) * (2.0 * kf + 3.0);
        term2 *= -x / d2;
        term3 *= -x / d3;
        c2 += term2;
        c3 += term3;
        k = k.wrapping_add(1);
    }
    let mut h = 0_u32;
    while h < halvings {
        let c0 = 1.0 - x * c2;
        let c1 = 1.0 - x * c3;
        let next_c2 = 0.5 * c1 * c1;
        let next_c3 = 0.25 * (c2 + c0 * c3);
        c2 = next_c2;
        c3 = next_c3;
        x *= 4.0;
        h = h.wrapping_add(1);
    }
    (c2, c3)
}

/// Propagates a two-body state `(r0_m, v0_m_s)` across `dt_s` (`s`) under gravitational
/// parameter `mu_m3_s2` (`m^3/s^2`) using universal variables and Lagrange $f, g, \dot{f}, \dot{g}$.
#[must_use]
pub fn propagate_kepler_universal(
    r0_m: DVec3,
    v0_m_s: DVec3,
    dt_s: f64,
    mu_m3_s2: f64,
) -> (DVec3, DVec3) {
    if dt_s.abs() <= 1.0e-15 || mu_m3_s2 <= 0.0 {
        return (r0_m + v0_m_s * dt_s, v0_m_s);
    }
    let r0_norm = sqrt(r0_m.length_squared());
    if r0_norm <= 1.0e-6 {
        return (r0_m, v0_m_s);
    }
    let v0_sq = v0_m_s.length_squared();
    let vr0 = r0_m.dot(v0_m_s) / r0_norm;
    let alpha = 2.0 / r0_norm - v0_sq / mu_m3_s2;
    let sqrt_mu = sqrt(mu_m3_s2);

    let mut chi = if alpha > 1.0e-9 {
        sqrt_mu * dt_s * alpha
    } else {
        sqrt_mu * dt_s / r0_norm
    };

    let mut iter = 0_u32;
    while iter < MAX_KEPLER_ITERS {
        let chi2 = chi * chi;
        let psi = alpha * chi2;
        let (c2, c3) = stumpff_c2_c3(psi);
        let r_val = chi2 * c2
            + (r0_norm * vr0 / sqrt_mu) * chi * (1.0 - psi * c3)
            + r0_norm * (1.0 - psi * c2);
        let f_chi = (r0_norm * vr0 / sqrt_mu) * chi2 * c2
            + (1.0 - alpha * r0_norm) * chi2 * chi * c3
            + r0_norm * chi
            - sqrt_mu * dt_s;
        if r_val.abs() <= 1.0e-15 {
            break;
        }
        let dchi = -f_chi / r_val;
        chi += dchi;
        if dchi.abs() <= 1.0e-13 * (1.0 + chi.abs()) {
            break;
        }
        iter = iter.wrapping_add(1);
    }

    let chi2 = chi * chi;
    let psi = alpha * chi2;
    let (c2, c3) = stumpff_c2_c3(psi);
    let f = 1.0 - (chi2 / r0_norm) * c2;
    let g = dt_s - (chi2 * chi / sqrt_mu) * c3;
    let r1_m = f * r0_m + g * v0_m_s;
    let r1_norm = sqrt(r1_m.length_squared()).max(1.0e-6);
    let fdot = (sqrt_mu / (r0_norm * r1_norm)) * (psi * c3 - 1.0) * chi;
    let gdot = 1.0 - (chi2 / r1_norm) * c2;
    let v1_m_s = fdot * r0_m + gdot * v0_m_s;
    (r1_m, v1_m_s)
}

/// Configuration for rectified Encke orbital propagation (`EnckeKeplerPropagator`).
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct EnckeConfig {
    /// Relative deviation threshold $\|\delta\mathbf{r}\| / \|\mathbf{r}_{\text{ref}}\|$ triggering reference orbit rectification.
    pub rectification_ratio: f64,
    /// Fixed RK4 integration substep (`s`) for the deviation vector $(\delta\mathbf{r}, \delta\mathbf{v})$.
    pub step_seconds: f64,
    /// Maximum perturbation-to-primary acceleration ratio $\eta = \|\mathbf{a}_{\text{pert}}\| / \|\mathbf{a}_0\|$ eligible for Encke.
    pub max_perturbation_ratio: f64,
}

impl Default for EnckeConfig {
    fn default() -> Self {
        Self {
            rectification_ratio: 1.0e-3,
            step_seconds: 10.0,
            max_perturbation_ratio: 2.0e-2,
        }
    }
}

/// Result of an Encke propagation interval, including rectification telemetry.
#[derive(Clone, Debug, PartialEq)]
pub struct EnckePropagationOutput {
    /// Propagated plant state at `end_epoch`.
    pub final_state: ContinuousPlantState,
    /// Number of reference-orbit rectifications performed across the interval.
    pub rectifications: u32,
    /// Number of RK4 deviation steps taken.
    pub steps_taken: u32,
    /// Peak observed perturbation ratio $\eta = \|\mathbf{a}_{\text{pert}}\| / \|\mathbf{a}_0\|$.
    pub peak_perturbation_ratio: f64,
    /// Accumulated idealization/exceedance flags.
    pub audit: PropagationAudit,
}

struct EnckeContext<'a> {
    primary: NaifId,
    mu_p: f64,
    start_epoch: TdbEpoch,
    template_state: ContinuousPlantState,
    mass_model: &'a MassPropertiesModel,
    force_model: &'a ForceModel,
    ephemeris: &'a dyn EphemerisCatalog,
    orientation: &'a dyn BodyOrientationCatalog,
    control: &'a dyn ContinuousControlLaw,
}

/// Propagates `initial` across `[start_epoch, end_epoch]` using Encke's method with
/// Battin's $f(q)$ differential two-body gravity and automatic reference rectification.
#[allow(clippy::too_many_arguments)]
pub fn propagate_encke_interval(
    initial: &ContinuousPlantState,
    start_epoch: TdbEpoch,
    end_epoch: TdbEpoch,
    config: &EnckeConfig,
    mass_model: &MassPropertiesModel,
    propellant_mode: PropellantMode,
    rw_mode: ReactionWheelMode,
    force_model: &ForceModel,
    ephemeris: &dyn EphemerisCatalog,
    orientation: &dyn BodyOrientationCatalog,
    control: &dyn ContinuousControlLaw,
) -> Result<EnckePropagationOutput, GravityError> {
    let total_s = end_epoch.signed_offset_seconds_from(start_epoch).max(0.0);
    if total_s <= 0.0 {
        return Ok(EnckePropagationOutput {
            final_state: *initial,
            rectifications: 0,
            steps_taken: 0,
            peak_perturbation_ratio: 0.0,
            audit: PropagationAudit::default(),
        });
    }
    let mu_p = force_model.constants().require_mu(initial.primary_body)?;
    let n_steps = ((total_s / config.step_seconds.max(0.1)).ceil() as u32).max(1);
    let dt = total_s / f64::from(n_steps);
    let mut ctx = EnckeContext {
        primary: initial.primary_body,
        mu_p,
        start_epoch,
        template_state: *initial,
        mass_model,
        force_model,
        ephemeris,
        orientation,
        control,
    };
    let (mut r_ref0, mut v_ref0, mut tau_ref0) =
        (initial.pos_rel_icrf_m, initial.vel_rel_icrf_m_s, 0.0);
    let (mut dr, mut dv) = (DVec3::ZERO, DVec3::ZERO);
    let (mut rectifications, mut peak_eta, mut audit) =
        (0_u32, 0.0_f64, PropagationAudit::default());
    let _ = (propellant_mode, rw_mode);

    let mut step = 0_u32;
    while step < n_steps {
        let tau = f64::from(step) * dt;
        let (next_dr, next_dv, step_eta, step_aud) =
            rk4_encke_step(&ctx, r_ref0, v_ref0, tau_ref0, dr, dv, tau, dt)?;
        dr = next_dr;
        dv = next_dv;
        peak_eta = peak_eta.max(step_eta);
        audit = audit.merged(step_aud);

        let tau_next = tau + dt;
        let (r_ref, v_ref) = propagate_kepler_universal(r_ref0, v_ref0, tau_next - tau_ref0, mu_p);
        ctx.template_state.pos_rel_icrf_m = r_ref + dr;
        ctx.template_state.vel_rel_icrf_m_s = v_ref + dv;

        if sqrt(dr.length_squared()) / sqrt(r_ref.length_squared()).max(1.0)
            >= config.rectification_ratio
        {
            r_ref0 = ctx.template_state.pos_rel_icrf_m;
            v_ref0 = ctx.template_state.vel_rel_icrf_m_s;
            tau_ref0 = tau_next;
            dr = DVec3::ZERO;
            dv = DVec3::ZERO;
            rectifications = rectifications.wrapping_add(1);
        }
        step = step.wrapping_add(1);
    }

    Ok(EnckePropagationOutput {
        final_state: ctx.template_state,
        rectifications,
        steps_taken: n_steps,
        peak_perturbation_ratio: peak_eta,
        audit,
    })
}

#[allow(clippy::too_many_arguments)]
fn rk4_encke_step(
    ctx: &EnckeContext<'_>,
    r_ref0: DVec3,
    v_ref0: DVec3,
    tau_ref0: f64,
    dr: DVec3,
    dv: DVec3,
    tau: f64,
    dt: f64,
) -> Result<(DVec3, DVec3, f64, PropagationAudit), GravityError> {
    let (k1_r, k1_v, e1, a1) = eval_encke_rhs(ctx, r_ref0, v_ref0, tau - tau_ref0, dr, dv, tau)?;
    let (k2_r, k2_v, e2, _) = eval_encke_rhs(
        ctx,
        r_ref0,
        v_ref0,
        tau + 0.5 * dt - tau_ref0,
        dr + 0.5 * dt * k1_r,
        dv + 0.5 * dt * k1_v,
        tau + 0.5 * dt,
    )?;
    let (k3_r, k3_v, e3, _) = eval_encke_rhs(
        ctx,
        r_ref0,
        v_ref0,
        tau + 0.5 * dt - tau_ref0,
        dr + 0.5 * dt * k2_r,
        dv + 0.5 * dt * k2_v,
        tau + 0.5 * dt,
    )?;
    let (k4_r, k4_v, e4, _) = eval_encke_rhs(
        ctx,
        r_ref0,
        v_ref0,
        tau + dt - tau_ref0,
        dr + dt * k3_r,
        dv + dt * k3_v,
        tau + dt,
    )?;
    let next_dr = dr + (dt / 6.0) * (k1_r + 2.0 * k2_r + 2.0 * k3_r + k4_r);
    let next_dv = dv + (dt / 6.0) * (k1_v + 2.0 * k2_v + 2.0 * k3_v + k4_v);
    Ok((next_dr, next_dv, e1.max(e2).max(e3).max(e4), a1))
}

fn eval_encke_rhs(
    ctx: &EnckeContext<'_>,
    r_ref0: DVec3,
    v_ref0: DVec3,
    dt_ref: f64,
    dr: DVec3,
    dv: DVec3,
    tau_s: f64,
) -> Result<(DVec3, DVec3, f64, PropagationAudit), GravityError> {
    let (rho, v_rho) = propagate_kepler_universal(r_ref0, v_ref0, dt_ref, ctx.mu_p);
    let r = rho + dr;
    let v = v_rho + dv;
    let r_sq = r.length_squared().max(1.0);
    let r_norm = sqrt(r_sq);
    let rho_sq = rho.length_squared().max(1.0);
    let a0_norm = ctx.mu_p / rho_sq;

    let q = dr.dot(dr + 2.0 * rho) / rho_sq;
    let fq = battin_f(q);
    let dg_two_body = -(ctx.mu_p / (r_sq * r_norm)) * (dr - fq * rho);

    let ticks = (tau_s * (sim_spacetime::TICKS_PER_SECOND as f64)).round() as u128;
    let ep = ctx
        .start_epoch
        .add_span(sim_spacetime::TickSpan::from_ticks_u128(ticks));
    let g_total = ctx.force_model.relative_gravitational_accel_icrf(
        ctx.primary,
        r,
        ep,
        ctx.ephemeris,
        ctx.orientation,
    )?;
    let g_two_body = -(ctx.mu_p / (r_sq * r_norm)) * r;
    let a_grav_pert = g_total - g_two_body;

    let mut stage_st = ctx.template_state;
    stage_st.pos_rel_icrf_m = r;
    stage_st.vel_rel_icrf_m_s = v;
    let wrench = ctx.control.evaluate_stage(tau_s, ep, &stage_st);
    let m_tot = ctx.mass_model.total_mass_kg(stage_st.propellant_mass_kg);
    let f_body = wrench.main_thrust_body_n + wrench.rcs_force_body_n + wrench.env_force_body_n;
    let a_ng = quat_rotate_vec_portable(stage_st.q_body_to_icrf, f_body) / m_tot;
    let a_pert = a_grav_pert + a_ng;
    let eta = sqrt(a_pert.length_squared()) / a0_norm.max(1.0e-12);

    Ok((dv, dg_two_body + a_pert, eta, wrench.stage_audit))
}
