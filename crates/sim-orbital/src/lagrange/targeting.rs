//! 1D single-shooting terminal-$x$ differential correction for finite-horizon $L_2$ arcs.

use glam::{DQuat, DVec3};
use serde::{Deserialize, Serialize};
use sim_spacetime::TickSpan;

use crate::{
    dop853::{Dop853Tolerances, propagate_plant_interval},
    ephemeris::EphemerisCatalog,
    gravity::ForceModel,
    ids::NaifId,
    lagrange::{L2OrbitRegimeKind, LagrangeError, SynodicFrame, approximate_l2_libration_seed},
    orientation::BodyOrientationCatalog,
    plant::{
        CoastControlLaw, ContinuousPlantState, MassPropertiesModel, PropellantMode,
        ReactionWheelMode,
    },
    time::TdbEpoch,
};

/// Failure reason when the 1D single-shooting terminal-$x$ corrector does not converge.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub enum L2SingleShootingFailureReason {
    /// Reached the configured maximum number of shooting iterations without meeting tolerance.
    MaxIterationsReached {
        /// Configured iteration cap.
        max_iterations: u32,
    },
    /// Secant denominator $\Delta f = f_1 - f_0$ collapsed below numerical threshold before convergence.
    SecantSlopeDegenerate {
        /// Observed residual difference $f_1 - f_0$ (`m`).
        delta_f_m: f64,
    },
    /// Propagation produced a non-finite terminal synodic residual.
    NonFiniteResidual,
}

/// Solver outcome status for the 1D single-shooting terminal-$x$ corrector.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub enum L2SingleShootingStatus {
    /// Terminal synodic residual $|x(t_f) - x_{L_2}|$ converged within `tolerance_x_m`.
    Converged,
    /// Corrector failed to converge.
    Failed(L2SingleShootingFailureReason),
}

/// Problem specification for 1D single-shooting finite-horizon $L_2$ arc targeting.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct L2BoundedArcTargetSpec {
    /// Primary body (`NaifId::SUN` for Sun–Earth $L_2$).
    pub primary: NaifId,
    /// Secondary body (`NaifId::EARTH`).
    pub secondary: NaifId,
    /// Initial epoch $t_0$.
    pub start_epoch: TdbEpoch,
    /// Fixed propagation horizon $T = t_f - t_0$ (`s`).
    pub horizon_s: f64,
    /// Out-of-plane seed amplitude $A_z$ (`m`).
    pub az_amplitude_m: f64,
    /// Whether to seed a Southern ($z_0 < 0$) or Northern ($z_0 > 0$) $L_2$ excursion.
    pub southern: bool,
    /// Convergence threshold on terminal radial residual $|x_{\text{syn}}(t_f) - x_{L_2}|$ (`m`).
    pub tolerance_x_m: f64,
    /// Maximum number of secant shooting iterations permitted.
    pub max_iterations: u32,
}

impl L2BoundedArcTargetSpec {
    /// Standard Sun–Earth Southern $L_2$ finite-horizon arc specification with `5,000 km` terminal-$x$ tolerance.
    #[must_use]
    pub const fn sun_earth_southern(
        start_epoch: TdbEpoch,
        horizon_s: f64,
        az_amplitude_m: f64,
    ) -> Self {
        Self {
            primary: NaifId::SUN,
            secondary: NaifId::EARTH,
            start_epoch,
            horizon_s,
            az_amplitude_m,
            southern: true,
            tolerance_x_m: 5.0e6,
            max_iterations: 12,
        }
    }
}

/// Output of 1D single-shooting terminal-$x$ differential correction for a finite-horizon $L_2$ arc.
///
/// The solver adjusts a single initial synodic velocity component ($\dot{y}_0$) to drive a single
/// terminal position residual $x_{\text{syn}}(t_0 + T) - x_{L_2}$ below `tolerance_x_m` at fixed horizon $T$.
#[derive(Clone, Debug, PartialEq)]
pub struct L2BoundedArcTargetReport {
    /// Dynamical regime solved (`FiniteHorizonTargetedArc`, not a closed periodic orbit).
    pub regime: L2OrbitRegimeKind,
    /// Convergence status of the single-shooting corrector.
    pub status: L2SingleShootingStatus,
    /// Refined initial synodic position relative to `secondary` (`m`).
    pub initial_pos_syn_m: DVec3,
    /// Refined initial synodic velocity relative to `secondary` (`m/s`).
    pub initial_vel_syn_m_s: DVec3,
    /// Final terminal synodic residual $x_{\text{syn}}(t_f) - x_{L_2}$ (`m`).
    pub terminal_x_residual_m: f64,
    /// Number of differential-corrector shooting iterations executed.
    pub iterations: u32,
    /// Maximum distance from $L_2$ observed across the discrete post-solve sample points (`m`).
    pub max_l2_distance_m: f64,
    /// Minimum distance from $L_2$ observed across the discrete post-solve sample points (`m`).
    pub min_l2_distance_m: f64,
    /// Discrete rotating synodic position samples `(x - x_L2, y, z)` along the targeted arc (`m`).
    pub synodic_l2_samples_m: Vec<[f64; 3]>,
}

struct L2ShootContext<'a> {
    frame0: &'a SynodicFrame,
    pos_syn0: DVec3,
    l2_x_m: f64,
    horizon_s: f64,
    force_model: &'a ForceModel,
    ephemeris: &'a dyn EphemerisCatalog,
    orientation: &'a dyn BodyOrientationCatalog,
}

/// Adjusts initial synodic $\dot{y}_0$ via secant single-shooting over full-ephemeris `DOP853`
/// so the terminal synodic residual $|x_{\text{syn}}(t_0 + T) - x_{L_2}| \le \text{tolerance\_x\_m}$.
///
/// Fails closed with [`LagrangeError::SingleShootingFailed`] if `max_iterations` is reached or if
/// the secant slope collapses, and preserves all underlying [`crate::Dop853Error`] / [`crate::EphemerisError`] sources.
pub fn target_l2_terminal_x_arc_dop853(
    spec: &L2BoundedArcTargetSpec,
    force_model: &ForceModel,
    ephemeris: &dyn EphemerisCatalog,
    orientation: &dyn BodyOrientationCatalog,
) -> Result<L2BoundedArcTargetReport, LagrangeError> {
    if !spec.horizon_s.is_finite()
        || !spec.az_amplitude_m.is_finite()
        || !spec.tolerance_x_m.is_finite()
        || spec.horizon_s <= 0.0
        || spec.az_amplitude_m <= 0.0
        || spec.tolerance_x_m <= 0.0
        || spec.max_iterations == 0
    {
        return Err(LagrangeError::InvalidTargetParameters {
            horizon_s: spec.horizon_s,
            az_amplitude_m: spec.az_amplitude_m,
            tolerance_x_m: spec.tolerance_x_m,
        });
    }
    let mu_1 = force_model.constants().require_mu(spec.primary)?;
    let mu_2 = force_model.constants().require_mu(spec.secondary)?;
    let frame0 =
        SynodicFrame::from_ephemeris(spec.primary, spec.secondary, spec.start_epoch, ephemeris)?;
    let seed =
        approximate_l2_libration_seed(&frame0, mu_1, mu_2, spec.az_amplitude_m, spec.southern);
    let ctx = L2ShootContext {
        frame0: &frame0,
        pos_syn0: seed.pos_syn_rel_secondary_m,
        l2_x_m: seed.l2_x_rel_secondary_m,
        horizon_s: spec.horizon_s,
        force_model,
        ephemeris,
        orientation,
    };

    let mut vy0 = seed.vel_syn_rel_secondary_m_s.y;
    let mut f0 = eval_l2_radial_drift(&ctx, vy0)?;
    if f0.abs() <= spec.tolerance_x_m {
        return sample_targeted_l2_arc(&ctx, DVec3::new(0.0, vy0, 0.0), f0, 1);
    }
    let mut vy1 = vy0 * 1.02;
    let mut iter = 0_u32;
    while iter < spec.max_iterations {
        iter = iter.wrapping_add(1);
        let f1 = eval_l2_radial_drift(&ctx, vy1)?;
        if !f1.is_finite() {
            return Err(LagrangeError::SingleShootingFailed {
                reason: L2SingleShootingFailureReason::NonFiniteResidual,
                iterations: iter,
                residual_x_m: f1,
            });
        }
        if f1.abs() <= spec.tolerance_x_m {
            return sample_targeted_l2_arc(&ctx, DVec3::new(0.0, vy1, 0.0), f1, iter);
        }
        let denom = f1 - f0;
        if denom.abs() < 1.0e-3 {
            return Err(LagrangeError::SingleShootingFailed {
                reason: L2SingleShootingFailureReason::SecantSlopeDegenerate { delta_f_m: denom },
                iterations: iter,
                residual_x_m: f1,
            });
        }
        let step = (f1 * (vy1 - vy0) / denom).clamp(-250.0, 250.0);
        vy0 = vy1;
        f0 = f1;
        vy1 -= step;
    }

    Err(LagrangeError::SingleShootingFailed {
        reason: L2SingleShootingFailureReason::MaxIterationsReached {
            max_iterations: spec.max_iterations,
        },
        iterations: spec.max_iterations,
        residual_x_m: f0,
    })
}

fn eval_l2_radial_drift(ctx: &L2ShootContext<'_>, vy_syn0: f64) -> Result<f64, LagrangeError> {
    let vel_syn0 = DVec3::new(0.0, vy_syn0, 0.0);
    let (p_icrf, v_icrf) = ctx
        .frame0
        .synodic_to_icrf_rel_secondary(ctx.pos_syn0, vel_syn0);
    let state0 = ContinuousPlantState {
        primary_body: ctx.frame0.secondary,
        pos_rel_icrf_m: p_icrf,
        vel_rel_icrf_m_s: v_icrf,
        q_body_to_icrf: DQuat::IDENTITY,
        omega_body_rad_s: DVec3::ZERO,
        propellant_mass_kg: 0.0,
        h_rw_body_nms: DVec3::ZERO,
    };
    let end_epoch = ctx
        .frame0
        .epoch
        .add_span(TickSpan::from_seconds_and_subsec_ticks(
            ctx.horizon_s.max(1.0).round() as u64,
            0,
        ));
    let out = propagate_plant_interval(
        &state0,
        ctx.frame0.epoch,
        end_epoch,
        Some(3_600.0),
        &l2_tolerances(),
        &MassPropertiesModel::diagonal(6_500.0, [15_000.0; 3], [0.0; 3]),
        PropellantMode::InfiniteConstantMass,
        ReactionWheelMode::ConserveMomentum {
            h_max_nms: 1.0e5,
            tau_max_nm: 1.0e3,
        },
        ctx.force_model,
        ctx.ephemeris,
        ctx.orientation,
        &CoastControlLaw,
    )?;
    let frame_end = SynodicFrame::from_ephemeris(
        ctx.frame0.primary,
        ctx.frame0.secondary,
        end_epoch,
        ctx.ephemeris,
    )?;
    let (pos_syn_end, _) = frame_end.icrf_rel_secondary_to_synodic(
        out.final_state.pos_rel_icrf_m,
        out.final_state.vel_rel_icrf_m_s,
    );
    Ok(pos_syn_end.x - ctx.l2_x_m)
}

fn sample_targeted_l2_arc(
    ctx: &L2ShootContext<'_>,
    vel_syn0: DVec3,
    terminal_x_residual_m: f64,
    iterations: u32,
) -> Result<L2BoundedArcTargetReport, LagrangeError> {
    let (p_icrf, v_icrf) = ctx
        .frame0
        .synodic_to_icrf_rel_secondary(ctx.pos_syn0, vel_syn0);
    let mut state = ContinuousPlantState {
        primary_body: ctx.frame0.secondary,
        pos_rel_icrf_m: p_icrf,
        vel_rel_icrf_m_s: v_icrf,
        q_body_to_icrf: DQuat::IDENTITY,
        omega_body_rad_s: DVec3::ZERO,
        propellant_mass_kg: 0.0,
        h_rw_body_nms: DVec3::ZERO,
    };
    let n_chunks = 10_u32;
    let chunk_s = (ctx.horizon_s.max(10.0) / f64::from(n_chunks)).round() as u64;
    let mut cur_epoch = ctx.frame0.epoch;
    let mut min_d = f64::INFINITY;
    let mut max_d = 0.0_f64;
    let mut samples = Vec::with_capacity((n_chunks as usize).wrapping_add(1));
    let l2_vec0 = ctx.pos_syn0 - DVec3::new(ctx.l2_x_m, 0.0, 0.0);
    min_d = min_d.min(l2_vec0.length());
    max_d = max_d.max(l2_vec0.length());
    samples.push(l2_vec0.to_array());

    let mut idx = 0_u32;
    while idx < n_chunks {
        let next_epoch = cur_epoch.add_span(TickSpan::from_seconds_and_subsec_ticks(chunk_s, 0));
        let out = propagate_plant_interval(
            &state,
            cur_epoch,
            next_epoch,
            Some(3_600.0),
            &l2_tolerances(),
            &MassPropertiesModel::diagonal(6_500.0, [15_000.0; 3], [0.0; 3]),
            PropellantMode::InfiniteConstantMass,
            ReactionWheelMode::ConserveMomentum {
                h_max_nms: 1.0e5,
                tau_max_nm: 1.0e3,
            },
            ctx.force_model,
            ctx.ephemeris,
            ctx.orientation,
            &CoastControlLaw,
        )?;
        state = out.final_state;
        cur_epoch = next_epoch;
        let f_cur = SynodicFrame::from_ephemeris(
            ctx.frame0.primary,
            ctx.frame0.secondary,
            cur_epoch,
            ctx.ephemeris,
        )?;
        let (p_syn, _) =
            f_cur.icrf_rel_secondary_to_synodic(state.pos_rel_icrf_m, state.vel_rel_icrf_m_s);
        let rel_l2 = p_syn - DVec3::new(ctx.l2_x_m, 0.0, 0.0);
        let d = rel_l2.length();
        min_d = min_d.min(d);
        max_d = max_d.max(d);
        samples.push(rel_l2.to_array());
        idx = idx.wrapping_add(1);
    }

    Ok(L2BoundedArcTargetReport {
        regime: L2OrbitRegimeKind::FiniteHorizonTargetedArc,
        status: L2SingleShootingStatus::Converged,
        initial_pos_syn_m: ctx.pos_syn0,
        initial_vel_syn_m_s: vel_syn0,
        terminal_x_residual_m,
        iterations,
        max_l2_distance_m: max_d,
        min_l2_distance_m: min_d,
        synodic_l2_samples_m: samples,
    })
}

fn l2_tolerances() -> Dop853Tolerances {
    Dop853Tolerances {
        rtol: 1.0e-9,
        atol_pos_m: 1.0e-2,
        atol_vel_m_s: 1.0e-5,
        atol_att_rad: 1.0e-6,
        atol_omega_rad_s: 1.0e-6,
        atol_mass_kg: 1.0e-3,
        atol_h_rw_nms: 1.0,
        h_min_s: 1.0,
        h_max_s: 43_200.0,
        h_init_s: 3_600.0,
        max_steps: 50_000,
    }
}
