//! Assisted $L_2$ Planner combining linear/empirical libration seed generation,
//! finite-horizon targeted arc solving, local sensitivity analysis, and explicit
//! periodic-closure verification (`FiniteHorizonTargetedArc` vs. `PeriodicHaloOrbitCr3bp`).

use glam::{DQuat, DVec3};
use serde::{Deserialize, Serialize};
use sim_orbital::{
    ApproximateL2LibrationSeed, BodyOrientationCatalog, CoastControlLaw, ContinuousPlantState,
    Dop853Tolerances, EphemerisCatalog, ForceModel, L2BoundedArcTargetReport,
    L2BoundedArcTargetSpec, L2OrbitRegimeKind, LagrangeError, MassPropertiesModel, PropellantMode,
    ReactionWheelMode, SynodicFrame, TdbEpoch, approximate_l2_libration_seed,
    propagate_plant_interval, target_l2_terminal_x_arc_dop853,
};
use sim_spacetime::TickSpan;

/// Sensitivity and excursion bounds of a targeted finite-horizon $L_2$ arc.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct L2SensitivitySummary {
    /// Finite-difference sensitivity $\partial x_{\text{syn}}(t_f) / \partial \dot{y}_{\text{syn}}(t_0)$ (`s`).
    pub dx_terminal_dvy0_s: f64,
    /// Required change in $\dot{y}_{\text{syn}}(t_0)$ from the initial linear seed (`m/s`).
    pub delta_vy0_from_seed_m_s: f64,
    /// Minimum observed distance from $L_2$ along the targeted horizon (`m`).
    pub min_l2_distance_m: f64,
    /// Maximum observed distance from $L_2$ along the targeted horizon (`m`).
    pub max_l2_distance_m: f64,
}

/// Explicit verification of whether a trajectory segment satisfies 6D periodic synodic closure.
///
/// Prevents a finite-horizon $L_2$ targeted arc (`FiniteHorizonTargetedArc`) from being falsely
/// labeled as a closed periodic Halo orbit (`PeriodicHaloOrbitCr3bp`).
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct L2PeriodicClosureCheck {
    /// Certified dynamical regime (`FiniteHorizonTargetedArc` unless both position and velocity closure hold).
    pub certified_regime: L2OrbitRegimeKind,
    /// `true` iff $\|\Delta \mathbf{r}_{\text{syn}}\| \le \text{pos\_tolerance\_m}$ and $\|\Delta \mathbf{v}_{\text{syn}}\| \le \text{vel\_tolerance\_m\_s}$.
    pub satisfies_periodic_closure: bool,
    /// Terminal-minus-initial synodic position residual $\|\mathbf{r}_{\text{syn}}(t_f) - \mathbf{r}_{\text{syn}}(t_0)\|$ (`m`).
    pub synodic_position_closure_residual_m: f64,
    /// Terminal-minus-initial synodic velocity residual $\|\mathbf{v}_{\text{syn}}(t_f) - \mathbf{v}_{\text{syn}}(t_0)\|$ (`m/s`).
    pub synodic_velocity_closure_residual_m_s: f64,
    /// Required position closure tolerance (`m`).
    pub pos_tolerance_m: f64,
    /// Required velocity closure tolerance (`m/s`).
    pub vel_tolerance_m_s: f64,
}

/// Complete report produced by [`plan_assisted_l2_bounded_arc`].
#[derive(Clone, Debug, PartialEq)]
pub struct AssistedL2PlanReport {
    /// Input targeting specification.
    pub spec: L2BoundedArcTargetSpec,
    /// Approximate linear/empirical seed state around $L_2$.
    pub seed: ApproximateL2LibrationSeed,
    /// Solved finite-horizon targeted arc report.
    pub targeted_arc: L2BoundedArcTargetReport,
    /// Local sensitivity and distance excursion metrics.
    pub sensitivity: L2SensitivitySummary,
    /// Explicit periodic-closure verification proving whether full periodic closure holds.
    pub periodic_closure: L2PeriodicClosureCheck,
}

/// Runs the Assisted $L_2$ Planner:
/// 1. Computes the initial libration seed around $L_2$,
/// 2. Solves for $\dot{y}_{\text{syn}}(t_0)$ over the finite horizon $T$ using `DOP853`,
/// 3. Evaluates local terminal sensitivity $\partial x(t_f) / \partial \dot{y}_0$, and
/// 4. Performs an explicit 6D synodic periodic-closure check so finite-horizon targeted arcs
///    are never misreported as closed periodic Halo orbits.
pub fn plan_assisted_l2_bounded_arc(
    spec: &L2BoundedArcTargetSpec,
    force_model: &ForceModel,
    ephemeris: &dyn EphemerisCatalog,
    orientation: &dyn BodyOrientationCatalog,
    periodic_pos_tolerance_m: f64,
    periodic_vel_tolerance_m_s: f64,
) -> Result<AssistedL2PlanReport, LagrangeError> {
    let mu_1 = force_model.constants().require_mu(spec.primary)?;
    let mu_2 = force_model.constants().require_mu(spec.secondary)?;
    let frame0 =
        SynodicFrame::from_ephemeris(spec.primary, spec.secondary, spec.start_epoch, ephemeris)?;
    let seed =
        approximate_l2_libration_seed(&frame0, mu_1, mu_2, spec.az_amplitude_m, spec.southern);
    let targeted_arc = target_l2_terminal_x_arc_dop853(spec, force_model, ephemeris, orientation)?;

    let end_epoch = spec
        .start_epoch
        .add_span(TickSpan::from_seconds_and_subsec_ticks(
            spec.horizon_s.max(1.0).round() as u64,
            0,
        ));
    let (end_pos_syn, end_vel_syn) = evaluate_terminal_synodic_state(
        &frame0,
        end_epoch,
        targeted_arc.initial_pos_syn_m,
        targeted_arc.initial_vel_syn_m_s,
        force_model,
        ephemeris,
        orientation,
    )?;
    let dvy = 0.5_f64;
    let (pert_pos_syn, _) = evaluate_terminal_synodic_state(
        &frame0,
        end_epoch,
        targeted_arc.initial_pos_syn_m,
        targeted_arc.initial_vel_syn_m_s + DVec3::new(0.0, dvy, 0.0),
        force_model,
        ephemeris,
        orientation,
    )?;
    let dx_dvy0_s = (pert_pos_syn.x - end_pos_syn.x) / dvy;
    let delta_vy0 = targeted_arc.initial_vel_syn_m_s.y - seed.vel_syn_rel_secondary_m_s.y;

    let periodic_closure = verify_l2_periodic_closure(
        targeted_arc.initial_pos_syn_m,
        targeted_arc.initial_vel_syn_m_s,
        end_pos_syn,
        end_vel_syn,
        periodic_pos_tolerance_m,
        periodic_vel_tolerance_m_s,
    );

    Ok(AssistedL2PlanReport {
        spec: *spec,
        seed,
        sensitivity: L2SensitivitySummary {
            dx_terminal_dvy0_s: dx_dvy0_s,
            delta_vy0_from_seed_m_s: delta_vy0,
            min_l2_distance_m: targeted_arc.min_l2_distance_m,
            max_l2_distance_m: targeted_arc.max_l2_distance_m,
        },
        targeted_arc,
        periodic_closure,
    })
}

/// Verifies whether initial and terminal synodic states satisfy 6D periodic orbit closure.
#[must_use]
pub fn verify_l2_periodic_closure(
    initial_pos_syn_m: DVec3,
    initial_vel_syn_m_s: DVec3,
    terminal_pos_syn_m: DVec3,
    terminal_vel_syn_m_s: DVec3,
    pos_tolerance_m: f64,
    vel_tolerance_m_s: f64,
) -> L2PeriodicClosureCheck {
    let pos_res = (terminal_pos_syn_m - initial_pos_syn_m).length();
    let vel_res = (terminal_vel_syn_m_s - initial_vel_syn_m_s).length();
    let closed = pos_tolerance_m.is_finite()
        && vel_tolerance_m_s.is_finite()
        && pos_tolerance_m > 0.0
        && vel_tolerance_m_s > 0.0
        && pos_res <= pos_tolerance_m
        && vel_res <= vel_tolerance_m_s;
    L2PeriodicClosureCheck {
        certified_regime: if closed {
            L2OrbitRegimeKind::PeriodicHaloOrbitCr3bp
        } else {
            L2OrbitRegimeKind::FiniteHorizonTargetedArc
        },
        satisfies_periodic_closure: closed,
        synodic_position_closure_residual_m: pos_res,
        synodic_velocity_closure_residual_m_s: vel_res,
        pos_tolerance_m,
        vel_tolerance_m_s,
    }
}

fn evaluate_terminal_synodic_state(
    frame0: &SynodicFrame,
    end_epoch: TdbEpoch,
    pos_syn0: DVec3,
    vel_syn0: DVec3,
    force_model: &ForceModel,
    ephemeris: &dyn EphemerisCatalog,
    orientation: &dyn BodyOrientationCatalog,
) -> Result<(DVec3, DVec3), LagrangeError> {
    let (p_icrf, v_icrf) = frame0.synodic_to_icrf_rel_secondary(pos_syn0, vel_syn0);
    let state0 = ContinuousPlantState {
        primary_body: frame0.secondary,
        pos_rel_icrf_m: p_icrf,
        vel_rel_icrf_m_s: v_icrf,
        q_body_to_icrf: DQuat::IDENTITY,
        omega_body_rad_s: DVec3::ZERO,
        propellant_mass_kg: 0.0,
        h_rw_body_nms: DVec3::ZERO,
    };
    let tols = Dop853Tolerances {
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
    };
    let out = propagate_plant_interval(
        &state0,
        frame0.epoch,
        end_epoch,
        Some(3_600.0),
        &tols,
        &MassPropertiesModel::diagonal(6_500.0, [15_000.0; 3], [0.0; 3]),
        PropellantMode::InfiniteConstantMass,
        ReactionWheelMode::ConserveMomentum {
            h_max_nms: 1.0e5,
            tau_max_nm: 1.0e3,
        },
        force_model,
        ephemeris,
        orientation,
        &CoastControlLaw,
    )?;
    let frame_end =
        SynodicFrame::from_ephemeris(frame0.primary, frame0.secondary, end_epoch, ephemeris)?;
    Ok(frame_end.icrf_rel_secondary_to_synodic(
        out.final_state.pos_rel_icrf_m,
        out.final_state.vel_rel_icrf_m_s,
    ))
}
