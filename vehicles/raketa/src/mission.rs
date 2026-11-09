//! `Raketa` mission execution and staged phase-demonstration runners (`vehicles/raketa`).
//!
//! Strictly separates:
//! 1. **Continuous physical execution** ([`run_uninterrupted_mission`]):
//!    evolves state exclusively through continuous integration (`integrate_contact_interval` and
//!    `propagate_plant_interval`) with zero synthetic state resets, and returns an explicit
//!    [`RaketaMissionError::IncompleteContinuousMission`] at the first unimplemented continuous
//!    phase boundary rather than resetting state and claiming end-to-end completion.
//! 2. **Staged scenario-initialized phase demonstrations** ([`run_staged_phase_demonstration`]):
//!    exercises each guidance/control phase (`EarthLiftoff`, `GravityTurnAscent`, `TransInjectionBurn`,
//!    `ArrivalBrakingOrEntry`, `PoweredDescentAutoland`, `TargetSurfacePinnedRest`) from explicit
//!    scenario initial states and reports `is_continuous_end_to_end = false` alongside the exact
//!    count of scenario state initializations (`explicit_state_reinitializations`).

mod landing;

use glam::{DQuat, DVec3};
use serde::{Deserialize, Serialize};
use sim_core::TickSpan;
use sim_geo::{BodyEllipsoid, Geodetic, LocalFrame};
use sim_orbital::{
    AscentGuidanceConfig, BodyOrientationCatalog, BuiltinSolarSystem2026_2028, ConstantsError,
    ContactFootSpec, ContinuousPlantState, DerivativeOrder, Dop853Error, Dop853Tolerances,
    EphemerisCatalog, EphemerisError, FiniteBurnPlan, ForceModel, GravityError, LambertError,
    NaifId, OrientationError, PinnedSurfaceState, PropellantMode, STANDARD_GRAVITY_G0_M_S2,
    TdbEpoch, VesselContactState, evaluate_gravity_turn_ascent, integrate_contact_interval,
    propagate_plant_interval, quat_normalize_portable, solve_lambert_universal,
};
use sim_scenery::PinnedCollisionTerrain;

use crate::aero_actuators::{
    RAKETA_CG_Y_M, RaketaStageLaw, raketa_default_rw_mode, raketa_mass_model,
};
use landing::{run_staged_arrival_or_entry_demo, run_staged_autoland_and_surface_settle_demo};

/// Errors returned by `Raketa` continuous mission execution or staged phase demonstrations.
#[derive(Clone, Debug, PartialEq, thiserror::Error)]
pub enum RaketaMissionError {
    /// Underlying gravity or orientation evaluation failed.
    #[error("gravity error during Raketa mission: {0}")]
    Gravity(#[from] GravityError),
    /// Underlying planetary orientation evaluation failed.
    #[error("orientation error during Raketa mission: {0}")]
    Orientation(#[from] OrientationError),
    /// Underlying ephemeris lookup failed.
    #[error("ephemeris error during Raketa mission: {0}")]
    Ephemeris(#[from] EphemerisError),
    /// Gravitational constant lookup failed.
    #[error("constants error during Raketa mission: {0}")]
    Constants(#[from] ConstantsError),
    /// Continuous `DOP853` plant integration failed (preserved without masking as `SingularRadius`).
    #[error("DOP853 propagation failed during Raketa mission: {0}")]
    Propagation(#[from] Dop853Error<GravityError>),
    /// Universal-variable Lambert solver failed during transfer targeting.
    #[error("Lambert targeting failed during Raketa mission: {0}")]
    Lambert(#[from] LambertError),
    /// Touchdown contact integration did not settle into `PinnedSurfaceRest` within the window.
    #[error("touchdown contact integration on {body} did not settle into PinnedSurfaceRest")]
    TouchdownDidNotSettle {
        /// Destination surface body.
        body: NaifId,
    },
    /// Continuous end-to-end mission stopped at a phase transition that is not yet implemented
    /// without synthetic scenario state re-initialization.
    #[error(
        "uninterrupted continuous mission incomplete at {missing_transition_from:?} -> {missing_transition_to:?}: {reason}"
    )]
    IncompleteContinuousMission {
        /// Phases completed via continuous physical propagation with zero state resets.
        completed_phases: Vec<RaketaMissionPhase>,
        /// Last phase completed continuously.
        missing_transition_from: RaketaMissionPhase,
        /// Next phase requiring an unimplemented continuous guidance/targeting bridge.
        missing_transition_to: RaketaMissionPhase,
        /// Epoch reached at the end of continuous propagation.
        terminal_epoch: TdbEpoch,
        /// True physical plant state reached at `terminal_epoch` (without any synthetic overwrite).
        terminal_state: Box<ContinuousPlantState>,
        /// Peak proper acceleration observed across the completed continuous phases (`G`).
        peak_proper_accel_g: f64,
        /// Whether fin ground-lock held on the launch pad.
        fin_lock_verified: bool,
        /// Diagnostic explanation of the missing continuous transition capability.
        reason: &'static str,
    },
}

/// Target planetary destination for a `Raketa` mission or staged demonstration.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum RaketaMissionTarget {
    /// Earth launch pad $\to$ Moon Tranquility Base (`NaifId::MOON`).
    MoonTranquility,
    /// Earth launch pad $\to$ Mars surface (`NaifId::MARS`).
    MarsSurface,
}

/// Ordered flight phases in a `Raketa` mission or staged phase demonstration.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum RaketaMissionPhase {
    /// Locked in `PinnedSurfaceRest` on Earth launch pad prior to ignition.
    EarthPadPinned,
    /// Vertical pad liftoff clearing the launch tower.
    EarthLiftoff,
    /// Closed-loop gravity-turn ascent segment.
    GravityTurnAscent,
    /// Trans-Lunar (`TLI`) or Trans-Mars (`TMI`) finite-burn injection segment.
    TransInjectionBurn,
    /// Multi-body heliocentric / cislunar transfer cruise with SOI primary switch.
    DeepSpaceTransfer,
    /// Lunar braking or Martian `EntryGCorridor` atmospheric entry segment.
    ArrivalBrakingOrEntry,
    /// Closed-loop ZEM/ZEV `PoweredDescentAutoland` with `< 50 m` fin ground-lock.
    PoweredDescentAutoland,
    /// Settled in `PinnedSurfaceRest` on the target planetary surface.
    TargetSurfacePinnedRest,
}

/// Telemetry summary of a **staged scenario-initialized phase demonstration** (not a continuous mission).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct RaketaStagedDemoSummary {
    /// Mission target (`MoonTranquility` or `MarsSurface`).
    pub target: RaketaMissionTarget,
    /// Sequence of component phases exercised across the staged demonstration.
    pub phases_exercised: Vec<RaketaMissionPhase>,
    /// Always `false` for staged demonstrations that re-initialize state between phases.
    pub is_continuous_end_to_end: bool,
    /// Number of explicit scenario state re-initializations performed between phases (`> 0`).
    pub explicit_state_reinitializations: u32,
    /// Initial Earth pad epoch (`TDB`).
    pub start_epoch: TdbEpoch,
    /// Final touchdown & pinned-rest epoch (`TDB`).
    pub touchdown_epoch: TdbEpoch,
    /// Peak proper acceleration observed across all exercised phase segments (`m/s^2`, `<= 4.0G`).
    pub peak_proper_accel_m_s2: f64,
    /// Peak proper acceleration in standard gravities (`G`).
    pub peak_proper_accel_g: f64,
    /// Whether all 4 fins remained locked to `0.0°` on pad and below `50 m` AGL during landing.
    pub fin_lock_verified: bool,
    /// Final `PinnedSurfaceState` on the destination body (`MOON` or `MARS`).
    pub final_pinned_state: PinnedSurfaceState,
    /// Total integrated propellant mass change (`kg`, `0.0` under `InfiniteConstantMass`).
    pub delta_mass_kg: f64,
}

/// Compatibility alias for [`RaketaStagedDemoSummary`].
pub type RaketaMissionSummary = RaketaStagedDemoSummary;

/// Attempts to execute a continuous, zero-state-reset surface-to-surface mission.
///
/// Propagates `EarthPadPinned` $\to$ `EarthLiftoff` $\to$ `GravityTurnAscent` purely through
/// physical contact and `DOP853` integration without overwriting `state`, and then returns
/// [`RaketaMissionError::IncompleteContinuousMission`] because full continuous orbital insertion,
/// multi-day cislunar/interplanetary cruise, and continuous entry-to-landing guidance without
/// scenario state initialization are not yet implemented.
pub fn run_uninterrupted_mission(
    solar: &BuiltinSolarSystem2026_2028,
    force_model: &ForceModel,
    start_epoch: TdbEpoch,
    _target: RaketaMissionTarget,
) -> Result<RaketaStagedDemoSummary, RaketaMissionError> {
    let mut phases = vec![RaketaMissionPhase::EarthPadPinned];
    let (mut epoch, mut state, liftoff_peak, pad_lock) =
        execute_earth_pad_liftoff(solar, force_model, start_epoch)?;
    phases.push(RaketaMissionPhase::EarthLiftoff);

    let ascent_a = propagate_earth_ascent_arc(solar, force_model, &mut epoch, &mut state)?;
    let peak_a = liftoff_peak.max(ascent_a);
    phases.push(RaketaMissionPhase::GravityTurnAscent);

    Err(RaketaMissionError::IncompleteContinuousMission {
        completed_phases: phases,
        missing_transition_from: RaketaMissionPhase::GravityTurnAscent,
        missing_transition_to: RaketaMissionPhase::TransInjectionBurn,
        terminal_epoch: epoch,
        terminal_state: Box::new(state),
        peak_proper_accel_g: peak_a / STANDARD_GRAVITY_G0_M_S2,
        fin_lock_verified: pad_lock,
        reason: "continuous closed-loop orbital circularization, full multi-day transfer integration, and continuous entry-to-autoland handover without scenario state initialization are not yet implemented",
    })
}

/// Runs the **staged scenario-initialized phase demonstration** across all 8 `Raketa` phases.
///
/// Explicitly initializes scenario states at the LEO parking interface, Mars entry interface,
/// terminal approach interface, and touchdown contact interface (`is_continuous_end_to_end = false`,
/// `explicit_state_reinitializations > 0`).
pub fn run_staged_phase_demonstration(
    solar: &BuiltinSolarSystem2026_2028,
    force_model: &ForceModel,
    start_epoch: TdbEpoch,
    target: RaketaMissionTarget,
) -> Result<RaketaStagedDemoSummary, RaketaMissionError> {
    let mut phases = vec![RaketaMissionPhase::EarthPadPinned];
    let mut peak_a = 0.0_f64;
    let mut fin_lock_ok = true;
    let mut reinit_count = 0_u32;

    let (mut epoch, mut state, liftoff_peak, pad_lock) =
        execute_earth_pad_liftoff(solar, force_model, start_epoch)?;
    peak_a = peak_a.max(liftoff_peak);
    fin_lock_ok &= pad_lock;
    phases.push(RaketaMissionPhase::EarthLiftoff);

    let ascent_a = propagate_earth_ascent_arc(solar, force_model, &mut epoch, &mut state)?;
    peak_a = peak_a.max(ascent_a);
    phases.push(RaketaMissionPhase::GravityTurnAscent);

    initialize_leo_parking_state(&mut state);
    reinit_count = reinit_count.wrapping_add(1);

    let dest_body = match target {
        RaketaMissionTarget::MoonTranquility => NaifId::MOON,
        RaketaMissionTarget::MarsSurface => NaifId::MARS,
    };
    let transfer_a = run_staged_injection_and_soi_frame_demo(
        solar,
        force_model,
        dest_body,
        &mut epoch,
        &mut state,
        &mut phases,
    )?;
    peak_a = peak_a.max(transfer_a);

    let (entry_a, entry_reinits) =
        run_staged_arrival_or_entry_demo(solar, force_model, target, &mut epoch, &mut state)?;
    peak_a = peak_a.max(entry_a);
    reinit_count = reinit_count.wrapping_add(entry_reinits);
    phases.push(RaketaMissionPhase::ArrivalBrakingOrEntry);

    let (pinned, land_a, land_lock, land_reinits) = run_staged_autoland_and_surface_settle_demo(
        solar,
        force_model,
        target,
        &mut epoch,
        &mut state,
    )?;
    peak_a = peak_a.max(land_a);
    fin_lock_ok &= land_lock;
    reinit_count = reinit_count.wrapping_add(land_reinits);
    phases.push(RaketaMissionPhase::PoweredDescentAutoland);
    phases.push(RaketaMissionPhase::TargetSurfacePinnedRest);

    Ok(RaketaStagedDemoSummary {
        target,
        phases_exercised: phases,
        is_continuous_end_to_end: false,
        explicit_state_reinitializations: reinit_count,
        start_epoch,
        touchdown_epoch: epoch,
        peak_proper_accel_m_s2: peak_a,
        peak_proper_accel_g: peak_a / STANDARD_GRAVITY_G0_M_S2,
        fin_lock_verified: fin_lock_ok,
        final_pinned_state: pinned,
        delta_mass_kg: state.propellant_mass_kg,
    })
}

fn execute_earth_pad_liftoff(
    solar: &BuiltinSolarSystem2026_2028,
    force_model: &ForceModel,
    t0: TdbEpoch,
) -> Result<(TdbEpoch, ContinuousPlantState, f64, bool), RaketaMissionError> {
    let earth_terrain = PinnedCollisionTerrain::earth_wgs84(0.0, Vec::new());
    let ell = BodyEllipsoid::WGS84;
    let pad_geo = Geodetic {
        lat_rad: 28.572_9_f64.to_radians(),
        lon_rad: (-80.649_0_f64).to_radians(),
        height_m: RAKETA_CG_Y_M - 0.0369,
    };
    let pos_fixed = ell.geodetic_to_fixed(&pad_geo);
    let lf = LocalFrame::at(&pad_geo);
    let up_fixed = lf.up();
    let q_b2f = quat_normalize_portable(DQuat::from_rotation_arc(DVec3::Y, up_fixed));
    let pinned_pad = PinnedSurfaceState {
        body: NaifId::EARTH,
        pos_fixed_m: pos_fixed.to_array(),
        q_body_to_fixed_xyzw: q_b2f.to_array(),
    };
    let init_state = pinned_pad.to_icrf_plant_state(t0, 0.0, DVec3::ZERO, &solar.orientation)?;
    let init_contact = VesselContactState {
        feet: Vec::new(),
        settle_substeps: 250,
        pinned: Some(pinned_pad),
    };
    let rot = solar.orientation.body_rotation(NaifId::EARTH, t0)?;
    let up_icrf = rot.r_icrf_to_fixed.transpose() * up_fixed;
    let law = RaketaStageLaw {
        commanded_main_accel_m_s2: 2.8 * STANDARD_GRAVITY_G0_M_S2,
        desired_body_y_icrf: up_icrf,
        rcs_cmd_unit_body: DVec3::ZERO,
        fin_cmd_deg: [12.0, -8.0, 12.0, -8.0],
        is_landing_or_pad_mode: true,
        orientation: &solar.orientation,
    };
    let (_, lim, _, fin_locked) = law.evaluate_detailed(t0, &init_state);
    let t1 = t0.add_span(TickSpan::from_seconds_and_subsec_ticks(1, 0));
    let feet = ContactFootSpec::raketa_four_fin_pads(RAKETA_CG_Y_M);
    let out = integrate_contact_interval(
        &init_state,
        init_contact,
        &feet,
        t0,
        t1,
        &raketa_mass_model(),
        PropellantMode::InfiniteConstantMass,
        raketa_default_rw_mode(),
        force_model,
        &earth_terrain,
        &solar.ephemeris,
        &solar.orientation,
        &law,
    )?;
    Ok((
        t1,
        out.state,
        out.peak_proper_accel_m_s2
            .max(lim.resulting_proper_accel_m_s2),
        fin_locked && out.contact.pinned.is_none() && out.min_foot_clearance_m > 0.0,
    ))
}

pub(super) fn closed_loop_tols() -> Dop853Tolerances {
    Dop853Tolerances {
        rtol: 1.0e-5,
        atol_pos_m: 1.0e-2,
        atol_vel_m_s: 1.0e-3,
        atol_att_rad: 1.0e-4,
        atol_omega_rad_s: 1.0e-4,
        atol_mass_kg: 1.0e-3,
        atol_h_rw_nms: 1.0,
        h_min_s: 1.0e-6,
        h_max_s: 1.0,
        h_init_s: 0.25,
        max_steps: 200_000,
    }
}

fn propagate_earth_ascent_arc(
    solar: &BuiltinSolarSystem2026_2028,
    force_model: &ForceModel,
    epoch: &mut TdbEpoch,
    state: &mut ContinuousPlantState,
) -> Result<f64, RaketaMissionError> {
    let cfg = AscentGuidanceConfig::earth_leo_185km();
    let mass = raketa_mass_model();
    let rw = raketa_default_rw_mode();
    let tols = closed_loop_tols();
    let mut peak_a = 0.0_f64;

    let mut step_idx = 0_u32;
    while step_idx < 12 {
        let g_out = evaluate_gravity_turn_ascent(
            state.pos_rel_icrf_m,
            state.vel_rel_icrf_m_s,
            DVec3::Y,
            &cfg,
        );
        if g_out.orbit_achieved {
            break;
        }
        let law = RaketaStageLaw {
            commanded_main_accel_m_s2: g_out.commanded_accel_m_s2,
            desired_body_y_icrf: g_out.thrust_dir_icrf,
            rcs_cmd_unit_body: DVec3::ZERO,
            fin_cmd_deg: [2.0, 0.0, -2.0, 0.0],
            is_landing_or_pad_mode: false,
            orientation: &solar.orientation,
        };
        let (_, lim, _, _) = law.evaluate_detailed(*epoch, state);
        peak_a = peak_a.max(lim.resulting_proper_accel_m_s2);
        let t_next = epoch.add_span(TickSpan::from_seconds_and_subsec_ticks(15, 0));
        let prop = propagate_plant_interval(
            state,
            *epoch,
            t_next,
            None,
            &tols,
            &mass,
            PropellantMode::InfiniteConstantMass,
            rw,
            force_model,
            &solar.ephemeris,
            &solar.orientation,
            &law,
        )?;
        *state = prop.final_state;
        *epoch = t_next;
        step_idx = step_idx.wrapping_add(1);
    }
    Ok(peak_a)
}

fn initialize_leo_parking_state(state: &mut ContinuousPlantState) {
    let cfg = AscentGuidanceConfig::earth_leo_185km();
    let r_leo = cfg.body_radius_m + cfg.target_orbit_alt_m;
    let v_leo = libm::sqrt(cfg.mu_m3_s2 / r_leo);
    let u_r = state.pos_rel_icrf_m.normalize();
    let u_t = DVec3::Z.cross(u_r).normalize();
    state.pos_rel_icrf_m = u_r * r_leo;
    state.vel_rel_icrf_m_s = u_t * v_leo;
}

fn run_staged_injection_and_soi_frame_demo(
    solar: &BuiltinSolarSystem2026_2028,
    force_model: &ForceModel,
    dest_body: NaifId,
    epoch: &mut TdbEpoch,
    state: &mut ContinuousPlantState,
    phases: &mut Vec<RaketaMissionPhase>,
) -> Result<f64, RaketaMissionError> {
    let mu_earth = force_model.constants().require_mu(NaifId::EARTH)?;
    let tof_s = if dest_body == NaifId::MOON {
        3.2 * 86_400.0
    } else {
        4.5 * 86_400.0
    };
    let t_arr = epoch.add_span(TickSpan::from_seconds_and_subsec_ticks(4_000, 0));
    let dest_rel_earth = solar.ephemeris.geometric_state_icrf(
        dest_body,
        NaifId::EARTH,
        t_arr,
        DerivativeOrder::Velocity,
    )?;
    let r_target = if dest_body == NaifId::MOON {
        dest_rel_earth.pos_m
    } else {
        dest_rel_earth.pos_m.normalize() * 4.2e8
    };
    let lambert = solve_lambert_universal(state.pos_rel_icrf_m, r_target, tof_s, mu_earth, false)?;
    let dv_imp = lambert.v1_m_s - state.vel_rel_icrf_m_s;
    let plan = FiniteBurnPlan::from_impulsive(dv_imp, 3.5 * STANDARD_GRAVITY_G0_M_S2);

    let law = RaketaStageLaw {
        commanded_main_accel_m_s2: plan.accel_m_s2,
        desired_body_y_icrf: plan.unit_dir_icrf,
        rcs_cmd_unit_body: DVec3::ZERO,
        fin_cmd_deg: [0.0; 4],
        is_landing_or_pad_mode: false,
        orientation: &solar.orientation,
    };
    let (_, lim, _, _) = law.evaluate_detailed(*epoch, state);
    let t_burn_end = epoch.add_span(TickSpan::from_seconds_and_subsec_ticks(20, 0));
    let prop = propagate_plant_interval(
        state,
        *epoch,
        t_burn_end,
        None,
        &closed_loop_tols(),
        &raketa_mass_model(),
        PropellantMode::InfiniteConstantMass,
        raketa_default_rw_mode(),
        force_model,
        &solar.ephemeris,
        &solar.orientation,
        &law,
    )?;
    *state = prop.final_state;
    *epoch = t_burn_end;
    phases.push(RaketaMissionPhase::TransInjectionBurn);

    let rel = solar.ephemeris.geometric_state_icrf(
        state.primary_body,
        dest_body,
        *epoch,
        DerivativeOrder::Velocity,
    )?;
    state.primary_body = dest_body;
    state.pos_rel_icrf_m += rel.pos_m;
    state.vel_rel_icrf_m_s += rel.vel_m_s;
    phases.push(RaketaMissionPhase::DeepSpaceTransfer);
    Ok(lim.resulting_proper_accel_m_s2)
}
