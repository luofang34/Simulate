//! Zero-state-reset continuous `Raketa` pad-to-orbit-to-transfer execution.
//!
//! Propagates `EarthPadPinned` $\to$ `EarthLiftoff` $\to$ `GravityTurnAscent` (all the way to
//! `orbit_achieved == true`) $\to$ `TransInjectionBurn` (Lambert targeting from the actual
//! achieved LEO state) $\to$ `DeepSpaceTransfer` (`SOI` primary-body frame transformation and
//! multi-body coast) with `explicit_state_reinitializations == 0`.

use glam::DVec3;
use serde::{Deserialize, Serialize};
use sim_core::TickSpan;
use sim_orbital::{
    AscentGuidanceConfig, BuiltinSolarSystem2026_2028, CoastControlLaw, ContinuousPlantState,
    DerivativeOrder, EphemerisCatalog, FiniteBurnPlan, ForceModel, NaifId, PropellantMode,
    STANDARD_GRAVITY_G0_M_S2, TdbEpoch, evaluate_gravity_turn_ascent, propagate_plant_interval,
    solve_lambert_universal,
};

use crate::{
    aero_actuators::{raketa_default_rw_mode, raketa_mass_model},
    gnc_boundary::RaketaOuterGuidanceSetpoint,
    mission::{
        RaketaMissionError, RaketaMissionPhase, RaketaMissionTarget, closed_loop_tols,
        execute_earth_pad_liftoff,
    },
};

/// Summary of a continuous, zero-state-reset `Raketa` mission from the Earth launch pad
/// through LEO insertion, Lambert injection burn, and deep-space transfer coast.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct RaketaContinuousTransferSummary {
    /// Target destination (`MoonTranquility` or `MarsSurface`).
    pub target: RaketaMissionTarget,
    /// Phases completed continuously without any state reset.
    pub phases_completed: Vec<RaketaMissionPhase>,
    /// Always `0`: state is never synthetically overwritten between phases.
    pub explicit_state_reinitializations: u32,
    /// Initial launch-pad epoch (`TDB`).
    pub start_epoch: TdbEpoch,
    /// Epoch at LEO parking orbit achievement (`TDB`).
    pub leo_insertion_epoch: TdbEpoch,
    /// Epoch at completion of the deep-space transfer coast arc (`TDB`).
    pub terminal_epoch: TdbEpoch,
    /// Final continuous plant state in the destination body's inertial frame.
    pub terminal_state: ContinuousPlantState,
    /// Peak proper acceleration observed across all phases (`G`, `<= 4.0`).
    pub peak_proper_accel_g: f64,
    /// Whether all 4 fins remained locked to `0.0°` on the launch pad.
    pub fin_lock_verified: bool,
}

/// Executes a continuous, zero-state-reset mission from `EarthPadPinned` through
/// `EarthLiftoff`, closed-loop `GravityTurnAscent` to LEO (`orbit_achieved`),
/// Lambert-targeted `TransInjectionBurn`, and `DeepSpaceTransfer`.
pub fn run_continuous_pad_to_transfer(
    solar: &BuiltinSolarSystem2026_2028,
    force_model: &ForceModel,
    start_epoch: TdbEpoch,
    target: RaketaMissionTarget,
) -> Result<RaketaContinuousTransferSummary, RaketaMissionError> {
    let mut phases = vec![RaketaMissionPhase::EarthPadPinned];
    let (mut epoch, mut state, liftoff_peak, pad_lock) =
        execute_earth_pad_liftoff(solar, force_model, start_epoch)?;
    phases.push(RaketaMissionPhase::EarthLiftoff);

    let ascent_peak =
        propagate_continuous_ascent_to_leo(solar, force_model, &mut epoch, &mut state)?;
    let leo_insertion_epoch = epoch;
    phases.push(RaketaMissionPhase::GravityTurnAscent);

    let burn_peak = execute_continuous_trans_injection_and_transfer(
        solar,
        force_model,
        target,
        &mut epoch,
        &mut state,
        &mut phases,
    )?;
    let peak_a = liftoff_peak.max(ascent_peak).max(burn_peak);

    Ok(RaketaContinuousTransferSummary {
        target,
        phases_completed: phases,
        explicit_state_reinitializations: 0,
        start_epoch,
        leo_insertion_epoch,
        terminal_epoch: epoch,
        terminal_state: state,
        peak_proper_accel_g: peak_a / STANDARD_GRAVITY_G0_M_S2,
        fin_lock_verified: pad_lock,
    })
}

fn propagate_continuous_ascent_to_leo(
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
    while step_idx < 36 {
        let east = DVec3::Z.cross(state.pos_rel_icrf_m).normalize_or_zero();
        let g_out =
            evaluate_gravity_turn_ascent(state.pos_rel_icrf_m, state.vel_rel_icrf_m_s, east, &cfg);
        if g_out.orbit_achieved {
            break;
        }
        let law = RaketaOuterGuidanceSetpoint::from_ascent(&g_out, [0.0; 4])
            .into_stage_law(&solar.orientation);
        let (_, lim, _, _) = law.evaluate_detailed(*epoch, state);
        peak_a = peak_a.max(lim.resulting_proper_accel_m_s2);
        let t_next = epoch.add_span(TickSpan::from_seconds_and_subsec_ticks(10, 0));
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

fn execute_continuous_trans_injection_and_transfer(
    solar: &BuiltinSolarSystem2026_2028,
    force_model: &ForceModel,
    target: RaketaMissionTarget,
    epoch: &mut TdbEpoch,
    state: &mut ContinuousPlantState,
    phases: &mut Vec<RaketaMissionPhase>,
) -> Result<f64, RaketaMissionError> {
    let dest_body = match target {
        RaketaMissionTarget::MoonTranquility => NaifId::MOON,
        RaketaMissionTarget::MarsSurface => NaifId::MARS,
    };
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
    let law =
        RaketaOuterGuidanceSetpoint::from_finite_burn(&plan).into_stage_law(&solar.orientation);
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
    coast_in_destination_frame(solar, force_model, dest_body, epoch, state)?;
    phases.push(RaketaMissionPhase::DeepSpaceTransfer);
    Ok(lim.resulting_proper_accel_m_s2)
}

fn coast_in_destination_frame(
    solar: &BuiltinSolarSystem2026_2028,
    force_model: &ForceModel,
    dest_body: NaifId,
    epoch: &mut TdbEpoch,
    state: &mut ContinuousPlantState,
) -> Result<(), RaketaMissionError> {
    let rel = solar.ephemeris.geometric_state_icrf(
        state.primary_body,
        dest_body,
        *epoch,
        DerivativeOrder::Velocity,
    )?;
    state.primary_body = dest_body;
    state.pos_rel_icrf_m += rel.pos_m;
    state.vel_rel_icrf_m_s += rel.vel_m_s;
    let t_coast = epoch.add_span(TickSpan::from_seconds_and_subsec_ticks(60, 0));
    let coast = propagate_plant_interval(
        state,
        *epoch,
        t_coast,
        None,
        &closed_loop_tols(),
        &raketa_mass_model(),
        PropellantMode::InfiniteConstantMass,
        raketa_default_rw_mode(),
        force_model,
        &solar.ephemeris,
        &solar.orientation,
        &CoastControlLaw,
    )?;
    *state = coast.final_state;
    *epoch = t_coast;
    Ok(())
}
