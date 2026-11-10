//! Zero-state-reset continuous `Raketa` pad-to-orbit-to-transfer execution.
//!
//! Propagates `EarthPadPinned` $\to$ `EarthLiftoff` $\to$ `GravityTurnAscent` (all the way to
//! `orbit_achieved == true`) $\to$ `TransInjectionBurn` (Lambert targeting from the actual
//! achieved LEO state) $\to$ `DeepSpaceTransfer` (`SOI` primary-body frame transformation and
//! multi-body coast) with `explicit_state_reinitializations == 0`.

use glam::DVec3;
use serde::{Deserialize, Serialize};
use sim_orbital::{
    AscentGuidanceConfig, BuiltinSolarSystem2026_2028, ContinuousPlantState, ForceModel, NaifId,
    STANDARD_GRAVITY_G0_M_S2, TdbEpoch, evaluate_gravity_turn_ascent,
};

use super::voyage::Voyage;
use crate::{
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
    /// Measured count of state re-initializations (`0` when state is never synthetically overwritten).
    pub explicit_state_reinitializations: u32,
    /// Initial launch-pad epoch (`TDB`).
    pub start_epoch: TdbEpoch,
    /// Epoch at LEO parking orbit achievement (`TDB`).
    pub leo_insertion_epoch: TdbEpoch,
    /// Epoch at completion of the deep-space transfer coast arc (`TDB`).
    pub terminal_epoch: TdbEpoch,
    /// Final continuous plant state in the destination body's inertial frame.
    pub terminal_state: ContinuousPlantState,
    /// Final vessel position in Solar System Barycenter (`SSB`) `ICRF` coordinates (`m`).
    #[serde(with = "sim_orbital::serde_dvec3")]
    pub terminal_vessel_pos_ssb_m: DVec3,
    /// Distance from the destination body's centre at `terminal_epoch` (`m`).
    pub terminal_distance_to_target_m: f64,
    /// Integrated thrust $\Delta v$ expended during pad liftoff and gravity-turn ascent to LEO (`m/s`).
    pub ascent_delta_v_m_s: f64,
    /// Integrated thrust $\Delta v$ expended during the trans-injection burn (`TLI` or `TMI`, `m/s`).
    pub injection_burn_delta_v_m_s: f64,
    /// Integrated thrust $\Delta v$ expended during midcourse trajectory correction burns (`m/s`).
    pub midcourse_correction_delta_v_m_s: f64,
    /// Total integrated transfer $\Delta v$ after LEO insertion (`injection + midcourse`, `m/s`).
    pub total_transfer_delta_v_m_s: f64,
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
    let (mut voyage, departure) =
        fly_pad_to_destination_soi(solar, force_model, start_epoch, target)?;
    voyage.audit_continuity();
    let dest = destination_body(target);
    let terminal_vessel_pos_ssb_m = voyage.vessel_ssb_state()?.0;
    let terminal_distance_to_target_m = voyage.distance_to_target_m(dest)?;
    let total_transfer_delta_v_m_s =
        voyage.injection_burn_delta_v_m_s + voyage.midcourse_correction_delta_v_m_s;

    Ok(RaketaContinuousTransferSummary {
        target,
        phases_completed: departure.phases,
        explicit_state_reinitializations: voyage.state_reinitializations,
        start_epoch,
        leo_insertion_epoch: departure.leo_insertion_epoch,
        terminal_epoch: voyage.epoch,
        terminal_state: voyage.state,
        terminal_vessel_pos_ssb_m,
        terminal_distance_to_target_m,
        ascent_delta_v_m_s: departure.ascent_delta_v_m_s,
        injection_burn_delta_v_m_s: voyage.injection_burn_delta_v_m_s,
        midcourse_correction_delta_v_m_s: voyage.midcourse_correction_delta_v_m_s,
        total_transfer_delta_v_m_s,
        peak_proper_accel_g: voyage.peak_proper_accel_m_s2 / STANDARD_GRAVITY_G0_M_S2,
        fin_lock_verified: departure.fin_lock_verified,
    })
}

/// Bookkeeping of the flight from the pad to the destination's sphere of influence.
pub(super) struct Departure {
    pub(super) phases: Vec<RaketaMissionPhase>,
    pub(super) leo_insertion_epoch: TdbEpoch,
    pub(super) ascent_delta_v_m_s: f64,
    pub(super) fin_lock_verified: bool,
}

/// The destination body of `target`.
pub(super) const fn destination_body(target: RaketaMissionTarget) -> NaifId {
    match target {
        RaketaMissionTarget::MoonTranquility => NaifId::MOON,
        RaketaMissionTarget::MarsSurface => NaifId::MARS,
    }
}

/// Flies from the Earth pad into the destination's sphere of influence and hands back the
/// live voyage, so later phases continue the same propagated state and ledger.
pub(super) fn fly_pad_to_destination_soi<'a>(
    solar: &'a BuiltinSolarSystem2026_2028,
    force_model: &'a ForceModel,
    start_epoch: TdbEpoch,
    target: RaketaMissionTarget,
) -> Result<(Voyage<'a>, Departure), RaketaMissionError> {
    let mut phases = vec![RaketaMissionPhase::EarthPadPinned];
    let (epoch, state, liftoff_peak, liftoff_dv, pad_lock) =
        execute_earth_pad_liftoff(solar, force_model, start_epoch)?;
    phases.push(RaketaMissionPhase::EarthLiftoff);

    let mut voyage = Voyage::new(solar, force_model, epoch, state);
    voyage.peak_proper_accel_m_s2 = liftoff_peak;
    let ascent_dv = fly_ascent_to_leo(&mut voyage)?;
    let leo_insertion_epoch = voyage.epoch;
    phases.push(RaketaMissionPhase::GravityTurnAscent);

    match target {
        RaketaMissionTarget::MoonTranquility => voyage.transfer_to_the_moon()?,
        RaketaMissionTarget::MarsSurface => voyage.transfer_to_mars()?,
    }
    phases.push(RaketaMissionPhase::TransInjectionBurn);
    phases.push(RaketaMissionPhase::DeepSpaceTransfer);
    Ok((
        voyage,
        Departure {
            phases,
            leo_insertion_epoch,
            ascent_delta_v_m_s: liftoff_dv + ascent_dv,
            fin_lock_verified: pad_lock,
        },
    ))
}

fn fly_ascent_to_leo(voyage: &mut Voyage<'_>) -> Result<f64, RaketaMissionError> {
    let cfg = AscentGuidanceConfig::earth_leo_185km();
    let tols = closed_loop_tols();
    let mut ascent_dv = 0.0_f64;
    for _ in 0..36 {
        let state = voyage.state;
        let east = DVec3::Z.cross(state.pos_rel_icrf_m).normalize_or_zero();
        let g_out =
            evaluate_gravity_turn_ascent(state.pos_rel_icrf_m, state.vel_rel_icrf_m_s, east, &cfg);
        if g_out.orbit_achieved {
            break;
        }
        let law = RaketaOuterGuidanceSetpoint::from_ascent(&g_out, [0.0; 4])
            .into_stage_law(&voyage.solar.orientation);
        ascent_dv += voyage.fly_with_tols(10.0, &tols, &law)?;
    }
    Ok(ascent_dv)
}
