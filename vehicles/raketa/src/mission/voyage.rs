//! One vessel in flight from the pad to the destination surface, and the only three ways its
//! state may change: `DOP853` propagation, compliant contact integration, and origin changes
//! that re-express the same state about another body.
//!
//! Every flight primitive first compares the state it is about to propagate with the last
//! state a propagator produced, so any write that bypasses those three paths is counted as a
//! state reset rather than silently accepted.

use glam::DVec3;
use serde::{Deserialize, Serialize};
use sim_orbital::{
    BuiltinSolarSystem2026_2028, ContactStepOutput, ContinuousControlLaw, ContinuousPlantState,
    DerivativeOrder, Dop853Tolerances, EphemerisCatalog, ForceModel, NaifId, PlantStageWrench,
    PropellantMode, STANDARD_GRAVITY_G0_M_S2, TdbEpoch, VesselContactState,
    integrate_contact_interval, propagate_plant_until_step,
};
use sim_scenery::PinnedCollisionTerrain;

use crate::{
    aero_actuators::{raketa_default_rw_mode, raketa_mass_model},
    gnc_boundary::RaketaOuterGuidanceSetpoint,
    mission::{RaketaMissionError, closed_loop_tols},
};

/// Specific thrust of a finite orbital burn.
pub(super) const BURN_ACCEL_M_S2: f64 = 3.5 * STANDARD_GRAVITY_G0_M_S2;
/// A burn is over when this little velocity is still owed (`m/s`).
const VELOCITY_TO_GO_DONE_M_S: f64 = 0.2;
/// The engine lights once the nose is within this of the burn direction (`rad`).
const POINTING_TOLERANCE_RAD: f64 = 0.03;

/// One change of the coordinate origin, with the vessel's Solar System Barycenter (`SSB`)
/// state computed independently on both sides of it.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct RaketaOriginChange {
    /// Epoch of the origin change (`TDB`).
    pub epoch: TdbEpoch,
    /// Origin before the change.
    pub from_body: NaifId,
    /// Origin after the change.
    pub to_body: NaifId,
    /// Vessel `SSB` position before the change (`m`).
    #[serde(with = "sim_orbital::serde_dvec3")]
    pub ssb_pos_before_m: DVec3,
    /// Vessel `SSB` position after the change (`m`).
    #[serde(with = "sim_orbital::serde_dvec3")]
    pub ssb_pos_after_m: DVec3,
    /// Vessel `SSB` velocity before the change (`m/s`).
    #[serde(with = "sim_orbital::serde_dvec3")]
    pub ssb_vel_before_m_s: DVec3,
    /// Vessel `SSB` velocity after the change (`m/s`).
    #[serde(with = "sim_orbital::serde_dvec3")]
    pub ssb_vel_after_m_s: DVec3,
}

/// One vessel in flight and the solar system it flies in.
pub(super) struct Voyage<'a> {
    pub(super) solar: &'a BuiltinSolarSystem2026_2028,
    pub(super) force_model: &'a ForceModel,
    pub(super) epoch: TdbEpoch,
    pub(super) state: ContinuousPlantState,
    pub(super) peak_proper_accel_m_s2: f64,
    pub(super) injection_burn_delta_v_m_s: f64,
    pub(super) midcourse_correction_delta_v_m_s: f64,
    pub(super) state_reinitializations: u32,
    pub(super) origin_changes: Vec<RaketaOriginChange>,
    /// Whether thrust and slew laws also cancel the gyroscopic torque of the body rate
    /// acting on stored wheel momentum, which otherwise stalls the pointing loop once the
    /// wheels hold a few `1e5 N*m*s`.
    pub(super) cancel_gyroscopic_torque: bool,
    ledger_epoch: TdbEpoch,
    ledger_state: ContinuousPlantState,
}

impl<'a> Voyage<'a> {
    /// Starts a voyage from a state a propagator produced at `epoch`.
    pub(super) fn new(
        solar: &'a BuiltinSolarSystem2026_2028,
        force_model: &'a ForceModel,
        epoch: TdbEpoch,
        state: ContinuousPlantState,
    ) -> Self {
        Self {
            solar,
            force_model,
            epoch,
            state,
            peak_proper_accel_m_s2: 0.0,
            injection_burn_delta_v_m_s: 0.0,
            midcourse_correction_delta_v_m_s: 0.0,
            state_reinitializations: 0,
            origin_changes: Vec::new(),
            cancel_gyroscopic_torque: false,
            ledger_epoch: epoch,
            ledger_state: state,
        }
    }
}

impl Voyage<'_> {
    /// Counts a reset when the state or epoch differs from what the last propagator produced.
    pub(super) fn audit_continuity(&mut self) {
        if self.epoch != self.ledger_epoch || self.state != self.ledger_state {
            self.state_reinitializations = self.state_reinitializations.wrapping_add(1);
        }
    }

    fn commit(&mut self, epoch: TdbEpoch, state: ContinuousPlantState) {
        self.epoch = epoch;
        self.state = state;
        self.ledger_epoch = epoch;
        self.ledger_state = state;
    }

    /// Flies `seconds` under `law`, sampling the proper acceleration after every accepted
    /// step and returning the thrust $\Delta v$ integrated over those steps (`m/s`).
    pub(super) fn fly_with_tols(
        &mut self,
        seconds: f64,
        tols: &Dop853Tolerances,
        law: &dyn ContinuousControlLaw,
    ) -> Result<f64, RaketaMissionError> {
        self.audit_continuity();
        let until = self.epoch.advanced_seconds(seconds);
        let mass = raketa_mass_model();
        let mass_kg = mass.total_mass_kg(self.state.propellant_mass_kg);
        let mut previous = thrust_and_proper_accel(law, self.epoch, &self.state, mass_kg);
        let mut peak = previous.1;
        let mut delta_v = 0.0_f64;
        let out = propagate_plant_until_step(
            &self.state,
            self.epoch,
            until,
            None,
            tols,
            &mass,
            PropellantMode::InfiniteConstantMass,
            raketa_default_rw_mode(),
            self.force_model,
            &self.solar.ephemeris,
            &self.solar.orientation,
            law,
            false,
            &mut |segment| {
                let end = segment.start_epoch.advanced_seconds(segment.duration_s);
                let now = thrust_and_proper_accel(law, end, &segment.end_state, mass_kg);
                delta_v += 0.5 * (previous.0 + now.0) * segment.duration_s;
                peak = peak.max(now.1);
                previous = now;
                false
            },
        )?;
        self.peak_proper_accel_m_s2 = self.peak_proper_accel_m_s2.max(peak);
        self.commit(until, out.final_state);
        Ok(delta_v)
    }

    /// Integrates `seconds` of compliant ground contact under `law`, returning the contact
    /// output and the thrust $\Delta v$ integrated across the interval (`m/s`).
    pub(super) fn fly_in_contact(
        &mut self,
        seconds: f64,
        contact: VesselContactState,
        terrain: &PinnedCollisionTerrain,
        law: &dyn ContinuousControlLaw,
    ) -> Result<(ContactStepOutput, f64), RaketaMissionError> {
        self.audit_continuity();
        let until = self.epoch.advanced_seconds(seconds);
        let mass = raketa_mass_model();
        let mass_kg = mass.total_mass_kg(self.state.propellant_mass_kg);
        let before = thrust_and_proper_accel(law, self.epoch, &self.state, mass_kg).0;
        let feet = sim_orbital::ContactFootSpec::raketa_four_fin_pads(crate::RAKETA_CG_Y_M);
        let out = integrate_contact_interval(
            &self.state,
            contact,
            &feet,
            self.epoch,
            until,
            &mass,
            PropellantMode::InfiniteConstantMass,
            raketa_default_rw_mode(),
            self.force_model,
            terrain,
            &self.solar.ephemeris,
            &self.solar.orientation,
            law,
        )?;
        let after = thrust_and_proper_accel(law, until, &out.state, mass_kg).0;
        self.peak_proper_accel_m_s2 = self.peak_proper_accel_m_s2.max(out.peak_proper_accel_m_s2);
        self.commit(until, out.state);
        Ok((out, 0.5 * (before + after) * seconds))
    }

    pub(super) fn coast_with_tols(
        &mut self,
        seconds: f64,
        tols: &Dop853Tolerances,
    ) -> Result<(), RaketaMissionError> {
        self.fly_with_tols(seconds, tols, &RateDampCoastLaw { kd_s_inv: 0.0 })
            .map(|_| ())
    }

    pub(super) fn damp_rates(&mut self) -> Result<(), RaketaMissionError> {
        let tols = closed_loop_tols();
        self.fly_with_tols(10.0, &tols, &RateDampCoastLaw { kd_s_inv: 2.5 })
            .map(|_| ())
    }

    /// Flies `seconds` pointing along `direction` at `specific_force_m_s2` of thrust,
    /// returning the integrated thrust $\Delta v$ (`m/s`).
    pub(super) fn thrust(
        &mut self,
        direction: DVec3,
        specific_force_m_s2: f64,
        seconds: f64,
    ) -> Result<f64, RaketaMissionError> {
        let law = RaketaOuterGuidanceSetpoint::ThrustVector {
            direction_unit_icrf: direction,
            specific_force_m_s2,
            velocity_to_go_m_s: specific_force_m_s2 * seconds,
            cutoff: specific_force_m_s2 <= 0.0,
            fin_trim_deg: [0.0; 4],
            is_landing_or_pad_mode: false,
        }
        .into_stage_law(&self.solar.orientation);
        let tols = closed_loop_tols();
        if self.cancel_gyroscopic_torque {
            self.fly_with_tols(seconds.max(0.0), &tols, &GyroscopicCancelling(&law))
        } else {
            self.fly_with_tols(seconds.max(0.0), &tols, &law)
        }
    }

    /// Turns the nose towards `direction` for `seconds` with the engine off, going round
    /// through a perpendicular when `direction` is behind the nose (where the pointing
    /// error's cross product vanishes).
    pub(super) fn slew_towards(
        &mut self,
        direction: DVec3,
        seconds: f64,
    ) -> Result<(), RaketaMissionError> {
        let nose = self.state.q_body_to_icrf * DVec3::Y;
        let aim = if nose.dot(direction) < -0.5 {
            let perp = nose
                .cross(DVec3::Z)
                .try_normalize()
                .unwrap_or_else(|| nose.cross(DVec3::X).normalize_or_zero());
            (nose + perp).normalize_or_zero()
        } else {
            direction
        };
        self.thrust(aim, 0.0, seconds).map(|_| ())
    }

    /// Burns until the vessel has the velocity `wanted` asks for, re-asking as it goes:
    /// the nose is brought round first, and the engine throttles down into the last of
    /// the velocity owed. Returns the total integrated thrust $\Delta v$ (`m/s`).
    pub(super) fn burn_until(
        &mut self,
        wanted: impl Fn(&Self) -> Result<DVec3, RaketaMissionError>,
    ) -> Result<f64, RaketaMissionError> {
        let mut burn_dv = 0.0_f64;
        for _ in 0..4_000 {
            let owed = wanted(self)? - self.state.vel_rel_icrf_m_s;
            let to_go = owed.length();
            let nose = self.state.q_body_to_icrf * DVec3::Y;
            if to_go < VELOCITY_TO_GO_DONE_M_S
                || (burn_dv > 25.0 && to_go < 25.0 && owed.dot(nose) <= VELOCITY_TO_GO_DONE_M_S)
            {
                self.damp_rates()?;
                return Ok(burn_dv);
            }
            let direction = owed / to_go;
            if nose.angle_between(direction) > POINTING_TOLERANCE_RAD {
                self.slew_towards(direction, 2.0)?;
                continue;
            }
            let seconds = (to_go / BURN_ACCEL_M_S2).clamp(0.05, 5.0);
            burn_dv += self.thrust(direction, (to_go / seconds).min(BURN_ACCEL_M_S2), seconds)?;
        }
        Err(RaketaMissionError::TransferBurnDidNotConverge { epoch: self.epoch })
    }

    /// Position and velocity of `body` from the vessel's origin.
    pub(super) fn body_from_origin(
        &self,
        body: NaifId,
        at: TdbEpoch,
    ) -> Result<(DVec3, DVec3), RaketaMissionError> {
        if body == self.state.primary_body {
            return Ok((DVec3::ZERO, DVec3::ZERO));
        }
        let state = self.solar.ephemeris.geometric_state_icrf(
            body,
            self.state.primary_body,
            at,
            DerivativeOrder::Velocity,
        )?;
        Ok((state.pos_m, state.vel_m_s))
    }

    /// Re-expresses the same state about `origin`, recording the vessel's `SSB` position
    /// and velocity on both sides from independent body-to-`SSB` ephemeris lookups.
    pub(super) fn move_origin_to(&mut self, origin: NaifId) -> Result<(), RaketaMissionError> {
        self.audit_continuity();
        let from_body = self.state.primary_body;
        let (ssb_pos_before_m, ssb_vel_before_m_s) = self.vessel_ssb_state()?;
        let (position, velocity) = self.force_model.switch_primary_frame(
            from_body,
            origin,
            self.state.pos_rel_icrf_m,
            self.state.vel_rel_icrf_m_s,
            self.epoch,
            &self.solar.ephemeris,
        )?;
        let moved = ContinuousPlantState {
            primary_body: origin,
            pos_rel_icrf_m: position,
            vel_rel_icrf_m_s: velocity,
            ..self.state
        };
        self.commit(self.epoch, moved);
        let (ssb_pos_after_m, ssb_vel_after_m_s) = self.vessel_ssb_state()?;
        self.origin_changes.push(RaketaOriginChange {
            epoch: self.epoch,
            from_body,
            to_body: origin,
            ssb_pos_before_m,
            ssb_pos_after_m,
            ssb_vel_before_m_s,
            ssb_vel_after_m_s,
        });
        Ok(())
    }

    /// Vessel position and velocity about the Solar System Barycenter (`SSB`).
    pub(super) fn vessel_ssb_state(&self) -> Result<(DVec3, DVec3), RaketaMissionError> {
        if self.state.primary_body == NaifId::SSB {
            return Ok((self.state.pos_rel_icrf_m, self.state.vel_rel_icrf_m_s));
        }
        let origin = self.solar.ephemeris.geometric_state_icrf(
            self.state.primary_body,
            NaifId::SSB,
            self.epoch,
            DerivativeOrder::Velocity,
        )?;
        Ok((
            origin.pos_m + self.state.pos_rel_icrf_m,
            origin.vel_m_s + self.state.vel_rel_icrf_m_s,
        ))
    }
}

/// `(thrust specific force, proper acceleration)` of `law` at `state` (`m/s^2`).
fn thrust_and_proper_accel(
    law: &dyn ContinuousControlLaw,
    epoch: TdbEpoch,
    state: &ContinuousPlantState,
    mass_kg: f64,
) -> (f64, f64) {
    let wrench = law.evaluate_stage(0.0, epoch, state);
    let thrust = wrench.main_thrust_body_n + wrench.rcs_force_body_n;
    let proper = thrust + wrench.env_force_body_n;
    (thrust.length() / mass_kg, proper.length() / mass_kg)
}

/// Gyroscopic torque $\boldsymbol\omega \times (I\boldsymbol\omega + \mathbf{h}_{rw})$ on
/// the hull (`N*m`, body axes).
pub(super) fn gyroscopic_torque_body_nm(state: &ContinuousPlantState) -> DVec3 {
    let i_mat = raketa_mass_model().inertia_tensor(state.propellant_mass_kg);
    let omega = state.omega_body_rad_s;
    omega.cross(i_mat * omega + state.h_rw_body_nms)
}

/// Wraps a law with rotational-RCS torque that cancels the gyroscopic torque.
pub(super) struct GyroscopicCancelling<'l>(pub(super) &'l dyn ContinuousControlLaw);

impl ContinuousControlLaw for GyroscopicCancelling<'_> {
    fn evaluate_stage(
        &self,
        tau_s: f64,
        epoch: TdbEpoch,
        state: &ContinuousPlantState,
    ) -> PlantStageWrench {
        let mut wrench = self.0.evaluate_stage(tau_s, epoch, state);
        wrench.ext_torque_body_nm += gyroscopic_torque_body_nm(state);
        wrench
    }
}

/// Smooth RCS rate-damping and gyroscopic-precession-cancelling cruise control law.
pub(super) struct RateDampCoastLaw {
    pub(super) kd_s_inv: f64,
}

impl ContinuousControlLaw for RateDampCoastLaw {
    fn evaluate_stage(
        &self,
        _tau_s: f64,
        _epoch: TdbEpoch,
        state: &ContinuousPlantState,
    ) -> PlantStageWrench {
        let i_mat = raketa_mass_model().inertia_tensor(state.propellant_mass_kg);
        let damp = i_mat * (state.omega_body_rad_s * self.kd_s_inv);
        PlantStageWrench {
            ext_torque_body_nm: gyroscopic_torque_body_nm(state) - damp,
            ..PlantStageWrench::default()
        }
    }
}

#[cfg(test)]
mod tests;
