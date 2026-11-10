//! Arrival from the edge of the destination's sphere of influence to rest above the landing
//! site: a Lambert approach aimed at a gate point over the site, re-aimed as the approach
//! closes, then a propulsive braking burn that cancels the velocity relative to the turning
//! surface. Raketa carries unlimited propellant, so the capture is entirely propulsive.

use glam::DVec3;
use sim_geo::Geodetic;
use sim_orbital::{BodyOrientationCatalog, TdbEpoch, solve_lambert_universal};

use super::{
    landing::TargetSiteSpec,
    voyage::{BURN_ACCEL_M_S2, Voyage},
};
use crate::{aero_actuators::RAKETA_CG_Y_M, mission::RaketaMissionError};

/// Shallowest allowed arrival: the approach must meet the gate within this angle of the
/// local vertical, so braking happens well above the ground (`rad`).
const MAX_ARRIVAL_ZENITH_RAD: f64 = 60.0 * core::f64::consts::PI / 180.0;
/// Fractions of the approach time at which the aim is re-solved.
const CORRECTIONS_AT: [f64; 4] = [0.5, 0.8, 0.95, 0.99];
/// Braking distance margin over the constant-deceleration estimate.
const IGNITION_MARGIN: f64 = 1.1;
/// Fixed distance added to the braking estimate (`m`).
const IGNITION_PAD_M: f64 = 2_000.0;
/// Braking ends when this little velocity relative to the surface is left (`m/s`).
const BRAKING_DONE_M_S: f64 = 0.5;
/// Below this residual the descent law takes over rather than waiting for the nose (`m/s`).
const BRAKING_HANDOFF_M_S: f64 = 15.0;
/// Main thrust is allowed within this angle of the braking direction (`rad`).
const BRAKING_POINTING_RAD: f64 = 0.2;
/// Before ignition the nose is brought round this long in advance (`s`).
const PRE_SLEW_S: f64 = 150.0;

/// Where the arrival aims and how much it cost.
pub(super) struct Arrival {
    /// CG position at touchdown above the landing site, body-fixed (`m`).
    pub(super) touchdown_fixed_m: DVec3,
    /// Thrust $\Delta v$ of the approach aiming burns (`m/s`).
    pub(super) targeting_delta_v_m_s: f64,
    /// Thrust $\Delta v$ of the braking burn (`m/s`).
    pub(super) braking_delta_v_m_s: f64,
}

impl Voyage<'_> {
    /// Flies from the sphere of influence to rest at roughly `gate_alt_m` above `site`.
    pub(super) fn approach_and_brake(
        &mut self,
        site: &TargetSiteSpec,
        gate_alt_m: f64,
    ) -> Result<Arrival, RaketaMissionError> {
        let at = |height_m: f64| {
            site.ellipsoid.geodetic_to_fixed(&Geodetic {
                height_m: site.ground_h_m + height_m,
                ..site.site_geo
            })
        };
        self.cancel_gyroscopic_torque = true;
        let gate_fixed = at(gate_alt_m);
        let touchdown_fixed_m = at(RAKETA_CG_Y_M - 0.0369);
        let arrival = self.choose_arrival_time(site, gate_fixed)?;
        let gate_icrf = self.fixed_to_icrf(site, gate_fixed, arrival)?;
        let targeting_delta_v_m_s = self.close_on_gate(site, gate_icrf, arrival)?;
        let braking_delta_v_m_s = self.brake_to_surface_rest()?;
        Ok(Arrival {
            touchdown_fixed_m,
            targeting_delta_v_m_s,
            braking_delta_v_m_s,
        })
    }

    /// Velocity of the turning surface beneath the vessel, in `ICRF` about the body.
    fn surface_velocity_icrf(&self) -> Result<DVec3, RaketaMissionError> {
        let rot = self
            .solar
            .orientation
            .body_rotation(self.state.primary_body, self.epoch)?;
        let p_fixed = rot.r_icrf_to_fixed * self.state.pos_rel_icrf_m;
        Ok(rot.fixed_to_icrf_pv(p_fixed, DVec3::ZERO).1)
    }

    /// Burns against the velocity relative to the surface until almost none is left,
    /// thrusting whenever the nose is within [`BRAKING_POINTING_RAD`] of the burn direction
    /// (the slow attitude loop never holds a tighter cone while the direction turns).
    /// Returns the thrust $\Delta v$ (`m/s`).
    fn brake_to_surface_rest(&mut self) -> Result<f64, RaketaMissionError> {
        let mut braking_dv = 0.0_f64;
        for _ in 0..20_000 {
            let owed = self.surface_velocity_icrf()? - self.state.vel_rel_icrf_m_s;
            let to_go = owed.length();
            let direction = owed / to_go.max(1.0e-9);
            let nose = self.state.q_body_to_icrf * DVec3::Y;
            let off = nose.angle_between(direction);
            if to_go < BRAKING_DONE_M_S
                || (to_go < BRAKING_HANDOFF_M_S && off > BRAKING_POINTING_RAD)
            {
                return Ok(braking_dv);
            }
            if off > BRAKING_POINTING_RAD {
                self.slew_towards(direction, 1.0)?;
                continue;
            }
            let seconds = (to_go / BURN_ACCEL_M_S2).clamp(0.05, 1.0);
            braking_dv +=
                self.thrust(direction, (to_go / seconds).min(BURN_ACCEL_M_S2), seconds)?;
        }
        Err(RaketaMissionError::TransferBurnDidNotConverge { epoch: self.epoch })
    }

    /// The short-way Lambert velocity to `aim` at `arrival`: the long way round would pass
    /// periapsis, inside the body, before reaching a gate this close to the surface.
    fn short_way_velocity_to(
        &self,
        aim: DVec3,
        arrival: TdbEpoch,
    ) -> Result<DVec3, RaketaMissionError> {
        let mu = self
            .force_model
            .constants()
            .require_mu(self.state.primary_body)?;
        let time_s = arrival.signed_offset_seconds_from(self.epoch);
        Ok(solve_lambert_universal(self.state.pos_rel_icrf_m, aim, time_s, mu, false)?.v1_m_s)
    }

    fn fixed_to_icrf(
        &self,
        site: &TargetSiteSpec,
        p_fixed: DVec3,
        at: TdbEpoch,
    ) -> Result<DVec3, RaketaMissionError> {
        let rot = self.solar.orientation.body_rotation(site.body, at)?;
        Ok(rot.fixed_to_icrf_pv(p_fixed, DVec3::ZERO).0)
    }

    /// The arrival time whose short-way Lambert arc reaches the gate point descending at
    /// least as steeply as [`MAX_ARRIVAL_ZENITH_RAD`] and costs the least to aim at. A
    /// short-way arc that is still descending at the gate has not yet passed periapsis, so it
    /// never dips below the gate's radius on the way in.
    fn choose_arrival_time(
        &self,
        site: &TargetSiteSpec,
        gate_fixed: DVec3,
    ) -> Result<TdbEpoch, RaketaMissionError> {
        let mu = self.force_model.constants().require_mu(site.body)?;
        let r = self.state.pos_rel_icrf_m;
        let v = self.state.vel_rel_icrf_m_s;
        let straight_s = (r.length() - site.ellipsoid.semi_major_m) / v.length().max(1.0);
        let mut best: Option<(f64, TdbEpoch)> = None;
        let mut steepest_rad = f64::INFINITY;
        for i in 0..=90_u32 {
            let tof_s = straight_s * (0.3 + 0.03 * f64::from(i));
            let arrival = self.epoch.advanced_seconds(tof_s);
            let gate = self.fixed_to_icrf(site, gate_fixed, arrival)?;
            let Ok(arc) = solve_lambert_universal(r, gate, tof_s, mu, false) else {
                continue;
            };
            let zenith = (-arc.v2_m_s).angle_between(gate);
            steepest_rad = steepest_rad.min(zenith);
            if zenith > MAX_ARRIVAL_ZENITH_RAD {
                continue;
            }
            let cost = (arc.v1_m_s - v).length();
            if best.is_none_or(|(least, _)| cost < least) {
                best = Some((cost, arrival));
            }
        }
        best.map(|(_, arrival)| arrival)
            .ok_or(RaketaMissionError::LandingSiteOutOfView {
                body: site.body,
                steepest_arrival_zenith_deg: steepest_rad.to_degrees(),
            })
    }

    /// Coasts towards `gate_icrf`, re-aiming at the set fractions of the way, and stops
    /// when braking at full thrust has to begin. Returns the aiming $\Delta v$ (`m/s`).
    fn close_on_gate(
        &mut self,
        site: &TargetSiteSpec,
        gate_icrf: DVec3,
        arrival: TdbEpoch,
    ) -> Result<f64, RaketaMissionError> {
        let mu = self.force_model.constants().require_mu(site.body)?;
        let local_g = mu / gate_icrf.length_squared();
        let decel = (BURN_ACCEL_M_S2 - local_g).max(1.0);
        let departed = self.epoch;
        let trip_s = arrival.signed_offset_seconds_from(departed);
        let mut aiming_dv =
            self.burn_until(|voyage| voyage.short_way_velocity_to(gate_icrf, arrival))?;
        let mut corrections = CORRECTIONS_AT.iter().copied().peekable();
        loop {
            let offset = gate_icrf - self.state.pos_rel_icrf_m;
            let speed = self.state.vel_rel_icrf_m_s.length().max(1.0);
            let ignition_m = IGNITION_MARGIN * speed * speed / (2.0 * decel) + IGNITION_PAD_M;
            let to_ignition_s = (offset.length() - ignition_m) / speed;
            if to_ignition_s <= 0.5 {
                return Ok(aiming_dv);
            }
            let flown_s = self.epoch.signed_offset_seconds_from(departed);
            if flown_s > 1.2 * trip_s || offset.dot(self.state.vel_rel_icrf_m_s) < 0.0 {
                return Err(RaketaMissionError::TransferDidNotArrive {
                    target: site.body,
                    distance_m: offset.length(),
                });
            }
            if corrections.next_if(|at| flown_s >= at * trip_s).is_some() {
                aiming_dv +=
                    self.burn_until(|voyage| voyage.short_way_velocity_to(gate_icrf, arrival))?;
                continue;
            }
            if to_ignition_s < PRE_SLEW_S {
                let retro = -self.state.vel_rel_icrf_m_s.normalize_or_zero();
                self.slew_towards(retro, (0.5 * to_ignition_s).clamp(0.5, 10.0))?;
            } else {
                self.coast((0.5 * (to_ignition_s - PRE_SLEW_S)).clamp(1.0, 3_600.0))?;
            }
        }
    }
}
