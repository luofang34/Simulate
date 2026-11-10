//! The flight from a parking orbit to another body's sphere of influence, with no state
//! written in between: Lambert-guided finite burns that keep burning until the velocity
//! still owed is gone, coasts under the full force model, and origin changes that only
//! re-express the same state about another body.

use glam::DVec3;
use sim_orbital::{
    DerivativeOrder, Dop853Tolerances, EphemerisCatalog, NaifId, TdbEpoch,
    propagate_kepler_universal, solve_lambert_universal,
};

use super::voyage::Voyage;
use crate::mission::RaketaMissionError;

/// Radius inside which a body's own gravity leads (`m`).
const MOON_SOI_M: f64 = 66_183_000.0;
const EARTH_SOI_M: f64 = 924_600_000.0;
const MARS_SOI_M: f64 = 577_200_000.0;
const DAY_S: f64 = 86_400.0;

/// Tolerances for unpowered deep-space coast strides (`CoastControlLaw`), allowing the
/// 8th-order adaptive `DOP853` integrator to take steps up to `3,600 s` once reaction-wheel
/// rates are damped.
pub(super) fn coast_tols(stride_s: f64) -> Dop853Tolerances {
    Dop853Tolerances {
        rtol: 1.0e-6,
        atol_pos_m: 1.0,
        atol_vel_m_s: 1.0e-2,
        atol_att_rad: 100.0,
        atol_omega_rad_s: 100.0,
        atol_mass_kg: 1.0,
        atol_h_rw_nms: 100.0,
        h_min_s: 1.0e-9,
        h_max_s: stride_s.clamp(1.0, 3_600.0),
        h_init_s: stride_s.clamp(1.0, 60.0),
        max_steps: 200_000,
    }
}

impl Voyage<'_> {
    pub(super) fn coast(&mut self, seconds: f64) -> Result<(), RaketaMissionError> {
        let tols = coast_tols(seconds);
        self.coast_with_tols(seconds, &tols)
    }

    fn distance_to(&self, body: NaifId) -> Result<f64, RaketaMissionError> {
        Ok((self.state.pos_rel_icrf_m - self.body_from_origin(body, self.epoch)?.0).length())
    }

    /// The velocity that carries the vessel from where it is to where `target` will be
    /// at `arrival`, about the current origin, selecting the branch (`short_way` vs `long_way`)
    /// closest to the vessel's current velocity.
    fn intercept_velocity(
        &self,
        target: NaifId,
        arrival: TdbEpoch,
    ) -> Result<DVec3, RaketaMissionError> {
        let aim = self.body_from_origin(target, arrival)?.0;
        self.lambert_velocity_to_point(aim, arrival)
    }

    pub(super) fn lambert_velocity_to_point(
        &self,
        aim: DVec3,
        arrival: TdbEpoch,
    ) -> Result<DVec3, RaketaMissionError> {
        let mu = self
            .force_model
            .constants()
            .require_mu(self.state.primary_body)?;
        let time_s = arrival.signed_offset_seconds_from(self.epoch);
        let short = solve_lambert_universal(self.state.pos_rel_icrf_m, aim, time_s, mu, false);
        let long = solve_lambert_universal(self.state.pos_rel_icrf_m, aim, time_s, mu, true);
        match (short, long) {
            (Ok(s), Ok(l)) => {
                let ds = (s.v1_m_s - self.state.vel_rel_icrf_m_s).length_squared();
                let dl = (l.v1_m_s - self.state.vel_rel_icrf_m_s).length_squared();
                Ok(if ds <= dl { s.v1_m_s } else { l.v1_m_s })
            }
            (Ok(s), Err(_)) => Ok(s.v1_m_s),
            (Err(_), Ok(l)) => Ok(l.v1_m_s),
            (Err(e), Err(_)) => Err(e.into()),
        }
    }

    /// Coasts in strides of `stride_s` towards `target`, correcting course at the given
    /// fractions of the way, until the vessel is `within_m` of it. Fails past `arrival`
    /// plus a tenth of the trip.
    fn coast_to(
        &mut self,
        target: NaifId,
        arrival: TdbEpoch,
        within_m: f64,
        stride_s: f64,
        corrections_at: &[f64],
    ) -> Result<(), RaketaMissionError> {
        let trip_s = arrival.signed_offset_seconds_from(self.epoch);
        let departed = self.epoch;
        let mut corrections = corrections_at.iter().copied().peekable();
        loop {
            let dist_m = self.distance_to(target)?;
            if dist_m <= within_m {
                return Ok(());
            }
            let flown_s = self.epoch.signed_offset_seconds_from(departed);
            if flown_s > 1.1 * trip_s {
                return Err(RaketaMissionError::TransferDidNotArrive {
                    target,
                    distance_m: dist_m,
                });
            }
            if corrections.next_if(|at| flown_s >= at * trip_s).is_some() {
                let mcc_dv =
                    self.burn_until(|voyage| voyage.intercept_velocity(target, arrival))?;
                self.midcourse_correction_delta_v_m_s += mcc_dv;
            }
            let step_s = ((dist_m - 0.85 * within_m) / 60_000.0).clamp(60.0, stride_s);
            self.coast(step_s)?;
        }
    }

    /// From an Earth parking orbit to the Moon's sphere of influence, three and a half
    /// days out.
    pub(super) fn transfer_to_the_moon(&mut self) -> Result<(), RaketaMissionError> {
        let mu_earth = self.force_model.constants().require_mu(NaifId::EARTH)?;
        let h_hat = self
            .state
            .pos_rel_icrf_m
            .cross(self.state.vel_rel_icrf_m_s)
            .normalize_or_zero();
        let best_tof_s = 3.5 * DAY_S;
        let mut best_coast_s = 0.0_f64;
        let mut best_cost = f64::INFINITY;
        for step in (0..=5_400).step_by(20) {
            let dt = f64::from(step);
            let (r_step, v_step) = propagate_kepler_universal(
                self.state.pos_rel_icrf_m,
                self.state.vel_rel_icrf_m_s,
                dt,
                mu_earth,
            );
            let t_arr = self.epoch.advanced_seconds(dt + best_tof_s);
            let moon_pos = self.body_from_origin(NaifId::MOON, t_arr)?.0;
            let moon_in_plane =
                (moon_pos - h_hat * moon_pos.dot(h_hat)).normalize_or_zero() * moon_pos.length();
            for long_way in [false, true] {
                if let Ok(sol) =
                    solve_lambert_universal(r_step, moon_in_plane, best_tof_s, mu_earth, long_way)
                {
                    let cost = (sol.v1_m_s - v_step).length();
                    if cost < best_cost {
                        best_cost = cost;
                        best_coast_s = dt;
                    }
                }
            }
        }
        let pre_burn_coast_s = (best_coast_s - 45.0).max(0.0);
        if pre_burn_coast_s > 0.0 {
            self.coast(pre_burn_coast_s)?;
        }
        let arrival = self.epoch.advanced_seconds(best_tof_s);
        let moon_pos = self.body_from_origin(NaifId::MOON, arrival)?.0;
        let moon_in_plane =
            (moon_pos - h_hat * moon_pos.dot(h_hat)).normalize_or_zero() * moon_pos.length();
        self.injection_burn_delta_v_m_s +=
            self.burn_until(|voyage| voyage.lambert_velocity_to_point(moon_in_plane, arrival))?;
        self.coast_to(NaifId::MOON, arrival, 0.95 * MOON_SOI_M, 600.0, &[0.3, 0.7])?;
        self.move_origin_to(NaifId::MOON)
    }

    /// From an Earth parking orbit out of Earth's sphere of influence, then round the
    /// Sun to Mars's.
    pub(super) fn transfer_to_mars(&mut self) -> Result<(), RaketaMissionError> {
        let mu_earth = self.force_model.constants().require_mu(NaifId::EARTH)?;
        let h_hat = self
            .state
            .pos_rel_icrf_m
            .cross(self.state.vel_rel_icrf_m_s)
            .normalize_or_zero();
        let (arrival, v_inf) = self.cheapest_earth_to_mars_departure(h_hat)?;
        let v_inf_in_plane = v_inf - h_hat * v_inf.dot(h_hat);
        let u_inf = v_inf_in_plane.normalize_or_zero();
        let v_inf_sq = v_inf.length_squared();
        let r0 = self.state.pos_rel_icrf_m.length();
        let ecc = 1.0 + r0 * v_inf_sq / mu_earth;
        let beta = (1.0 / ecc).clamp(0.0, 1.0).asin();
        let v_p_hat = glam::DQuat::from_axis_angle(h_hat, -(beta + 0.075)) * u_inf;

        let mut best_coast_s = 0.0_f64;
        let mut best_align = f64::NEG_INFINITY;
        for step in (0..=5_400).step_by(20) {
            let dt = f64::from(step);
            let (_, v_step) = propagate_kepler_universal(
                self.state.pos_rel_icrf_m,
                self.state.vel_rel_icrf_m_s,
                dt,
                mu_earth,
            );
            let align = v_step.normalize_or_zero().dot(v_p_hat);
            if align > best_align {
                best_align = align;
                best_coast_s = dt;
            }
        }
        if best_coast_s > 0.0 {
            self.coast(best_coast_s)?;
        }
        self.injection_burn_delta_v_m_s += self.burn_until(|voyage| {
            let state = &voyage.state;
            let v_hat = state.vel_rel_icrf_m_s.normalize_or_zero();
            let leaving_speed =
                (2.0 * mu_earth / state.pos_rel_icrf_m.length().max(1.0) + v_inf_sq).sqrt();
            Ok(v_hat * leaving_speed)
        })?;
        while self.state.pos_rel_icrf_m.length() < 1.3 * EARTH_SOI_M {
            self.coast(3_600.0)?;
        }
        self.move_origin_to(NaifId::SUN)?;

        self.midcourse_correction_delta_v_m_s +=
            self.burn_until(|voyage| voyage.intercept_velocity(NaifId::MARS, arrival))?;
        self.coast_to(
            NaifId::MARS,
            arrival,
            0.95 * MARS_SOI_M,
            DAY_S / 4.0,
            &[0.5, 0.85],
        )?;
        self.move_origin_to(NaifId::MARS)
    }

    /// The arrival date and Earth hyperbolic excess velocity $\mathbf{v}_\infty$, among trips
    /// of 120 to 320 days, that asks least of the engine.
    fn cheapest_earth_to_mars_departure(
        &self,
        h_hat: DVec3,
    ) -> Result<(TdbEpoch, DVec3), RaketaMissionError> {
        let mu_sun = self.force_model.constants().require_mu(NaifId::SUN)?;
        let escape_epoch = self.epoch.advanced_seconds(3.5 * DAY_S);
        let earth = self.solar.ephemeris.geometric_state_icrf(
            NaifId::EARTH,
            NaifId::SUN,
            escape_epoch,
            DerivativeOrder::Velocity,
        )?;
        let mut best: Option<(f64, TdbEpoch, DVec3)> = None;
        for days in (120..=350).step_by(5) {
            let tof_s = f64::from(days) * DAY_S;
            let arrival = escape_epoch.advanced_seconds(tof_s);
            let mars = self.solar.ephemeris.geometric_state_icrf(
                NaifId::MARS,
                NaifId::SUN,
                arrival,
                DerivativeOrder::Position,
            )?;
            for long_way in [false, true] {
                let Ok(sol) =
                    solve_lambert_universal(earth.pos_m, mars.pos_m, tof_s, mu_sun, long_way)
                else {
                    continue;
                };
                let v_inf = sol.v1_m_s - earth.vel_m_s;
                let out_of_plane = v_inf.dot(h_hat).abs();
                let cost = v_inf.length() + 0.8 * out_of_plane;
                if best.is_none_or(|(least, _, _)| cost < least) {
                    best = Some((cost, arrival, v_inf));
                }
            }
        }
        best.map(|(_, arrival, v_inf)| (arrival, v_inf)).ok_or(
            RaketaMissionError::TransferDidNotArrive {
                target: NaifId::MARS,
                distance_m: f64::INFINITY,
            },
        )
    }

    /// Distance to `target`, for the summary.
    pub(super) fn distance_to_target_m(&self, target: NaifId) -> Result<f64, RaketaMissionError> {
        self.distance_to(target)
    }
}
