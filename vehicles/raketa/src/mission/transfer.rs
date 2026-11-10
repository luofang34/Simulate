//! The flight from a parking orbit to another body's sphere of influence, with no state
//! written in between: Lambert-guided finite burns that keep burning until the velocity
//! still owed is gone, coasts under the full force model, and origin changes that only
//! re-express the same state about another body.

use glam::DVec3;
use sim_orbital::{
    BuiltinSolarSystem2026_2028, ContinuousControlLaw, ContinuousPlantState, DerivativeOrder,
    Dop853Tolerances, EphemerisCatalog, ForceModel, NaifId, PlantStageWrench, PropellantMode,
    STANDARD_GRAVITY_G0_M_S2, TdbEpoch, propagate_plant_interval, solve_lambert_universal,
};

use crate::{
    aero_actuators::{raketa_default_rw_mode, raketa_mass_model},
    gnc_boundary::RaketaOuterGuidanceSetpoint,
    mission::{RaketaMissionError, closed_loop_tols},
};

/// Specific thrust of a transfer burn.
const BURN_ACCEL_M_S2: f64 = 3.5 * STANDARD_GRAVITY_G0_M_S2;
/// A burn is over when this little velocity is still owed (`m/s`).
const VELOCITY_TO_GO_DONE_M_S: f64 = 0.2;
/// The engine lights once the nose is within this of the burn direction (`rad`).
const POINTING_TOLERANCE_RAD: f64 = 0.03;
/// Radius inside which a body's own gravity leads (`m`).
const MOON_SOI_M: f64 = 66_183_000.0;
const EARTH_SOI_M: f64 = 924_600_000.0;
const MARS_SOI_M: f64 = 577_200_000.0;
const DAY_S: f64 = 86_400.0;

/// Tolerances for unpowered deep-space coast strides (`CoastControlLaw`), allowing the
/// 8th-order adaptive `DOP853` integrator to take steps up to `3,600 s` once reaction-wheel
/// rates are damped.
fn coast_tols(stride_s: f64) -> Dop853Tolerances {
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

/// One vessel in flight and the solar system it flies in.
pub(super) struct Voyage<'a> {
    pub(super) solar: &'a BuiltinSolarSystem2026_2028,
    pub(super) force_model: &'a ForceModel,
    pub(super) epoch: TdbEpoch,
    pub(super) state: ContinuousPlantState,
    pub(super) peak_proper_accel_m_s2: f64,
}

impl Voyage<'_> {
    fn fly_with_tols(
        &mut self,
        seconds: f64,
        tols: &Dop853Tolerances,
        law: &dyn ContinuousControlLaw,
    ) -> Result<(), RaketaMissionError> {
        let until = self.epoch.advanced_seconds(seconds);
        self.state = propagate_plant_interval(
            &self.state,
            self.epoch,
            until,
            None,
            tols,
            &raketa_mass_model(),
            PropellantMode::InfiniteConstantMass,
            raketa_default_rw_mode(),
            self.force_model,
            &self.solar.ephemeris,
            &self.solar.orientation,
            law,
        )?
        .final_state;
        self.epoch = until;
        Ok(())
    }

    fn coast(&mut self, seconds: f64) -> Result<(), RaketaMissionError> {
        let tols = coast_tols(seconds);
        self.fly_with_tols(seconds, &tols, &RateDampCoastLaw { kd_s_inv: 0.0 })
    }

    fn damp_rates(&mut self) -> Result<(), RaketaMissionError> {
        let tols = closed_loop_tols();
        self.fly_with_tols(10.0, &tols, &RateDampCoastLaw { kd_s_inv: 2.5 })
    }

    /// Flies `seconds` pointing along `direction` at `specific_force_m_s2` of thrust.
    fn thrust(
        &mut self,
        direction: DVec3,
        specific_force_m_s2: f64,
        seconds: f64,
    ) -> Result<(), RaketaMissionError> {
        let law = RaketaOuterGuidanceSetpoint::ThrustVector {
            direction_unit_icrf: direction,
            specific_force_m_s2,
            velocity_to_go_m_s: specific_force_m_s2 * seconds,
            cutoff: specific_force_m_s2 <= 0.0,
            fin_trim_deg: [0.0; 4],
            is_landing_or_pad_mode: false,
        }
        .into_stage_law(&self.solar.orientation);
        let (_, limited, _, _) = law.evaluate_detailed(self.epoch, &self.state);
        self.peak_proper_accel_m_s2 = self
            .peak_proper_accel_m_s2
            .max(limited.resulting_proper_accel_m_s2);
        let tols = closed_loop_tols();
        self.fly_with_tols(seconds, &tols, &law)
    }

    /// Burns until the vessel has the velocity `wanted` asks for, re-asking as it goes:
    /// the nose is brought round first, and the engine throttles down into the last of
    /// the velocity owed.
    fn burn_until(
        &mut self,
        wanted: impl Fn(&Self) -> Result<DVec3, RaketaMissionError>,
    ) -> Result<(), RaketaMissionError> {
        for _ in 0..4_000 {
            let owed = wanted(self)? - self.state.vel_rel_icrf_m_s;
            let to_go = owed.length();
            if to_go < VELOCITY_TO_GO_DONE_M_S {
                return self.damp_rates();
            }
            let direction = owed / to_go;
            let nose = self.state.q_body_to_icrf * DVec3::Y;
            if nose.angle_between(direction) > POINTING_TOLERANCE_RAD {
                self.thrust(direction, 0.0, 2.0)?;
                continue;
            }
            let seconds = (to_go / BURN_ACCEL_M_S2).clamp(0.05, 5.0);
            self.thrust(direction, (to_go / seconds).min(BURN_ACCEL_M_S2), seconds)?;
        }
        Err(RaketaMissionError::TransferBurnDidNotConverge { epoch: self.epoch })
    }

    /// Position and velocity of `body` from the vessel's origin.
    fn body_from_origin(
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

    fn distance_to(&self, body: NaifId) -> Result<f64, RaketaMissionError> {
        Ok((self.state.pos_rel_icrf_m - self.body_from_origin(body, self.epoch)?.0).length())
    }

    /// Re-expresses the same state about `origin`.
    fn move_origin_to(&mut self, origin: NaifId) -> Result<(), RaketaMissionError> {
        let (position, velocity) = self.force_model.switch_primary_frame(
            self.state.primary_body,
            origin,
            self.state.pos_rel_icrf_m,
            self.state.vel_rel_icrf_m_s,
            self.epoch,
            &self.solar.ephemeris,
        )?;
        self.state.primary_body = origin;
        self.state.pos_rel_icrf_m = position;
        self.state.vel_rel_icrf_m_s = velocity;
        Ok(())
    }

    /// The velocity that carries the vessel from where it is to where `target` will be
    /// at `arrival`, about the current origin.
    fn intercept_velocity(
        &self,
        target: NaifId,
        arrival: TdbEpoch,
    ) -> Result<DVec3, RaketaMissionError> {
        let mu = self
            .force_model
            .constants()
            .require_mu(self.state.primary_body)?;
        let aim = self.body_from_origin(target, arrival)?.0;
        let time_s = arrival.signed_offset_seconds_from(self.epoch);
        Ok(solve_lambert_universal(self.state.pos_rel_icrf_m, aim, time_s, mu, false)?.v1_m_s)
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
                self.burn_until(|voyage| voyage.intercept_velocity(target, arrival))?;
            }
            let step_s = ((dist_m - 0.85 * within_m) / 60_000.0).clamp(60.0, stride_s);
            self.coast(step_s)?;
        }
    }

    /// From an Earth parking orbit to the Moon's sphere of influence, three and a half
    /// days out.
    pub(super) fn transfer_to_the_moon(&mut self) -> Result<(), RaketaMissionError> {
        let arrival = self.epoch.advanced_seconds(3.5 * DAY_S);
        self.burn_until(|voyage| voyage.intercept_velocity(NaifId::MOON, arrival))?;
        self.coast_to(NaifId::MOON, arrival, 0.95 * MOON_SOI_M, 600.0, &[0.3, 0.7])?;
        self.move_origin_to(NaifId::MOON)
    }

    /// From an Earth parking orbit out of Earth's sphere of influence, then round the
    /// Sun to Mars's.
    pub(super) fn transfer_to_mars(&mut self) -> Result<(), RaketaMissionError> {
        let (arrival, v_inf) = self.cheapest_earth_to_mars_departure()?;
        for _ in 0..60 {
            if self.state.pos_rel_icrf_m.dot(v_inf) >= 0.0 {
                break;
            }
            self.coast(60.0)?;
        }
        let mu_earth = self.force_model.constants().require_mu(NaifId::EARTH)?;
        let v_inf_hat = v_inf.normalize_or_zero();
        let v_inf_sq = v_inf.length_squared();
        self.burn_until(|voyage| {
            let state = &voyage.state;
            let r_hat = state.pos_rel_icrf_m.normalize_or_zero();
            let dir = (v_inf_hat + 0.35 * r_hat).normalize_or_zero();
            let escape_speed = (2.0 * mu_earth / state.pos_rel_icrf_m.length()).sqrt();
            let leaving_speed = (escape_speed * escape_speed + v_inf_sq).sqrt();
            Ok(dir * leaving_speed)
        })?;
        while self.state.pos_rel_icrf_m.length() < 1.3 * EARTH_SOI_M {
            self.coast(3_600.0)?;
        }
        self.move_origin_to(NaifId::SUN)?;

        self.burn_until(|voyage| voyage.intercept_velocity(NaifId::MARS, arrival))?;
        self.coast_to(
            NaifId::MARS,
            arrival,
            0.95 * MARS_SOI_M,
            DAY_S / 4.0,
            &[0.5, 0.85, 0.97],
        )?;
        self.move_origin_to(NaifId::MARS)
    }

    /// The arrival date and Earth hyperbolic excess velocity $\mathbf{v}_\infty$, among trips
    /// of 120 to 320 days, that asks least of the engine.
    fn cheapest_earth_to_mars_departure(&self) -> Result<(TdbEpoch, DVec3), RaketaMissionError> {
        let mu_sun = self.force_model.constants().require_mu(NaifId::SUN)?;
        let earth = self.solar.ephemeris.geometric_state_icrf(
            NaifId::EARTH,
            NaifId::SUN,
            self.epoch,
            DerivativeOrder::Velocity,
        )?;
        let mut best: Option<(f64, TdbEpoch, DVec3)> = None;
        for days in (120..=320).step_by(10) {
            let tof_s = f64::from(days) * DAY_S;
            let arrival = self.epoch.advanced_seconds(tof_s);
            let mars = self.solar.ephemeris.geometric_state_icrf(
                NaifId::MARS,
                NaifId::SUN,
                arrival,
                DerivativeOrder::Position,
            )?;
            let Ok(sol) = solve_lambert_universal(earth.pos_m, mars.pos_m, tof_s, mu_sun, false)
            else {
                continue;
            };
            let v_inf = sol.v1_m_s - earth.vel_m_s;
            let cost = v_inf.length();
            if best.is_none_or(|(least, _, _)| cost < least) {
                best = Some((cost, arrival, v_inf));
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

/// Smooth RCS rate-damping and gyroscopic-precession-cancelling cruise control law.
struct RateDampCoastLaw {
    kd_s_inv: f64,
}

impl ContinuousControlLaw for RateDampCoastLaw {
    fn evaluate_stage(
        &self,
        _tau_s: f64,
        _epoch: TdbEpoch,
        state: &ContinuousPlantState,
    ) -> PlantStageWrench {
        let i_mat = raketa_mass_model().inertia_tensor(state.propellant_mass_kg);
        let omega = state.omega_body_rad_s;
        let gyro = omega.cross(i_mat * omega + state.h_rw_body_nms);
        let damp = i_mat * (omega * self.kd_s_inv);
        PlantStageWrench {
            ext_torque_body_nm: gyro - damp,
            ..PlantStageWrench::default()
        }
    }
}
