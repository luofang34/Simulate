//! Closed-loop EFB LNAV/VNAV, localizer/glidepath approach, flare, and centreline braking
//! rollout autopilot for both charted and ad-hoc runways (including short bridge decks
//! such as `ZMRS` Bolshoy Moskvoretsky Bridge).

mod enroute;

use std::f64::consts::PI;

use glam::{DVec2, DVec3};
use sim_core::{AirframeArticulation, ControlInputs, VehicleState};
use sim_geo::{Geodetic, LocalFrame, geodetic_to_ecef};
use sim_maplibre::RouteProgress;
use sim_scenery::LandingRunwayGeometry;

use super::tokens::RouteToken;

const GRAVITY_M_S2: f64 = 9.806_65;
const RATE_KP: f64 = 0.55;
const RATE_KI: f64 = 1.6;
const BANK_GAIN: f64 = 1.65;
const FLIGHT_PATH_GAIN: f64 = 1.15;
const MAX_BANK_RAD: f64 = 25.0 * PI / 180.0;
const MAX_TURN_RATE_RAD_S: f64 = 0.45;
const SPEED_KP: f64 = 0.15;
const SPEED_KI: f64 = 0.04;

/// Active guidance sub-mode of [`EfbAutopilot`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum EfbAutopilotPhase {
    /// Autopilot disconnected; manual stick/throttle in direct control.
    #[default]
    Off,
    /// Enroute great-circle LNAV + altitude VNAV (or racetrack hold).
    EnrouteLnav,
    /// Coupled final approach localizer + `3.0°` glidepath capture.
    Approach,
    /// Idle-thrust exponential landing flare over the runway/bridge deck.
    Flare,
    /// Post-touchdown wings-level centreline rudder tracking and progressive wheel braking.
    Rollout,
}

impl EfbAutopilotPhase {
    /// Short FMA / EFB badge label (`OFF`, `LNAV`, `APP`, `FLARE`, `ROLL`).
    #[must_use]
    pub const fn badge(self) -> &'static str {
        match self {
            Self::Off => "AP OFF",
            Self::EnrouteLnav => "AP LNAV",
            Self::Approach => "AP APP",
            Self::Flare => "AP FLARE",
            Self::Rollout => "AP ROLL",
        }
    }

    /// Lateral FMA mode string for PFD/HUD scoreboard integration.
    #[must_use]
    pub const fn lateral_fma(self) -> Option<&'static str> {
        match self {
            Self::Off => None,
            Self::EnrouteLnav => Some("LNAV"),
            Self::Approach | Self::Flare | Self::Rollout => Some("LOC"),
        }
    }

    /// Vertical FMA mode string for PFD/HUD scoreboard integration.
    #[must_use]
    pub const fn vertical_fma(self) -> Option<&'static str> {
        match self {
            Self::Off => None,
            Self::EnrouteLnav => Some("ALT"),
            Self::Approach => Some("GS"),
            Self::Flare => Some("FLARE"),
            Self::Rollout => Some("ROLLOUT"),
        }
    }
}

/// Flight-control and aircraft-systems command produced by one [`EfbAutopilot::step`] tick.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AutopilotOutput {
    /// Primary stick/rudder/throttle flight controls.
    pub controls: ControlInputs,
    /// Target normalized flap setting (`0.0..=1.0`).
    pub flaps_norm: f64,
    /// Target symmetric wheel-brake pressure (`0.0..=1.0`).
    pub wheel_brake_norm: f64,
    /// Active phase after mode transitions.
    pub phase: EfbAutopilotPhase,
}

#[derive(Debug, Clone, Copy, Default)]
struct RatePi {
    integral: f64,
}

impl RatePi {
    fn step(&mut self, error: f64, dt_s: f64) -> f64 {
        self.integral = (self.integral + RATE_KI * error * dt_s).clamp(-0.85, 0.85);
        (RATE_KP * error + self.integral).clamp(-1.0, 1.0)
    }

    fn reset(&mut self) {
        self.integral = 0.0;
    }
}

/// Closed-loop EFB route & autoland flight director / autopilot.
#[derive(Debug, Clone, Default)]
pub struct EfbAutopilot {
    phase: EfbAutopilotPhase,
    roll_pi: RatePi,
    pitch_pi: RatePi,
    yaw_pi: RatePi,
    speed_integral: f64,
    rollout_elapsed_s: f64,
    hold_elapsed_s: f64,
}

#[derive(Debug, Clone, Copy)]
pub(super) struct AeroState {
    pub(super) geo: Geodetic,
    pub(super) speed_m_s: f64,
    pub(super) track_rad: f64,
    pub(super) heading_rad: f64,
    pub(super) flight_path_rad: f64,
    pub(super) bank_rad: f64,
    pub(super) pitch_rad: f64,
    pub(super) body_rate: DVec3,
}

#[derive(Debug, Clone, Copy)]
pub(super) struct GuidanceTarget {
    pub(super) line_origin_ecef: DVec3,
    pub(super) line_frame: LocalFrame,
    pub(super) line_bearing_rad: f64,
    pub(super) lookahead_m: f64,
    pub(super) target_ellip_m: f64,
    pub(super) height_gain: f64,
    pub(super) target_speed_m_s: f64,
    pub(super) flaps_norm: f64,
}

impl EfbAutopilot {
    /// Current autopilot phase (`Off` when disengaged).
    #[must_use]
    pub const fn phase(&self) -> EfbAutopilotPhase {
        self.phase
    }

    /// Whether the autopilot is actively driving controls (`phase != Off`).
    #[must_use]
    pub const fn is_engaged(&self) -> bool {
        !matches!(self.phase, EfbAutopilotPhase::Off)
    }

    /// Engages LNAV/APP guidance, resetting inner-loop integrators cleanly.
    pub fn engage(&mut self) {
        if !self.is_engaged() {
            self.reset_integrators();
            self.phase = EfbAutopilotPhase::EnrouteLnav;
        }
    }

    /// Disengages the autopilot to manual control.
    pub fn disengage(&mut self) {
        self.phase = EfbAutopilotPhase::Off;
        self.reset_integrators();
    }

    /// Toggles engagement state.
    pub fn toggle(&mut self) {
        if self.is_engaged() {
            self.disengage();
        } else {
            self.engage();
        }
    }

    fn reset_integrators(&mut self) {
        self.roll_pi.reset();
        self.pitch_pi.reset();
        self.yaw_pi.reset();
        self.speed_integral = 0.0;
        self.rollout_elapsed_s = 0.0;
        self.hold_elapsed_s = 0.0;
    }

    /// Steps guidance by `dt_s` seconds, returning `None` when disengaged.
    #[allow(clippy::too_many_arguments)]
    pub fn step(
        &mut self,
        state: &VehicleState,
        articulation: &AirframeArticulation,
        tokens: &[RouteToken],
        progress: RouteProgress,
        landing_rwy: Option<&LandingRunwayGeometry>,
        geoid_undulation_m: f64,
        dt_s: f64,
    ) -> Option<AutopilotOutput> {
        if !self.is_engaged() {
            return None;
        }
        let step_dt = dt_s.clamp(1.0 / 240.0, 0.1);
        let aero = enroute::observe_aero(state);
        if let Some(rwy) = landing_rwy
            && let Some(out) = self.try_step_runway(&aero, state, articulation, rwy, step_dt)
        {
            return Some(out);
        }
        self.phase = EfbAutopilotPhase::EnrouteLnav;
        let target = enroute::build_enroute_target(
            &aero,
            tokens,
            progress,
            geoid_undulation_m,
            &mut self.hold_elapsed_s,
            step_dt,
        )?;
        let controls = self.track_target(&aero, state.position_ecef, &target, step_dt);
        Some(AutopilotOutput {
            controls,
            flaps_norm: target.flaps_norm,
            wheel_brake_norm: 0.0,
            phase: self.phase,
        })
    }

    fn try_step_runway(
        &mut self,
        aero: &AeroState,
        state: &VehicleState,
        art: &AirframeArticulation,
        rwy: &LandingRunwayGeometry,
        dt_s: f64,
    ) -> Option<AutopilotOutput> {
        let thr_ecef = geodetic_to_ecef(&rwy.threshold);
        let thr_frame = LocalFrame::at(&rwy.threshold);
        let enu = thr_frame.ecef_to_enu_vector(state.position_ecef - thr_ecef);
        let (sin_b, cos_b) = rwy.true_bearing_rad.sin_cos();
        let along_m = enu.y * cos_b + enu.x * sin_b;
        let right_m = enu.x * cos_b - enu.y * sin_b;
        let in_capture_box = (-8_500.0..=rwy.landing_length_m + 600.0).contains(&along_m)
            && right_m.abs() <= 1_800.0;
        if !in_capture_box
            && !matches!(
                self.phase,
                EfbAutopilotPhase::Flare | EfbAutopilotPhase::Rollout
            )
        {
            return None;
        }
        let frac = (along_m / rwy.landing_length_m.max(1.0)).clamp(0.0, 1.0);
        let deck_ellip_m =
            rwy.threshold.height_m + (rwy.far_end.height_m - rwy.threshold.height_m) * frac;
        let deck_agl_m = aero.geo.height_m - deck_ellip_m;
        let wow = art.gear_wow.into_iter().any(|w| w);

        if matches!(self.phase, EfbAutopilotPhase::Rollout)
            || (along_m >= -25.0 && (wow || deck_agl_m <= 1.52))
        {
            self.phase = EfbAutopilotPhase::Rollout;
            return Some(self.step_rollout(aero, rwy.true_bearing_rad, right_m, dt_s));
        }
        if matches!(self.phase, EfbAutopilotPhase::Flare) || (along_m >= -75.0 && deck_agl_m <= 6.2)
        {
            self.phase = EfbAutopilotPhase::Flare;
            return Some(self.step_flare(aero, rwy.true_bearing_rad, right_m, deck_agl_m, dt_s));
        }
        self.phase = EfbAutopilotPhase::Approach;
        Some(self.step_approach(
            aero,
            state.position_ecef,
            rwy,
            (thr_ecef, thr_frame, along_m, deck_ellip_m),
            dt_s,
        ))
    }

    fn step_approach(
        &mut self,
        aero: &AeroState,
        pos_ecef: DVec3,
        rwy: &LandingRunwayGeometry,
        (thr_ecef, thr_frame, along_m, deck_ellip_m): (DVec3, LocalFrame, f64, f64),
        dt_s: f64,
    ) -> AutopilotOutput {
        let ultra_short = rwy.landing_length_m < 360.0;
        let short_strip = rwy.landing_length_m < 850.0;
        let aimpoint_m = if ultra_short {
            22.0
        } else if short_strip {
            42.0
        } else {
            135.0
        };
        let dist_to_aim = (aimpoint_m - along_m).max(0.0);
        let glide_above_m = dist_to_aim * rwy.glidepath_rad.tan() + 1.35;
        let dist_out = (-along_m).max(0.0);
        let flaps_norm = if dist_out < 950.0 && aero.speed_m_s < 43.5 {
            1.0
        } else if dist_out < 1_850.0 && aero.speed_m_s < 47.0 {
            0.67
        } else {
            0.33
        };
        let base_app_speed = if ultra_short {
            28.2
        } else if short_strip {
            29.8
        } else {
            33.0
        };
        let target_speed_m_s = if dist_out > 1_800.0 {
            base_app_speed + 5.0
        } else {
            base_app_speed
        };
        let target = GuidanceTarget {
            line_origin_ecef: thr_ecef,
            line_frame: thr_frame,
            line_bearing_rad: rwy.true_bearing_rad,
            lookahead_m: dist_out.clamp(135.0, 260.0),
            target_ellip_m: deck_ellip_m + glide_above_m,
            height_gain: 0.30,
            target_speed_m_s,
            flaps_norm,
        };
        let controls = self.track_target(aero, pos_ecef, &target, dt_s);
        AutopilotOutput {
            controls,
            flaps_norm,
            wheel_brake_norm: 0.0,
            phase: EfbAutopilotPhase::Approach,
        }
    }

    fn step_flare(
        &mut self,
        aero: &AeroState,
        rwy_bearing_rad: f64,
        right_m: f64,
        deck_agl_m: f64,
        dt_s: f64,
    ) -> AutopilotOutput {
        let speed = aero.speed_m_s.max(16.0);
        let bank_cmd = (-0.018 * right_m).clamp(-0.10, 0.10);
        let target_sink_m_s = (-0.20 * (deck_agl_m - 1.25).max(0.0) - 0.35).clamp(-1.6, -0.25);
        let path_cmd = (target_sink_m_s / speed).clamp(-0.09, 0.04).asin();
        let bank_rate = BANK_GAIN * angle_diff(bank_cmd, aero.bank_rad);
        let pitch_rate = 1.35 * (path_cmd - aero.flight_path_rad);
        let hdg_err = angle_diff(rwy_bearing_rad, aero.heading_rad);
        let yaw_rate_cmd = (0.55 * hdg_err).clamp(-0.25, 0.25);
        let controls = ControlInputs {
            roll: self.roll_pi.step(bank_rate - aero.body_rate.x, dt_s),
            pitch: self.pitch_pi.step(pitch_rate - aero.body_rate.y, dt_s),
            yaw: self.yaw_pi.step(yaw_rate_cmd - aero.body_rate.z, dt_s),
            throttle: 0.0,
        };
        AutopilotOutput {
            controls,
            flaps_norm: 1.0,
            wheel_brake_norm: 0.0,
            phase: EfbAutopilotPhase::Flare,
        }
    }

    fn step_rollout(
        &mut self,
        aero: &AeroState,
        rwy_bearing_rad: f64,
        right_m: f64,
        dt_s: f64,
    ) -> AutopilotOutput {
        self.rollout_elapsed_s += dt_s;
        let hdg_err = angle_diff(rwy_bearing_rad, aero.heading_rad);
        let steer_track = (-0.032 * right_m).clamp(-0.28, 0.28);
        let yaw_cmd = (1.6 * (hdg_err + steer_track) - 0.35 * aero.body_rate.z).clamp(-0.85, 0.85);
        let roll_level = (-1.2 * aero.bank_rad - 0.25 * aero.body_rate.x).clamp(-0.6, 0.6);
        let brake = if aero.speed_m_s <= 0.20 {
            1.0
        } else {
            (0.28 + 0.55 * (self.rollout_elapsed_s / 1.4).clamp(0.0, 1.0)).clamp(0.0, 0.88)
        };
        AutopilotOutput {
            controls: ControlInputs {
                roll: roll_level,
                pitch: 0.08,
                yaw: yaw_cmd,
                throttle: 0.0,
            },
            flaps_norm: 0.0,
            wheel_brake_norm: brake,
            phase: EfbAutopilotPhase::Rollout,
        }
    }

    fn track_target(
        &mut self,
        aero: &AeroState,
        pos_ecef: DVec3,
        target: &GuidanceTarget,
        dt_s: f64,
    ) -> ControlInputs {
        let speed = aero.speed_m_s.max(18.0);
        let enu = target
            .line_frame
            .ecef_to_enu_vector(pos_ecef - target.line_origin_ecef);
        let ne = DVec2::new(enu.y, enu.x);
        let (sin_l, cos_l) = target.line_bearing_rad.sin_cos();
        let cross_right_m = ne.y * cos_l - ne.x * sin_l;
        let intercept = (cross_right_m / target.lookahead_m)
            .atan()
            .clamp(-0.80, 0.80);
        let desired_track = target.line_bearing_rad - intercept;
        let turn_cmd = 0.42 * angle_diff(desired_track, aero.track_rad);
        let bank_cmd = (turn_cmd * speed / GRAVITY_M_S2)
            .atan()
            .clamp(-MAX_BANK_RAD, MAX_BANK_RAD);

        let climb_cmd =
            (target.height_gain * (target.target_ellip_m - aero.geo.height_m)).clamp(-4.2, 3.4);
        let path_cmd = (climb_cmd / speed).clamp(-0.13, 0.13).asin();

        let (sin_bank, cos_bank) = aero.bank_rad.sin_cos();
        let (sin_pitch, cos_pitch) = aero.pitch_rad.sin_cos();
        let bank_rate = BANK_GAIN * angle_diff(bank_cmd, aero.bank_rad);
        let pitch_rate = FLIGHT_PATH_GAIN * (path_cmd - aero.flight_path_rad);
        let turn_rate = (GRAVITY_M_S2 * aero.bank_rad.tan() / speed)
            .clamp(-MAX_TURN_RATE_RAD_S, MAX_TURN_RATE_RAD_S);
        let p = bank_rate - turn_rate * sin_pitch;
        let q = pitch_rate * cos_bank + turn_rate * cos_pitch * sin_bank;
        let r = turn_rate * cos_pitch * cos_bank - pitch_rate * sin_bank;

        let speed_err = target.target_speed_m_s - aero.speed_m_s;
        self.speed_integral =
            (self.speed_integral + SPEED_KI * speed_err * dt_s).clamp(-0.62, 0.35);
        let throttle = (0.58
            + SPEED_KP * speed_err
            + self.speed_integral
            + 0.055 * climb_cmd
            + 0.07 * target.flaps_norm)
            .clamp(0.02, 1.0);

        ControlInputs {
            roll: self.roll_pi.step(p - aero.body_rate.x, dt_s),
            pitch: self.pitch_pi.step(q - aero.body_rate.y, dt_s),
            yaw: self.yaw_pi.step(r - aero.body_rate.z, dt_s),
            throttle,
        }
    }
}

fn angle_diff(target_rad: f64, current_rad: f64) -> f64 {
    (target_rad - current_rad + PI).rem_euclid(2.0 * PI) - PI
}
