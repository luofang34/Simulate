//! Closed-loop 5-leg C172 airport traffic-pattern state machine and autopilot.

use std::f64::consts::{FRAC_PI_2, PI};

use glam::{DVec2, DVec3};
use sim_core::{ControlInputs, VehicleState};
use sim_geo::{Geodetic, LocalFrame, ecef_to_geodetic};

use super::{PatternLeg, TrafficDirection};
use crate::corridor::angle_difference;

const GRAVITY_M_S2: f64 = 9.806_65;
/// Standard C172 traffic pattern altitude: `1,000 ft` (`304.8 m`) AGL.
pub(super) const PATTERN_ALT_AGL_M: f64 = 304.8;
/// Lateral spacing of the downwind leg from the runway centerline (`~0.73 NM`).
pub(super) const PATTERN_WIDTH_M: f64 = 1_350.0;
/// Distance behind the landing threshold where the base leg tracks (`~0.84 NM`).
pub(super) const BASE_ALONG_M: f64 = -1_550.0;
const GLIDESLOPE_RAD: f64 = 3.0 * PI / 180.0;
const THRESHOLD_CROSSING_AGL_M: f64 = 15.0;

const RATE_KP: f64 = 0.55;
const RATE_KI: f64 = 1.6;
const BANK_GAIN: f64 = 1.6;
const FLIGHT_PATH_GAIN: f64 = 1.05;
const HEIGHT_GAIN: f64 = 0.22;
const MAX_CLIMB_M_S: f64 = 3.4;
const MAX_DESCENT_M_S: f64 = 4.0;
const PATH_GAIN: f64 = 0.38;
const MAX_BANK_RAD: f64 = 25.0 * PI / 180.0;
const MAX_INTERCEPT_RAD: f64 = 0.85;
const MAX_TURN_RATE_RAD_S: f64 = 0.45;
const BASE_THROTTLE: f64 = 0.65;
const SPEED_KP: f64 = 0.14;
const SPEED_KI: f64 = 0.04;

/// Runway-relative aircraft coordinates (`m`) and horizon state.
#[derive(Debug, Clone, Copy)]
pub(super) struct RunwayReading {
    /// Distance past the landing threshold along the runway heading (`< 0` on final approach).
    pub(super) along_m: f64,
    /// Signed lateral distance toward the active pattern side (`> 0` on downwind).
    pub(super) pattern_offset_m: f64,
    /// Signed cross-track distance right of the runway centerline (`m`).
    pub(super) right_of_centerline_m: f64,
    /// Height above airport field elevation (`m` AGL).
    pub(super) agl_m: f64,
    /// Ground/inertial speed (`m/s`).
    pub(super) speed_m_s: f64,
    /// Ground track clockwise from true north (`rad`).
    pub(super) track_rad: f64,
    /// Flight-path angle above horizon (`rad`).
    pub(super) flight_path_rad: f64,
    /// Roll bank angle (`rad`, right wing down positive).
    pub(super) bank_rad: f64,
    /// Pitch angle (`rad`, nose up positive).
    pub(super) pitch_rad: f64,
    /// Body angular rates `(p, q, r)` (`rad/s`).
    pub(super) body_rate: DVec3,
    /// North/East offset (`m`) from threshold anchor.
    pub(super) north_east_m: DVec2,
}

impl RunwayReading {
    pub(super) fn observe(
        state: &VehicleState,
        threshold_ecef: DVec3,
        threshold_frame: &LocalFrame,
        field_elevation_m: f64,
        runway_bearing_rad: f64,
        direction: TrafficDirection,
    ) -> Self {
        let geo = ecef_to_geodetic(state.position_ecef);
        let local = LocalFrame::at(&geo);
        let [north, east, up] = [local.north(), local.east(), local.up()];
        let vel = state.velocity_ecef;
        let speed_m_s = vel.length();
        let enu = threshold_frame.ecef_to_enu_vector(state.position_ecef - threshold_ecef);
        let north_east_m = DVec2::new(enu.y, enu.x);
        let (sin_b, cos_b) = runway_bearing_rad.sin_cos();
        let along_m = north_east_m.x * cos_b + north_east_m.y * sin_b;
        let right_m = north_east_m.y * cos_b - north_east_m.x * sin_b;
        let pattern_sign = direction.right_sign();
        let right_axis = state.attitude * DVec3::Y;
        Self {
            along_m,
            pattern_offset_m: right_m * pattern_sign,
            right_of_centerline_m: right_m,
            agl_m: geo.height_m - field_elevation_m,
            speed_m_s,
            track_rad: vel.dot(east).atan2(vel.dot(north)),
            flight_path_rad: (vel.dot(up) / speed_m_s.max(1.0)).clamp(-1.0, 1.0).asin(),
            bank_rad: (-right_axis.dot(up)).atan2(-state.down_ecef().dot(up)),
            pitch_rad: state.forward_ecef().dot(up).clamp(-1.0, 1.0).asin(),
            body_rate: state.angular_rate_body,
            north_east_m,
        }
    }
}

/// Commanded line, speed, altitude, and flap setting for the current pattern instant.
#[derive(Debug, Clone, Copy)]
pub(super) struct LegCommand {
    pub(super) line_through_ne_m: DVec2,
    pub(super) line_bearing_rad: f64,
    pub(super) lookahead_m: f64,
    pub(super) target_agl_m: f64,
    pub(super) target_speed_m_s: f64,
    pub(super) flaps_norm: f64,
}

/// Advances `leg` when turn-lead capture conditions are met; returns `true` when a full
/// circuit completes (touch-and-go / low-approach climbout back to `Upwind`).
pub(super) fn advance_leg(
    leg: &mut PatternLeg,
    reading: &RunwayReading,
    crosswind_along_m: f64,
) -> bool {
    match *leg {
        PatternLeg::Upwind => {
            if reading.along_m >= crosswind_along_m - 320.0 && reading.agl_m >= 175.0 {
                *leg = PatternLeg::Crosswind;
            }
            false
        }
        PatternLeg::Crosswind => {
            if reading.pattern_offset_m >= PATTERN_WIDTH_M - 380.0 {
                *leg = PatternLeg::Downwind;
            }
            false
        }
        PatternLeg::Downwind => {
            if reading.along_m <= BASE_ALONG_M + 390.0 {
                *leg = PatternLeg::Base;
            }
            false
        }
        PatternLeg::Base => {
            if reading.pattern_offset_m <= 360.0 {
                *leg = PatternLeg::Final;
            }
            false
        }
        PatternLeg::Final => {
            if reading.along_m >= 240.0 {
                *leg = PatternLeg::Upwind;
                return true;
            }
            false
        }
    }
}

pub(super) fn command_for_leg(
    leg: PatternLeg,
    reading: &RunwayReading,
    runway_bearing_rad: f64,
    direction: TrafficDirection,
    crosswind_along_m: f64,
) -> LegCommand {
    let sign = direction.right_sign();
    let (sin_b, cos_b) = runway_bearing_rad.sin_cos();
    let rwy_dir = DVec2::new(cos_b, sin_b);
    let pat_dir = DVec2::new(-sin_b, cos_b) * sign;

    match leg {
        PatternLeg::Upwind => LegCommand {
            line_through_ne_m: DVec2::ZERO,
            line_bearing_rad: runway_bearing_rad,
            lookahead_m: 260.0,
            target_agl_m: PATTERN_ALT_AGL_M,
            target_speed_m_s: 39.5,
            flaps_norm: 0.0,
        },
        PatternLeg::Crosswind => LegCommand {
            line_through_ne_m: rwy_dir * crosswind_along_m,
            line_bearing_rad: runway_bearing_rad + sign * FRAC_PI_2,
            lookahead_m: 280.0,
            target_agl_m: PATTERN_ALT_AGL_M,
            target_speed_m_s: 43.5,
            flaps_norm: 0.0,
        },
        PatternLeg::Downwind => {
            let abeam_past_threshold = reading.along_m <= 0.0;
            let (target_agl_m, target_speed_m_s, flaps_norm) = if abeam_past_threshold {
                let frac = ((-reading.along_m) / (-BASE_ALONG_M)).clamp(0.0, 1.0);
                (PATTERN_ALT_AGL_M - 85.0 * frac, 41.0, 0.33)
            } else {
                (PATTERN_ALT_AGL_M, 46.5, 0.0)
            };
            LegCommand {
                line_through_ne_m: pat_dir * PATTERN_WIDTH_M,
                line_bearing_rad: runway_bearing_rad + PI,
                lookahead_m: 290.0,
                target_agl_m,
                target_speed_m_s,
                flaps_norm,
            }
        }
        PatternLeg::Base => {
            let frac = (1.0 - (reading.pattern_offset_m / PATTERN_WIDTH_M)).clamp(0.0, 1.0);
            let glide_intercept_agl =
                (-BASE_ALONG_M) * GLIDESLOPE_RAD.tan() + THRESHOLD_CROSSING_AGL_M;
            LegCommand {
                line_through_ne_m: rwy_dir * BASE_ALONG_M,
                line_bearing_rad: runway_bearing_rad - sign * FRAC_PI_2,
                lookahead_m: 260.0,
                target_agl_m: 220.0 * (1.0 - frac) + glide_intercept_agl * frac,
                target_speed_m_s: 37.0,
                flaps_norm: 0.67,
            }
        }
        PatternLeg::Final => {
            let dist_to_thresh = (-reading.along_m).max(0.0);
            let glide_agl = if reading.along_m < 60.0 {
                (dist_to_thresh * GLIDESLOPE_RAD.tan() + THRESHOLD_CROSSING_AGL_M)
                    .clamp(12.0, 165.0)
            } else {
                // Smooth low-approach / touch-and-go climbout transition.
                65.0
            };
            let go_around = reading.along_m >= 60.0;
            LegCommand {
                line_through_ne_m: DVec2::ZERO,
                line_bearing_rad: runway_bearing_rad,
                lookahead_m: 195.0,
                target_agl_m: glide_agl,
                target_speed_m_s: if go_around { 38.5 } else { 34.0 },
                flaps_norm: if go_around { 0.33 } else { 1.0 },
            }
        }
    }
}

#[derive(Debug, Clone, Copy, Default)]
struct RatePi {
    integral: f64,
}

impl RatePi {
    fn step(&mut self, error: f64, dt_s: f64) -> f64 {
        self.integral = (self.integral + RATE_KI * error * dt_s).clamp(-0.9, 0.9);
        (RATE_KP * error + self.integral).clamp(-1.0, 1.0)
    }
}

/// Inner/outer-loop autopilot tuned for `c172_class_reference` traffic pattern speeds and flaps.
#[derive(Debug, Clone, Copy, Default)]
pub(super) struct PatternAutopilot {
    roll: RatePi,
    pitch: RatePi,
    yaw: RatePi,
    speed_integral: f64,
}

impl PatternAutopilot {
    pub(super) fn step(
        &mut self,
        cmd: &LegCommand,
        reading: &RunwayReading,
        dt_s: f64,
    ) -> ControlInputs {
        let speed = reading.speed_m_s.max(15.0);
        let (sin_l, cos_l) = cmd.line_bearing_rad.sin_cos();
        let offset = reading.north_east_m - cmd.line_through_ne_m;
        let cross_right_m = offset.y * cos_l - offset.x * sin_l;
        let intercept = (cross_right_m / cmd.lookahead_m)
            .atan()
            .clamp(-MAX_INTERCEPT_RAD, MAX_INTERCEPT_RAD);
        let desired_track = cmd.line_bearing_rad - intercept;
        let turn_cmd = PATH_GAIN * angle_difference(desired_track, reading.track_rad);
        let bank_cmd = (turn_cmd * speed / GRAVITY_M_S2)
            .atan()
            .clamp(-MAX_BANK_RAD, MAX_BANK_RAD);

        let speed_climb_factor = ((reading.speed_m_s - 30.0) / 6.5).clamp(0.0, 1.0);
        let bank_climb_factor = reading.bank_rad.cos().powi(2).clamp(0.65, 1.0);
        let max_climb = MAX_CLIMB_M_S * bank_climb_factor * speed_climb_factor;
        let climb_cmd =
            (HEIGHT_GAIN * (cmd.target_agl_m - reading.agl_m)).clamp(-MAX_DESCENT_M_S, max_climb);
        let path_cmd = (climb_cmd / speed).clamp(-0.12, 0.12).asin();

        let (sin_bank, cos_bank) = reading.bank_rad.sin_cos();
        let (sin_pitch, cos_pitch) = reading.pitch_rad.sin_cos();
        let bank_rate = BANK_GAIN * angle_difference(bank_cmd, reading.bank_rad);
        let pitch_rate = FLIGHT_PATH_GAIN * (path_cmd - reading.flight_path_rad);
        let turn_rate = (GRAVITY_M_S2 * reading.bank_rad.tan() / speed)
            .clamp(-MAX_TURN_RATE_RAD_S, MAX_TURN_RATE_RAD_S);

        let p = bank_rate - turn_rate * sin_pitch;
        let q = pitch_rate * cos_bank + turn_rate * cos_pitch * sin_bank;
        let r = turn_rate * cos_pitch * cos_bank - pitch_rate * sin_bank;

        let speed_err = cmd.target_speed_m_s - reading.speed_m_s;
        self.speed_integral =
            (self.speed_integral + SPEED_KI * speed_err * dt_s).clamp(-0.60, 0.35);
        let throttle = (BASE_THROTTLE
            + SPEED_KP * speed_err
            + self.speed_integral
            + 0.055 * climb_cmd
            + 0.08 * cmd.flaps_norm)
            .clamp(0.05, 1.0);

        ControlInputs {
            roll: self.roll.step(p - reading.body_rate.x, dt_s),
            pitch: self.pitch.step(q - reading.body_rate.y, dt_s),
            yaw: self.yaw.step(r - reading.body_rate.z, dt_s),
            throttle,
        }
    }
}

/// Computes the airborne initial position on `Upwind` (`180 m` past threshold, `38 m` AGL).
pub(super) fn initial_upwind_geodetic(
    threshold_lat_deg: f64,
    threshold_lon_deg: f64,
    field_elevation_m: f64,
    runway_bearing_rad: f64,
) -> Geodetic {
    let thresh = Geodetic::from_degrees(threshold_lat_deg, threshold_lon_deg, field_elevation_m);
    let frame = LocalFrame::at(&thresh);
    let (sin_b, cos_b) = runway_bearing_rad.sin_cos();
    let fwd = frame.north() * cos_b + frame.east() * sin_b;
    let mut start = ecef_to_geodetic(sim_geo::geodetic_to_ecef(&thresh) + fwd * 180.0);
    start.height_m = field_elevation_m + 38.0;
    start
}
