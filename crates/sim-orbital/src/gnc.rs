//! Closed-loop orbital Guidance, Navigation, and Control (`GNC`) primitives:
//! `GravityTurnAscent`, universal `Lambert/FiniteBurn` targeting, `EntryGCorridor`,
//! and ZEM/ZEV `PoweredDescentAutoland`.

pub mod lambert;

use glam::DVec3;
use serde::{Deserialize, Serialize};

use crate::scalar_math::{cos, sin, sqrt};

pub use lambert::{FiniteBurnPlan, LambertError, LambertSolution, solve_lambert_universal};

/// Standard Earth gravity $g_0 = 9.80665\text{ m/s}^2$.
pub const STANDARD_GRAVITY_G0_M_S2: f64 = 9.806_65;

/// `Raketa`'s peak proper-acceleration limit `4.0G` (`39.2266 m/s^2`).
pub const FOUR_G_PROPER_ACCEL_M_S2: f64 = 4.0 * STANDARD_GRAVITY_G0_M_S2;

/// Configuration for closed-loop gravity-turn ascent to circular parking orbit.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct AscentGuidanceConfig {
    /// Mean equatorial/spherical radius of the launch body (`m`).
    pub body_radius_m: f64,
    /// Gravitational parameter $\mu$ of the launch body (`m^3/s^2`).
    pub mu_m3_s2: f64,
    /// Altitude (`m` AGL) above which vertical rise transitions into gravity-turn pitchover.
    pub pitchover_alt_m: f64,
    /// Target circular parking-orbit altitude (`m` above `body_radius_m`).
    pub target_orbit_alt_m: f64,
    /// Maximum commanded proper acceleration (`m/s^2`, default `4.0G = 39.2266`).
    pub max_proper_accel_m_s2: f64,
}

impl AscentGuidanceConfig {
    /// Standard Earth launch-to-LEO (`185 km`) ascent configuration within `4.0G`.
    #[must_use]
    pub const fn earth_leo_185km() -> Self {
        Self {
            body_radius_m: 6_378_137.0,
            mu_m3_s2: 3.986_004_418e14,
            pitchover_alt_m: 350.0,
            target_orbit_alt_m: 185_000.0,
            max_proper_accel_m_s2: FOUR_G_PROPER_ACCEL_M_S2,
        }
    }
}

/// Commanded thrust vector and phase flag from [`evaluate_gravity_turn_ascent`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AscentGuidanceOutput {
    /// Desired unit thrust axis in primary-centered `ICRF`.
    pub thrust_dir_icrf: DVec3,
    /// Desired specific thrust magnitude (`m/s^2`, `0..=max_proper_accel_m_s2`).
    pub commanded_accel_m_s2: f64,
    /// `true` once the circular parking orbit gate is satisfied (`MECO`).
    pub orbit_achieved: bool,
}

/// Evaluates closed-loop gravity-turn ascent and circularization guidance in primary `ICRF`.
#[must_use]
pub fn evaluate_gravity_turn_ascent(
    pos_rel_icrf_m: DVec3,
    vel_rel_icrf_m_s: DVec3,
    east_tangent_icrf: DVec3,
    config: &AscentGuidanceConfig,
) -> AscentGuidanceOutput {
    let r = sqrt(pos_rel_icrf_m.length_squared()).max(1.0);
    let u_up = pos_rel_icrf_m / r;
    let alt_m = r - config.body_radius_m;
    let r_target = config.body_radius_m + config.target_orbit_alt_m;
    let v_circ = sqrt(config.mu_m3_s2 / r_target);

    let vr = vel_rel_icrf_m_s.dot(u_up);
    let v_horiz_vec = vel_rel_icrf_m_s - vr * u_up;
    let vh = sqrt(v_horiz_vec.length_squared());
    let u_horiz = if vh > 5.0 {
        v_horiz_vec / vh
    } else {
        let e_proj = east_tangent_icrf - east_tangent_icrf.dot(u_up) * u_up;
        let e_norm = sqrt(e_proj.length_squared());
        if e_norm > 1.0e-6 {
            e_proj / e_norm
        } else {
            DVec3::Y
        }
    };

    let alt_err = config.target_orbit_alt_m - alt_m;
    let vh_err = v_circ - vh;
    if alt_err.abs() <= 1_000.0 && vr.abs() <= 2.0 && vh_err.abs() <= 1.5 {
        return AscentGuidanceOutput {
            thrust_dir_icrf: u_horiz,
            commanded_accel_m_s2: 0.0,
            orbit_achieved: true,
        };
    }

    if alt_m < config.pitchover_alt_m {
        return AscentGuidanceOutput {
            thrust_dir_icrf: u_up,
            commanded_accel_m_s2: config.max_proper_accel_m_s2,
            orbit_achieved: false,
        };
    }

    let g_local = config.mu_m3_s2 / (r * r);
    let radial_eff_g = (g_local - (vh * vh) / r).max(0.0);
    let a_rad_des = (radial_eff_g + 0.0018 * alt_err - 0.14 * vr).clamp(
        -0.65 * config.max_proper_accel_m_s2,
        0.88 * config.max_proper_accel_m_s2,
    );
    let a_tan_des = (0.25 * vh_err).clamp(0.0, config.max_proper_accel_m_s2);
    let a_vec = a_rad_des * u_up + a_tan_des * u_horiz;
    let a_mag = sqrt(a_vec.length_squared());
    if a_mag <= 1.0e-6 {
        return AscentGuidanceOutput {
            thrust_dir_icrf: u_horiz,
            commanded_accel_m_s2: 0.0,
            orbit_achieved: false,
        };
    }
    let frac = (alt_m - config.pitchover_alt_m)
        / (config.target_orbit_alt_m - config.pitchover_alt_m).max(1.0);
    let min_pitch_rad = (1.0 - frac.clamp(0.0, 1.0)) * 0.25;
    let blended = if frac < 0.15 {
        let p = 0.5 * core::f64::consts::PI
            - (frac / 0.15) * (0.5 * core::f64::consts::PI - min_pitch_rad);
        sin(p) * u_up + cos(p) * u_horiz
    } else {
        a_vec / a_mag
    };
    AscentGuidanceOutput {
        thrust_dir_icrf: blended / sqrt(blended.length_squared()).max(1.0e-9),
        commanded_accel_m_s2: a_mag.min(config.max_proper_accel_m_s2),
        orbit_achieved: false,
    }
}

/// Configuration for atmospheric entry / aerocapture g-corridor guidance (`EntryGCorridor`).
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct EntryCorridorConfig {
    /// Target handoff altitude (`m` AGL) where powered descent autoland takes over.
    pub handoff_altitude_m: f64,
    /// Target maximum speed (`m/s`) at powered-descent handoff.
    pub handoff_speed_m_s: f64,
    /// Maximum total proper acceleration (`m/s^2`, default `4.0G = 39.2266`).
    pub max_proper_accel_m_s2: f64,
}

impl Default for EntryCorridorConfig {
    fn default() -> Self {
        Self {
            handoff_altitude_m: 8_000.0,
            handoff_speed_m_s: 320.0,
            max_proper_accel_m_s2: FOUR_G_PROPER_ACCEL_M_S2,
        }
    }
}

/// Output of [`evaluate_entry_corridor_guidance`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct EntryCorridorOutput {
    /// Desired body `+Y` thrust/retro-attitude unit vector in `ICRF`.
    pub attitude_dir_icrf: DVec3,
    /// Supplemental retro-thrust acceleration (`m/s^2`) budgeted inside the remaining `4.0G` corridor.
    pub retro_thrust_accel_m_s2: f64,
    /// Commanded bank angle (`rad`) for crossrange/vertical lift modulation.
    pub bank_angle_rad: f64,
    /// `true` once altitude and speed enter the `PoweredDescentAutoland` handoff gate.
    pub ready_for_autoland: bool,
}

/// Evaluates `EntryGCorridor` guidance during planetary atmospheric entry or aerocapture,
/// budgeting retro-propulsion inside the remaining `4.0G` margin above passive aerodynamic drag.
#[must_use]
pub fn evaluate_entry_corridor_guidance(
    pos_rel_icrf_m: DVec3,
    vel_rel_body_fixed_icrf_m_s: DVec3,
    aero_accel_icrf_m_s2: DVec3,
    altitude_agl_m: f64,
    config: &EntryCorridorConfig,
) -> EntryCorridorOutput {
    let r = sqrt(pos_rel_icrf_m.length_squared()).max(1.0);
    let u_up = pos_rel_icrf_m / r;
    let speed = sqrt(vel_rel_body_fixed_icrf_m_s.length_squared());
    let u_retro = if speed > 1.0e-3 {
        -vel_rel_body_fixed_icrf_m_s / speed
    } else {
        u_up
    };

    if altitude_agl_m <= config.handoff_altitude_m && speed <= config.handoff_speed_m_s {
        return EntryCorridorOutput {
            attitude_dir_icrf: u_up,
            retro_thrust_accel_m_s2: 0.0,
            bank_angle_rad: 0.0,
            ready_for_autoland: true,
        };
    }

    let aero_mag = sqrt(aero_accel_icrf_m_s2.length_squared());
    let g_headroom = (config.max_proper_accel_m_s2 - aero_mag).max(0.0);
    let speed_excess = (speed - config.handoff_speed_m_s).max(0.0);
    let dist_to_handoff = (altitude_agl_m - config.handoff_altitude_m).max(500.0);
    let req_decel = (speed_excess * speed_excess) / (2.0 * dist_to_handoff);
    let retro_cmd = (req_decel - aero_mag).clamp(0.0, g_headroom);

    let vr = vel_rel_body_fixed_icrf_m_s.dot(u_up);
    let bank_rad = if vr < -120.0 { 0.0 } else { 0.65 };

    EntryCorridorOutput {
        attitude_dir_icrf: u_retro,
        retro_thrust_accel_m_s2: retro_cmd,
        bank_angle_rad: bank_rad,
        ready_for_autoland: false,
    }
}

/// Configuration for closed-loop ZEM/ZEV powered-descent autoland (`PoweredDescentAutoland`).
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct AutolandGuidanceConfig {
    /// Radar altitude (`m` AGL) below which ZEM/ZEV transitions to vertical constant-rate touchdown.
    pub terminal_vertical_alt_m: f64,
    /// Target downward speed (`m/s`) during terminal touchdown (`<= 1.2 m/s`).
    pub touchdown_sink_rate_m_s: f64,
    /// Maximum proper acceleration (`m/s^2`, default `4.0G = 39.2266`).
    pub max_proper_accel_m_s2: f64,
}

impl Default for AutolandGuidanceConfig {
    fn default() -> Self {
        Self {
            terminal_vertical_alt_m: 25.0,
            touchdown_sink_rate_m_s: 0.75,
            max_proper_accel_m_s2: FOUR_G_PROPER_ACCEL_M_S2,
        }
    }
}

/// Output of [`evaluate_powered_descent_autoland`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AutolandGuidanceOutput {
    /// Desired unit thrust vector in `ICRF`.
    pub thrust_dir_icrf: DVec3,
    /// Commanded proper acceleration (`m/s^2`, `0..=max_proper_accel_m_s2`).
    pub commanded_accel_m_s2: f64,
    /// Estimated time-to-go (`s`) until touchdown.
    pub time_to_go_s: f64,
    /// `true` when the lander is inside the terminal `< 50 m` AGL fin-lock zone.
    pub lock_fins_neutral: bool,
}

/// Evaluates closed-loop ZEM/ZEV (`Zero-Effort-Miss / Zero-Effort-Velocity`) powered descent
/// guidance toward a surface target `(target_pos_icrf_m, target_vel_icrf_m_s)`.
#[allow(clippy::too_many_arguments)]
#[must_use]
pub fn evaluate_powered_descent_autoland(
    pos_rel_icrf_m: DVec3,
    vel_rel_icrf_m_s: DVec3,
    target_pos_icrf_m: DVec3,
    target_vel_icrf_m_s: DVec3,
    surface_up_icrf: DVec3,
    grav_accel_icrf_m_s2: DVec3,
    altitude_agl_m: f64,
    config: &AutolandGuidanceConfig,
) -> AutolandGuidanceOutput {
    let lock_fins = altitude_agl_m <= 50.0;
    let rel_r = target_pos_icrf_m - pos_rel_icrf_m;
    let rel_v = vel_rel_icrf_m_s - target_vel_icrf_m_s;
    let dist = sqrt(rel_r.length_squared());
    let speed = sqrt(rel_v.length_squared());

    if altitude_agl_m <= config.terminal_vertical_alt_m {
        let v_up = rel_v.dot(surface_up_icrf);
        let v_horiz = rel_v - v_up * surface_up_icrf;
        let v_up_des = -config.touchdown_sink_rate_m_s;
        let a_vert = 2.8 * (v_up_des - v_up);
        let a_horiz = -2.4 * v_horiz;
        let a_cmd_vec = -grav_accel_icrf_m_s2 + a_vert * surface_up_icrf + a_horiz;
        let a_mag = sqrt(a_cmd_vec.length_squared());
        let dir = if a_mag > 1.0e-6 {
            a_cmd_vec / a_mag
        } else {
            surface_up_icrf
        };
        let tgo = (altitude_agl_m.max(0.0)) / config.touchdown_sink_rate_m_s.max(0.1);
        return AutolandGuidanceOutput {
            thrust_dir_icrf: dir,
            commanded_accel_m_s2: a_mag.min(config.max_proper_accel_m_s2),
            time_to_go_s: tgo,
            lock_fins_neutral: true,
        };
    }

    let g_mag = sqrt(grav_accel_icrf_m_s2.length_squared());
    let a_net_avail = (0.65 * config.max_proper_accel_m_s2 - g_mag).max(2.0);
    let tgo_vel = speed / a_net_avail;
    let tgo_pos = sqrt((2.0 * dist) / a_net_avail);
    let tgo = tgo_vel.max(tgo_pos).max(1.5);

    let v_term_icrf = target_vel_icrf_m_s - config.touchdown_sink_rate_m_s * surface_up_icrf;
    let zem = target_pos_icrf_m
        - (pos_rel_icrf_m + vel_rel_icrf_m_s * tgo + 0.5 * grav_accel_icrf_m_s2 * tgo * tgo);
    let zev = v_term_icrf - (vel_rel_icrf_m_s + grav_accel_icrf_m_s2 * tgo);
    let a_cmd_vec = (6.0 / (tgo * tgo)) * zem - (2.0 / tgo) * zev;
    let a_mag = sqrt(a_cmd_vec.length_squared());
    let dir = if a_mag > 1.0e-6 {
        a_cmd_vec / a_mag
    } else {
        surface_up_icrf
    };

    AutolandGuidanceOutput {
        thrust_dir_icrf: dir,
        commanded_accel_m_s2: a_mag.min(config.max_proper_accel_m_s2),
        time_to_go_s: tgo,
        lock_fins_neutral: lock_fins,
    }
}
