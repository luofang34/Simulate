use std::f64::consts::{PI, TAU};

use glam::{DVec2, DVec3};
use sim_atmosphere::{AtmosphereField, StandardAtmosphere};
use sim_core::{ControlInputs, VehicleState};
use sim_devices::{
    AirDataSystem, AirFlow, GaussMarkov, PitotStatic, PressureErrorSpec, sensed_true_airspeed_m_s,
};
use sim_geo::{LocalFrame, UtcTime, ecef_to_geodetic};

use crate::ScenarioError;

const GRAVITY_M_S2: f64 = 9.806_65;
/// Rate-loop gains, stick per rad/s and per rad. The proportional gain stays
/// under 1 because a full stick is 1 rad/s of roll in the kinematic model,
/// which responds within one step.
const RATE_KP: f64 = 0.5;
const RATE_KI: f64 = 1.5;
/// Attitude errors to attitude rates, 1/s.
const BANK_GAIN: f64 = 1.5;
const PITCH_GAIN: f64 = 1.2;
const FLIGHT_PATH_GAIN: f64 = 0.9;
/// Height error to climb rate, 1/s, and the steepest climb it may ask for.
const HEIGHT_GAIN: f64 = 0.25;
const MAX_CLIMB_M_S: f64 = 7.0;
/// Track error to turn rate, 1/s: gentle when holding a track, firmer when
/// staying on a path.
const TRACK_GAIN: f64 = 0.1;
const PATH_GAIN: f64 = 0.3;
/// Largest bank the path modes ask for, radians.
const MAX_PATH_BANK_RAD: f64 = 0.8;
/// Distance over which a line or orbit error turns into an intercept, metres.
const LINE_LOOKAHEAD_M: f64 = 500.0;
const ORBIT_LOOKAHEAD_M: f64 = 120.0;
/// Largest intercept off the line, radians.
const MAX_INTERCEPT_RAD: f64 = 0.8;
/// Throttle gains, per m/s and per m.
const SPEED_KP: f64 = 0.08;
const SPEED_KI: f64 = 0.02;
const CRUISE_THROTTLE: f64 = 0.35;
/// Fastest turn the coordination term will feed forward, rad/s.
const MAX_TURN_RATE_RAD_S: f64 = 0.6;

/// What the path part of the autopilot follows.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Steering {
    /// Hold a true track, radians from north.
    Track(f64),
    /// Converge on a straight line through a point, ENU metres, along a
    /// bearing.
    Line {
        /// A point on the line.
        through_m: DVec2,
        /// Along-line direction, radians from north.
        bearing_rad: f64,
    },
    /// Circle a point, ENU metres, clockwise seen from above.
    Orbit {
        /// Centre, ENU metres.
        centre_m: DVec2,
        /// Radius, metres.
        radius_m: f64,
    },
    /// Hold a bank and pitch attitude, radians.
    Attitude {
        /// Bank, right wing down positive.
        bank_rad: f64,
        /// Pitch, nose up positive.
        pitch_rad: f64,
    },
}

/// One instant's commands.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Guidance {
    /// What the path part follows.
    pub steering: Steering,
    /// Height above the ellipsoid the path modes hold, metres.
    pub height_m: f64,
    /// Airspeed the throttle holds, m/s.
    pub speed_m_s: f64,
}

/// A vehicle's flight condition against the local horizon.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FlightReading {
    /// Position in the anchor's ENU axes from the origin, metres.
    pub position_enu_m: DVec3,
    /// Height above the ellipsoid, metres.
    pub height_m: f64,
    /// Ground/inertial speed, m/s.
    pub speed_m_s: f64,
    /// Airspeed measured from the pitot-static sensor, m/s; when `None`, the
    /// throttle loop falls back to [`Self::speed_m_s`].
    pub airspeed_m_s: Option<f64>,
    /// True track, radians from north.
    pub track_rad: f64,
    /// Flight-path angle, climbing positive, radians.
    pub flight_path_rad: f64,
    /// Bank, right wing down positive, radians.
    pub bank_rad: f64,
    /// Pitch, nose up positive, radians.
    pub pitch_rad: f64,
    /// Body rates, forward-right-down axes, rad/s.
    pub body_rate: DVec3,
}

impl FlightReading {
    /// The reading of `state`, with its position measured from
    /// `origin_ecef` in `anchor`'s ENU axes.
    pub fn of(state: &VehicleState, origin_ecef: DVec3, anchor: &LocalFrame) -> Self {
        let position = ecef_to_geodetic(state.position_ecef);
        let frame = LocalFrame::at(&position);
        let [north, east, up] = [frame.north(), frame.east(), frame.up()];
        let velocity = state.velocity_ecef;
        let speed_m_s = velocity.length();
        let right = state.attitude * DVec3::Y;
        Self {
            position_enu_m: anchor.ecef_to_enu_vector(state.position_ecef - origin_ecef),
            height_m: position.height_m,
            speed_m_s,
            airspeed_m_s: None,
            track_rad: velocity.dot(east).atan2(velocity.dot(north)),
            flight_path_rad: (velocity.dot(up) / speed_m_s.max(1.0))
                .clamp(-1.0, 1.0)
                .asin(),
            bank_rad: (-right.dot(up)).atan2(-state.down_ecef().dot(up)),
            pitch_rad: state.forward_ecef().dot(up).clamp(-1.0, 1.0).asin(),
            body_rate: state.angular_rate_body,
        }
    }

    /// Attaches a pitot-static airspeed reading (`m/s`) for throttle control.
    #[must_use]
    pub const fn with_airspeed(mut self, airspeed_m_s: f64) -> Self {
        self.airspeed_m_s = Some(airspeed_m_s);
        self
    }

    fn north_east_m(&self) -> DVec2 {
        DVec2::new(self.position_enu_m.y, self.position_enu_m.x)
    }
}

/// The shortest signed turn from `from` to `to`.
pub(crate) fn angle_difference(to: f64, from: f64) -> f64 {
    (to - from + PI).rem_euclid(TAU) - PI
}

fn bearing_of(north_east: DVec2) -> f64 {
    north_east.y.atan2(north_east.x)
}

/// Bank and pitch command for one instant, and whether the pitch is the path
/// or an attitude.
#[derive(Debug, Clone, Copy, PartialEq)]
enum PitchCommand {
    FlightPath(f64),
    Attitude(f64),
}

fn turn_rate_command(steering: Steering, reading: &FlightReading) -> Option<f64> {
    let speed = reading.speed_m_s.max(1.0);
    let error = |bearing: f64| angle_difference(bearing, reading.track_rad);
    let toward = |bearing: f64| PATH_GAIN * error(bearing);
    match steering {
        Steering::Track(track_rad) => Some(TRACK_GAIN * error(track_rad)),
        Steering::Line {
            through_m,
            bearing_rad,
        } => {
            let (sin, cos) = bearing_rad.sin_cos();
            let offset = reading.north_east_m() - through_m;
            // Positive when the vehicle is to the right of the line.
            let cross = offset.y * cos - offset.x * sin;
            let intercept = (cross / LINE_LOOKAHEAD_M)
                .atan()
                .clamp(-MAX_INTERCEPT_RAD, MAX_INTERCEPT_RAD);
            Some(toward(bearing_rad - intercept))
        }
        Steering::Orbit { centre_m, radius_m } => {
            let spoke = reading.north_east_m() - centre_m;
            let tangent = bearing_of(spoke) + PI / 2.0;
            let radial_error = spoke.length() - radius_m;
            let correction = (radial_error / ORBIT_LOOKAHEAD_M).atan();
            Some(speed / radius_m + toward(tangent + correction))
        }
        Steering::Attitude { .. } => None,
    }
}

/// A proportional-integral loop from a rate error to a stick position. The
/// integrator finds each dynamics source's trim, and stops at the stick's
/// travel so saturation does not wind it up.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
struct RateLoop {
    integral: f64,
}

impl RateLoop {
    fn update(&mut self, error: f64, dt_s: f64) -> f64 {
        self.integral = (self.integral + RATE_KI * error * dt_s).clamp(-1.0, 1.0);
        (RATE_KP * error + self.integral).clamp(-1.0, 1.0)
    }
}

/// Flies either dynamics source to a [`Guidance`] through its stick and
/// throttle alone: everything above the rate loops is kinematics, and the
/// rate loops adapt to the airframe.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Autopilot {
    roll: RateLoop,
    pitch: RateLoop,
    yaw: RateLoop,
    speed_integral: f64,
}

impl Autopilot {
    /// The stick and throttle that fly `guidance` from `reading`, `dt_s`
    /// after the previous update.
    pub fn update(
        &mut self,
        guidance: &Guidance,
        reading: &FlightReading,
        dt_s: f64,
    ) -> ControlInputs {
        let speed = reading.speed_m_s.max(1.0);
        let (bank_command, pitch_command) = match guidance.steering {
            Steering::Attitude {
                bank_rad,
                pitch_rad,
            } => (bank_rad, PitchCommand::Attitude(pitch_rad)),
            steering => {
                let turn = turn_rate_command(steering, reading).unwrap_or(0.0);
                let bank = (turn * speed / GRAVITY_M_S2)
                    .atan()
                    .clamp(-MAX_PATH_BANK_RAD, MAX_PATH_BANK_RAD);
                let climb = (HEIGHT_GAIN * (guidance.height_m - reading.height_m))
                    .clamp(-MAX_CLIMB_M_S, MAX_CLIMB_M_S);
                let flight_path = (climb / speed).clamp(-1.0, 1.0).asin();
                (bank, PitchCommand::FlightPath(flight_path))
            }
        };
        let (sin_bank, cos_bank) = reading.bank_rad.sin_cos();
        let (sin_pitch, cos_pitch) = reading.pitch_rad.sin_cos();
        let bank_rate = BANK_GAIN * angle_difference(bank_command, reading.bank_rad);
        let pitch_rate = match pitch_command {
            PitchCommand::FlightPath(path) => FLIGHT_PATH_GAIN * (path - reading.flight_path_rad),
            PitchCommand::Attitude(pitch) => PITCH_GAIN * (pitch - reading.pitch_rad),
        };
        let turn_rate = (GRAVITY_M_S2 * reading.bank_rad.tan() / speed)
            .clamp(-MAX_TURN_RATE_RAD_S, MAX_TURN_RATE_RAD_S);

        // Euler rates to body rates, so a banked aircraft pulls round the turn.
        let p = bank_rate - turn_rate * sin_pitch;
        let q = pitch_rate * cos_bank + turn_rate * cos_pitch * sin_bank;
        let r = turn_rate * cos_pitch * cos_bank - pitch_rate * sin_bank;
        let throttle_speed = reading.airspeed_m_s.unwrap_or(reading.speed_m_s);
        ControlInputs {
            roll: self.roll.update(p - reading.body_rate.x, dt_s),
            pitch: self.pitch.update(q - reading.body_rate.y, dt_s),
            yaw: self.yaw.update(r - reading.body_rate.z, dt_s),
            throttle: self.throttle(guidance.speed_m_s - throttle_speed, dt_s),
        }
    }

    fn throttle(&mut self, error_m_s: f64, dt_s: f64) -> f64 {
        self.speed_integral = (self.speed_integral + SPEED_KI * error_m_s * dt_s).clamp(-0.5, 0.5);
        (CRUISE_THROTTLE + SPEED_KP * error_m_s + self.speed_integral).clamp(0.0, 1.0)
    }
}

/// Simulated `24 Hz` C172-class pitot-static airspeed probe for scenario autopilots.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct ScenarioPitot {
    probe: PitotStatic,
    ticks: u64,
    held_airspeed_m_s: Option<f64>,
}

impl ScenarioPitot {
    /// Creates a forward-axis pitot-static probe seeded deterministically from `seed`.
    pub(crate) fn new(seed: u64) -> Result<Self, ScenarioError> {
        let static_spec = PressureErrorSpec {
            white_sigma_pa: 3.0,
            bias: GaussMarkov {
                tau_s: 600.0,
                sigma: 20.0,
            },
        };
        let pitot_spec = PressureErrorSpec {
            white_sigma_pa: 3.0,
            bias: GaussMarkov {
                tau_s: 600.0,
                sigma: 10.0,
            },
        };
        Ok(Self {
            probe: PitotStatic::with_pitot(static_spec, pitot_spec, DVec3::X, seed)?,
            ticks: 0,
            held_airspeed_m_s: None,
        })
    }

    /// Samples at `24 Hz` (every 5th `120 Hz` tick) and returns the held pitot airspeed (`m/s`).
    pub(crate) fn sense(&mut self, truth: &VehicleState, dt_s: f64) -> Result<f64, ScenarioError> {
        const EVERY: u64 = 5;
        if self.held_airspeed_m_s.is_none() || self.ticks.is_multiple_of(EVERY) {
            let pos = ecef_to_geodetic(truth.position_ecef);
            let air = StandardAtmosphere.sample(&pos, UtcTime::from_unix_seconds(0.0))?;
            let sample = self
                .probe
                .sense(&AirFlow::of(&air, truth), dt_s * (EVERY as f64));
            if let Some(tas) = sensed_true_airspeed_m_s(&sample) {
                self.held_airspeed_m_s = Some(tas);
            }
        }
        self.ticks = self.ticks.wrapping_add(1);
        Ok(self
            .held_airspeed_m_s
            .unwrap_or_else(|| truth.velocity_ecef.length()))
    }
}
