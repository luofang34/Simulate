use std::f64::consts::{PI, TAU};

use navigate_contract::{
    ClockDomainId, Ellipsoid, FaultDetection, GuidanceSetpoint, IntegrityAssessment,
    MonotonicNanos, NavigationSolution, NedVelocity, Redundancy, SensorClass, SolutionQuality,
    SolutionStamp, SourceComposition, SourceEpoch, SymmetricCov3, WrappingSequence,
};
use sim_core::{ControlInputs, VehicleState};
use sim_geo::{LocalFrame, ecef_to_geodetic};
use sim_scenario::{Autopilot, FlightReading, Guidance, Steering};

use super::geodetic;
use crate::Truth;
use crate::trial::VehicleSpec;

/// Standard gravity for the coordinated-turn bank, m/s².
const GRAVITY_M_S2: f64 = 9.806_65;
/// Track error to turn rate, 1/s.
const TRACK_GAIN: f64 = 0.3;
/// Seconds of flight over which a cross-track offset turns into an
/// intercept: the offset decays with about this time constant.
const LOOKAHEAD_S: f64 = 10.0;
/// Vertical deviation to climb rate, 1/s.
const CLIMB_GAIN: f64 = 0.2;
/// Variance the truth solution declares, m² and (m/s)²: exact, but
/// positive-definite as Navigate's contract requires.
const TRUTH_VARIANCE: f64 = 1e-6;

/// Turns Navigate's deviation-tracking setpoint into stick and throttle for
/// the kinematic vehicle, through Simulate's attitude autopilot.
///
/// The lateral law steers toward the reference course plus an intercept of
/// `atan(lateral / lookahead)` and banks for the turn rate that closes the
/// track error, within the vehicle's bank limit, the same limit the plan's
/// fly-by geometry assumes. The vertical law climbs or descends against the
/// vertical deviation within the climb limit. The kinematic vehicle's
/// velocity lies along its nose, so the pitch command is the flight path.
#[derive(Debug, Clone, Default)]
pub(crate) struct RouteFollower {
    autopilot: Autopilot,
    bank_limit_rad: f64,
    max_climb_m_s: f64,
}

impl RouteFollower {
    pub(crate) fn new(vehicle: &VehicleSpec) -> Self {
        Self {
            autopilot: Autopilot::default(),
            bank_limit_rad: vehicle.bank_limit_deg.to_radians(),
            max_climb_m_s: vehicle.max_climb_m_s,
        }
    }

    /// Stick and throttle for `setpoint` from the vehicle believed to be in
    /// `state`; `None` holds the present track and height.
    pub(crate) fn controls(
        &mut self,
        setpoint: Option<&GuidanceSetpoint>,
        state: &VehicleState,
        speed_m_s: f64,
        dt_s: f64,
    ) -> ControlInputs {
        let frame = LocalFrame::at(&ecef_to_geodetic(state.position_ecef));
        let reading = FlightReading::of(state, state.position_ecef, &frame);
        let speed = reading.speed_m_s.max(1.0);
        let (desired_track, vertical_m) = match setpoint.copied() {
            Some(GuidanceSetpoint::DeviationTracking {
                lateral_m,
                vertical_m,
                course_rad,
            }) => (
                course_rad - (lateral_m / (speed * LOOKAHEAD_S)).atan(),
                vertical_m,
            ),
            // Deviation guidance gives no other setpoint; anything else, or
            // no setpoint at all, holds the present track and height.
            _ => (reading.track_rad, 0.0),
        };
        let turn_rate = TRACK_GAIN * wrapped(desired_track - reading.track_rad);
        let bank = (turn_rate * speed / GRAVITY_M_S2)
            .atan()
            .clamp(-self.bank_limit_rad, self.bank_limit_rad);
        let climb = (-CLIMB_GAIN * vertical_m).clamp(-self.max_climb_m_s, self.max_climb_m_s);
        let pitch = (climb / speed).clamp(-1.0, 1.0).asin();
        let guidance = Guidance {
            steering: Steering::Attitude {
                bank_rad: bank,
                pitch_rad: pitch,
            },
            height_m: reading.height_m,
            speed_m_s,
        };
        self.autopilot.update(&guidance, &reading, dt_s)
    }
}

/// `angle` folded into `[-π, π)`.
fn wrapped(angle: f64) -> f64 {
    (angle + PI).rem_euclid(TAU) - PI
}

/// The true state as a navigation solution, for guidance in open loop. It
/// stays wrapped as truth: only the route flight reads it.
pub(super) fn truth_solution(
    state: &Truth<VehicleState>,
    now: MonotonicNanos,
    clock: ClockDomainId,
) -> Truth<NavigationSolution> {
    let state = state.get();
    let frame = LocalFrame::at(&ecef_to_geodetic(state.position_ecef));
    let velocity = frame.ecef_to_ned_vector(state.velocity_ecef);
    let exact = SymmetricCov3::from_diagonal(TRUTH_VARIANCE, TRUTH_VARIANCE, TRUTH_VARIANCE);
    let composition = SourceComposition::of(SensorClass::Gnss).with(SensorClass::Inertial);
    let sigma = TRUTH_VARIANCE.sqrt();
    Truth::new(NavigationSolution::new(
        SolutionStamp::new(SourceEpoch::new(0), WrappingSequence::new(0), now, clock),
        geodetic(state),
        NedVelocity::new(velocity.x, velocity.y, velocity.z),
        exact,
        exact,
        IntegrityAssessment::new(
            SolutionQuality::Good,
            composition,
            Redundancy::IndependentClasses,
            sigma,
            sigma,
            FaultDetection::Monitoring,
        ),
        composition,
        Ellipsoid::WGS84,
    ))
}
