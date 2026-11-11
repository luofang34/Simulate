//! The closed-loop autopilot: the linked flight director sequences the
//! route and computes the guidance setpoint from the navigator's solution,
//! and Simulate's route follower turns that setpoint into stick and
//! throttle from the attitude the navigator believes and the IMU's sensed
//! rates. It holds no truth and takes none.

use std::collections::BTreeMap;

use glam::{DQuat, DVec3};
use navigate_contract::{MonotonicNanos, NavigationSolution};
use navigate_onboard::{DirectorEvent, FlightDirector};
use sim_core::{ControlInputs, VehicleState};
use sim_geo::{LocalFrame, ecef_to_geodetic};

use crate::trial::VehicleSpec;
use crate::trial::route::RouteFollower;
use crate::{Estimate, solution_position_ecef};

/// Throttle before the navigator's first solution: about the kinematic
/// vehicle's cruise.
const STARTING_THROTTLE: f64 = 0.35;

/// What the autopilot reads at one step: nothing but the navigator's
/// estimate, the IMU's sensed body rate, and the clock.
#[derive(Debug, Clone, PartialEq)]
pub struct PilotInput {
    /// The navigator's solution at this step, when it has one.
    pub estimate: Option<Estimate<NavigationSolution>>,
    /// Body rate the IMU sensed over the last increment, rad/s.
    pub sensed_rate_body: DVec3,
    /// The navigator's clock.
    pub now: MonotonicNanos,
}

/// What the autopilot decided at one step.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PilotOutput {
    /// Stick and throttle for the vehicle.
    pub controls: ControlInputs,
    /// The director's sequencing decision, if it made one.
    pub event: Option<DirectorEvent>,
    /// The waypoint being flown to after this step, or `None` once the plan
    /// is complete.
    pub active_index: Option<usize>,
}

/// The flight director and the route follower flying the vehicle on the
/// navigator's solution.
pub struct Autopilot {
    director: Box<dyn FlightDirector>,
    follower: RouteFollower,
    refusals: BTreeMap<String, u64>,
}

impl Autopilot {
    /// `director` flown by a follower within `vehicle`'s limits.
    pub fn new(director: Box<dyn FlightDirector>, vehicle: &VehicleSpec) -> Self {
        Self {
            director,
            follower: RouteFollower::new(vehicle),
            refusals: BTreeMap::new(),
        }
    }

    /// Guidance refusals so far, by Navigate's refusal kind.
    pub const fn refusals(&self) -> &BTreeMap<String, u64> {
        &self.refusals
    }

    /// One step: the director on the estimate, then the follower.
    pub fn step(&mut self, input: &PilotInput, dt_s: f64) -> PilotOutput {
        let starting = ControlInputs {
            throttle: STARTING_THROTTLE,
            ..ControlInputs::default()
        };
        let Some(estimate) = input.estimate.as_ref() else {
            return PilotOutput {
                controls: starting,
                event: None,
                active_index: self.director.active_index(),
            };
        };
        let solution = estimate.get();
        let output = self.director.step(Some(solution), input.now);
        if let Some(refusal) = output.refusal {
            let text = format!("{refusal:?}");
            let kind = text.split([' ', '{', '(']).next().unwrap_or_default();
            let count = self.refusals.entry(kind.to_owned()).or_default();
            *count = count.wrapping_add(1);
        }
        let controls = match believed_state(solution, input.sensed_rate_body) {
            Some(state) => self.follower.controls(
                output.setpoint.as_ref(),
                &state,
                output.commanded_speed_mps,
                dt_s,
            ),
            None => starting,
        };
        PilotOutput {
            controls,
            event: output.event,
            active_index: output.active_index,
        }
    }
}

/// The vehicle as the navigator believes it to be: its solution's position,
/// velocity and attitude, with the sensed body rate. `None` without an
/// attitude.
fn believed_state(solution: &NavigationSolution, rate_body: DVec3) -> Option<VehicleState> {
    let attitude = solution.attitude?;
    let position_ecef = solution_position_ecef(solution);
    let ned_to_ecef = LocalFrame::at(&ecef_to_geodetic(position_ecef)).ned_to_ecef();
    let body_to_ned = DQuat::from_xyzw(attitude.x, attitude.y, attitude.z, attitude.w);
    let v = solution.velocity;
    Some(VehicleState {
        position_ecef,
        velocity_ecef: ned_to_ecef * DVec3::new(v.north_mps, v.east_mps, v.down_mps),
        attitude: (DQuat::from_mat3(&ned_to_ecef) * body_to_ned).normalize(),
        angular_rate_body: rate_body,
    })
}
