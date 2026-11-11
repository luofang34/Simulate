//! The closed-loop pilot: Navigate's plan executor and guidance, and the
//! route follower, all driven by the navigator's own solution and the IMU's
//! sensed rates. It holds no truth and takes none.

use std::collections::BTreeMap;

use glam::{DQuat, DVec3};
use navigate_contract::{
    ClockDomainId, Ellipsoid, FlightPlan, GuidanceSetpoint, Ident, MonotonicNanos,
    NavigationSolution, PlanRole, SolutionQuality, Waypoint,
};
use navigate_guidance::{GuidanceConfig, guide};
use navigate_route::{PlanExecution, SequenceEvent, SequenceReason};
use sim_core::{ControlInputs, VehicleState};
use sim_geo::{LocalFrame, ecef_to_geodetic};

use crate::trial::route::{RouteFollower, execution_config, leg_speed};
use crate::trial::{QualityFloor, TrialError, VehicleSpec};
use crate::{Estimate, solution_position_ecef};

/// Throttle before the navigator's first solution: about the kinematic
/// vehicle's cruise.
const STARTING_THROTTLE: f64 = 0.35;

/// What the pilot reads at one step: nothing but the navigator's estimate,
/// the IMU's sensed body rate, and the clock.
#[derive(Debug, Clone, PartialEq)]
pub struct PilotInput {
    /// The navigator's solution at this step, when it has one.
    pub estimate: Option<Estimate<NavigationSolution>>,
    /// Body rate the IMU sensed over the last increment, rad/s.
    pub sensed_rate_body: DVec3,
    /// The navigator's clock.
    pub now: MonotonicNanos,
}

/// A waypoint the executor sequenced at this step.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PilotEvent {
    /// The sequenced waypoint.
    pub waypoint_index: usize,
    /// Whether it completed the plan.
    pub plan_complete: bool,
    /// Whether turn anticipation sequenced it.
    pub anticipated: bool,
}

/// What the pilot decided at one step.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PilotOutput {
    /// Stick and throttle for the vehicle.
    pub controls: ControlInputs,
    /// A sequencing decision, if one was made.
    pub event: Option<PilotEvent>,
    /// The waypoint being flown to after this step, or `None` once the plan
    /// is complete.
    pub active_index: Option<usize>,
}

/// Navigate's route execution and guidance flying the vehicle on the
/// navigator's solution.
#[derive(Debug, Clone)]
pub struct Pilot<'w> {
    execution: PlanExecution<'w>,
    waypoints: &'w [Waypoint],
    vehicle: VehicleSpec,
    follower: RouteFollower,
    guidance: GuidanceConfig,
    clock: ClockDomainId,
    refusals: BTreeMap<String, u64>,
}

impl<'w> Pilot<'w> {
    /// A pilot for `waypoints` reading solutions on `clock`.
    pub fn new(
        waypoints: &'w [Waypoint],
        vehicle: &VehicleSpec,
        clock: ClockDomainId,
    ) -> Result<Self, TrialError> {
        let ident = "trial-route";
        let route_id = Ident::try_new(ident).map_err(|source| TrialError::Ident {
            ident: ident.into(),
            source,
        })?;
        let plan = FlightPlan::new(route_id, PlanRole::Mission, Ellipsoid::WGS84, waypoints);
        let execution =
            PlanExecution::new(plan, execution_config(vehicle)).map_err(TrialError::Plan)?;
        let floor = match vehicle.guidance_min_quality {
            Some(QualityFloor::Good) => SolutionQuality::Good,
            Some(QualityFloor::Unusable) => SolutionQuality::Unusable,
            Some(QualityFloor::Degraded) | None => GuidanceConfig::default().minimum_quality,
        };
        let mut guidance = GuidanceConfig::default().with_expected_ellipsoid(Ellipsoid::WGS84);
        guidance.minimum_quality = floor;
        Ok(Self {
            execution,
            waypoints,
            vehicle: *vehicle,
            follower: RouteFollower::new(vehicle),
            guidance,
            clock,
            refusals: BTreeMap::new(),
        })
    }

    /// Guidance refusals so far, by Navigate's refusal kind.
    pub const fn refusals(&self) -> &BTreeMap<String, u64> {
        &self.refusals
    }

    /// One step: sequence the plan on the estimate, guide, and fly.
    pub fn step(&mut self, input: &PilotInput, dt_s: f64) -> PilotOutput {
        let Some(estimate) = input.estimate.as_ref() else {
            return PilotOutput {
                controls: ControlInputs {
                    throttle: STARTING_THROTTLE,
                    ..ControlInputs::default()
                },
                event: None,
                active_index: self.execution.active_leg().map(|leg| leg.index),
            };
        };
        let solution = estimate.get();
        let index = self.execution.active_index();
        let event = match self.execution.advance(
            &solution.position,
            leg_speed(self.waypoints, index, &self.vehicle),
        ) {
            SequenceEvent::LegAdvanced { reason, .. } => Some((false, reason)),
            SequenceEvent::PlanComplete { reason, .. } => Some((true, reason)),
            _ => None,
        }
        .map(|(plan_complete, reason)| PilotEvent {
            waypoint_index: index,
            plan_complete,
            anticipated: reason == SequenceReason::Anticipated,
        });
        let leg = self.execution.active_leg();
        let setpoint = leg.and_then(|leg| self.guide(solution, leg, input.now));
        let speed = leg_speed(
            self.waypoints,
            leg.map_or(index, |leg| leg.index),
            &self.vehicle,
        );
        let believed = believed_state(solution, input.sensed_rate_body);
        let controls = match believed {
            Some(state) => self
                .follower
                .controls(setpoint.as_ref(), &state, speed, dt_s),
            None => ControlInputs {
                throttle: STARTING_THROTTLE,
                ..ControlInputs::default()
            },
        };
        PilotOutput {
            controls,
            event,
            active_index: leg.map(|leg| leg.index),
        }
    }

    fn guide(
        &mut self,
        solution: &NavigationSolution,
        leg: navigate_route::Leg<'_>,
        now: MonotonicNanos,
    ) -> Option<GuidanceSetpoint> {
        match guide(
            solution,
            leg.lateral_reference(),
            leg.to,
            now,
            self.clock,
            &self.guidance,
        ) {
            Ok(command) => Some(command.setpoint),
            Err(refusal) => {
                let text = format!("{refusal:?}");
                let kind = text.split([' ', '{', '(']).next().unwrap_or_default();
                let count = self.refusals.entry(kind.to_owned()).or_default();
                *count = count.wrapping_add(1);
                None
            }
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
