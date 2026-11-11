//! The flown route: waypoints through Navigate's plan executor and
//! deviation guidance to the kinematic vehicle, on truth.

use navigate_contract::{
    AltitudeConstraint, Ellipsoid, FlightPlan, GeodeticPosition, Ident, PlanRole, TurnType,
    Waypoint,
};
use navigate_geodesy::{BodyEllipsoid, distance_m, initial_bearing_rad};
use navigate_guidance::{GuidanceConfig, guide};
use navigate_route::{
    EARTH_STANDARD_GRAVITY_MPS2, ExecutionConfig, PlanExecution, SequenceEvent, SequenceReason,
    turn_anticipation_m, turn_radius_m,
};
use serde::{Deserialize, Serialize};
use sim_core::{Block, KinematicInput, KinematicModel, SimInstant, TickSpan, VehicleState};
use sim_geo::{Geodetic, ecef_to_geodetic};

use super::{TrialError, TrialRequest, VehicleSpec, WaypointSpec};
use crate::{MasterClockBridge, Truth};

mod follow;

pub(super) use follow::RouteFollower;
use follow::truth_solution;

/// The kinematic vehicle's step, the simulation's step: 120 Hz.
pub(super) const STEP_HZ: u64 = 120;
/// Slowest speed a leg is planned at when bounding the flight time, m/s.
const SLOWEST_PLANNED_M_S: f64 = 30.0;
/// Time added to the bound for the turns and climbs, seconds.
const FLIGHT_TIME_MARGIN_S: f64 = 900.0;

/// One waypoint sequencing decision, as Navigate's executor made it.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct RouteEvent {
    /// Route time, seconds.
    pub time_s: f64,
    /// Simulation step.
    pub step: usize,
    /// Index of the captured waypoint.
    pub waypoint_index: usize,
    /// Its identifier.
    pub ident: String,
    /// Whether it completed the plan.
    pub plan_complete: bool,
    /// Whether turn anticipation sequenced it beyond the capture radius.
    pub anticipated: bool,
    /// True distance to the fix on Navigate's mean-radius sphere, metres.
    pub distance_m: f64,
}

/// The true flight of a route.
#[derive(Debug, Clone)]
pub struct RouteFlight {
    /// Every step's true state.
    pub truth: Vec<Truth<(SimInstant, VehicleState)>>,
    /// The executor's sequencing, in order.
    pub events: Vec<RouteEvent>,
    /// The plan's waypoints as Navigate holds them.
    pub waypoints: Vec<Waypoint>,
}

/// The simulation step.
pub fn step_span() -> TickSpan {
    TickSpan::from_ticks(sim_core::TICKS_PER_SECOND / STEP_HZ)
}

/// The plan executor's configuration: the vehicle's capture radius and bank
/// limit on the Earth's standard gravity.
pub fn execution_config(vehicle: &VehicleSpec) -> ExecutionConfig {
    let mut config = ExecutionConfig::new(vehicle.capture_radius_m)
        .with_surface_gravity_mps2(EARTH_STANDARD_GRAVITY_MPS2);
    config.bank_limit_rad = vehicle.bank_limit_deg.to_radians();
    config
}

/// The request's waypoints as Navigate waypoints on WGS84, each with an
/// `At` altitude constraint.
pub fn plan_waypoints(specs: &[WaypointSpec]) -> Result<Vec<Waypoint>, TrialError> {
    specs
        .iter()
        .map(|spec| {
            let ident = Ident::try_new(&spec.ident).map_err(|source| TrialError::Ident {
                ident: spec.ident.clone(),
                source,
            })?;
            let position = GeodeticPosition::new(
                spec.lat_deg.to_radians(),
                spec.lon_deg.to_radians(),
                spec.altitude_m,
            );
            let turn = if spec.fly_over {
                TurnType::FlyOver
            } else {
                TurnType::FlyBy
            };
            let waypoint = Waypoint::new(ident, position)
                .with_altitude(AltitudeConstraint::At(spec.altitude_m))
                .with_turn(turn);
            Ok(match spec.speed_m_s {
                Some(speed) => waypoint.with_max_speed(speed),
                None => waypoint,
            })
        })
        .collect()
}

/// Flies the request's route on truth until Navigate's executor reports the
/// plan complete.
pub fn fly_route(request: &TrialRequest) -> Result<RouteFlight, TrialError> {
    let waypoints = plan_waypoints(&request.waypoints)?;
    let truth_and_events = fly_plan(&waypoints, &request.vehicle)?;
    Ok(RouteFlight {
        truth: truth_and_events.0,
        events: truth_and_events.1,
        waypoints,
    })
}

type Flown = (Vec<Truth<(SimInstant, VehicleState)>>, Vec<RouteEvent>);

fn fly_plan(waypoints: &[Waypoint], vehicle: &VehicleSpec) -> Result<Flown, TrialError> {
    let route_id = Ident::try_new("trial-route").map_err(|source| TrialError::Ident {
        ident: "trial-route".into(),
        source,
    })?;
    let plan = FlightPlan::new(route_id, PlanRole::Mission, Ellipsoid::WGS84, waypoints);
    let mut execution =
        PlanExecution::new(plan, execution_config(vehicle)).map_err(TrialError::Plan)?;
    let mut model = KinematicModel::new(start_state(waypoints, vehicle)?);
    let mut follower = RouteFollower::new(vehicle);
    let guidance = GuidanceConfig::default().with_expected_ellipsoid(Ellipsoid::WGS84);
    let clock = MasterClockBridge::default();
    let step = step_span();
    let limit_s = flight_time_bound_s(waypoints);
    let mut at = SimInstant::START;
    let mut truth = vec![Truth::new((at, model.output()))];
    let mut events = Vec::new();
    loop {
        let state = model.output();
        let time_s = at.duration_since(SimInstant::START).as_seconds();
        let position = geodetic(&state);
        let index = execution.active_index();
        let speed = leg_speed(waypoints, index, vehicle);
        let event = execution.advance(&position, speed);
        if let Some(record) = route_event(event, waypoints, index, &position, time_s, &truth) {
            events.push(record);
        }
        let Some(leg) = execution.active_leg() else {
            break;
        };
        if time_s > limit_s {
            return Err(TrialError::RouteIncomplete {
                time_s,
                active_index: leg.index,
            });
        }
        let now = clock.to_monotonic(at)?;
        let solution = truth_solution(&Truth::new(state), now, clock.domain());
        let command = guide(
            solution.get(),
            leg.lateral_reference(),
            leg.to,
            now,
            clock.domain(),
            &guidance,
        )
        .map_err(|source| TrialError::Guidance { time_s, source })?;
        let speed = leg_speed(waypoints, leg.index, vehicle);
        let controls = follower.controls(Some(&command.setpoint), &state, speed, step.as_seconds());
        model
            .step(&KinematicInput { controls })
            .map_err(TrialError::Vehicle)?;
        at = at.advanced(step);
        truth.push(Truth::new((at, model.output())));
    }
    Ok((truth, events))
}

fn route_event(
    event: SequenceEvent,
    waypoints: &[Waypoint],
    index: usize,
    position: &GeodeticPosition,
    time_s: f64,
    truth: &[Truth<(SimInstant, VehicleState)>],
) -> Option<RouteEvent> {
    let (plan_complete, reason) = match event {
        SequenceEvent::LegAdvanced { reason, .. } => (false, reason),
        SequenceEvent::PlanComplete { reason, .. } => (true, reason),
        _ => return None,
    };
    let fix = waypoints.get(index)?;
    Some(RouteEvent {
        time_s,
        step: truth.len().saturating_sub(1),
        waypoint_index: index,
        ident: fix.ident.as_str().to_owned(),
        plan_complete,
        anticipated: reason == SequenceReason::Anticipated,
        distance_m: distance_m(position, &fix.position, BodyEllipsoid::WGS84),
    })
}

/// Speed on the leg toward waypoint `index`: cruise under that waypoint's
/// limit.
pub(super) fn leg_speed(waypoints: &[Waypoint], index: usize, vehicle: &VehicleSpec) -> f64 {
    waypoints
        .get(index)
        .and_then(|w| w.max_speed_mps)
        .map_or(vehicle.cruise_speed_m_s, |limit| {
            limit.min(vehicle.cruise_speed_m_s)
        })
}

/// Level over the first waypoint at its altitude, heading for the second.
pub(super) fn start_state(
    waypoints: &[Waypoint],
    vehicle: &VehicleSpec,
) -> Result<VehicleState, TrialError> {
    let (Some(first), Some(second)) = (waypoints.first(), waypoints.get(1)) else {
        return Err(TrialError::InvalidRequest {
            field: "waypoints",
            detail: "at least two".into(),
        });
    };
    let heading = initial_bearing_rad(&first.position, &second.position);
    let position = Geodetic {
        lat_rad: first.position.latitude_rad,
        lon_rad: first.position.longitude_rad,
        height_m: first.position.altitude_m,
    };
    Ok(VehicleState::level_flight(
        &position,
        heading,
        leg_speed(waypoints, 1, vehicle),
    ))
}

pub(super) fn flight_time_bound_s(waypoints: &[Waypoint]) -> f64 {
    let length_m: f64 = waypoints
        .windows(2)
        .map(|pair| match pair {
            [a, b] => distance_m(&a.position, &b.position, BodyEllipsoid::WGS84),
            _ => 0.0,
        })
        .sum();
    length_m / SLOWEST_PLANNED_M_S + FLIGHT_TIME_MARGIN_S
}

/// `state`'s position as Navigate's geodetic type.
pub(super) fn geodetic(state: &VehicleState) -> GeodeticPosition {
    let position = ecef_to_geodetic(state.position_ecef);
    GeodeticPosition::new(position.lat_rad, position.lon_rad, position.height_m)
}

/// The distance inside which Navigate's executor sequences waypoint
/// `index`: `max(capture radius, min(DTA, half the shorter adjoining leg))`
/// for a fly-by fix between two legs, the capture radius otherwise.
pub(super) fn capture_threshold_m(
    waypoints: &[Waypoint],
    index: usize,
    vehicle: &VehicleSpec,
) -> f64 {
    let capture = vehicle.capture_radius_m;
    let (Some(from), Some(at), Some(to)) = (
        index.checked_sub(1).and_then(|i| waypoints.get(i)),
        waypoints.get(index),
        waypoints.get(index.wrapping_add(1)),
    ) else {
        return capture;
    };
    if at.turn != TurnType::FlyBy {
        return capture;
    }
    let e = BodyEllipsoid::WGS84;
    let inbound = initial_bearing_rad(&at.position, &from.position) + std::f64::consts::PI;
    let outbound = initial_bearing_rad(&at.position, &to.position);
    let radius = turn_radius_m(
        leg_speed(waypoints, index, vehicle),
        vehicle.bank_limit_deg.to_radians(),
        EARTH_STANDARD_GRAVITY_MPS2,
    );
    let half_leg = 0.5
        * distance_m(&from.position, &at.position, e).min(distance_m(
            &at.position,
            &to.position,
            e,
        ));
    let change = (outbound - inbound).rem_euclid(std::f64::consts::TAU);
    let change = change.min(std::f64::consts::TAU - change);
    if change > 2.0 * std::f64::consts::FRAC_PI_3 {
        return capture;
    }
    capture.max(turn_anticipation_m(radius, change).min(half_leg))
}
