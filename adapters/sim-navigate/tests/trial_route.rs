//! The trial's route flight: Navigate's plan executor and deviation guidance
//! steer the kinematic vehicle round a four-waypoint route with two turns
//! over 60° and a climb, on truth; and a route through 300 m, 3 km and 9 km
//! flies its vertical plan.

use navigate_geodesy::{BodyEllipsoid, cross_track_m, initial_bearing_rad};
use navigate_route::{turn_anticipation_m, turn_radius_m};
use sim_devices::ImuErrorSpec;
use sim_geo::{LocalFrame, ecef_to_geodetic};
use sim_navigate::trial::{
    ImuGradeName, RouteFlight, TrialError, execution_config, fly_route, vertical_plan,
};

#[cfg(test)]
#[path = "trial_support/ideal.rs"]
mod ideal;
#[cfg(test)]
#[path = "trial_support/requests.rs"]
mod requests;

/// Largest cross-track error on a straight leg after capture, metres. A leg
/// is captured at the first instant the vehicle is both within this of the
/// track and heading along it.
const STRAIGHT_LEG_CROSS_TRACK_M: f64 = 30.0;
/// Track error under which the vehicle heads along the leg, radians.
const ALONG_TRACK_RAD: f64 = 0.035;

#[cfg(test)]
fn init_tracing() {
    tracing_subscriber::fmt().with_test_writer().try_init().ok();
}

#[cfg(test)]
fn flown() -> (sim_navigate::trial::TrialRequest, RouteFlight) {
    let request = requests::request(
        requests::four_waypoints(),
        ImuGradeName::Tactical,
        requests::reference_sets(),
    );
    let flight = fly_route(&request).unwrap();
    (request, flight)
}

/// The executor's own capture threshold for the waypoint at `index`:
/// `max(capture radius, min(DTA, half the shorter adjoining leg))` for a
/// fly-by fix between two legs, the capture radius otherwise.
#[cfg(test)]
fn capture_threshold_m(flight: &RouteFlight, index: usize, request_speed: f64, bank: f64) -> f64 {
    let capture = 100.0;
    let w = &flight.waypoints;
    if index == 0 || index + 1 >= w.len() {
        return capture;
    }
    let (from, at, to) = (
        &w[index - 1].position,
        &w[index].position,
        &w[index + 1].position,
    );
    let inbound = initial_bearing_rad(at, from) + std::f64::consts::PI;
    let outbound = initial_bearing_rad(at, to);
    let radius = turn_radius_m(request_speed, bank, 9.80665);
    let e = BodyEllipsoid::WGS84;
    let half_leg = 0.5
        * navigate_geodesy::distance_m(from, at, e).min(navigate_geodesy::distance_m(at, to, e));
    capture.max(turn_anticipation_m(radius, outbound - inbound).min(half_leg))
}

#[test]
fn the_executor_sequences_every_waypoint_once_in_order() {
    init_tracing();
    let (request, flight) = flown();
    let indices: Vec<usize> = flight.events.iter().map(|e| e.waypoint_index).collect();
    assert_eq!(indices, vec![0, 1, 2, 3], "{:?}", flight.events);
    let completes: Vec<bool> = flight.events.iter().map(|e| e.plan_complete).collect();
    assert_eq!(completes, vec![false, false, false, true]);
    let config = execution_config(&request.vehicle);
    for event in &flight.events {
        let threshold = capture_threshold_m(
            &flight,
            event.waypoint_index,
            request.vehicle.cruise_speed_m_s,
            config.bank_limit_rad,
        );
        tracing::info!(?event, threshold_m = threshold, "sequenced");
        assert!(event.distance_m < threshold, "{event:?} {threshold}");
    }
    // Both corners turn more than 60°, so both anticipate.
    assert!(flight.events[1].anticipated && flight.events[2].anticipated);
}

#[test]
fn straight_legs_hold_the_track_after_rollout() {
    init_tracing();
    let (_, flight) = flown();
    let e = BodyEllipsoid::WGS84;
    for leg in 1..flight.waypoints.len() {
        let from = &flight.waypoints[leg - 1].position;
        let to = &flight.waypoints[leg].position;
        let course = initial_bearing_rad(from, to);
        let start = flight.events[leg - 1].step;
        let end = flight.events[leg].step;
        let samples: Vec<(f64, f64)> = flight.truth[start..end]
            .iter()
            .map(|sample| {
                let state = sample.get().1;
                let geo = ecef_to_geodetic(state.position_ecef);
                let position = navigate_contract::GeodeticPosition::new(
                    geo.lat_rad,
                    geo.lon_rad,
                    geo.height_m,
                );
                let frame = LocalFrame::at(&geo);
                let v = state.velocity_ecef;
                let track = v.dot(frame.east()).atan2(v.dot(frame.north()));
                let track_error = (track - course + std::f64::consts::PI)
                    .rem_euclid(std::f64::consts::TAU)
                    - std::f64::consts::PI;
                (cross_track_m(&position, from, to, e).unwrap(), track_error)
            })
            .collect();
        let rollout = samples
            .iter()
            .position(|(xt, track_error)| {
                xt.abs() < STRAIGHT_LEG_CROSS_TRACK_M && track_error.abs() < ALONG_TRACK_RAD
            })
            .unwrap();
        let overshoot = samples.iter().map(|(xt, _)| xt.abs()).fold(0.0, f64::max);
        let worst = samples[rollout..]
            .iter()
            .map(|(xt, _)| xt.abs())
            .fold(0.0, f64::max);
        tracing::info!(
            leg,
            rollout_s = rollout as f64 / 120.0,
            leg_s = samples.len() as f64 / 120.0,
            worst_cross_track_m = worst,
            largest_before_capture_m = overshoot,
            "straight leg"
        );
        assert!(rollout < samples.len() / 2, "leg {leg} captured late");
        assert!(worst < STRAIGHT_LEG_CROSS_TRACK_M, "leg {leg}: {worst} m");
    }
}

#[test]
fn the_route_climbs_to_the_constrained_altitude() {
    let (_, flight) = flown();
    let end = flight.truth.last().unwrap().get().1;
    let height = ecef_to_geodetic(end.position_ecef).height_m;
    assert!((height - 1_300.0).abs() < 5.0, "{height}");
}

/// An ideal IMU through the route's turns and climb must dead-reckon to the
/// same level it does through the corridor manoeuvres, or the route's
/// trajectory is not one a strapdown IMU could sense.
#[test]
fn an_ideal_imu_follows_the_route() {
    init_tracing();
    let (_, flight) = flown();
    let drift = ideal::dead_reckon(&flight.truth, ImuErrorSpec::IDEAL);
    tracing::info!(?drift, "ideal IMU over the route");
    ideal::assert_ideal(&drift);
}

/// Five waypoints east of the corridor: level at 300 m, a 2.7 km climb to
/// 3 km, a 6 km climb to 9 km, and a 6 km descent back to 3 km, flown at
/// 100 m/s with a 20 m/s climb limit.
#[cfg(test)]
fn multi_altitude() -> sim_navigate::trial::TrialRequest {
    let waypoints = vec![
        requests::waypoint("V0", 39.0, -75.6, 300.0),
        requests::waypoint("V1", 39.0, -75.507, 300.0),
        requests::waypoint("V2", 39.02, -75.22, 3_000.0),
        requests::waypoint("V3", 39.0, -74.70, 9_000.0),
        requests::waypoint("V4", 39.0, -74.24, 3_000.0),
    ];
    let mut request = requests::request(
        waypoints,
        ImuGradeName::Tactical,
        vec![requests::set(
            "ins",
            &[sim_navigate::NavMeans::DeadReckoning],
        )],
    );
    request.vehicle.cruise_speed_m_s = 100.0;
    request.vehicle.max_climb_m_s = 20.0;
    request.vehicle.bank_limit_deg = 25.0;
    request
}

/// The multi-altitude route passes every waypoint inside the executor's
/// capture threshold, crossing each at its altitude, and its vertical plan
/// names each leg's demand within the climb limit.
#[test]
fn a_route_through_300_m_3_km_and_9_km_flies_its_vertical_plan() {
    init_tracing();
    let request = multi_altitude();
    request.validate().unwrap();
    let (legs, planned) = vertical_plan(&request.waypoints, &request.vehicle).unwrap();
    for leg in &legs {
        tracing::info!(?leg, "vertical demand");
        assert!(leg.required_rate_m_s <= leg.limit_m_s, "{leg:?}");
    }
    assert_eq!(planned.last().unwrap().altitude_m, 3_000.0);
    let flight = fly_route(&request).unwrap();
    let indices: Vec<usize> = flight.events.iter().map(|e| e.waypoint_index).collect();
    assert_eq!(indices, vec![0, 1, 2, 3, 4], "{:?}", flight.events);
    let config = execution_config(&request.vehicle);
    for event in &flight.events {
        let threshold = capture_threshold_m(
            &flight,
            event.waypoint_index,
            request.vehicle.cruise_speed_m_s,
            config.bank_limit_rad,
        );
        let state = flight.truth[event.step].get().1;
        let height = ecef_to_geodetic(state.position_ecef).height_m;
        let target = request.waypoints[event.waypoint_index].altitude_m;
        tracing::info!(?event, threshold, height, target, "sequenced");
        assert!(event.distance_m < threshold, "{event:?} {threshold}");
        assert!((height - target).abs() < 10.0, "{event:?}: {height} m");
    }
}

/// A climb the vehicle cannot make before the waypoint is refused, naming
/// the leg.
#[test]
fn an_impossible_climb_is_refused_naming_the_leg() {
    init_tracing();
    let mut request = multi_altitude();
    request.vehicle.max_climb_m_s = 5.0;
    let error = request.validate().unwrap_err();
    tracing::info!(%error, "refused");
    match error {
        TrialError::UnreachableAltitude { leg, from, to, .. } => {
            assert_eq!((leg, from.as_str(), to.as_str()), (2, "V1", "V2"));
        }
        other => panic!("{other:?}"),
    }
}
