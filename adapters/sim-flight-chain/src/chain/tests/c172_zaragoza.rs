//! The first shareable flight: the C172 around a closed route over land
//! near Zaragoza, on the flight controller's own estimate.

use sim_atmosphere::StandardAtmosphere;
use sim_geo::UtcTime;

use super::*;
use crate::{C172Setup, C172Vehicle, SharedAir, c172_sensor_suite, c172_vehicle};

/// Route fixes, degrees north and east.
const FIXES: [(f64, f64); 5] = [
    (41.70, -1.05),
    (41.70, -0.80),
    (41.80, -0.80),
    (41.80, -1.00),
    (41.74, -1.05),
];
/// Height of the route above the ellipsoid, metres.
const ROUTE_HEIGHT_M: f64 = 2000.0;
const INDICATED_M_S: f64 = 55.0;
/// True airspeed that reads 55 m/s indicated at the route height in the
/// standard atmosphere.
const TRUE_M_S: f64 = 60.7;
/// 2026-06-15 12:04 UTC.
const START_UNIX_S: f64 = 1_781_525_040.0;
/// How far short of the first fix the aeroplane starts, on the first leg
/// extended backwards, metres.
const START_SHORT_M: f64 = 400.0;

fn anchor() -> Geodetic {
    Geodetic::from_degrees(FIXES[0].0, FIXES[0].1, 250.0)
}

/// North and east of the anchor, metres, in its tangent plane.
fn north_east(position_ecef: DVec3) -> (f64, f64) {
    let offset =
        LocalFrame::at(&anchor()).ecef_to_ned_vector(position_ecef - geodetic_to_ecef(&anchor()));
    (offset.x, offset.y)
}

fn fix_north_east(index: usize) -> (f64, f64) {
    let (lat, lon) = FIXES[index];
    north_east(geodetic_to_ecef(&Geodetic::from_degrees(
        lat,
        lon,
        ROUTE_HEIGHT_M,
    )))
}

fn zaragoza_chain(seed: u64) -> FlightChain<C172Vehicle> {
    let (north, east) = fix_north_east(1);
    let heading_rad = east.atan2(north);
    let frame = LocalFrame::at(&anchor());
    let back = DVec3::new(
        -START_SHORT_M * heading_rad.cos(),
        -START_SHORT_M * heading_rad.sin(),
        0.0,
    );
    let over = ecef_to_geodetic(geodetic_to_ecef(&anchor()) + frame.ned_to_ecef() * back);
    let setup = C172Setup {
        vehicle: VEHICLE,
        origin: anchor(),
        position: Geodetic {
            height_m: ROUTE_HEIGHT_M,
            ..over
        },
        heading_rad,
        speed_m_s: TRUE_M_S,
        utc: UtcTime::from_unix_seconds(START_UNIX_S),
        decimal_year: 2026.45,
        seed,
    };
    let suite = c172_sensor_suite(setup.decimal_year, seed);
    let vehicle = c172_vehicle(&setup, &suite, SharedAir::new(StandardAtmosphere)).unwrap();
    let mission = MissionSetup {
        vehicle: VEHICLE,
        anchor: anchor(),
        fixes: FIXES
            .iter()
            .map(|&(lat_deg, lon_deg)| RouteFix {
                lat_deg,
                lon_deg,
                altitude_m: ROUTE_HEIGHT_M,
            })
            .collect(),
        capture_radius_m: 2.0 * INDICATED_M_S,
        cruise_mps: INDICATED_M_S,
        climb_rate_mps: 2.0,
        ownship: OwnshipFeed::FlightController,
    };
    FlightChain::over(vehicle, &mission).unwrap()
}

#[derive(Clone, Copy)]
struct Sample {
    t_s: f64,
    state: MissionState,
    north_m: f64,
    east_m: f64,
    ellipsoidal_m: f64,
    roll_deg: f64,
    sideslip_deg: f64,
    true_m_s: f64,
    indicated_m_s: f64,
    lanes: [f64; 4],
    attitude_error_deg: f64,
    position_error_m: f64,
    mode: ControlMode,
}

fn sample(chain: &FlightChain<C172Vehicle>, t_s: f64, state: MissionState) -> Sample {
    let truth = chain.true_state();
    let position = ecef_to_geodetic(truth.position_ecef);
    let frame = LocalFrame::at(&position);
    let right = frame.ecef_to_ned_vector(truth.attitude * DVec3::Y);
    let down = frame.ecef_to_ned_vector(truth.attitude * DVec3::Z);
    let body_velocity = truth.attitude.inverse() * truth.velocity_ecef;
    let board = chain.vehicle().board();
    let lanes = board.last_command().outputs;
    let (north_m, east_m) = north_east(truth.position_ecef);
    let (attitude_error_deg, position_error_m) =
        board
            .estimated_state()
            .map_or((f64::NAN, f64::NAN), |estimate| {
                let angle = (estimate.attitude.inverse() * truth.attitude)
                    .to_axis_angle()
                    .1;
                (
                    angle.min(std::f64::consts::TAU - angle).to_degrees(),
                    (estimate.position_ecef - truth.position_ecef).length(),
                )
            });
    Sample {
        t_s,
        state,
        north_m,
        east_m,
        ellipsoidal_m: position.height_m,
        roll_deg: right.z.atan2(down.z).to_degrees(),
        sideslip_deg: (body_velocity.y / body_velocity.length())
            .asin()
            .to_degrees(),
        true_m_s: truth.velocity_ecef.length(),
        indicated_m_s: f64::from(
            board
                .controller()
                .board()
                .kernel()
                .state
                .controller
                .air
                .indicated_mps,
        ),
        lanes: [lanes[0], lanes[1], lanes[2], lanes[3]].map(f64::from),
        attitude_error_deg,
        position_error_m,
        mode: chain.flight_mode(),
    }
}

/// Flies the route and `orbit_s` of what follows it.
fn fly_route(
    chain: &mut FlightChain<C172Vehicle>,
    orbit_s: f64,
) -> (Vec<Sample>, Vec<(f64, MissionEvent)>) {
    let (mut samples, mut events) = (Vec::new(), Vec::new());
    let mut completed_at = None::<f64>;
    for frame in 1..=1500 * MISSION_FRAME_HZ {
        let t_s = frame as f64 / MISSION_FRAME_HZ as f64;
        let Ok(report) = chain.advance_frame() else {
            break;
        };
        for event in report.events {
            if matches!(event, MissionEvent::MissionComplete) {
                completed_at = Some(t_s);
            }
            events.push((t_s, event));
        }
        samples.push(sample(chain, t_s, report.state));
        if completed_at.is_some_and(|at| t_s - at >= orbit_s) {
            break;
        }
    }
    (samples, events)
}

/// Signed distance right of the leg from fix `to - 1` to fix `to`, metres.
fn right_of_leg_m(sample: &Sample, to: usize) -> f64 {
    let (a, b) = (fix_north_east(to - 1), fix_north_east(to));
    let (dn, de) = (b.0 - a.0, b.1 - a.1);
    ((sample.east_m - a.1) * dn - (sample.north_m - a.0) * de) / dn.hypot(de)
}

fn mean_sd(values: impl Iterator<Item = f64>) -> (f64, f64) {
    let values: Vec<f64> = values.collect();
    let mean = values.iter().sum::<f64>() / values.len().max(1) as f64;
    let var = values.iter().map(|v| (v - mean).powi(2)).sum::<f64>() / values.len().max(1) as f64;
    (mean, var.sqrt())
}

fn span(values: impl Iterator<Item = f64>) -> (f64, f64) {
    values.fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), v| {
        (lo.min(v), hi.max(v))
    })
}

fn at_event(events: &[(f64, MissionEvent)], wanted: impl Fn(&MissionEvent) -> bool) -> Option<f64> {
    events
        .iter()
        .find_map(|(t, event)| wanted(event).then_some(*t))
}

/// When the aeroplane began flying to fix `to`.
fn leg_start(events: &[(f64, MissionEvent)], to: usize) -> Option<f64> {
    at_event(
        events,
        |event| matches!(event, MissionEvent::LegAdvanced { to_index, .. } if *to_index == to),
    )
}

/// Level cruise on the long first leg: the engine has room left, the
/// surfaces are quiet and nothing drifts.
fn assert_cruise(samples: &[Sample], from_s: f64) {
    let cruise = || {
        samples
            .iter()
            .filter(move |s| s.t_s >= from_s && s.t_s < from_s + 150.0)
    };
    let (throttle, throttle_sd) = mean_sd(cruise().map(|s| s.lanes[0]));
    assert!((0.8..0.97).contains(&throttle), "throttle {throttle}");
    assert!(throttle_sd < 0.06, "throttle wanders by {throttle_sd}");
    assert!(
        mean_sd(cruise().map(|s| s.lanes[2])).1 < 0.01,
        "elevator is busy"
    );
    let (indicated, _) = mean_sd(cruise().map(|s| s.indicated_m_s));
    assert!(
        (indicated - INDICATED_M_S).abs() < 0.5,
        "{indicated} m/s indicated"
    );
    let (truly, _) = mean_sd(cruise().map(|s| s.true_m_s));
    assert!((truly - TRUE_M_S).abs() < 1.5, "{truly} m/s true");
    assert!(cruise().all(|s| s.sideslip_deg.abs() < 1.0 && s.roll_deg.abs() < 3.0));
    assert!(cruise().all(|s| right_of_leg_m(s, 1).abs() < 10.0));
}

/// Each turn stays near the route bank and ends on the new leg.
fn assert_turns_and_legs(samples: &[Sample], events: &[(f64, MissionEvent)], done_s: f64) {
    for to in 2..FIXES.len() {
        let start = leg_start(events, to).unwrap();
        let end = leg_start(events, to + 1).unwrap_or(done_s);
        let turn = || {
            samples
                .iter()
                .filter(move |s| s.t_s >= start - 15.0 && s.t_s < start + 45.0)
        };
        let (left, right) = span(turn().map(|s| s.roll_deg));
        assert!(
            left > -32.0 && right < 32.0,
            "turn to fix {to} banked {left} to {right} deg"
        );
        assert!(
            turn().all(|s| s.sideslip_deg.abs() < 5.0),
            "turn to fix {to} slipped"
        );
        // The turn overshoots the new leg: Pilotage sizes it for the
        // commanded indicated airspeed, and the aeroplane is faster over
        // the ground at this height.
        let on_leg = || {
            samples
                .iter()
                .filter(move |s| s.t_s >= start && s.t_s < end)
        };
        let (_, overshoot) = span(on_leg().map(|s| right_of_leg_m(s, to)));
        assert!(
            overshoot < 200.0,
            "turn to fix {to} overshot by {overshoot} m"
        );
        let settled = || on_leg().filter(move |s| s.t_s >= start + 60.0);
        assert!(
            settled().all(|s| right_of_leg_m(s, to).abs() < 10.0),
            "leg to fix {to} is not held"
        );
    }
}

/// Eighteen simulated minutes; about two wall minutes unoptimised.
#[test]
#[ignore = "the long acceptance flight; run it by name"]
fn pilotage_mission_flies_the_c172_around_the_zaragoza_route() {
    let mut chain = zaragoza_chain(1);
    let (samples, events) = fly_route(&mut chain, 180.0);
    let done_s = at_event(&events, |e| matches!(e, MissionEvent::MissionComplete)).unwrap();
    for to in 1..FIXES.len() {
        assert!(
            leg_start(&events, to).is_some(),
            "fix {to} was never flown to"
        );
    }
    assert!(
        at_event(&events, |e| matches!(
            e,
            MissionEvent::GuidanceRefused { .. }
        ))
        .is_none()
    );
    assert!(at_event(&events, |e| matches!(e, MissionEvent::OrbitStarted { .. })).is_some());
    let commanded = || {
        samples
            .iter()
            .filter(|s| matches!(s.state, MissionState::Enroute | MissionState::Complete))
    };
    assert!(commanded().all(|s| s.mode == ControlMode::VelocityControl));
    let (low, high) = span(commanded().map(|s| s.ellipsoidal_m));
    assert!(
        low > ROUTE_HEIGHT_M - 10.0 && high < ROUTE_HEIGHT_M + 10.0,
        "{low} to {high} m above the ellipsoid"
    );
    let estimated = || samples.iter().filter(|s| !s.attitude_error_deg.is_nan());
    assert!(estimated().all(|s| s.attitude_error_deg < 15.0));
    assert!(estimated().all(|s| s.position_error_m < 10.0));

    assert_cruise(&samples, leg_start(&events, 1).unwrap() + 60.0);
    assert_turns_and_legs(&samples, &events, done_s);

    // After the plan the aeroplane circles the last fix to the right.
    let last = fix_north_east(FIXES.len() - 1);
    let end_s = samples[samples.len() - 1].t_s;
    let orbit = || samples.iter().filter(move |s| s.t_s >= end_s - 80.0);
    let (inner, outer) = span(orbit().map(|s| (s.north_m - last.0).hypot(s.east_m - last.1)));
    // It cannot hold the circle Pilotage asks for at the route bank and
    // flies the wider one that bank gives at its ground speed.
    assert!(
        inner > 600.0 && outer < 1000.0,
        "orbit between {inner} and {outer} m"
    );
    assert!(orbit().all(|s| s.roll_deg > 5.0), "the orbit turns right");
    assert_eq!(chain.vehicle().refused_uplink_bytes(), 0);
}
