//! Pilotage's mission engine flying the C172 around a box and into the
//! orbit that follows a fixed-wing plan.

use sim_atmosphere::StandardAtmosphere;
use sim_geo::UtcTime;

use super::*;
use crate::{C172Setup, C172Vehicle, SharedAir, c172_sensor_suite, c172_vehicle};

const HEIGHT_M: f64 = 600.0;
const CRUISE_M_S: f64 = 55.0;
/// The distance flown in two seconds at cruise, which is how Pilotage sizes
/// a capture radius.
const CAPTURE_M: f64 = 2.0 * CRUISE_M_S;
/// Box corners, metres north and east of the anchor.
/// Distance of the start south of the first fix, metres.
const START_SOUTH_M: f64 = 1000.0;
/// Distance from every fix beyond which the aeroplane is on a straight:
/// clear of the turn onto the leg and of the one that ends it.
const STRAIGHT_FROM_FIX_M: f64 = 1200.0;
const BOX: [(f64, f64); 4] = [(0.0, 0.0), (3000.0, 0.0), (3000.0, 3000.0), (0.0, 3000.0)];

fn box_fix(north_m: f64, east_m: f64) -> RouteFix {
    RouteFix {
        altitude_m: pad().height_m + HEIGHT_M,
        ..fix(north_m, east_m)
    }
}

/// The start: on the extended first leg, far enough south of the first fix
/// that the aeroplane is armed and commanded before it reaches it.
fn start() -> Geodetic {
    let fix = fix(-START_SOUTH_M, 0.0);
    Geodetic::from_degrees(fix.lat_deg, fix.lon_deg, pad().height_m + HEIGHT_M)
}

fn c172_chain(seed: u64) -> FlightChain<C172Vehicle> {
    let setup = C172Setup {
        vehicle: VEHICLE,
        origin: pad(),
        position: start(),
        heading_rad: 0.0,
        speed_m_s: CRUISE_M_S,
        utc: UtcTime::from_unix_seconds(1_783_000_000.0),
        decimal_year: 2026.5,
        seed,
    };
    let suite = c172_sensor_suite(2026.5, seed);
    let vehicle = c172_vehicle(&setup, &suite, SharedAir::new(StandardAtmosphere)).unwrap();
    let mission = MissionSetup {
        vehicle: VEHICLE,
        anchor: pad(),
        fixes: BOX.iter().map(|&(n, e)| box_fix(n, e)).collect(),
        capture_radius_m: CAPTURE_M,
        cruise_mps: CRUISE_M_S,
        climb_rate_mps: 2.0,
        ownship: OwnshipFeed::FlightController,
    };
    FlightChain::over(vehicle, &mission).unwrap()
}

/// Distance from `p` to the nearest leg of the box, metres.
fn off_box_m(p: DVec3) -> f64 {
    BOX.windows(2)
        .map(|leg| {
            let a = DVec3::new(leg[0].0, leg[0].1, 0.0);
            let b = DVec3::new(leg[1].0, leg[1].1, 0.0);
            let along = ((p - a).dot(b - a) / (b - a).length_squared()).clamp(0.0, 1.0);
            let nearest = a + (b - a) * along;
            (p.x - nearest.x).hypot(p.y - nearest.y)
        })
        .fold(f64::INFINITY, f64::min)
}

/// What a flight of the box looked like against truth.
#[derive(Default)]
struct BoxFlight {
    events: Vec<MissionEvent>,
    states: Vec<MissionState>,
    worst_off_straight_m: f64,
    height_span_m: (f64, f64),
    worst_attitude_error_deg: f64,
    orbit_radius_span_m: (f64, f64),
    orbit_swept_deg: f64,
    modes: Vec<ControlMode>,
}

fn widen(span: &mut (f64, f64), value: f64) {
    *span = (span.0.min(value), span.1.max(value));
}

/// Flies the box and `orbit_s` of what follows it. The orbit is measured
/// once the aeroplane has had forty seconds to settle onto it.
fn fly_box(chain: &mut FlightChain<C172Vehicle>, orbit_s: u32) -> BoxFlight {
    let mut flown = BoxFlight {
        height_span_m: (f64::INFINITY, f64::NEG_INFINITY),
        orbit_radius_span_m: (f64::INFINITY, f64::NEG_INFINITY),
        ..BoxFlight::default()
    };
    let (mut frames, mut complete) = (0_u32, 0_u32);
    let last = DVec3::new(BOX[BOX.len() - 1].0, BOX[BOX.len() - 1].1, 0.0);
    let mut bearing = None::<f64>;
    while complete < orbit_s * MISSION_FRAME_HZ as u32 && frames < 700 * 20 {
        let Ok(report) = chain.advance_frame() else {
            break;
        };
        frames += 1;
        if flown.states.last() != Some(&report.state) {
            flown.states.push(report.state);
        }
        let on_the_box = flown
            .events
            .iter()
            .any(|event| matches!(event, MissionEvent::LegAdvanced { .. }));
        flown.events.extend(report.events);
        let truth = chain.true_state();
        let p = ned(&truth);
        widen(&mut flown.height_span_m, -p.z);
        if let Some(estimate) = chain.vehicle().board().estimated_state() {
            let error = (estimate.attitude.inverse() * truth.attitude)
                .to_axis_angle()
                .1;
            let error = error.min(std::f64::consts::TAU - error).to_degrees();
            flown.worst_attitude_error_deg = flown.worst_attitude_error_deg.max(error);
        }
        let mode = chain.flight_mode();
        let commanded = matches!(report.state, MissionState::Enroute | MissionState::Complete);
        if commanded && !flown.modes.contains(&mode) {
            flown.modes.push(mode);
        }
        let from_fix = BOX
            .iter()
            .map(|&(n, e)| (p.x - n).hypot(p.y - e))
            .fold(f64::INFINITY, f64::min);
        if report.state == MissionState::Enroute && on_the_box && from_fix > STRAIGHT_FROM_FIX_M {
            flown.worst_off_straight_m = flown.worst_off_straight_m.max(off_box_m(p));
        }
        if report.state == MissionState::Complete {
            complete += 1;
            let now = (p.y - last.y).atan2(p.x - last.x);
            if complete > 40 * MISSION_FRAME_HZ as u32 {
                widen(
                    &mut flown.orbit_radius_span_m,
                    (p - last).truncate().length(),
                );
            }
            if let Some(before) = bearing {
                let step = (now - before + std::f64::consts::PI).rem_euclid(std::f64::consts::TAU)
                    - std::f64::consts::PI;
                flown.orbit_swept_deg += step.to_degrees();
            }
            bearing = Some(now);
        }
    }
    flown
}

fn count(flown: &BoxFlight, wanted: fn(&MissionEvent) -> bool) -> usize {
    flown.events.iter().filter(|event| wanted(event)).count()
}

/// The mission went through its phases once each and sequenced every fix.
fn assert_the_plan_ran(flown: &BoxFlight) {
    // The climb phase is skipped or brief: the aeroplane starts at height.
    let phases: Vec<_> = flown
        .states
        .iter()
        .filter(|state| **state != MissionState::Climb)
        .collect();
    assert_eq!(
        phases,
        [
            &MissionState::AwaitSolution,
            &MissionState::Arming,
            &MissionState::Enroute,
            &MissionState::Complete
        ]
    );
    assert_eq!(
        count(flown, |e| matches!(e, MissionEvent::ArmAccepted { .. })),
        1
    );
    assert_eq!(
        count(flown, |e| matches!(e, MissionEvent::LegAdvanced { .. })),
        BOX.len() - 1
    );
    assert_eq!(
        count(flown, |e| matches!(
            e,
            MissionEvent::LastWaypointSequenced { .. }
        )),
        1
    );
    assert_eq!(
        count(flown, |e| matches!(e, MissionEvent::MissionComplete)),
        1
    );
    assert_eq!(
        count(flown, |e| matches!(e, MissionEvent::GuidanceRefused { .. })),
        0
    );
}

#[test]
fn pilotage_mission_flies_the_c172_around_a_box_and_into_its_orbit() {
    let mut chain = c172_chain(22);
    let flown = fly_box(&mut chain, 100);
    assert_the_plan_ran(&flown);
    let orbit = flown.events.iter().find_map(|event| match event {
        MissionEvent::OrbitStarted { radius_m, .. } => Some(*radius_m),
        _ => None,
    });
    // The orbit Pilotage sizes: the turn at cruise speed and 25 degrees of
    // bank, v² / (g tan 25°).
    let orbit_m = orbit.unwrap();
    assert!((orbit_m - 661.5).abs() < 5.0, "{orbit_m} m");
    assert_eq!(flown.modes, [ControlMode::VelocityControl]);

    assert!(
        flown.worst_off_straight_m < 30.0,
        "{} m off a straight",
        flown.worst_off_straight_m
    );
    let (low, high) = flown.height_span_m;
    assert!(
        low > HEIGHT_M - 15.0 && high < HEIGHT_M + 15.0,
        "{low} to {high} m"
    );
    assert!(
        flown.worst_attitude_error_deg < 15.0,
        "attitude estimate off by {} deg",
        flown.worst_attitude_error_deg
    );

    // The aeroplane circles the last fix clockwise, on a circle wider than
    // the one asked for: it chases a point led ahead of it on that circle.
    assert!(
        flown.orbit_swept_deg > 360.0,
        "{} deg",
        flown.orbit_swept_deg
    );
    let (inner, outer) = flown.orbit_radius_span_m;
    assert!(
        inner > orbit_m && outer < 1.35 * orbit_m,
        "{inner} to {outer} m"
    );
    assert!(outer - inner < 30.0, "{inner} to {outer} m");
    assert_eq!(chain.vehicle().refused_uplink_bytes(), 0);
}
