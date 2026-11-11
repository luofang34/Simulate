use std::collections::BTreeMap;

use aviate_core::control::fixed_wing::FixedWingController;
use aviate_core::mixer::fixed_wing::FixedWingMixer;
use pilotage_mavlink::codec::parse_datagram;
use pilotage_mission::{MissionEvent, MissionState};
use sim_atmosphere::StandardAtmosphere;
use sim_geo::{Geodetic, egm96_undulation_m};

use super::*;
use crate::{
    C172_RATE_HZ, C172Airframe, C172Board, C172Setup, SharedAir, VehicleId, c172_board,
    c172_sensor_suite,
};

mod zaragoza;

/// The first shareable link's route, degrees north and east.
const FIXES: [(f64, f64); 5] = [
    (41.70, -1.05),
    (41.70, -0.80),
    (41.80, -0.80),
    (41.80, -1.00),
    (41.74, -1.05),
];
/// Where the aeroplane starts, degrees north and east, heading east.
const START: (f64, f64) = (41.70, -1.07);
const START_HEADING_DEG: f64 = 90.0;
/// Route and start height above mean sea level, metres.
const ROUTE_MSL_M: f64 = 2000.0;
const INDICATED_M_S: f64 = 55.0;
/// True airspeed that reads 55 m/s indicated at 2000 m in the standard
/// atmosphere.
const TRUE_M_S: f64 = 60.7;
/// 2026-06-15 12:04 UTC.
const START_UNIX_S: f64 = 1_781_525_040.0;
/// When the crew hands the plan to the host after the flight controller
/// powers up, seconds.
const PLAN_LOADED_AT_S: f64 = 2.0;

/// Height above the ellipsoid of the route at `lat_deg`, `lon_deg`: the link
/// states mean sea level, and the chain flies ellipsoidal heights.
fn ellipsoidal_m(lat_deg: f64, lon_deg: f64) -> f64 {
    ROUTE_MSL_M + egm96_undulation_m(lat_deg.to_radians(), lon_deg.to_radians())
}

fn start() -> Geodetic {
    Geodetic::from_degrees(START.0, START.1, ellipsoidal_m(START.0, START.1))
}

fn route() -> ChainRoute {
    ChainRoute {
        fixes: FIXES
            .iter()
            .map(|&(lat_deg, lon_deg)| RouteFix {
                lat_deg,
                lon_deg,
                altitude_m: ellipsoidal_m(lat_deg, lon_deg),
            })
            .collect(),
        cruise_mps: INDICATED_M_S,
        route_bank_deg: crate::C172_PLANNING_BANK_DEG,
        climb_rate_mps: 2.0,
        max_horizontal_mps: INDICATED_M_S,
        ownship: OwnshipFeed::FlightController,
        class: VehicleClass::FixedWing,
        airframe: crate::c172_pilotage_airframe().unwrap(),
        plan_loaded_at_s: PLAN_LOADED_AT_S,
        utc: UtcTime::from_unix_seconds(START_UNIX_S),
    }
}

fn c172(seed: u64) -> C172Board {
    let setup = C172Setup {
        vehicle: VehicleId::new(1),
        origin: start(),
        position: start(),
        heading_rad: START_HEADING_DEG.to_radians(),
        speed_m_s: TRUE_M_S,
        utc: UtcTime::from_unix_seconds(START_UNIX_S),
        decimal_year: 2026.45,
        seed,
    };
    let suite = c172_sensor_suite(setup.decimal_year, seed);
    c172_board(&setup, &suite, SharedAir::new(StandardAtmosphere)).unwrap()
}

fn chain(seed: u64) -> HostChain<C172Airframe, FixedWingController, FixedWingMixer> {
    HostChain::new(c172(seed), &route()).unwrap()
}

/// When each step of getting under way happened, seconds into the run.
#[derive(Debug, Default)]
struct Timeline {
    fc_ready: Option<f64>,
    first_frame_to_fc: Option<f64>,
    fc_armed: Option<f64>,
    arm_confirmed: Option<f64>,
    enroute: Option<f64>,
    velocity_control: Option<f64>,
}

/// What a run saw, for the report and the checks.
#[derive(Debug, Default)]
struct Run {
    timeline: Timeline,
    /// Telemetry messages from the flight controller, by kind.
    messages: BTreeMap<String, u64>,
    /// The last message of each kind, as text.
    last: BTreeMap<String, String>,
    crc_failures: u64,
    unknown_ids: u64,
    events: Vec<(f64, String)>,
    last_state: Option<MissionState>,
    /// The link's bytes, one line per frame or datagram, when captured.
    capture: Option<Vec<String>>,
}

/// How much of a run the byte capture keeps, seconds.
const CAPTURE_S: f64 = 60.0;

fn hex(bytes: &[u8]) -> String {
    bytes.iter().map(|b| format!("{b:02x}")).collect()
}

fn seconds(at: SimInstant) -> f64 {
    at.duration_since(SimInstant::START).as_seconds()
}

fn kind(message: &impl core::fmt::Debug) -> String {
    let text = format!("{message:?}");
    text.split([' ', '{', '(']).next().unwrap_or("").to_owned()
}

/// Flies `chain` for `seconds`, noting the arm timeline and the link.
fn fly(
    chain: &mut HostChain<C172Airframe, FixedWingController, FixedWingMixer>,
    seconds_to_fly: f64,
) -> Run {
    let mut run = Run {
        capture: std::env::var_os("CHAIN_LINK_CAPTURE").map(|_| Vec::new()),
        ..Run::default()
    };
    let mut parsed = Vec::new();
    for _ in 0..(seconds_to_fly * C172_RATE_HZ as f64) as u64 {
        let frames_before = chain.counters().frames_to_fc;
        chain.advance_tick().unwrap();
        let t = seconds(chain.now());
        let line = &mut run.timeline;
        if line.fc_ready.is_none() && chain.board().controller().is_ready() {
            line.fc_ready = Some(t);
        }
        if line.first_frame_to_fc.is_none() && chain.counters().frames_to_fc > frames_before {
            line.first_frame_to_fc = Some(t);
        }
        if line.fc_armed.is_none() && chain.board().controller().is_armed() {
            line.fc_armed = Some(t);
        }
        if line.velocity_control.is_none() && chain.flight_mode() == ControlMode::VelocityControl {
            line.velocity_control = Some(t);
        }
        if let Some(capture) = run.capture.as_mut()
            && t <= CAPTURE_S
        {
            let ns = (t * 1e9).round() as u64;
            for frame in chain.last_uplink() {
                capture.push(format!("{ns} to_fc {}", hex(frame)));
            }
            if !chain.last_downlink().is_empty() {
                capture.push(format!("{ns} from_fc {}", hex(chain.last_downlink())));
            }
        }
        parsed.clear();
        let stats = parse_datagram(chain.last_downlink(), &mut parsed);
        run.crc_failures += u64::from(stats.crc_failures);
        run.unknown_ids += u64::from(stats.unknown_ids);
        for (_, message) in &parsed {
            *run.messages.entry(kind(message)).or_default() += 1;
            run.last.insert(kind(message), format!("{message:?}"));
        }
        for (at, event) in chain.drain_events() {
            let at = seconds(at);
            match &event {
                HostEvent::ArmConfirmed => line.arm_confirmed.get_or_insert(at),
                HostEvent::Mission(MissionEvent::EnrouteStarted) => line.enroute.get_or_insert(at),
                _ => &mut 0.0,
            };
            run.events.push((at, format!("{event:?}")));
        }
        run.last_state = chain.status().mission_state;
    }
    run
}

/// Ninety simulated seconds from the link's start.
#[test]
#[ignore = "Pilotage's host core loses the motion lease while it waits a heartbeat for the arm"]
fn the_host_core_arms_the_c172_and_sets_out_on_the_route() {
    tracing_subscriber::fmt()
        .with_env_filter(tracing_subscriber::EnvFilter::from_default_env())
        .with_test_writer()
        .try_init()
        .ok();
    let mut chain = chain(1);
    let run = fly(&mut chain, 90.0);
    if let (Some(path), Some(lines)) = (std::env::var_os("CHAIN_LINK_CAPTURE"), &run.capture) {
        std::fs::write(path, lines.join("\n")).unwrap();
    }
    tracing::info!(
        timeline = ?run.timeline,
        messages = ?run.messages,
        crc_failures = run.crc_failures,
        unknown_ids = run.unknown_ids,
        events = ?run.events,
        counters = ?chain.counters(),
        status = ?chain.status()
    );
    assert_eq!(run.crc_failures, 0);
    assert!(
        run.timeline.arm_confirmed.is_some(),
        "the arm was never confirmed"
    );
    assert_eq!(run.last_state, Some(MissionState::Enroute));
}

#[test]
fn the_same_seed_flies_the_same_flight() {
    let flown = |seed| {
        let mut chain = chain(seed);
        fly(&mut chain, 45.0);
        (
            chain.status().frame_digest,
            chain.true_state().position_ecef,
        )
    };
    assert_eq!(flown(3), flown(3));
}
