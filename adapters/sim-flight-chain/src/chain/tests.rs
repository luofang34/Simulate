use glam::DVec3;
use pilotage_adapter_api::{
    AdapterCapabilities, ApplyOutcome, LinkLossEnactError, LinkLossPolicy, StepOutcome,
    TelemetryBatch, VideoSource,
};
use pilotage_mission::{MissionEvent, MissionState};
use pilotage_protocol::{ScopeId, ScopedControlFrame, VehicleId};
use sim_geo::{Geodetic, LocalFrame, ecef_to_geodetic, geodetic_to_ecef};

use super::*;
use crate::RouteFix;

const VEHICLE: VehicleId = VehicleId::new(7);
const CRUISE_HEIGHT_M: f64 = 60.0;
/// Navigate's default capture radius, which Pilotage's route missions fly.
const CAPTURE_RADIUS_M: f64 = 100.0;

fn pad() -> Geodetic {
    Geodetic::from_degrees(39.1582, -75.5244, 15.0)
}

/// Route corners, metres north and east of the pad.
const CORNERS: [(f64, f64); 5] = [
    (0.0, 0.0),
    (800.0, 0.0),
    (800.0, 600.0),
    (0.0, 600.0),
    (0.0, 0.0),
];

fn fix(north_m: f64, east_m: f64) -> RouteFix {
    let frame = LocalFrame::at(&pad());
    let ecef = geodetic_to_ecef(&pad()) + frame.ned_to_ecef() * DVec3::new(north_m, east_m, 0.0);
    let position = ecef_to_geodetic(ecef);
    RouteFix {
        lat_deg: position.lat_deg(),
        lon_deg: position.lon_deg(),
    }
}

fn setup() -> ChainSetup {
    ChainSetup {
        mission: MissionSetup {
            vehicle: VEHICLE,
            anchor: pad(),
            fixes: CORNERS.iter().map(|&(n, e)| fix(n, e)).collect(),
            cruise_height_m: CRUISE_HEIGHT_M,
            cruise_mps: 12.0,
            climb_rate_mps: 3.0,
        },
        heading_rad: 0.0,
        noise: DirectNoiseTier::Industrial,
        noise_seed: 11,
    }
}

fn ned(state: &VehicleState) -> DVec3 {
    LocalFrame::at(&pad()).ecef_to_ned_vector(state.position_ecef - geodetic_to_ecef(&pad()))
}

/// Horizontal distance from `p` to the nearest leg of the route, metres.
fn off_route_m(p: DVec3) -> f64 {
    CORNERS
        .windows(2)
        .map(|leg| {
            let a = DVec3::new(leg[0].0, leg[0].1, 0.0);
            let b = DVec3::new(leg[1].0, leg[1].1, 0.0);
            let along = ((p - a).dot(b - a) / (b - a).length_squared()).clamp(0.0, 1.0);
            let nearest = a + (b - a) * along;
            (p.x - nearest.x).hypot(p.y - nearest.y)
        })
        .fold(f64::INFINITY, f64::min)
}

/// Whether `p` is on a straight: clear of every fix by more than the capture
/// radius and the turn that follows it.
fn far_from_fixes(p: DVec3) -> bool {
    CORNERS
        .iter()
        .all(|&(north, east)| (p.x - north).hypot(p.y - east) > 2.0 * CAPTURE_RADIUS_M)
}

/// What a flight looked like against truth.
#[derive(Default)]
struct Flown {
    events: Vec<MissionEvent>,
    worst_off_route_m: f64,
    worst_off_straight_m: f64,
    worst_height_error_m: f64,
    worst_estimate_error_m: f64,
    top_speed_m_s: f64,
    modes_enroute: Vec<ControlMode>,
}

/// Flies the mission to completion and ten seconds beyond.
fn fly_to_completion(chain: &mut FlightChain) -> Flown {
    let mut flown = Flown::default();
    let (mut frames, mut frames_complete) = (0_u32, 0_u32);
    while frames_complete < 10 * MISSION_FRAME_HZ as u32 {
        let report = chain.advance_frame().unwrap();
        frames += 1;
        assert!(
            frames < 600 * MISSION_FRAME_HZ as u32,
            "mission did not complete"
        );
        flown.events.extend(report.events);
        let truth = chain.true_state();
        let p = ned(&truth);
        if report.state == MissionState::Enroute {
            flown.worst_off_route_m = flown.worst_off_route_m.max(off_route_m(p));
            if far_from_fixes(p) {
                flown.worst_off_straight_m = flown.worst_off_straight_m.max(off_route_m(p));
            }
            let height_error = (-p.z - CRUISE_HEIGHT_M).abs();
            flown.worst_height_error_m = flown.worst_height_error_m.max(height_error);
            let mode = chain.flight_mode();
            if !flown.modes_enroute.contains(&mode) {
                flown.modes_enroute.push(mode);
            }
        }
        flown.top_speed_m_s = flown.top_speed_m_s.max(truth.velocity_ecef.length());
        if let Some(estimate) = chain.vehicle().estimated_state() {
            let error = (estimate.position_ecef - truth.position_ecef).length();
            flown.worst_estimate_error_m = flown.worst_estimate_error_m.max(error);
        }
        if report.state == MissionState::Complete {
            frames_complete += 1;
        }
    }
    flown
}

#[test]
fn pilotage_mission_flies_the_route_on_the_flight_controllers_estimate() {
    let mut chain = FlightChain::new(&setup()).unwrap();
    assert_eq!(chain.host().record().waypoint_count, CORNERS.len());
    let flown = fly_to_completion(&mut chain);

    let count =
        |wanted: fn(&MissionEvent) -> bool| flown.events.iter().filter(|e| wanted(e)).count();
    assert_eq!(count(|e| matches!(e, MissionEvent::ArmAccepted { .. })), 1);
    assert_eq!(count(|e| matches!(e, MissionEvent::ClimbStarted)), 1);
    assert_eq!(
        count(|e| matches!(e, MissionEvent::LegAdvanced { .. })),
        CORNERS.len() - 1
    );
    assert_eq!(count(|e| matches!(e, MissionEvent::MissionComplete)), 1);
    assert_eq!(
        count(|e| matches!(e, MissionEvent::GuidanceRefused { .. })),
        0
    );
    assert_eq!(flown.modes_enroute, [ControlMode::VelocityControl]);

    // Each fix sequences one capture radius early, which cuts a right-angle
    // corner by 100 m · (1 - 1/√2) ≈ 29 m before the turn itself is flown.
    assert!(
        flown.worst_off_route_m > 20.0,
        "{} m",
        flown.worst_off_route_m
    );
    assert!(
        flown.worst_off_route_m < 0.7 * CAPTURE_RADIUS_M,
        "{} m",
        flown.worst_off_route_m
    );
    assert!(
        flown.worst_height_error_m < 2.0,
        "{} m",
        flown.worst_height_error_m
    );
    assert!(
        flown.worst_estimate_error_m < 3.0,
        "{} m",
        flown.worst_estimate_error_m
    );
    assert!(
        flown.top_speed_m_s > 9.0 && flown.top_speed_m_s < 11.0,
        "{}",
        flown.top_speed_m_s
    );

    assert!(
        flown.worst_off_straight_m < 3.0,
        "{} m off a straight",
        flown.worst_off_straight_m
    );
    let truth = chain.true_state();
    let p = ned(&truth);
    assert!(
        p.x.hypot(p.y) < CAPTURE_RADIUS_M,
        "stopped {:?} from the last fix",
        p
    );
    assert!(
        (-p.z - CRUISE_HEIGHT_M).abs() < 2.0,
        "hovering at {} m",
        -p.z
    );
    assert!(
        truth.velocity_ecef.length() < 0.5,
        "{} m/s",
        truth.velocity_ecef.length()
    );
    assert_eq!(chain.vehicle().refused_uplink_bytes(), 0);
}

/// The vehicle with the simulation oracle's report replaced by a lie.
struct FalseOracle {
    vehicle: AviateX500Vehicle,
    offset_m: f32,
}

impl VehicleAdapter for FalseOracle {
    fn capabilities(&self) -> AdapterCapabilities {
        self.vehicle.capabilities()
    }

    fn apply_control(&mut self, frame: &ScopedControlFrame) -> ApplyOutcome {
        self.vehicle.apply_control(frame)
    }

    fn sample_telemetry(&mut self) -> TelemetryBatch {
        let mut batch = self.vehicle.sample_telemetry();
        for sample in &mut batch.samples {
            let truth = sample.sim_truth.as_mut().unwrap();
            truth.pos_ned_m = truth.pos_ned_m.map(|metres| metres + self.offset_m);
            truth.vel_ned_mps = [self.offset_m; 3];
            truth.quat_wxyz = [0.0, 1.0, 0.0, 0.0];
        }
        batch
    }

    fn video_sources(&self) -> Vec<VideoSource> {
        self.vehicle.video_sources()
    }

    fn set_link_loss_policy(
        &mut self,
        vehicle: VehicleId,
        scope: &ScopeId,
        policy: Option<LinkLossPolicy>,
    ) -> Result<(), LinkLossEnactError> {
        self.vehicle.set_link_loss_policy(vehicle, scope, policy)
    }

    fn step(&mut self, budget: StepBudget) -> StepOutcome {
        self.vehicle.step(budget)
    }
}

/// Flies `seconds` of the mission with the oracle's report shifted by
/// `offset_m`, and returns the true states at every frame.
fn fly_with_oracle_offset(offset_m: f32, seconds: u32) -> Vec<VehicleState> {
    let setup = setup();
    let mut vehicle = FalseOracle {
        vehicle: AviateX500Vehicle::new(AviateVehicleConfig {
            vehicle: VEHICLE,
            pad: pad(),
            heading_rad: setup.heading_rad,
            noise: setup.noise,
            noise_seed: setup.noise_seed,
        })
        .unwrap(),
        offset_m,
    };
    let mut host = MissionHost::new(&setup.mission, &vehicle.capabilities()).unwrap();
    let mut flown = Vec::new();
    for frame in 0..u64::from(seconds) * MISSION_FRAME_HZ {
        let now = MonotonicNanos::from_nanos(frame * 1_000_000_000 / MISSION_FRAME_HZ);
        host.frame(&mut vehicle, now);
        assert_eq!(
            vehicle.step(StepBudget { ticks: FRAME_TICKS }).advanced,
            FRAME_TICKS
        );
        flown.push(vehicle.vehicle.true_state());
    }
    flown
}

#[test]
fn nothing_the_chain_flies_on_reads_the_simulation_oracle() {
    let honest = fly_with_oracle_offset(0.0, 60);
    let lied_to = fly_with_oracle_offset(750.0, 60);
    let moved = ned(honest.last().unwrap());
    assert!(
        moved.x > 200.0 && -moved.z > 50.0,
        "flight got under way: {moved:?}"
    );
    let estimate_differs_from_truth = honest != fly_with_noise_seed(12, 60);
    assert!(
        estimate_differs_from_truth,
        "the flown track depends on the estimate's noise"
    );
    assert_eq!(
        honest, lied_to,
        "a false oracle report changed the flown track"
    );
}

/// The honest flight with another realisation of the flight controller's
/// sensor noise.
fn fly_with_noise_seed(noise_seed: u64, seconds: u32) -> Vec<VehicleState> {
    let mut setup = setup();
    setup.noise_seed = noise_seed;
    let mut chain = FlightChain::new(&setup).unwrap();
    (0..u64::from(seconds) * MISSION_FRAME_HZ)
        .map(|_| {
            chain.advance_frame().unwrap();
            chain.true_state()
        })
        .collect()
}
