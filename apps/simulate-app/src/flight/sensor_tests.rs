//! The `30 Hz` optical sensor channel and manual-control takeover.

use sim_core::{SensorExposureEvent, SystemControls};
use sim_scenario::{ScenarioProfile, SimTimeScale};

use super::sensors::{OPTICAL_SENSOR_CHANNEL, OPTICAL_SENSOR_PERIOD};
use super::*;
use crate::vehicles::FreeFlightVehicle;

fn cruise() -> ControlInputs {
    ControlInputs {
        throttle: 0.72,
        ..ControlInputs::default()
    }
}

/// Flies `total_s` in frames of `frame_s`, draining exposures every frame.
fn exposures_at_frame_rate(
    vehicle: FreeFlightVehicle,
    frame_s: f64,
    total_s: f64,
) -> Vec<SensorExposureEvent> {
    let mut flight = Flight::new().expect("flight");
    flight.select_vehicle(vehicle).expect("vehicle");
    let frames = (total_s / frame_s).round() as usize;
    let mut out = Vec::new();
    for _ in 0..frames {
        flight
            .advance_by(frame_s, cruise(), SimTimeScale::RealTime)
            .expect("frame");
        out.extend(flight.drain_sensor_exposures());
    }
    assert_eq!(flight.sensor_exposures_drained(), out.len() as u64);
    out
}

fn assert_strict_30_hz(exposures: &[SensorExposureEvent]) {
    for (i, pair) in exposures.windows(2).enumerate() {
        assert_eq!(pair[0].channel, OPTICAL_SENSOR_CHANNEL);
        assert_eq!(
            pair[1].exposure_at.duration_since(pair[0].exposure_at),
            OPTICAL_SENSOR_PERIOD,
            "exposure {i}"
        );
        assert_eq!(pair[1].sequence, pair[0].sequence + 1);
    }
}

#[test]
fn free_flight_exposes_at_30_hz_independent_of_frame_rate() {
    for vehicle in [FreeFlightVehicle::Kinematic, FreeFlightVehicle::JsbsimC172] {
        let slow = exposures_at_frame_rate(vehicle, 1.0 / 30.0, 2.0);
        let fast = exposures_at_frame_rate(vehicle, 1.0 / 144.0, 2.0);
        assert!(
            slow.len() >= 59 && fast.len() >= 59,
            "{} {}",
            slow.len(),
            fast.len()
        );
        assert!(slow.len().abs_diff(fast.len()) <= 1);
        assert_strict_30_hz(&slow);
        assert_strict_30_hz(&fast);
        for (a, b) in slow.iter().zip(&fast) {
            assert_eq!(
                a, b,
                "{vehicle:?}: exposures must not depend on frame chunking"
            );
        }
        let first = slow.first().expect("an exposure");
        assert_eq!(
            first.exposure_at,
            SimInstant::START.advanced(OPTICAL_SENSOR_PERIOD)
        );
    }
}

#[test]
fn scenario_exposures_carry_exact_step_truth() {
    let mut flight = Flight::new().expect("flight");
    flight
        .select(Some(ScenarioProfile::CorridorNominalCruise))
        .expect("scenario");
    let mut exposures = Vec::new();
    for _ in 0..45 {
        flight
            .advance_by(1.0 / 45.0, cruise(), SimTimeScale::RealTime)
            .expect("frame");
        let drained = flight.drain_sensor_exposures();
        if let Some(last) = drained.last()
            && last.exposure_at == flight.now()
        {
            assert_eq!(last.state, flight.vehicle_state(), "uninterpolated truth");
        }
        exposures.extend(drained);
    }
    assert!(exposures.len() >= 29, "{}", exposures.len());
    assert_strict_30_hz(&exposures);
}

#[test]
fn paused_flight_exposes_nothing_and_rewind_rephases() {
    let mut flight = Flight::new().expect("flight");
    flight
        .select_vehicle(FreeFlightVehicle::Kinematic)
        .expect("kinematic");
    for _ in 0..10 {
        flight
            .advance_by(0.1, cruise(), SimTimeScale::Paused)
            .expect("paused");
    }
    assert!(flight.drain_sensor_exposures().is_empty());
    for _ in 0..8 {
        flight
            .advance_by(0.25, cruise(), SimTimeScale::RealTime)
            .expect("frame");
    }
    assert_eq!(flight.drain_sensor_exposures().len(), 60);
    flight.rewind_by_seconds(1.0).expect("rewind");
    let rewound_to = flight.now();
    flight
        .advance_by(0.5, cruise(), SimTimeScale::RealTime)
        .expect("frame");
    let after = flight.drain_sensor_exposures();
    let first = after.first().expect("exposures after rewind");
    assert_eq!(
        first.exposure_at,
        rewound_to.advanced(OPTICAL_SENSOR_PERIOD),
        "sensors re-phase to the checkpoint"
    );
    assert_strict_30_hz(&after);
}

#[test]
fn taking_manual_control_flies_on_from_the_scenario_state() {
    let mut flight = Flight::new().expect("flight");
    flight
        .select(Some(ScenarioProfile::CorridorNominalCruise))
        .expect("scenario");
    for _ in 0..8 {
        flight
            .advance_by(0.25, cruise(), SimTimeScale::RealTime)
            .expect("frame");
    }
    let before = flight.vehicle_state();
    let epoch = flight.discontinuity();
    flight.take_manual_control().expect("takeover");
    assert_eq!(flight.profile(), None);
    assert_eq!(flight.free_vehicle(), Some(flight.preferred_vehicle()));
    assert_ne!(flight.discontinuity(), epoch);
    let after = flight.vehicle_state();
    let moved_m = (after.position_ecef - before.position_ecef).length();
    assert!(
        moved_m < 50.0,
        "spawned {moved_m} m from the scenario state"
    );
    let turned = (manual::heading_rad(&after) - manual::heading_rad(&before))
        .rem_euclid(std::f64::consts::TAU);
    assert!(
        turned.min(std::f64::consts::TAU - turned) < 0.05,
        "{turned}"
    );
    flight
        .advance_by(1.0, cruise(), SimTimeScale::RealTime)
        .expect("free flight continues");
    flight.restart().expect("restart");
    let restarted = flight.vehicle_state();
    assert!((restarted.position_ecef - after.position_ecef).length() < 1e-6);
    flight
        .take_manual_control()
        .expect("already manual is a no-op");
}

#[test]
fn system_controls_queued_while_paused_apply_on_the_next_step() {
    let mut flight = Flight::new().expect("flight");
    let start = flight.system_controls().expect("JSBSim systems");
    flight.update_system_controls(|sys| sys.flaps_norm = 1.0);
    flight.update_system_controls(|sys| sys.wheel_brake_norm = 1.0);
    let queued: SystemControls = flight.system_controls().expect("queued");
    assert!(
        (queued.flaps_norm - 1.0).abs() < 1e-12 && (queued.wheel_brake_norm - 1.0).abs() < 1e-12
    );
    assert!(start.flaps_norm < 1.0);
    flight
        .advance_by(0.1, cruise(), SimTimeScale::RealTime)
        .expect("frame");
    let applied = flight.system_controls().expect("applied");
    assert!((applied.flaps_norm - 1.0).abs() < 1e-12);
}
