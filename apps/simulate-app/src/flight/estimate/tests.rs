use glam::DVec3;
use sim_core::{ControlInputs, SimInstant, VehicleState};
use sim_scenario::SimTimeScale;

use super::super::Flight;
use crate::sensor_bus::SensorRealism;

const FRAME_S: f64 = 0.25;

/// Flies `frames` quarter-second frames, recording `(instant, truth, estimate)` after each.
fn fly(flight: &mut Flight, frames: usize) -> Vec<(SimInstant, DVec3, VehicleState)> {
    let controls = ControlInputs {
        throttle: 0.72,
        ..ControlInputs::default()
    };
    (0..frames)
        .map(|_| {
            flight
                .advance_by(FRAME_S, controls, SimTimeScale::RealTime)
                .expect("flight step");
            (
                flight.now(),
                flight.vehicle_state().position_ecef,
                flight.estimated_vehicle_state(),
            )
        })
        .collect()
}

#[test]
fn realistic_navigation_is_never_raw_truth_yet_replays_bit_for_bit() {
    let mut flight = Flight::new().expect("flight");
    assert_eq!(flight.sensor_realism(), SensorRealism::Realistic);
    let track = fly(&mut flight, 80);
    for (_, truth, estimate) in &track {
        assert_ne!(estimate.position_ecef, *truth);
        let error_m = (estimate.position_ecef - *truth).length();
        assert!(error_m < 15.0, "navigation error {error_m} m");
    }
    let display_error = flight.display_estimated_vehicle_state().position_ecef
        - flight.display_vehicle_state().position_ecef;
    assert!(display_error.length() > 0.0 && display_error.length() < 15.0);

    flight.rewind_by_seconds(10.0).expect("rewind");
    let resumed_at = flight.now();
    let replay = fly(&mut flight, 8);
    for (at, truth, estimate) in &replay {
        let (_, recorded_truth, recorded_estimate) = track
            .iter()
            .find(|(t, _, _)| t == at)
            .expect("replayed instant was recorded");
        assert_eq!(truth, recorded_truth);
        assert_eq!(
            estimate, recorded_estimate,
            "after rewind to {resumed_at:?}"
        );
    }

    flight.restart().expect("restart");
    let again = fly(&mut flight, 8);
    for (fresh, recorded) in again.iter().zip(&track) {
        assert_eq!(
            fresh.2, recorded.2,
            "restart replays the same sensor errors"
        );
    }
}

#[test]
fn ideal_sensors_hand_the_displays_truth() {
    let mut flight = Flight::new().expect("flight");
    flight.set_sensor_realism(SensorRealism::Ideal);
    let _ = fly(&mut flight, 4);
    assert_eq!(flight.estimated_vehicle_state(), flight.vehicle_state());
    let sensed = flight.onboard_sensors();
    assert_eq!(sensed.estimated_vehicle, flight.display_vehicle_state());
    assert!(sensed.air_data.is_some());
    assert_eq!(flight.cycle_sensor_realism(), SensorRealism::Realistic);
}
