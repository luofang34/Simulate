//! The ideal-IMU level: what an error-free strapdown IMU leaves after
//! Navigate's filter dead-reckons a trajectory.

#![allow(
    dead_code,
    reason = "each test crate that includes this support uses part of it"
)]

use navigate_contract::SourceId;
use sim_core::{SimInstant, VehicleState};
use sim_devices::ImuErrorSpec;
use sim_navigate::{
    InertialNavigator, InitialAlignment, MasterClockBridge, SimulatedImu, Truth,
    compare_attitude_with_truth, compare_with_truth, inertial_config,
};

const IMU_SOURCE: SourceId = SourceId::new(63);

/// Worst errors over a dead-reckoned trajectory.
#[derive(Debug, Default, Clone, Copy)]
pub struct IdealDrift {
    /// Metres.
    pub worst_horizontal_m: f64,
    /// Metres.
    pub worst_vertical_m: f64,
    /// Radians.
    pub worst_attitude_rad: f64,
}

/// Dead-reckons `truth` from an exact alignment with an IMU of `spec`,
/// scoring every tenth of a second.
pub fn dead_reckon(truth: &[Truth<(SimInstant, VehicleState)>], spec: ImuErrorSpec) -> IdealDrift {
    let clock = MasterClockBridge::default();
    let mut imu = SimulatedImu::new(spec, 0).unwrap();
    let mut navigator = InertialNavigator::new(inertial_config(&spec), clock, IMU_SOURCE);
    let (start, initial) = *truth[0].get();
    navigator
        .align(&InitialAlignment::EXACT.measure(&initial), start)
        .unwrap();
    assert!(imu.sense(start, &initial).unwrap().is_none());
    let mut drift = IdealDrift::default();
    for (step, sample) in truth.iter().enumerate().skip(1) {
        let (at, state) = *sample.get();
        let increment = imu.sense(at, &state).unwrap().unwrap();
        navigator.ingest(&increment, at).unwrap();
        if step % 12 != 0 && step != truth.len() - 1 {
            continue;
        }
        let solution = navigator.solution(at).unwrap().unwrap();
        let position = compare_with_truth(&solution, state.position_ecef).unwrap();
        let attitude = compare_attitude_with_truth(&solution, state.attitude).unwrap();
        drift.worst_horizontal_m = drift.worst_horizontal_m.max(position.horizontal_error_m);
        drift.worst_vertical_m = drift.worst_vertical_m.max(position.vertical_error_m);
        drift.worst_attitude_rad = drift.worst_attitude_rad.max(attitude.error_rad);
    }
    drift
}

/// An ideal IMU must leave only what a parked one leaves, the 1e-4 m of the
/// two `J2` constants: a sculling, coning or truth-integration mismatch under
/// turn rate and acceleration shows up as centimetres to metres.
pub fn assert_ideal(drift: &IdealDrift) {
    assert!(drift.worst_horizontal_m < 1e-3, "{drift:?}");
    assert!(drift.worst_vertical_m < 1e-3, "{drift:?}");
    assert!(drift.worst_attitude_rad < 1e-9, "{drift:?}");
}
