//! Stress cases for the C172 on its own sensors: an uncalibrated inertial
//! unit and a large turn the moment the flight controller is armed.

use super::*;
use crate::{AccelCalibration, c172_sensor_suite_with};

/// The course change asked for right after arming.
const TURN_DEG: f64 = 200.0;

fn c172_with(seed: u64, calibration: AccelCalibration) -> C172Board {
    let suite = c172_sensor_suite_with(2026.5, seed, calibration);
    c172_board(
        &c172_setup(seed),
        &suite,
        SharedAir::new(StandardAtmosphere),
    )
    .unwrap()
}

/// The lowest height and the steepest bank of two minutes after a
/// [`TURN_DEG`] course change commanded on arming.
fn turn_on_arming(seed: u64, calibration: AccelCalibration) -> (f64, f64) {
    let mut board = c172_with(seed, calibration);
    boot_and_arm(&mut board);
    let mut sequence = 0;
    let probes = fly(
        &mut board,
        &mut sequence,
        120.0,
        TURN_DEG.to_radians(),
        CRUISE_M_S,
        START_HEIGHT_M,
    );
    let lowest = probes
        .iter()
        .map(|p| p.height_m)
        .fold(f64::INFINITY, f64::min);
    let steepest = probes
        .iter()
        .map(|p| p.roll_rad.abs().to_degrees())
        .fold(0.0, f64::max);
    (lowest, steepest)
}

/// Seed 21 draws an accelerometer bias that, uncalibrated, has flown the
/// aeroplane into the ground in a turn like this one. The turn must stay a
/// turn: little height lost and the bank inside the law's limit.
#[test]
#[ignore = "stress case; run it by name"]
fn an_uncalibrated_seed_21_survives_a_large_turn_on_arming() {
    for calibration in [
        AccelCalibration::Uncalibrated20Mg,
        AccelCalibration::Calibrated,
    ] {
        let (lowest, steepest) = turn_on_arming(21, calibration);
        tracing::info!(?calibration, lowest, steepest, "turn on arming");
        assert!(
            lowest > START_HEIGHT_M - 30.0,
            "{calibration:?}: down to {lowest} m"
        );
        assert!(steepest < 35.0, "{calibration:?}: banked {steepest} deg");
    }
}
