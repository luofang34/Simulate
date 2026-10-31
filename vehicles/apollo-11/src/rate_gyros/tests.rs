use glam::{BVec3, DQuat};
use sim_core::TICKS_PER_SECOND;

use super::*;

const PERIOD: TickSpan = TickSpan::from_ticks(TICKS_PER_SECOND / 100);

fn turning(rate: DVec3) -> InertialSiteSample {
    InertialSiteSample {
        site_to_inertial: DQuat::IDENTITY,
        specific_force_site_m_s2: DVec3::ZERO,
        angular_rate_site_rad_s: rate,
    }
}

#[test]
fn the_reading_carries_the_turn_on_bias() {
    let bias = DVec3::new(1e-3, -2e-3, 3e-3);
    let mut block = RateGyroAssemblyBlock::new(RateGyroConfig::nominal(PERIOD, bias)).unwrap();
    assert_eq!(block.output().rate_rad_s, DVec3::ZERO);
    block.step(&turning(DVec3::new(0.1, 0.0, -0.1))).unwrap();
    assert_eq!(block.output().rate_rad_s, DVec3::new(0.1, 0.0, -0.1) + bias);
    assert_eq!(block.output().saturated, BVec3::FALSE);
}

#[test]
fn each_axis_pins_at_its_range_on_its_own() {
    let mut block =
        RateGyroAssemblyBlock::new(RateGyroConfig::nominal(PERIOD, DVec3::ZERO)).unwrap();
    let range = NOMINAL_RANGE_RAD_S;
    block.step(&turning(DVec3::new(2.0, -2.0, 0.1))).unwrap();
    let output = block.output();
    assert_eq!(output.rate_rad_s, DVec3::new(range, -range, 0.1));
    assert_eq!(output.saturated, BVec3::new(true, true, false));
}

#[test]
fn a_snapshot_restores_and_bad_setup_is_rejected() {
    let mut block =
        RateGyroAssemblyBlock::new(RateGyroConfig::nominal(PERIOD, DVec3::ZERO)).unwrap();
    block.step(&turning(DVec3::X)).unwrap();
    let saved = block.snapshot();
    block.step(&turning(DVec3::ZERO)).unwrap();
    block.restore(&saved);
    assert_eq!(block.output(), saved);
    assert!(block.step(&turning(DVec3::INFINITY)).is_err());
    assert!(
        RateGyroAssemblyBlock::new(RateGyroConfig {
            range_rad_s: -1.0,
            ..RateGyroConfig::nominal(PERIOD, DVec3::ZERO)
        })
        .is_err()
    );
}
