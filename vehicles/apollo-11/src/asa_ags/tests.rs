use sim_core::TICKS_PER_SECOND;

use super::*;

const PERIOD: TickSpan = TickSpan::from_ticks(TICKS_PER_SECOND / 100);

fn perfect() -> StrapdownAsaConfig {
    StrapdownAsaConfig {
        gyro_bias_rad_s: DVec3::ZERO,
        ..StrapdownAsaConfig::nominal(PERIOD, DQuat::IDENTITY)
    }
}

fn turning(rate: DVec3) -> InertialSiteSample {
    InertialSiteSample {
        site_to_inertial: DQuat::IDENTITY,
        specific_force_site_m_s2: DVec3::ZERO,
        angular_rate_site_rad_s: rate,
    }
}

#[test]
fn attitude_integrates_through_the_orientation_that_locks_a_gimballed_platform() {
    let mut block = StrapdownAsaBlock::new(perfect()).unwrap();
    let rate = DVec3::new(0.0, 0.0, 1.0);
    for _ in 0..200 {
        block.step(&turning(rate)).unwrap();
    }
    let expected = DQuat::from_rotation_z(2.0);
    assert!(block.output().site_to_inertial.angle_between(expected) < 1e-9);
}

#[test]
fn velocity_change_accumulates_in_inertial_axes() {
    let mut block = StrapdownAsaBlock::new(StrapdownAsaConfig {
        initial_site_to_inertial: DQuat::from_rotation_z(std::f64::consts::FRAC_PI_2),
        ..perfect()
    })
    .unwrap();
    let thrust = InertialSiteSample {
        specific_force_site_m_s2: DVec3::new(2.0, 0.0, 0.0),
        ..turning(DVec3::ZERO)
    };
    for _ in 0..100 {
        block.step(&thrust).unwrap();
    }
    let dv = block.output().delta_velocity_inertial_m_s;
    assert!((dv - DVec3::new(0.0, 2.0, 0.0)).length() < 1e-9, "{dv}");
}

#[test]
fn its_nominal_drift_is_worse_than_the_platforms() {
    let period = PERIOD;
    let mut asa =
        StrapdownAsaBlock::new(StrapdownAsaConfig::nominal(period, DQuat::IDENTITY)).unwrap();
    let mut imu = crate::ThreeGimbalImuBlock::new(crate::ThreeGimbalImuConfig::nominal(
        period,
        DQuat::IDENTITY,
    ))
    .unwrap();
    for _ in 0..1_000 {
        asa.step(&turning(DVec3::ZERO)).unwrap();
        imu.step(&turning(DVec3::ZERO)).unwrap();
    }
    let asa_error = asa.output().site_to_inertial.angle_between(DQuat::IDENTITY);
    let imu_error = imu
        .output()
        .indicated_navbase_to_inertial
        .angle_between(DQuat::IDENTITY);
    assert!(asa_error > 2.0 * imu_error, "{asa_error} vs {imu_error}");
}

#[test]
fn a_restored_block_replays_identically_and_bad_input_changes_nothing() {
    let mut block = StrapdownAsaBlock::new(perfect()).unwrap();
    let input = turning(DVec3::new(0.1, 0.2, 0.3));
    block.step(&input).unwrap();
    let saved = block.snapshot();
    block.step(&input).unwrap();
    let expected = block.output();
    block.restore(&saved);
    assert!(block.step(&turning(DVec3::NAN)).is_err());
    assert_eq!(block.snapshot(), saved);
    block.step(&input).unwrap();
    assert_eq!(block.output(), expected);
    assert!(
        StrapdownAsaBlock::new(StrapdownAsaConfig {
            period: TickSpan::ZERO,
            ..perfect()
        })
        .is_err()
    );
}
