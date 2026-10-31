use std::f64::consts::TAU;

use super::*;
use sim_core::TICKS_PER_SECOND;

const PERIOD: TickSpan = TickSpan::from_ticks(TICKS_PER_SECOND / 50);
const DT_S: f64 = 0.02;

fn still(attitude: DQuat) -> InertialSiteSample {
    InertialSiteSample {
        site_to_inertial: attitude,
        specific_force_site_m_s2: DVec3::ZERO,
        angular_rate_site_rad_s: DVec3::ZERO,
    }
}

fn perfect() -> ThreeGimbalImuConfig {
    ThreeGimbalImuConfig {
        gyro_drift_rad_s: DVec3::ZERO,
        ..ThreeGimbalImuConfig::nominal(PERIOD, DQuat::IDENTITY)
    }
}

fn imu(config: ThreeGimbalImuConfig) -> ThreeGimbalImuBlock {
    ThreeGimbalImuBlock::new(config).unwrap()
}

fn yawed(degrees: f64) -> DQuat {
    DQuat::from_rotation_z(degrees.to_radians())
}

#[test]
fn gimbal_angles_round_trip_away_from_lock() {
    for (oga, mga, iga) in [(0.3, 0.2, -0.4), (-2.0, -1.0, 2.5), (3.0, 1.2, 0.1)] {
        let angles = GimbalAngles {
            oga_rad: oga,
            mga_rad: mga,
            iga_rad: iga,
        };
        let solved = GimbalAngles::of_navbase_to_platform(angles.navbase_to_platform());
        assert!((solved.oga_rad - oga).abs() < 1e-12, "{solved:?}");
        assert!((solved.mga_rad - mga).abs() < 1e-12, "{solved:?}");
        assert!((solved.iga_rad - iga).abs() < 1e-12, "{solved:?}");
    }
}

#[test]
fn each_vehicle_axis_drives_its_own_gimbal() {
    let solve = GimbalAngles::of_navbase_to_platform;
    assert!((solve(DQuat::from_rotation_x(0.5)).oga_rad - 0.5).abs() < 1e-12);
    assert!((solve(DQuat::from_rotation_y(0.5)).iga_rad - 0.5).abs() < 1e-12);
    assert!((solve(DQuat::from_rotation_z(0.5)).mga_rad - 0.5).abs() < 1e-12);
}

#[test]
fn the_warning_follows_the_angle_and_the_capture_latches() {
    let mut block = imu(perfect());
    block.step(&still(yawed(69.9))).unwrap();
    assert!(!block.output().gimbal_lock_warning);
    block.step(&still(yawed(-70.1))).unwrap();
    assert!(block.output().gimbal_lock_warning);
    block.step(&still(yawed(10.0))).unwrap();
    assert!(!block.output().gimbal_lock_warning, "level-triggered");
    assert!(!block.is_gimbal_locked());

    block.step(&still(yawed(84.9))).unwrap();
    assert!(!block.is_gimbal_locked());
    block.step(&still(yawed(85.1))).unwrap();
    assert!(block.is_gimbal_locked());
    assert!(!block.output().platform_valid);
    block.step(&still(yawed(0.0))).unwrap();
    assert!(block.is_gimbal_locked(), "latched");
}

#[test]
fn a_captured_platform_tumbles_even_on_a_still_vehicle() {
    let mut block = imu(perfect());
    let base = yawed(86.0);
    block.step(&still(base)).unwrap();
    let captured = block.output().platform_to_inertial;
    for _ in 0..50 {
        block.step(&still(base)).unwrap();
    }
    let drift = block.output().platform_to_inertial.angle_between(captured);
    let expected = CAPTURE_TUMBLE_RATE_RAD_S.length();
    assert!((drift - expected).abs() < 1e-6, "{drift}");
}

#[test]
fn caging_zeroes_the_gimbals_but_leaves_the_computer_wrong() {
    let mut block = imu(perfect());
    let base = yawed(86.0);
    block.step(&still(base)).unwrap();
    block.cage_to_zero();
    let caged = block.output();
    assert!(!caged.gimbal_locked && caged.platform_valid);
    assert!(caged.mga_rad.abs() < 1e-12);
    assert_eq!(caged.platform_to_inertial, base);
    assert_eq!(caged.estimated_platform_to_inertial, DQuat::IDENTITY);
    assert!(caged.indicated_navbase_to_inertial.angle_between(base) > 1.0);
}

#[test]
fn a_coarse_align_moves_the_platform_and_the_belief() {
    let mut block = imu(perfect());
    block.step(&still(yawed(86.0))).unwrap();
    let target = DQuat::from_rotation_y(0.3);
    block.coarse_align(target);
    let aligned = block.output();
    assert!(!aligned.gimbal_locked);
    assert_eq!(aligned.platform_to_inertial, target);
    assert_eq!(aligned.estimated_platform_to_inertial, target);
    assert!(
        aligned
            .indicated_navbase_to_inertial
            .angle_between(yawed(86.0))
            < 1e-12
    );
}

#[test]
fn a_fine_align_corrects_the_belief_without_moving_the_platform() {
    let mut block = imu(perfect());
    let platform = block.output().platform_to_inertial;
    block.fine_align(DQuat::from_rotation_x(0.2));
    let output = block.output();
    assert_eq!(output.platform_to_inertial, platform);
    assert_eq!(
        output.estimated_platform_to_inertial,
        DQuat::from_rotation_x(0.2)
    );
    assert!((output.indicated_navbase_to_inertial.angle_between(platform) - 0.2).abs() < 1e-12);
}

#[test]
fn drift_opens_an_attitude_error_at_the_configured_rate() {
    let rate = DVec3::new(0.0, 1e-3, 0.0);
    let mut block = imu(ThreeGimbalImuConfig {
        gyro_drift_rad_s: rate,
        ..perfect()
    });
    for _ in 0..500 {
        block.step(&still(DQuat::IDENTITY)).unwrap();
    }
    let error = block
        .output()
        .indicated_navbase_to_inertial
        .angle_between(DQuat::IDENTITY);
    assert!((error - 1e-2).abs() < 1e-9, "{error}");
}

#[test]
fn pipas_count_whole_pulses_in_stable_member_axes_and_keep_the_remainder() {
    let mut block = imu(perfect());
    let base = yawed(90.0 - 45.0);
    let input = InertialSiteSample {
        specific_force_site_m_s2: DVec3::new(1.0, 0.0, -0.1234),
        ..still(base)
    };
    for _ in 0..250 {
        block.step(&input).unwrap();
    }
    let in_platform = base * input.specific_force_site_m_s2 * (250.0 * DT_S);
    let [x, y, z] = block.output().pipa_counts;
    let expected = (in_platform / PIPA_PULSE_LM_M_S).trunc();
    assert_eq!(x as f64, expected.x);
    assert_eq!(y as f64, expected.y);
    assert_eq!(z as f64, expected.z);
    assert!(z < 0, "negative pulses count down");

    let before = block.snapshot();
    block.step(&still(base)).unwrap();
    assert_eq!(block.output().pipa_counts, before.output.pipa_counts);
}

#[test]
fn cdu_counts_encode_fifteen_bits_in_agc_axis_order() {
    let mut block = imu(perfect());
    block
        .step(&still(DQuat::from_rotation_x(TAU / 4.0)))
        .unwrap();
    assert_eq!(block.output().cdu_counts, [8192, 0, 0]);
    block
        .step(&still(DQuat::from_rotation_y(-TAU / 4.0)))
        .unwrap();
    assert_eq!(block.output().cdu_counts, [0, 24_576, 0]);
    block
        .step(&still(DQuat::from_rotation_z(TAU / 16.0)))
        .unwrap();
    assert_eq!(block.output().cdu_counts, [0, 0, 2048]);
}

#[test]
fn cdu_pulses_take_the_short_way_across_the_wrap() {
    let frames = pulses([32_760, 5, 100], [8, 32_760, 100]);
    let deltas = frames.map(|frame| match frame {
        DigitalBusFrame::AgcCduPulses { delta_pulses, .. } => delta_pulses,
        other => panic!("{other:?}"),
    });
    assert_eq!(deltas, [16, -13, 0]);
    for frame in frames {
        frame.validate().unwrap();
    }
}

#[test]
fn resolvers_decode_to_the_gimbal_angles() {
    let mut block = imu(perfect());
    block
        .step(&still(
            GimbalAngles {
                oga_rad: 0.4,
                mga_rad: 0.3,
                iga_rad: -0.2,
            }
            .navbase_to_platform(),
        ))
        .unwrap();
    let [outer, inner, middle] = block.output().resolvers;
    assert!((outer.decode_angle_rad().unwrap() - 0.4).abs() < 1e-12);
    assert!((inner.decode_angle_rad().unwrap() + 0.2).abs() < 1e-12);
    assert!((middle.decode_angle_rad().unwrap() - 0.3).abs() < 1e-12);
}

#[test]
fn the_lamp_lights_with_the_warning() {
    let mut block = imu(perfect());
    let mut port = CrewStationPort::new();
    block.step(&still(yawed(75.0))).unwrap();
    block.publish_lamp(SimInstant::START, &mut port).unwrap();
    assert_eq!(
        port.latest_observations().first().unwrap().observation,
        CrewObservation::AnnunciatorLamp {
            lamp_id: GIMBAL_LOCK_LAMP,
            lit: true
        }
    );
}

#[test]
fn a_restored_block_replays_identically() {
    let mut block = imu(ThreeGimbalImuConfig::nominal(PERIOD, DQuat::IDENTITY));
    let input = InertialSiteSample {
        specific_force_site_m_s2: DVec3::new(0.3, 0.2, 0.1),
        ..still(yawed(30.0))
    };
    block.step(&input).unwrap();
    let saved = block.snapshot();
    block.step(&input).unwrap();
    let expected = block.output();
    block.restore(&saved);
    block.step(&input).unwrap();
    assert_eq!(block.output(), expected);
}

#[test]
fn bad_setup_and_inputs_are_rejected() {
    assert!(
        ThreeGimbalImuBlock::new(ThreeGimbalImuConfig {
            period: TickSpan::ZERO,
            ..perfect()
        })
        .is_err()
    );
    assert!(
        ThreeGimbalImuBlock::new(ThreeGimbalImuConfig {
            pipa_m_s_per_pulse: 0.0,
            ..perfect()
        })
        .is_err()
    );
    let mut block = imu(perfect());
    let saved = block.snapshot();
    let bad = InertialSiteSample {
        specific_force_site_m_s2: DVec3::NAN,
        ..still(DQuat::IDENTITY)
    };
    assert!(block.step(&bad).is_err());
    assert_eq!(block.snapshot(), saved);
}
