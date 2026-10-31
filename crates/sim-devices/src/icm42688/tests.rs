use glam::DQuat;
use mbd_core::{KinematicsFrame, LocalSiteKinematics, PowerRailState};

use super::*;

fn site(specific_force: DVec3, rate: DVec3) -> SiteEnvironment {
    SiteEnvironment::ideal(LocalSiteKinematics {
        frame: KinematicsFrame::Body,
        position_m: DVec3::ZERO,
        velocity_m_s: DVec3::ZERO,
        attitude: DQuat::IDENTITY,
        specific_force_site_m_s2: specific_force,
        angular_rate_site_rad_s: rate,
    })
}

fn at_temperature(temperature_k: f64) -> SiteEnvironment {
    SiteEnvironment {
        temperature_k,
        ..site(DVec3::new(0.0, 0.0, -9.8), DVec3::new(0.1, -0.2, 0.3))
    }
}

/// A part whose noise is off, so a test can reason about exact values.
fn quiet() -> Icm42688Config {
    Icm42688Config {
        accel_noise_rms_m_s2: DVec3::ZERO,
        gyro_noise_rms_rad_s: DVec3::ZERO,
        ..Icm42688Config::nominal(1)
    }
}

fn read(config: Icm42688Config, input: &SiteEnvironment) -> Icm42688Sample {
    let mut block = Icm42688Block::new(config).unwrap();
    block.step(input).unwrap();
    block.output().unwrap()
}

fn vector(value: EngineeringValue) -> DVec3 {
    match value {
        EngineeringValue::Vec3(v) => v,
        other => panic!("{other:?} is not a vector"),
    }
}

fn scalar(value: EngineeringValue) -> f64 {
    match value {
        EngineeringValue::Scalar(v) => v,
        other => panic!("{other:?} is not a scalar"),
    }
}

fn burst(frame: DigitalBusFrame) -> (u8, [u8; 8], u8) {
    match frame {
        DigitalBusFrame::Spi {
            register,
            payload,
            len,
            ..
        } => (register, payload, len),
        other => panic!("{other:?} is not SPI"),
    }
}

fn be(high: u8, low: u8) -> i16 {
    i16::from_be_bytes([high, low])
}

#[test]
fn a_quiet_part_reads_the_site_within_one_count() {
    let input = at_temperature(BIAS_REFERENCE_K);
    let sample = read(quiet(), &input);
    let accel_error = vector(sample.accel_m_s2) - input.kinematics.specific_force_site_m_s2;
    let gyro_error = vector(sample.gyro_rad_s) - input.kinematics.angular_rate_site_rad_s;
    assert!(accel_error.abs().max_element() <= ACCEL_LSB_M_S2 / 2.0 + 1e-12);
    assert!(gyro_error.abs().max_element() <= GYRO_LSB_RAD_S / 2.0 + 1e-12);
    assert!((scalar(sample.temperature_k) - BIAS_REFERENCE_K).abs() < 1e-9);
}

#[test]
fn bias_drifts_linearly_from_the_reference_temperature() {
    let config = Icm42688Config {
        accel_temp_coeff: DVec3::new(0.002, 0.0, 0.0),
        gyro_temp_coeff: DVec3::new(0.0, 0.001, 0.0),
        ..quiet()
    };
    let cold = read(config, &at_temperature(BIAS_REFERENCE_K - 40.0));
    let reference = read(config, &at_temperature(BIAS_REFERENCE_K));
    let hot = read(config, &at_temperature(BIAS_REFERENCE_K + 40.0));

    let tolerance = 1.5 * ACCEL_LSB_M_S2;
    let accel = |s: &Icm42688Sample| vector(s.accel_m_s2);
    assert!((accel(&hot) - accel(&reference) - DVec3::X * 0.08).length() < tolerance);
    assert!((accel(&cold) - accel(&reference) + DVec3::X * 0.08).length() < tolerance);
    let gyro = |s: &Icm42688Sample| vector(s.gyro_rad_s);
    assert!((gyro(&hot) - gyro(&reference) - DVec3::Y * 0.04).length() < 1.5 * GYRO_LSB_RAD_S);
    assert!(scalar(hot.temperature_k) > scalar(cold.temperature_k) + 79.0);
}

#[test]
fn the_part_saturates_at_full_scale() {
    let input = site(
        DVec3::new(1_000.0, -1_000.0, 0.0),
        DVec3::new(100.0, -100.0, 0.0),
    );
    let sample = read(quiet(), &input);
    let accel = vector(sample.accel_m_s2);
    let gyro = vector(sample.gyro_rad_s);
    assert!((accel.x - (ACCEL_FULL_SCALE_M_S2 - ACCEL_LSB_M_S2)).abs() < 1e-9);
    assert!((accel.y + ACCEL_FULL_SCALE_M_S2).abs() < 1e-9);
    assert!((gyro.x - (GYRO_FULL_SCALE_RAD_S - GYRO_LSB_RAD_S)).abs() < 1e-9);
    assert!((gyro.y + GYRO_FULL_SCALE_RAD_S).abs() < 1e-9);
    let (_, payload, _) = burst(sample.spi_bursts[0]);
    assert_eq!(be(payload[2], payload[3]), i16::MAX);
    assert_eq!(be(payload[4], payload[5]), i16::MIN);
}

#[test]
fn outputs_are_quantised_to_whole_counts() {
    let lsb = ACCEL_LSB_M_S2;
    let just_under = site(DVec3::new(0.4 * lsb, 0.6 * lsb, 0.0), DVec3::ZERO);
    let accel = vector(read(quiet(), &just_under).accel_m_s2);
    assert_eq!(accel, DVec3::new(0.0, lsb, 0.0));
}

#[test]
fn spi_bursts_carry_the_same_counts_as_the_engineering_values() {
    let config = Icm42688Config {
        bus_id: 2,
        chip_select: 3,
        ..Icm42688Config::nominal(9)
    };
    let sample = read(config, &at_temperature(310.0));

    let DigitalBusFrame::Spi { bus_id, cs, .. } = sample.spi_bursts[0] else {
        panic!("not SPI");
    };
    assert_eq!((bus_id, cs), (2, 3));
    sample.spi_bursts[0].validate().unwrap();
    sample.spi_bursts[1].validate().unwrap();

    let (register, p, len) = burst(sample.spi_bursts[0]);
    assert_eq!((register, len), (TEMP_DATA1, 8));
    let temperature = f64::from(be(p[0], p[1])) / TEMP_COUNTS_PER_K + TEMP_OFFSET_K;
    assert_eq!(temperature, scalar(sample.temperature_k));
    let accel = DVec3::new(
        be(p[2], p[3]).into(),
        be(p[4], p[5]).into(),
        be(p[6], p[7]).into(),
    );
    assert_eq!(accel * ACCEL_LSB_M_S2, vector(sample.accel_m_s2));

    let (register, p, len) = burst(sample.spi_bursts[1]);
    assert_eq!((register, len), (GYRO_DATA_X1, 6));
    let gyro = DVec3::new(
        be(p[0], p[1]).into(),
        be(p[2], p[3]).into(),
        be(p[4], p[5]).into(),
    );
    assert_eq!(gyro * GYRO_LSB_RAD_S, vector(sample.gyro_rad_s));
}

#[test]
fn a_disabled_rail_silences_the_part_and_recovery_needs_no_restart() {
    let live = at_temperature(300.0);
    let dead = SiteEnvironment {
        power_rail: PowerRailState {
            enabled: false,
            ..PowerRailState::nominal_3v3()
        },
        ..live
    };
    let mut block = Icm42688Block::new(quiet()).unwrap();
    assert_eq!(block.output(), None);
    block.step(&live).unwrap();
    assert!(block.output().is_some());
    block.step(&dead).unwrap();
    assert_eq!(block.output(), None, "no stale sample, no bus frames");
    block.step(&live).unwrap();
    assert!(block.output().is_some());
}

#[test]
fn a_sagging_rail_is_a_brownout_too() {
    let sagging = SiteEnvironment {
        power_rail: PowerRailState::nominal(1.5),
        ..at_temperature(300.0)
    };
    let mut block = Icm42688Block::new(quiet()).unwrap();
    block.step(&sagging).unwrap();
    assert_eq!(block.output(), None);
}

#[test]
fn noise_has_the_configured_spread_and_replays_exactly() {
    let config = Icm42688Config::nominal(77);
    let input = at_temperature(BIAS_REFERENCE_K);
    let mut block = Icm42688Block::new(config).unwrap();
    let mut twin = Icm42688Block::new(config).unwrap();
    let samples = 20_000;
    let mut squares = 0.0;
    for _ in 0..samples {
        block.step(&input).unwrap();
        twin.step(&input).unwrap();
        assert_eq!(block.output(), twin.output());
        let z = vector(block.output().unwrap().accel_m_s2).z;
        let error = z - input.kinematics.specific_force_site_m_s2.z;
        squares += error * error;
    }
    let rms = (squares / f64::from(samples)).sqrt();
    let expected = config.accel_noise_rms_m_s2.z;
    assert!((rms / expected - 1.0).abs() < 0.05, "{rms} vs {expected}");
}

#[test]
fn a_restored_part_repeats_its_noise() {
    let input = at_temperature(300.0);
    let mut block = Icm42688Block::new(Icm42688Config::nominal(3)).unwrap();
    block.step(&input).unwrap();
    let saved = block.snapshot();
    block.step(&input).unwrap();
    let next = block.output();
    block.restore(&saved);
    block.step(&input).unwrap();
    assert_eq!(block.output(), next);
}

#[test]
fn bad_inputs_are_refused() {
    let mut block = Icm42688Block::new(quiet()).unwrap();
    let bad = site(DVec3::NAN, DVec3::ZERO);
    assert!(matches!(
        block.step(&bad),
        Err(BlockError::InputOutOfRange { .. })
    ));
    let stopped = Icm42688Config {
        period: TickSpan::ZERO,
        ..quiet()
    };
    assert!(Icm42688Block::new(stopped).is_err());
}
