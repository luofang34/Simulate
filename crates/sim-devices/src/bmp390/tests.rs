use glam::{DQuat, DVec3};
use mbd_core::{KinematicsFrame, LocalSiteKinematics, PowerRailState};

use super::*;

fn site(temperature_k: f64) -> SiteEnvironment {
    SiteEnvironment {
        temperature_k,
        ..SiteEnvironment::ideal(LocalSiteKinematics {
            frame: KinematicsFrame::Body,
            position_m: DVec3::ZERO,
            velocity_m_s: DVec3::ZERO,
            attitude: DQuat::IDENTITY,
            specific_force_site_m_s2: DVec3::ZERO,
            angular_rate_site_rad_s: DVec3::ZERO,
        })
    }
}

fn read(static_pressure_pa: f64, temperature_k: f64) -> Bmp390Sample {
    let mut block = Bmp390Block::new(Bmp390Config::default()).unwrap();
    block
        .step(&Bmp390Input {
            static_pressure_pa,
            site: site(temperature_k),
        })
        .unwrap();
    block.output().unwrap()
}

fn scalar(value: EngineeringValue) -> f64 {
    match value {
        EngineeringValue::Scalar(v) => v,
        other => panic!("{other:?} is not a scalar"),
    }
}

#[test]
fn readings_are_quantised_to_the_finest_resolution() {
    let sample = read(101_325.004, 293.151);
    let pressure = scalar(sample.pressure_pa);
    let temperature = scalar(sample.temperature_k);
    assert!((pressure - 101_325.004).abs() <= PRESSURE_LSB_PA / 2.0 + 1e-9);
    assert!((temperature - 293.151).abs() <= TEMPERATURE_LSB_K / 2.0 + 1e-9);
    assert!((pressure / PRESSURE_LSB_PA).fract().abs() < 1e-6);
}

#[test]
fn the_i2c_burst_holds_the_same_counts_little_endian() {
    let sample = read(90_000.0, 300.0);
    sample.i2c_burst.validate().unwrap();
    let DigitalBusFrame::I2c {
        addr_7bit,
        register,
        payload: [p0, p1, p2, t0, t1, t2, ..],
        len,
        ..
    } = sample.i2c_burst
    else {
        panic!("not I2C");
    };
    assert_eq!((addr_7bit, register, len), (BMP390_ADDRESS, DATA_0, 6));
    let pressure = u32::from_le_bytes([p0, p1, p2, 0]);
    let temperature = u32::from_le_bytes([t0, t1, t2, 0]);
    assert_eq!(
        f64::from(pressure) * PRESSURE_LSB_PA,
        scalar(sample.pressure_pa)
    );
    assert_eq!(
        f64::from(temperature) * TEMPERATURE_LSB_K,
        scalar(sample.temperature_k)
    );
}

#[test]
fn inputs_outside_the_operating_range_pin_to_its_limits() {
    let high = read(200_000.0, 500.0);
    let low = read(1_000.0, 100.0);
    assert!((scalar(high.pressure_pa) - 125_000.0).abs() < PRESSURE_LSB_PA);
    assert!((scalar(low.pressure_pa) - 30_000.0).abs() < PRESSURE_LSB_PA);
    assert!((scalar(high.temperature_k) - 358.15).abs() < TEMPERATURE_LSB_K);
    assert!((scalar(low.temperature_k) - 233.15).abs() < TEMPERATURE_LSB_K);
}

#[test]
fn a_dead_rail_outputs_nothing() {
    let mut block = Bmp390Block::new(Bmp390Config::default()).unwrap();
    let mut input = Bmp390Input {
        static_pressure_pa: 100_000.0,
        site: site(300.0),
    };
    block.step(&input).unwrap();
    assert!(block.output().is_some());
    input.site.power_rail = PowerRailState {
        enabled: false,
        ..PowerRailState::nominal_3v3()
    };
    block.step(&input).unwrap();
    assert_eq!(block.output(), None);
}

#[test]
fn bad_setups_and_inputs_are_refused_and_snapshots_round_trip() {
    let bad = Bmp390Config {
        address: 0x80,
        ..Bmp390Config::default()
    };
    assert!(Bmp390Block::new(bad).is_err());

    let mut block = Bmp390Block::new(Bmp390Config::default()).unwrap();
    let nan = Bmp390Input {
        static_pressure_pa: f64::NAN,
        site: site(300.0),
    };
    assert!(block.step(&nan).is_err());

    let good = Bmp390Input {
        static_pressure_pa: 95_000.0,
        site: site(300.0),
    };
    block.step(&good).unwrap();
    let saved = block.snapshot();
    let mut twin = Bmp390Block::new(Bmp390Config::default()).unwrap();
    twin.restore(&saved);
    assert_eq!(twin.output(), block.output());
}
