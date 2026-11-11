use sim_atmosphere::{AtmosphereField, StandardAtmosphere};
use sim_geo::{Geodetic, UtcTime};

use super::*;
use crate::earth_field_ned_ut;

const CYCLE_S: f64 = 1.0 / 480.0;
const GRAVITY_M_S2: f64 = 9.801;

fn spec(seed: u64) -> SensorSuiteSpec {
    SensorSuiteSpec {
        imu: ImuErrorSpec::IDEAL,
        static_port: PressureErrorSpec::IDEAL,
        pitot: None,
        magnetometer: MagnetometerSpec::IDEAL,
        gnss: GnssErrorSpec::OPEN_SKY,
        magnetic: MagneticEnvironment::main_field(2026.5),
        air_data_every: 10,
        magnetometer_every: 5,
        gnss_every: 48,
        seed,
    }
}

fn at_rest(heading_rad: f64) -> (Geodetic, SensedTruth) {
    let at = Geodetic::from_degrees(39.1582, -75.5244, 15.0);
    let air = StandardAtmosphere
        .sample(&at, UtcTime::from_unix_seconds(0.0))
        .unwrap();
    let truth = SensedTruth {
        vehicle: VehicleState::level_flight(&at, heading_rad, 0.0),
        specific_force_body_m_s2: DVec3::new(0.0, 0.0, -GRAVITY_M_S2),
        angular_rate_body_rad_s: DVec3::ZERO,
        air,
    };
    (at, truth)
}

#[test]
fn each_device_samples_at_its_own_rate() {
    let (_, truth) = at_rest(0.0);
    let mut suite = SensorSuite::new(&spec(1)).unwrap();
    let (mut imu, mut air, mut magnetometer, mut gnss) = (0, 0, 0, 0);
    for _ in 0..480 {
        let samples = suite.sense(&truth, CYCLE_S).unwrap();
        imu += usize::from(samples.imu.is_some());
        air += usize::from(samples.air_data.is_some());
        magnetometer += usize::from(samples.magnetic_field_body_ut.is_some());
        gnss += usize::from(samples.gnss.is_some());
    }
    assert_eq!((imu, air, magnetometer, gnss), (480, 48, 96, 10));
}

#[test]
fn ideal_devices_at_rest_report_the_pad_reaction_the_air_and_the_field() {
    let heading = std::f64::consts::FRAC_PI_2;
    let (at, truth) = at_rest(heading);
    let samples = SensorSuite::new(&spec(1))
        .unwrap()
        .sense(&truth, CYCLE_S)
        .unwrap();
    let imu = samples.imu.unwrap();
    assert_eq!(imu.dt_s, CYCLE_S);
    assert!((imu.delta_velocity_m_s / CYCLE_S - truth.specific_force_body_m_s2).length() < 1e-12);
    assert_eq!(imu.delta_theta_rad, DVec3::ZERO);
    let air = samples.air_data.unwrap();
    assert_eq!(air.static_pressure_pa, truth.air.pressure_pa);
    assert_eq!(air.differential_pressure_pa, None);
    // Nose east: the field's east part lies along the nose, its north part
    // out of the left wing.
    let field = earth_field_ned_ut(&at, 2026.5).unwrap();
    let sensed = samples.magnetic_field_body_ut.unwrap();
    assert!(
        (sensed - DVec3::new(field.y, -field.x, field.z)).length() < 1e-9,
        "{sensed:?}"
    );
    assert!(samples.gnss.is_some());
}

#[test]
fn a_seed_replays_the_suite_and_the_receiver_can_lose_the_sky() {
    let (_, truth) = at_rest(0.3);
    let run = |seed: u64| {
        let mut suite = SensorSuite::new(&spec(seed)).unwrap();
        (0..96)
            .map(|_| suite.sense(&truth, CYCLE_S).unwrap())
            .collect::<Vec<_>>()
    };
    assert_eq!(run(5), run(5));
    assert_ne!(run(5), run(6));

    let mut suite = SensorSuite::new(&spec(5)).unwrap();
    suite.receiver_mut().set_tracking(false);
    let fixes = (0..96)
        .filter(|_| suite.sense(&truth, CYCLE_S).unwrap().gnss.is_some())
        .count();
    assert_eq!(fixes, 0);
}

#[test]
fn a_device_that_never_samples_is_refused() {
    let never = SensorSuiteSpec {
        gnss_every: 0,
        ..spec(1)
    };
    assert!(SensorSuite::new(&never).is_err());
}
