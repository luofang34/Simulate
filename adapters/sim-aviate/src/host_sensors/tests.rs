use glam::DVec3;
use sim_atmosphere::{AtmosphereField, StandardAtmosphere};
use sim_core::{SimInstant, TICKS_PER_SECOND, TickSpan, VehicleState};
use sim_devices::{
    AirDataSample, GaussMarkov, GnssErrorSpec, ImuGrade, InertialIncrement, MagneticEnvironment,
    MagnetometerSpec, PressureErrorSpec, SensedTruth, SensorSuite, SensorSuiteSpec,
};
use sim_geo::{BodyEllipsoid, Geodetic, UtcTime, ecef_to_geodetic, geodetic_to_ecef};

use super::*;
use crate::{AviateDirectConfig, AviateDirectRunner, build_direct_x500_kernel};

const RATE_HZ: u64 = 480;

fn origin() -> Geodetic {
    Geodetic::from_degrees(39.1582, -75.5244, 15.0)
}

#[test]
fn device_samples_become_hal_readings_in_the_boards_units_and_frame() {
    let world = NedWorld::new(&BodyEllipsoid::WGS84, &origin());
    let north_ecef =
        geodetic_to_ecef(&origin()) + world.frame().ned_to_ecef() * DVec3::new(100.0, -40.0, -20.0);
    let dt_s = 1.0 / RATE_HZ as f64;
    let samples = DeviceSamples {
        imu: Some(InertialIncrement {
            dt_s,
            delta_theta_rad: DVec3::new(0.1, -0.2, 0.3) * dt_s,
            delta_velocity_m_s: DVec3::new(0.5, 0.0, -9.8) * dt_s,
        }),
        air_data: Some(AirDataSample {
            static_pressure_pa: 100_900.0,
            static_temperature_k: 288.15,
            differential_pressure_pa: Some(1_850.0),
        }),
        magnetic_field_body_ut: Some(DVec3::new(20.0, -4.0, 47.0)),
        gnss: Some(sim_devices::GnssReading {
            position: ecef_to_geodetic(north_ecef),
            velocity_ned_m_s: DVec3::new(3.0, 4.0, -1.0),
            position_sigma_ned_m: DVec3::new(1.2, 1.6, 2.5),
            velocity_sigma_m_s: 0.05,
        }),
    };
    let host = world.host_sensors(2_500_000, 7, &samples);
    assert_eq!((host.time_us, host.reset_generation), (2_500_000, 7));
    let imu = host.imu.unwrap();
    assert_eq!(imu.timestamp_us, 2_500_000);
    assert!((imu.reading.accel[2] + 9.8).abs() < 1e-5 && (imu.reading.accel[0] - 0.5).abs() < 1e-6);
    assert!((imu.reading.gyro[1] + 0.2).abs() < 1e-6);
    let baro = host.baro.unwrap().reading;
    assert!(
        (baro.pressure_pa - 100_900.0).abs() < 0.01 && (baro.temperature_c - 15.0).abs() < 1e-3
    );
    let airspeed = host.airspeed.unwrap().reading;
    assert_eq!(airspeed.differential_pressure_pa, 1_850.0);
    assert_eq!(airspeed.static_pressure_pa, Some(100_900.0));
    assert_eq!(host.mag.unwrap().reading.field_ut, [20.0, -4.0, 47.0]);
    let gnss = host.gnss.unwrap().reading;
    for (got, want) in gnss.position_ned.iter().zip([100.0, -40.0, -20.0]) {
        assert!((got - want).abs() < 0.01, "{:?}", gnss.position_ned);
    }
    assert_eq!(gnss.vel_ned, [3.0, 4.0, -1.0]);
    assert!(
        (gnss.alt_m - 35.0).abs() < 0.01,
        "ellipsoidal height {}",
        gnss.alt_m
    );
    assert_eq!(gnss.fix, GnssFix::ThreeD);
    assert!((gnss.h_acc - 2.0).abs() < 1e-5 && (gnss.v_acc - 2.5).abs() < 1e-6);
    assert!(host.geomagnetic.is_none());

    let silent = world.host_sensors(2_502_083, 7, &DeviceSamples::default());
    assert!(silent.imu.is_none() && silent.baro.is_none() && silent.gnss.is_none());
    assert!(silent.mag.is_none() && silent.airspeed.is_none());
}

fn suite() -> SensorSuite {
    SensorSuite::new(&SensorSuiteSpec {
        imu: ImuGrade::ConsumerMems.spec(),
        static_port: PressureErrorSpec {
            white_sigma_pa: 3.0,
            bias: GaussMarkov {
                tau_s: 600.0,
                sigma: 20.0,
            },
        },
        pitot: None,
        magnetometer: MagnetometerSpec {
            white_sigma_ut: 0.2,
            hard_iron_sigma_ut: 0.5,
            scale_factor_sigma: 0.0,
        },
        gnss: GnssErrorSpec::OPEN_SKY,
        magnetic: MagneticEnvironment::main_field(2026.5),
        air_data_every: 10,
        magnetometer_every: 5,
        gnss_every: 48,
        seed: 17,
    })
    .unwrap()
}

/// A flight controller that is given device samples and nothing else comes
/// up on a vehicle at rest and places it where it is, level, on its heading.
#[test]
fn a_board_fed_by_the_devices_alone_finds_the_vehicle_at_rest() {
    let heading_rad = 0.6;
    let truth_state = VehicleState::level_flight(&origin(), heading_rad, 0.0);
    let truth = SensedTruth {
        vehicle: truth_state,
        specific_force_body_m_s2: DVec3::new(0.0, 0.0, -9.801),
        angular_rate_body_rad_s: DVec3::ZERO,
        air: StandardAtmosphere
            .sample(&origin(), UtcTime::from_unix_seconds(0.0))
            .unwrap(),
    };
    let period = TickSpan::from_ticks(TICKS_PER_SECOND / RATE_HZ);
    let mut suite = suite();
    let mut runner = AviateDirectRunner::new(
        build_direct_x500_kernel().unwrap(),
        AviateDirectConfig::ideal(origin()),
    )
    .unwrap();
    let mut now = SimInstant::START;
    for _ in 0..RATE_HZ * 8 {
        let samples = suite.sense(&truth, period.as_seconds()).unwrap();
        let command = runner.step_sensors(now, &samples).unwrap();
        assert!(!command.armed);
        now = now.advanced(period);
    }
    assert!(
        runner.is_ready(),
        "the flight controller never became ready"
    );
    let estimate = runner.estimated_state().expect("a usable estimate");
    let frame = *NedWorld::new(&BodyEllipsoid::WGS84, &origin()).frame();
    let error_ned = frame.ecef_to_ned_vector(estimate.position_ecef - truth_state.position_ecef);
    assert!(
        error_ned.x.hypot(error_ned.y) < 4.0,
        "horizontal error {error_ned:?}"
    );
    assert!(error_ned.z.abs() < 5.0, "height error {} m", error_ned.z);
    let attitude_error = estimate.attitude.angle_between(truth_state.attitude);
    assert!(
        attitude_error < 4_f64.to_radians(),
        "attitude error {attitude_error} rad"
    );
    assert!(
        estimate.velocity_ecef.length() < 0.5,
        "{:?}",
        estimate.velocity_ecef
    );
}
