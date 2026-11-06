use glam::DVec3;
use sim_atmosphere::{AtmosphereField, GeoidStandardAtmosphere};
use sim_core::{SimInstant, VehicleState};
use sim_geo::{Geodetic, LocalFrame, UtcTime};

use super::*;

const STEP_S: f64 = 1.0 / 120.0;

fn flown(realism: SensorRealism, steps: usize) -> OnboardSensorBus {
    let mut bus = OnboardSensorBus::new(realism);
    for _ in 0..steps {
        bus.step(STEP_S);
    }
    bus
}

#[test]
fn realistic_errors_are_nonzero_bounded_and_reproducible() {
    let a = flown(SensorRealism::Realistic, 12_000);
    let b = flown(SensorRealism::Realistic, 12_000);
    assert_eq!(a, b, "same seed and steps must give the same errors");
    let e = a.current();
    let horizontal = e.position_ned_m.truncate().length();
    assert!(horizontal > 0.0 && horizontal < 12.0, "{horizontal}");
    assert!(e.position_ned_m.z.abs() < 15.0);
    assert!(e.attitude_body_rad.length() < 0.012);
    assert!(e.static_pa.abs() < 50.0);
}

#[test]
fn ideal_sensors_read_truth_exactly() {
    let bus = flown(SensorRealism::Ideal, 500);
    assert_eq!(bus.current(), SensorErrors::default());
    let truth = VehicleState::level_flight(&Geodetic::from_degrees(39.1, -75.5, 450.0), 0.3, 55.0);
    assert_eq!(bus.current().estimate(BodyEllipsoid::WGS84, &truth), truth);
}

#[test]
fn estimate_rotates_ned_errors_on_non_earth_ellipsoids() {
    for body in [BodyEllipsoid::MOON_IAU, BodyEllipsoid::MARS_IAU] {
        let site = Geodetic::from_degrees(18.4446, 77.4509, 250.0);
        let local = LocalFrame::at(&site);
        let truth = VehicleState {
            position_ecef: body.geodetic_to_fixed(&site),
            velocity_ecef: local.north() * 12.0,
            attitude: glam::DQuat::from_mat3(&local.ned_to_ecef()),
            angular_rate_body: DVec3::ZERO,
        };
        let err = SensorErrors {
            position_ned_m: DVec3::new(3.0, -4.0, 1.5),
            ..SensorErrors::default()
        };
        let est = err.estimate(body, &truth);
        let delta_ned = local.ecef_to_ned_vector(est.position_ecef - truth.position_ecef);
        assert!((delta_ned - err.position_ned_m).length() < 1e-9);
    }
}

#[test]
fn degraded_navigation_wanders_further_than_realistic() {
    let spread = |realism| {
        let mut bus = OnboardSensorBus::new(realism);
        let mut sum = 0.0;
        for _ in 0..2_000 {
            for _ in 0..60 {
                bus.step(STEP_S);
            }
            sum += bus.current().position_ned_m.truncate().length_squared();
        }
        (sum / 2_000.0).sqrt()
    };
    assert!(spread(SensorRealism::DegradedNavigation) > 4.0 * spread(SensorRealism::Realistic));
}

#[test]
fn display_errors_interpolate_between_steps() {
    let bus = flown(SensorRealism::Realistic, 10);
    assert_eq!(bus.errors_at(0.0), bus.previous);
    assert_eq!(bus.errors_at(1.0), bus.current());
    let mid = bus.errors_at(0.5).position_ned_m;
    assert!(
        (mid - (bus.previous.position_ned_m + bus.current().position_ned_m) / 2.0).length() < 1e-12
    );
}

/// Air data samples the geoid-aware atmosphere (over New Jersey the EGM96 geoid lies about
/// 33 m below the ellipsoid) and measures speed through the air, not over the ground.
#[test]
fn air_data_is_geoid_aware_airspeed_through_the_air_mass() {
    let geo = Geodetic::from_degrees(40.55, -74.60, 300.0);
    let truth = VehicleState::level_flight(&geo, 0.0, 50.0);
    let bus = OnboardSensorBus::new(SensorRealism::Ideal);
    let reading = bus
        .air_data(SimInstant::START, &truth, &bus.current())
        .expect("air data");
    let geoid = GeoidStandardAtmosphere::new()
        .sample(&geo, UtcTime::from_unix_seconds(AIR_DATA_EPOCH_UNIX_S))
        .expect("geoid ISA");
    assert!((reading.value.density_kg_m3 - geoid.density_kg_m3).abs() < 1e-9);
    let pressure = reading.value.static_pressure_pa.expect("static");
    assert!((pressure - geoid.pressure_pa).abs() < 1e-4);
    assert!((reading.value.true_airspeed_m_s - 50.0).abs() < 1e-6);

    let headwind_ned = DVec3::new(-10.0, 0.0, 0.0);
    let windy = OnboardSensorBus {
        atmosphere: FullColumnAtmosphere::with_wind(headwind_ned),
        ..bus
    };
    let into_wind = windy
        .air_data(SimInstant::START, &truth, &bus.current())
        .expect("air data");
    assert!((into_wind.value.true_airspeed_m_s - 60.0).abs() < 1e-3);
    let ground_ned = LocalFrame::at(&geo).ecef_to_ned_vector(truth.velocity_ecef);
    assert!((ground_ned.length() - 50.0).abs() < 1e-6);
}

#[test]
fn static_port_goes_silent_in_near_space() {
    let bus = OnboardSensorBus::new(SensorRealism::Realistic);
    let low = VehicleState::level_flight(&Geodetic::from_degrees(0.0, 0.0, 40_000.0), 0.0, 300.0);
    let high = VehicleState::level_flight(&Geodetic::from_degrees(0.0, 0.0, 60_000.0), 0.0, 300.0);
    let quiet = SensorErrors::default();
    assert!(bus.air_data(SimInstant::START, &low, &quiet).is_some());
    assert!(bus.air_data(SimInstant::START, &high, &quiet).is_none());
}
