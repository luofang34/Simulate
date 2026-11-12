use glam::{DQuat, DVec3};
use navigate_fusion::InertialConfig;
use sim_core::VehicleState;
use sim_devices::{ImuErrorSpec, ImuGrade};
use sim_geo::{BodyEllipsoid, Geodetic, LocalFrame, ecef_to_geodetic};

use super::alignment::body_to_ned;
use super::*;

const DT_S: f64 = 1.0 / 120.0;

fn delaware() -> Geodetic {
    Geodetic {
        lat_rad: 39.0_f64.to_radians(),
        lon_rad: (-75.5_f64).to_radians(),
        height_m: 300.0,
    }
}

#[test]
fn level_flight_points_the_body_along_its_heading_in_ned() {
    let heading = 0.7;
    let state = VehicleState::level_flight(&delaware(), heading, 60.0);
    let frame = LocalFrame::at(&delaware());
    let rotation = body_to_ned(&frame, state.attitude);
    let expected = DQuat::from_rotation_z(heading);
    assert!(rotation.angle_between(expected) < 1e-12, "{rotation:?}");
}

#[test]
fn a_vehicle_at_rest_senses_earth_rate_and_the_ground_reaction() {
    let state = VehicleState::level_flight(&delaware(), 0.0, 0.0);
    let increment = ideal_increment(&BodyEllipsoid::WGS84, &state, &state, DT_S);
    let earth_rate_body = state.attitude.inverse() * DVec3::new(0.0, 0.0, 7.292_115e-5);
    assert!((increment.delta_theta_rad - earth_rate_body * DT_S).length() < 1e-15);
    // Level body axes put the ground reaction along body up, which is -Z.
    let specific_force = increment.delta_velocity_m_s / DT_S;
    assert!(specific_force.x.abs() < 0.02 && specific_force.y.abs() < 0.02);
    assert!((specific_force.z + 9.80).abs() < 0.01, "{specific_force}");
}

#[test]
fn a_body_rotation_relative_to_the_earth_adds_to_earth_rate() {
    let start = VehicleState::level_flight(&delaware(), 1.0, 0.0);
    let rates = DVec3::new(0.3, -0.1, 0.05);
    let end = VehicleState {
        attitude: start.attitude * DQuat::from_scaled_axis(rates * DT_S),
        ..start
    };
    let increment = ideal_increment(&BodyEllipsoid::WGS84, &start, &end, DT_S);
    let earth_rate_body = start.attitude.inverse() * DVec3::new(0.0, 0.0, 7.292_115e-5);
    let expected = (rates + earth_rate_body) * DT_S;
    // The two rotations do not commute; their product differs at second order.
    assert!((increment.delta_theta_rad - expected).length() < 1e-8);
}

/// The simulated world and Navigate's surface filter must share one gravity
/// model on every body: any difference reaches the filter as unmodelled
/// acceleration and grows into dead-reckoning drift.
#[test]
fn the_world_gravity_is_navigates_gravity_model() {
    use navigate_fusion::PlanetaryBody;
    let bodies = [
        (BodyEllipsoid::WGS84, PlanetaryBody::EARTH_WGS84),
        (BodyEllipsoid::MOON_IAU, PlanetaryBody::MOON_PA),
        (BodyEllipsoid::MARS_IAU, PlanetaryBody::MARS_IAU),
    ];
    for (world, filter) in bodies {
        assert_eq!(world.omega_rad_s, filter.rotation_rate_rad_per_s);
        let navigate = filter.central_gravity();
        for (lat, lon, height) in [
            (39.0, -75.5, 300.0),
            (0.0, -30.0, 0.0),
            (89.0, 120.0, 12_000.0),
            (-45.0, 170.0, 400_000.0),
        ] {
            let point = world.geodetic_to_fixed(&Geodetic::from_degrees(lat, lon, height));
            let expected =
                navigate.acceleration(&nalgebra::Vector3::new(point.x, point.y, point.z));
            let difference =
                DVec3::new(expected.x, expected.y, expected.z) - world.gravity_fixed(point);
            assert!(difference.length() < 1e-9, "{world:?} {point} {difference}");
        }
    }
}

#[test]
fn the_navigation_grade_maps_onto_navigates_default_model() {
    let config = inertial_config(&ImuGrade::Navigation.spec());
    let default = InertialConfig::default();
    assert_eq!(config, default);
    let ideal = inertial_config(&ImuErrorSpec::IDEAL);
    assert_eq!(ideal.gyro_bias.sigma(), 0.0);
}

#[test]
fn an_exact_alignment_returns_the_truth_with_the_floor_covariance() {
    let truth = VehicleState::level_flight(&delaware(), 0.3, 50.0);
    let fix = InitialAlignment::EXACT.measure(&truth);
    let position = ecef_to_geodetic(truth.position_ecef);
    assert!((fix.position.latitude_rad - position.lat_rad).abs() < 1e-14);
    assert!((fix.position.altitude_m - position.height_m).abs() < 1e-6);
    let speed = fix.velocity.north_mps.hypot(fix.velocity.east_mps);
    assert!((speed - 50.0).abs() < 1e-9);
    let (xx, _, _) = fix.position_cov.diagonal();
    assert!((xx - 1e-6).abs() < 1e-18);
}

#[test]
fn a_seeded_alignment_misses_by_its_declared_sigma() {
    let truth = VehicleState::level_flight(&delaware(), 0.3, 50.0);
    let frame = LocalFrame::at(&ecef_to_geodetic(truth.position_ecef));
    let mut squares = 0.0;
    let draws = 400_u64;
    for seed in 0..draws {
        let alignment = InitialAlignment {
            position_sigma_m: DVec3::splat(5.0),
            seed,
            ..InitialAlignment::EXACT
        };
        let fix = alignment.measure(&truth);
        let measured = sim_geo::geodetic_to_ecef(&Geodetic {
            lat_rad: fix.position.latitude_rad,
            lon_rad: fix.position.longitude_rad,
            height_m: fix.position.altitude_m,
        });
        let error = frame.ecef_to_ned_vector(measured - truth.position_ecef);
        squares += error.length_squared();
        assert_eq!(fix.position_cov.diagonal(), (25.0, 25.0, 25.0));
    }
    let variance = squares / (3 * draws) as f64;
    assert!((variance / 25.0 - 1.0).abs() < 0.15, "{variance}");
}

#[test]
fn rotation_vectors_keep_their_digits_near_the_identity() {
    for angle in [1e-12, 6.1e-7, 1e-3, 3.0] {
        let axis = DVec3::new(0.3, -0.5, 0.8).normalize();
        let vector = super::rotation_vector(DQuat::from_axis_angle(axis, angle));
        assert!(
            (vector - axis * angle).length() < angle * 1e-12,
            "{angle}: {vector}"
        );
    }
}

/// The consumer gyroscope's residual after a minute at rest, about a
/// thousandth of a degree a second, from its own statistics.
#[test]
fn a_minute_on_the_ground_leaves_the_consumer_gyroscope_a_small_residual() {
    let calibration = GroundCalibration::at_rest(0);
    let spec = ImuGrade::ConsumerMemsCalibrated.spec();
    let sigma_deg_s = calibration
        .residual_variance(&spec.gyro)
        .sqrt()
        .to_degrees();
    assert!((0.0008..0.001).contains(&sigma_deg_s), "{sigma_deg_s}");
    assert!(sigma_deg_s < spec.gyro.turn_on_bias_sigma.to_degrees() / 500.0);
    assert!(
        calibration
            .prior(&ImuErrorSpec::IDEAL, DVec3::ZERO)
            .is_none()
    );
}

#[test]
fn a_ground_calibration_misses_by_its_declared_sigma() {
    let spec = ImuGrade::ConsumerMemsCalibrated.spec();
    let accel_sigma = inertial_config(&spec).accel_bias.sigma();
    let (mut squares, draws) = (0.0, 400_u64);
    let mut declared = 0.0;
    for seed in 0..draws {
        let calibration = GroundCalibration::at_rest(seed);
        let bias = DVec3::new(0.01, -0.004, 0.007);
        let prior = calibration.prior(&spec, bias).unwrap();
        squares += (DVec3::from_array(prior.gyro_bias_body_rad_s) - bias).length_squared();
        declared = prior.gyro_cov_body.diagonal().0;
        assert_eq!(prior.accel_bias_body_mps2, [0.0; 3]);
        assert!(
            (prior.accel_cov_body.diagonal().0 / (accel_sigma * accel_sigma) - 1.0).abs() < 1e-12
        );
    }
    let variance = squares / (3 * draws) as f64;
    assert!(
        (variance / declared - 1.0).abs() < 0.15,
        "{variance} {declared}"
    );
}

/// Unaided at rest for a minute, the consumer part's turn-on bias turns the
/// heading by degrees; the ground calibration's prior holds it to a small
/// fraction of a degree. A sign slip between the unit and the filter would
/// double the turn instead.
#[test]
fn a_ground_calibrated_unit_holds_its_heading_unaided() {
    let truth = VehicleState::level_flight(&delaware(), 0.3, 0.0);
    let spec = ImuGrade::ConsumerMemsCalibrated.spec();
    let heading_after_a_minute = |calibrated: bool| {
        let mut imu = SimulatedImu::new(spec, 11).unwrap();
        let mut fix = InitialAlignment::EXACT.measure(&truth);
        let bias = imu.errors().gyro_bias_rad_s();
        if calibrated {
            let calibration = GroundCalibration::at_rest(3);
            fix.imu_bias_prior = calibration.prior(&spec, bias);
        }
        let mut navigator = InertialNavigator::new(
            inertial_config(&spec),
            crate::MasterClockBridge::default(),
            navigate_contract::SourceId::new(1),
        );
        let period = sim_core::TickSpan::period_of_hz(120).unwrap().ticks_u128();
        let at = |step: u64| sim_core::SimInstant::from_ticks_u128(u128::from(step) * period);
        navigator.align(&fix, at(0)).unwrap();
        imu.sense(at(0), &truth).unwrap();
        let steps = (60.0 / DT_S).round() as u64;
        for step in 1..=steps {
            let increment = imu.sense(at(step), &truth).unwrap().unwrap();
            navigator.ingest(&increment, at(step)).unwrap();
        }
        let solution = navigator.solution(at(steps)).unwrap().unwrap();
        let comparison = crate::compare_attitude_with_truth(&solution, truth.attitude).unwrap();
        (comparison.heading_rad.abs().to_degrees(), bias)
    };
    let (unaided_deg, bias) = heading_after_a_minute(false);
    assert!(bias.z.abs().to_degrees() > 0.1, "{bias}");
    let (calibrated_deg, _) = heading_after_a_minute(true);
    assert!(unaided_deg > 5.0, "{unaided_deg}");
    assert!(
        calibrated_deg < 0.2,
        "{calibrated_deg} against {unaided_deg}"
    );
}
