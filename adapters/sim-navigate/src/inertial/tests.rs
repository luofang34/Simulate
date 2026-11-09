use glam::{DQuat, DVec3};
use navigate_fusion::InertialConfig;
use sim_core::VehicleState;
use sim_devices::{ImuErrorSpec, ImuGrade};
use sim_geo::{Geodetic, LocalFrame, ecef_to_geodetic};

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
    let increment = ideal_increment(TruthGravity::WgsJ2, &state, &state, DT_S);
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
    let increment = ideal_increment(TruthGravity::WgsJ2, &start, &end, DT_S);
    let earth_rate_body = start.attitude.inverse() * DVec3::new(0.0, 0.0, 7.292_115e-5);
    let expected = (rates + earth_rate_body) * DT_S;
    // The two rotations do not commute; their product differs at second order.
    assert!((increment.delta_theta_rad - expected).length() < 1e-8);
}

#[test]
fn point_mass_gravity_differs_from_j2_by_the_oblateness_term() {
    let position = sim_geo::geodetic_to_ecef(&delaware());
    let difference = TruthGravity::WgsJ2.attraction_ecef(position)
        - TruthGravity::PointMass.attraction_ecef(position);
    assert!(difference.length() > 5e-3 && difference.length() < 5e-2);
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
