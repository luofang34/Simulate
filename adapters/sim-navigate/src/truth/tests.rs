use navigate_contract::{
    ClockDomainId, Ellipsoid, FaultDetection, IntegrityAssessment, MonotonicNanos, NedVelocity,
    Redundancy, SensorClass, SolutionQuality, SolutionStamp, SourceComposition, SourceEpoch,
    SymmetricCov3, WrappingSequence,
};
use sim_geo::{Geodetic, LocalFrame, geodetic_to_ecef};

use super::*;

fn solution(position: &Geodetic, sigma_m: f64) -> NavigationSolution {
    let contributing = SourceComposition::of(SensorClass::VisualLandmark);
    NavigationSolution::new(
        SolutionStamp::new(
            SourceEpoch::new(0),
            WrappingSequence::new(0),
            MonotonicNanos::from_nanos(0),
            ClockDomainId::new(1),
        ),
        GeodeticPosition::new(position.lat_rad, position.lon_rad, position.height_m),
        NedVelocity::new(0.0, 0.0, 0.0),
        SymmetricCov3::from_diagonal(sigma_m * sigma_m, sigma_m * sigma_m, sigma_m * sigma_m),
        SymmetricCov3::from_diagonal(1.0, 1.0, 1.0),
        IntegrityAssessment::new(
            SolutionQuality::Good,
            contributing,
            Redundancy::None,
            sigma_m,
            sigma_m,
            FaultDetection::Unavailable,
        ),
        contributing,
        Ellipsoid::WGS84,
    )
}

#[test]
fn errors_and_nees_come_out_in_metres_and_sigmas() {
    let position = Geodetic::from_degrees(39.1, -75.6, 900.0);
    let frame = LocalFrame::at(&position);
    let truth =
        geodetic_to_ecef(&position) + frame.north() * 6.0 + frame.east() * 8.0 - frame.up() * 2.0;
    let comparison = compare_with_truth(&solution(&position, 2.0), truth).unwrap();
    assert!((comparison.horizontal_error_m - 10.0).abs() < 1e-3);
    assert!((comparison.vertical_error_m - 2.0).abs() < 1e-3);
    assert!((comparison.position_nees - 26.0).abs() < 1e-2);
}

#[test]
fn a_solution_returns_to_its_ecef_position() {
    let position = Geodetic::from_degrees(39.1, -75.6, 900.0);
    let ecef = solution_position_ecef(&solution(&position, 2.0));
    assert!((ecef - geodetic_to_ecef(&position)).length() < 1e-6);
}

#[test]
fn a_covariance_with_no_cholesky_factor_is_refused() {
    let position = Geodetic::from_degrees(39.1, -75.6, 900.0);
    let truth = geodetic_to_ecef(&position);
    assert!(matches!(
        compare_with_truth(&solution(&position, 0.0), truth),
        Err(NavigateBridgeError::SolutionCovariance)
    ));
}

#[test]
fn attitude_errors_split_into_tilt_and_heading_in_sigmas() {
    let position = Geodetic::from_degrees(39.1, -75.6, 900.0);
    let heading = 0.6;
    let truth_state = sim_core::VehicleState::level_flight(&position, heading, 50.0);
    // The estimate is the truth turned 2 mrad in heading and 1 mrad in roll.
    let estimate_ned =
        glam::DQuat::from_rotation_z(heading - 2e-3) * glam::DQuat::from_rotation_x(1e-3);
    let solution = solution(&position, 2.0)
        .with_attitude(navigate_contract::AttitudeQuaternion::new(
            estimate_ned.w,
            estimate_ned.x,
            estimate_ned.y,
            estimate_ned.z,
        ))
        .with_attitude_cov(SymmetricCov3::from_diagonal(1e-6, 1e-6, 1e-6));
    let comparison = compare_attitude_with_truth(&solution, truth_state.attitude).unwrap();
    assert!(
        (comparison.heading_rad - 2e-3).abs() < 1e-6,
        "{comparison:?}"
    );
    assert!((comparison.tilt_rad - 1e-3).abs() < 1e-6, "{comparison:?}");
    assert!(
        (comparison.attitude_nees - 5.0).abs() < 1e-2,
        "{comparison:?}"
    );
    assert!(matches!(
        compare_attitude_with_truth(&solution_without_attitude(&position), truth_state.attitude),
        Err(NavigateBridgeError::SolutionAttitudeMissing)
    ));
}

fn solution_without_attitude(position: &Geodetic) -> NavigationSolution {
    solution(position, 2.0)
}
