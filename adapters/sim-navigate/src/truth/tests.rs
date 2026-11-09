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
