use glam::{DQuat, DVec3};
use nalgebra::{Matrix3, Vector3};
use navigate_contract::{GeodeticPosition, NavigationSolution, SymmetricCov3};
use navigate_geodesy::{BodyEllipsoid, LocalTangentPlane};
use sim_geo::{LocalFrame, ecef_to_geodetic, geodetic_to_ecef};

use crate::NavigateBridgeError;
use crate::inertial::rotation_vector;

#[cfg(test)]
mod tests;

/// A solution scored against simulation truth.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TruthComparison {
    /// Horizontal position error, metres.
    pub horizontal_error_m: f64,
    /// Vertical position error, metres.
    pub vertical_error_m: f64,
    /// Position normalised estimation error squared, 3 degrees of freedom:
    /// consistent solutions average 3, and overconfident ones run far above.
    pub position_nees: f64,
}

/// A solution's attitude scored against the true attitude.
///
/// The error is the rotation taking the estimate onto the truth, as a rotation
/// vector in the solution's local north-east-down axes; Navigate's body-axis
/// right perturbation maps onto it through the estimated attitude, which is
/// how the published attitude covariance is resolved too.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AttitudeComparison {
    /// Total attitude error, radians.
    pub error_rad: f64,
    /// Error about the horizontal axes, the tilt, radians.
    pub tilt_rad: f64,
    /// Error about the local vertical, the heading, radians.
    pub heading_rad: f64,
    /// Attitude normalised estimation error squared, 3 degrees of freedom.
    pub attitude_nees: f64,
}

/// Scores `solution`'s attitude against `truth_attitude`, which rotates body
/// axes into ECEF.
pub fn compare_attitude_with_truth(
    solution: &NavigationSolution,
    truth_attitude: DQuat,
) -> Result<AttitudeComparison, NavigateBridgeError> {
    let (Some(attitude), Some(covariance)) = (solution.attitude, solution.attitude_cov) else {
        return Err(NavigateBridgeError::SolutionAttitudeMissing);
    };
    let frame = LocalFrame::at(&ecef_to_geodetic(solution_position_ecef(solution)));
    let ned_from_ecef = DQuat::from_mat3(&frame.ned_to_ecef()).inverse();
    let truth = ned_from_ecef * truth_attitude;
    let estimate = DQuat::from_xyzw(attitude.x, attitude.y, attitude.z, attitude.w);
    let error = rotation_vector((truth * estimate.inverse()).normalize());
    let [xx, xy, xz, yy, yz, zz] = covariance.upper_triangle();
    let matrix = Matrix3::new(xx, xy, xz, xy, yy, yz, xz, yz, zz);
    let vector = Vector3::new(error.x, error.y, error.z);
    let weighted = matrix
        .cholesky()
        .ok_or(NavigateBridgeError::SolutionCovariance)?
        .solve(&vector);
    Ok(AttitudeComparison {
        error_rad: error.length(),
        tilt_rad: error.x.hypot(error.y),
        heading_rad: error.z.abs(),
        attitude_nees: vector.dot(&weighted),
    })
}

/// A solution's position as ECEF metres.
pub fn solution_position_ecef(solution: &NavigationSolution) -> DVec3 {
    let position = solution.position;
    geodetic_to_ecef(&sim_geo::Geodetic {
        lat_rad: position.latitude_rad,
        lon_rad: position.longitude_rad,
        height_m: position.altitude_m,
    })
}

/// Scores `solution` against the true vehicle position.
pub fn compare_with_truth(
    solution: &NavigationSolution,
    truth_ecef: DVec3,
) -> Result<TruthComparison, NavigateBridgeError> {
    compare_position_with_truth(solution.position, &solution.position_cov, truth_ecef)
        .map(|(_, comparison)| comparison)
}

/// Scores a position with its north-east-down covariance against the true
/// position, and returns the truth's offset from the estimate in the
/// estimate's north-east-down axes beside the scores.
pub(crate) fn compare_position_with_truth(
    position: GeodeticPosition,
    position_cov: &SymmetricCov3,
    truth_ecef: DVec3,
) -> Result<(Vector3<f64>, TruthComparison), NavigateBridgeError> {
    let plane = LocalTangentPlane::new(position, BodyEllipsoid::WGS84)
        .map_err(NavigateBridgeError::SolutionPlane)?;
    let truth = ecef_to_geodetic(truth_ecef);
    let offset = plane.to_ned(&GeodeticPosition::new(
        truth.lat_rad,
        truth.lon_rad,
        truth.height_m,
    ));
    let error = Vector3::new(offset.north_m, offset.east_m, offset.down_m);
    let [xx, xy, xz, yy, yz, zz] = position_cov.upper_triangle();
    let covariance = Matrix3::new(xx, xy, xz, xy, yy, yz, xz, yz, zz);
    let weighted = covariance
        .cholesky()
        .ok_or(NavigateBridgeError::SolutionCovariance)?
        .solve(&error);
    Ok((
        error,
        TruthComparison {
            horizontal_error_m: offset.north_m.hypot(offset.east_m),
            vertical_error_m: offset.down_m.abs(),
            position_nees: error.dot(&weighted),
        },
    ))
}
