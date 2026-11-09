use glam::DVec3;
use nalgebra::{Matrix3, Vector3};
use navigate_contract::{GeodeticPosition, NavigationSolution};
use navigate_geodesy::{BodyEllipsoid, LocalTangentPlane};
use sim_geo::{ecef_to_geodetic, geodetic_to_ecef};

use crate::NavigateBridgeError;

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
    let plane = LocalTangentPlane::new(solution.position, BodyEllipsoid::WGS84)
        .map_err(NavigateBridgeError::SolutionPlane)?;
    let truth = ecef_to_geodetic(truth_ecef);
    let offset = plane.to_ned(&GeodeticPosition::new(
        truth.lat_rad,
        truth.lon_rad,
        truth.height_m,
    ));
    let error = Vector3::new(offset.north_m, offset.east_m, offset.down_m);
    let [xx, xy, xz, yy, yz, zz] = solution.position_cov.upper_triangle();
    let covariance = Matrix3::new(xx, xy, xz, xy, yy, yz, xz, yz, zz);
    let weighted = covariance
        .cholesky()
        .ok_or(NavigateBridgeError::SolutionCovariance)?
        .solve(&error);
    Ok(TruthComparison {
        horizontal_error_m: offset.north_m.hypot(offset.east_m),
        vertical_error_m: offset.down_m.abs(),
        position_nees: error.dot(&weighted),
    })
}
