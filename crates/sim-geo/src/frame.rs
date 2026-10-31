use glam::{DMat3, DVec3};

use crate::Geodetic;

#[cfg(test)]
mod tests;

/// An east-north-up tangent frame at a geodetic position, expressed as unit
/// axes in ECEF. NED is derived from the same axes so the two never disagree.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LocalFrame {
    enu_to_ecef: DMat3,
}

impl LocalFrame {
    /// The tangent frame at `position`; up is the ellipsoid normal, not the
    /// direction from the Earth's centre.
    pub fn at(position: &Geodetic) -> Self {
        let (sin_lat, cos_lat) = position.lat_rad.sin_cos();
        let (sin_lon, cos_lon) = position.lon_rad.sin_cos();
        let east = DVec3::new(-sin_lon, cos_lon, 0.0);
        let north = DVec3::new(-sin_lat * cos_lon, -sin_lat * sin_lon, cos_lat);
        let up = DVec3::new(cos_lat * cos_lon, cos_lat * sin_lon, sin_lat);
        Self {
            enu_to_ecef: DMat3::from_cols(east, north, up),
        }
    }

    /// East axis in ECEF.
    pub fn east(&self) -> DVec3 {
        self.enu_to_ecef.x_axis
    }

    /// North axis in ECEF.
    pub fn north(&self) -> DVec3 {
        self.enu_to_ecef.y_axis
    }

    /// Up (ellipsoid normal) axis in ECEF.
    pub fn up(&self) -> DVec3 {
        self.enu_to_ecef.z_axis
    }

    /// Rotation taking ENU components to ECEF components.
    pub fn enu_to_ecef(&self) -> DMat3 {
        self.enu_to_ecef
    }

    /// Rotation taking NED components to ECEF components.
    pub fn ned_to_ecef(&self) -> DMat3 {
        DMat3::from_cols(self.north(), self.east(), -self.up())
    }

    /// Expresses an ECEF vector in ENU components.
    pub fn ecef_to_enu_vector(&self, ecef: DVec3) -> DVec3 {
        self.enu_to_ecef.transpose() * ecef
    }

    /// Expresses an ECEF vector in NED components.
    pub fn ecef_to_ned_vector(&self, ecef: DVec3) -> DVec3 {
        self.ned_to_ecef().transpose() * ecef
    }
}
