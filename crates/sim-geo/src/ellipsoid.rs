use glam::DVec3;

use crate::BodyEllipsoid;

#[cfg(test)]
mod tests;

/// WGS84 semi-major axis, metres.
pub const WGS84_A: f64 = 6_378_137.0;
/// WGS84 flattening.
pub const WGS84_F: f64 = 1.0 / 298.257_223_563;
/// WGS84 semi-minor axis, metres.
pub const WGS84_B: f64 = WGS84_A * (1.0 - WGS84_F);
/// WGS84 first eccentricity squared.
pub const WGS84_E2: f64 = WGS84_F * (2.0 - WGS84_F);

/// A geodetic position with height above the body's reference ellipsoid (not
/// MSL): WGS84 unless a [`BodyEllipsoid`] says otherwise.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Geodetic {
    /// Geodetic latitude, radians, positive north.
    pub lat_rad: f64,
    /// Longitude, radians, positive east.
    pub lon_rad: f64,
    /// Height above the reference ellipsoid, metres.
    pub height_m: f64,
}

impl Geodetic {
    /// Builds a position from degrees and ellipsoidal height in metres.
    pub fn from_degrees(lat_deg: f64, lon_deg: f64, height_m: f64) -> Self {
        Self {
            lat_rad: lat_deg.to_radians(),
            lon_rad: lon_deg.to_radians(),
            height_m,
        }
    }

    /// Latitude in degrees.
    pub fn lat_deg(&self) -> f64 {
        self.lat_rad.to_degrees()
    }

    /// Longitude in degrees.
    pub fn lon_deg(&self) -> f64 {
        self.lon_rad.to_degrees()
    }
}

/// Converts a geodetic position to ECEF metres.
pub fn geodetic_to_ecef(position: &Geodetic) -> DVec3 {
    BodyEllipsoid::WGS84.geodetic_to_fixed(position)
}

/// Converts ECEF metres to a geodetic position on WGS84.
pub fn ecef_to_geodetic(ecef: DVec3) -> Geodetic {
    BodyEllipsoid::WGS84.fixed_to_geodetic(ecef)
}
