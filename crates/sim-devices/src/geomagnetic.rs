//! The magnetic field a magnetometer on the Earth sits in.

use glam::DVec3;
use sim_geo::Geodetic;
use world_core::{DecimalYear, GeodeticPosition};
use world_geomag::wmm2025;

use crate::DeviceError;

const NANOTESLA_PER_MICROTESLA: f64 = 1_000.0;

/// The magnetic field of a place: the Earth's main field and whatever the
/// ground there adds to it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MagneticEnvironment {
    /// Date the main field is evaluated for, decimal years.
    pub decimal_year: f64,
    /// Field the local geology and structures add to the main field,
    /// north-east-down, microtesla. A flight controller's reference model
    /// does not know it, so it is what makes the sensed declination differ
    /// from the reference; zero where none is declared.
    pub local_anomaly_ned_ut: DVec3,
}

impl MagneticEnvironment {
    /// The main field alone in `decimal_year`.
    pub const fn main_field(decimal_year: f64) -> Self {
        Self {
            decimal_year,
            local_anomaly_ned_ut: DVec3::ZERO,
        }
    }

    /// The field at `position`, north-east-down, microtesla.
    pub fn field_ned_ut(&self, position: &Geodetic) -> Result<DVec3, DeviceError> {
        Ok(earth_field_ned_ut(position, self.decimal_year)? + self.local_anomaly_ned_ut)
    }
}

/// The Earth's main field at `position` in `decimal_year`, north-east-down,
/// microtesla: the World Magnetic Model 2025. Crustal and local fields are
/// not in it.
pub fn earth_field_ned_ut(position: &Geodetic, decimal_year: f64) -> Result<DVec3, DeviceError> {
    let (lat_deg, lon_deg) = (position.lat_rad.to_degrees(), position.lon_rad.to_degrees());
    let at = GeodeticPosition::wgs84(lat_deg, lon_deg, position.height_m);
    let field =
        wmm2025(at, DecimalYear(decimal_year)).map_err(|source| DeviceError::NoMagneticField {
            lat_deg,
            lon_deg,
            decimal_year,
            source,
        })?;
    Ok(DVec3::new(field.north_nt, field.east_nt, field.down_nt) / NANOTESLA_PER_MICROTESLA)
}

#[cfg(test)]
mod tests;
