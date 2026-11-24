//! The magnetic field a magnetometer sits in.

use glam::DVec3;
use sim_geo::{LocalFrame, MagneticField, ecef_to_geodetic};

use crate::DeviceError;

const NANOTESLA_PER_MICROTESLA: f64 = 1_000.0;

/// When a vehicle's magnetometer reads the world's magnetic field, and what
/// its surroundings add to it.
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

    /// The field of the world `field` at the Earth-fixed position `fixed`,
    /// with the declared local anomaly, in Earth-fixed axes, microtesla.
    ///
    /// # Errors
    ///
    /// [`DeviceError::NoMagneticField`] where `field` has no value.
    pub fn field_fixed_ut(
        &self,
        field: &dyn MagneticField,
        fixed: DVec3,
    ) -> Result<DVec3, DeviceError> {
        let main_nt = field
            .field_fixed_nt(fixed, self.decimal_year)
            .map_err(|source| DeviceError::NoMagneticField { source })?;
        let anomaly = match self.local_anomaly_ned_ut == DVec3::ZERO {
            true => DVec3::ZERO,
            false => {
                LocalFrame::at(&ecef_to_geodetic(fixed)).ned_to_ecef() * self.local_anomaly_ned_ut
            }
        };
        Ok(main_nt / NANOTESLA_PER_MICROTESLA + anomaly)
    }
}

#[cfg(test)]
mod tests;
