//! A three-axis magnetometer: the true field in its axes, distorted by the
//! vehicle's own magnetism and the sensor's gain errors, plus noise.

use glam::DVec3;

use crate::{DeviceError, WhiteNoise};

#[cfg(test)]
mod tests;

/// Error statistics of a magnetometer, every axis independent with the same
/// statistics.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MagnetometerSpec {
    /// White noise of one sample, microtesla.
    pub white_sigma_ut: f64,
    /// 1-sigma of the constant offset the vehicle's hard iron adds,
    /// microtesla.
    pub hard_iron_sigma_ut: f64,
    /// 1-sigma scale-factor error, as a fraction.
    pub scale_factor_sigma: f64,
}

impl MagnetometerSpec {
    /// A sensor that reports the field exactly.
    pub const IDEAL: Self = Self {
        white_sigma_ut: 0.0,
        hard_iron_sigma_ut: 0.0,
        scale_factor_sigma: 0.0,
    };
}

/// A magnetometer with the errors it was built and installed with.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Magnetometer {
    spec: MagnetometerSpec,
    hard_iron_ut: DVec3,
    gain: DVec3,
    noise: WhiteNoise,
}

impl Magnetometer {
    /// A sensor whose offset and gain errors are drawn from `seed`.
    pub fn new(spec: MagnetometerSpec, seed: u64) -> Result<Self, DeviceError> {
        let fields = [
            ("white_sigma_ut", spec.white_sigma_ut),
            ("hard_iron_sigma_ut", spec.hard_iron_sigma_ut),
            ("scale_factor_sigma", spec.scale_factor_sigma),
        ];
        if let Some(&(field, value)) = fields.iter().find(|(_, v)| !(v.is_finite() && *v >= 0.0)) {
            return Err(DeviceError::InvalidStatistic {
                device: "magnetometer",
                field,
                value,
            });
        }
        let mut noise = WhiteNoise::from_seed(seed);
        let hard_iron_ut = noise.next_vec3(DVec3::splat(spec.hard_iron_sigma_ut));
        let gain = DVec3::ONE + noise.next_vec3(DVec3::splat(spec.scale_factor_sigma));
        Ok(Self {
            spec,
            hard_iron_ut,
            gain,
            noise,
        })
    }

    /// The offset the vehicle's hard iron adds to every reading, microtesla.
    pub const fn hard_iron_ut(&self) -> DVec3 {
        self.hard_iron_ut
    }

    /// The reading of the true field `field_body_ut` in the sensor's axes,
    /// microtesla.
    pub fn sense(&mut self, field_body_ut: DVec3) -> DVec3 {
        let white = self.noise.next_vec3(DVec3::splat(self.spec.white_sigma_ut));
        self.gain * field_body_ut + self.hard_iron_ut + white
    }
}
