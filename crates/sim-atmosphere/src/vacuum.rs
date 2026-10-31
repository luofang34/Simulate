use glam::DVec3;
use sim_geo::{Geodetic, UtcTime};

use crate::{AtmosphereError, AtmosphereField, AtmosphereSample};

#[cfg(test)]
mod tests;

/// Temperature of the cosmic microwave background, kelvin.
pub const COSMIC_BACKGROUND_K: f64 = 2.725;

/// No air at all: for the Moon and for deep space.
///
/// Speed of sound is zero, so Mach number is undefined here; consumers must
/// check dynamic pressure or density before dividing by it.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct VacuumAtmosphere;

impl VacuumAtmosphere {
    /// The sample every position and instant shares.
    pub(crate) const SAMPLE: AtmosphereSample = AtmosphereSample {
        temperature_k: COSMIC_BACKGROUND_K,
        pressure_pa: 0.0,
        density_kg_m3: 0.0,
        speed_of_sound_m_s: 0.0,
        wind_ned_m_s: DVec3::ZERO,
    };
}

impl AtmosphereField for VacuumAtmosphere {
    fn sample(
        &self,
        _position: &Geodetic,
        _utc: UtcTime,
    ) -> Result<AtmosphereSample, AtmosphereError> {
        Ok(Self::SAMPLE)
    }
}
