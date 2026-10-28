use fsg_geo::{Geodetic, UtcTime};
use glam::DVec3;

use crate::{geometric_to_geopotential_m, isa_at_geopotential};

/// Why an atmosphere sample is unavailable.
#[derive(Debug, Clone, PartialEq, thiserror::Error)]
pub enum AtmosphereError {
    /// The height lies outside the modelled layers.
    #[error(
        "geopotential height {geopotential_m} m is outside the model's {floor_m}..={ceiling_m} m"
    )]
    OutOfRange {
        /// Requested geopotential height, metres.
        geopotential_m: f64,
        /// Lowest supported height, metres.
        floor_m: f64,
        /// Highest supported height, metres.
        ceiling_m: f64,
    },
}

/// Air state at one point and instant.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AtmosphereSample {
    /// Static air temperature, kelvin.
    pub temperature_k: f64,
    /// Static pressure, pascals.
    pub pressure_pa: f64,
    /// Air density, kg/m³.
    pub density_kg_m3: f64,
    /// Speed of sound, m/s.
    pub speed_of_sound_m_s: f64,
    /// Wind velocity (where the air moves toward), north-east-down, m/s.
    pub wind_ned_m_s: DVec3,
}

/// A 4D atmosphere: the one place the flight model and the renderer read air
/// state from, so both always see the same weather.
pub trait AtmosphereField {
    /// The air state at `position` at instant `utc`.
    fn sample(
        &self,
        position: &Geodetic,
        utc: UtcTime,
    ) -> Result<AtmosphereSample, AtmosphereError>;
}

/// The ICAO standard atmosphere with still air.
///
/// Ellipsoidal height is used as height above MSL: with no geoid model the
/// error equals the local geoid undulation (within about ±110 m worldwide).
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct StandardAtmosphere;

impl AtmosphereField for StandardAtmosphere {
    fn sample(
        &self,
        position: &Geodetic,
        _utc: UtcTime,
    ) -> Result<AtmosphereSample, AtmosphereError> {
        let state = isa_at_geopotential(geometric_to_geopotential_m(position.height_m))?;
        Ok(AtmosphereSample {
            temperature_k: state.temperature_k,
            pressure_pa: state.pressure_pa,
            density_kg_m3: state.density_kg_m3,
            speed_of_sound_m_s: state.speed_of_sound_m_s,
            wind_ned_m_s: DVec3::ZERO,
        })
    }
}
