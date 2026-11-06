use glam::DVec3;
use sim_geo::{Geodetic, UtcTime};

use crate::{geometric_to_geopotential_m, isa_at_geopotential};

#[cfg(test)]
mod tests;

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

/// The ICAO standard atmosphere entered at mean sea level rather than the
/// ellipsoid, with an optional uniform wind.
///
/// Positions carry WGS84 ellipsoidal heights; the EGM96 geoid converts them to
/// orthometric heights first, so a field at `H = 0` reads exactly sea-level
/// ISA wherever the geoid lies.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct GeoidStandardAtmosphere {
    wind_ned_m_s: DVec3,
}

impl GeoidStandardAtmosphere {
    /// Still air.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// The same air moving at `wind_ned_m_s` (toward, north-east-down) at
    /// every point.
    #[must_use]
    pub fn with_wind(wind_ned_m_s: DVec3) -> Self {
        Self { wind_ned_m_s }
    }
}

impl AtmosphereField for GeoidStandardAtmosphere {
    fn sample(
        &self,
        position: &Geodetic,
        _utc: UtcTime,
    ) -> Result<AtmosphereSample, AtmosphereError> {
        let msl_m = sim_geo::ellipsoidal_to_msl_egm96(position).0;
        let state = isa_at_geopotential(geometric_to_geopotential_m(msl_m))?;
        Ok(AtmosphereSample {
            temperature_k: state.temperature_k,
            pressure_pa: state.pressure_pa,
            density_kg_m3: state.density_kg_m3,
            speed_of_sound_m_s: state.speed_of_sound_m_s,
            wind_ned_m_s: self.wind_ned_m_s,
        })
    }
}
