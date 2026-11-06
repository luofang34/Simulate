//! The whole atmospheric column over the geoid: the 1976 U.S. Standard
//! Atmosphere entered at EGM96 mean sea level, its thermosphere, and vacuum,
//! with a lower-atmosphere wind that dies out before the air does.

use glam::DVec3;
use sim_geo::{Geodetic, UtcTime};

use crate::{AtmosphereError, AtmosphereField, AtmosphereSample, ReentryAtmosphere};

#[cfg(test)]
mod tests;

/// Orthometric height where the wind starts to decay, metres: the top of the
/// ICAO standard atmosphere, above which no weather model is fed.
const WIND_FADE_START_M: f64 = 32_000.0;
/// Orthometric height from which the air is still, metres: the mesopause.
const WIND_FADE_END_M: f64 = 86_000.0;

/// Air from 5 km below mean sea level to vacuum at 200 km, measured from the
/// EGM96 geoid rather than the ellipsoid, with an optional uniform wind.
///
/// It is [`ReentryAtmosphere`] evaluated at orthometric height, so a field at
/// `H = 0` reads exactly sea-level ISA wherever the geoid lies, and every
/// handover (11, 32, 86, 180, 200 km MSL) stays continuous. The wind holds
/// through the troposphere and stratosphere, then fades smoothly to zero
/// between 32 km and 86 km MSL so the thermosphere and vacuum are still.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct FullColumnAtmosphere {
    wind_ned_m_s: DVec3,
}

impl FullColumnAtmosphere {
    /// Still air.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Air moving at `wind_ned_m_s` (toward, north-east-down) up to 32 km MSL.
    #[must_use]
    pub const fn with_wind(wind_ned_m_s: DVec3) -> Self {
        Self { wind_ned_m_s }
    }

    /// The wind at orthometric height `msl_m`, faded above 32 km.
    #[must_use]
    pub fn wind_at_msl(&self, msl_m: f64) -> DVec3 {
        let left =
            ((WIND_FADE_END_M - msl_m) / (WIND_FADE_END_M - WIND_FADE_START_M)).clamp(0.0, 1.0);
        self.wind_ned_m_s * (left * left * (3.0 - 2.0 * left))
    }
}

impl AtmosphereField for FullColumnAtmosphere {
    fn sample(
        &self,
        position: &Geodetic,
        utc: UtcTime,
    ) -> Result<AtmosphereSample, AtmosphereError> {
        let msl_m = sim_geo::ellipsoidal_to_msl_egm96(position).0;
        let orthometric = Geodetic {
            height_m: msl_m,
            ..*position
        };
        let mut sample = ReentryAtmosphere.sample(&orthometric, utc)?;
        sample.wind_ned_m_s = self.wind_at_msl(msl_m);
        Ok(sample)
    }
}
