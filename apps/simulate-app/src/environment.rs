//! Builds each frame's [`EnvironmentFrame`] from the ephemeris, and hands the truth scene its
//! share of it. The window, the simulated cameras, and the fallback globe all draw from that
//! one frame, so none of them can see a sky or air the others do not.

use glam::DVec3;
use sim_geo::BodyEllipsoid;
use sim_maplibre::{CelestialSkyInput, CelestialTwilight, MapAdapterError, TruthSceneMap};
use sim_navigate::CelestialFrame;
use sim_render::{EnvironmentFrame, SOLAR_ILLUMINANCE_1AU_LUX, Sunlight, TwilightRamp};

use crate::sky_rig::SkyFrame;

#[cfg(test)]
mod tests;

/// The environment over `body` under the ephemeris `frame`: clear air, with the body's
/// modelled atmosphere.
#[must_use]
pub fn environment_frame(frame: &CelestialFrame, body: BodyEllipsoid) -> EnvironmentFrame {
    let visible_fraction = frame.sun_visible_fraction.clamp(0.0, 1.0);
    let radius = frame.sun_angular_radius_deg.filter(|deg| *deg > 0.0);
    let sun = Sunlight {
        visible_fraction,
        illuminance_lux: SOLAR_ILLUMINANCE_1AU_LUX * f64::from(visible_fraction),
        ..Sunlight::uneclipsed(frame.sun_direction_fixed, radius)
    };
    let twilight = TwilightRamp {
        night_end_deg: frame.twilight_night_end_deg,
        day_start_deg: frame.twilight_day_start_deg,
        night_brightness: frame.night_brightness,
    };
    EnvironmentFrame::clear(body, sun, twilight)
}

/// What the truth scene takes from one [`SkyFrame`], through `sim-maplibre`'s setters.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TruthSky<'a> {
    /// Toward the Sun, body-fixed.
    pub sun_direction_ecef: DVec3,
    /// The full sky, or `None` to light the scene by the Sun alone.
    pub celestial: Option<CelestialSkyInput<'a>>,
    /// Whether the scene draws atmospheric scattering.
    pub atmosphere: bool,
}

impl<'a> TruthSky<'a> {
    /// The truth scene's sky: every Sun and twilight value from `sky.environment`, and the
    /// Moon, Earth's shadow, and stars from `sky.bodies`.
    #[must_use]
    pub fn of(sky: &SkyFrame<'a>) -> Self {
        let environment = &sky.environment;
        let sun = &environment.sun;
        let twilight = environment.twilight;
        let celestial = sky.bodies.map(|bodies| CelestialSkyInput {
            sun_direction_ecef: sun.direction_fixed,
            sun_angular_radius_deg: sun.angular_radius_deg,
            sun_visible_fraction: sun.visible_fraction,
            moon: bodies.moon,
            earth_shadow: bodies.earth_shadow,
            twilight: CelestialTwilight {
                night_end_deg: twilight.night_end_deg,
                day_start_deg: twilight.day_start_deg,
                night_brightness: twilight.night_brightness,
            },
            stars: bodies.stars,
        });
        Self {
            sun_direction_ecef: sun.direction_fixed,
            celestial,
            atmosphere: environment.atmosphere.is_some(),
        }
    }

    /// Applies this sky to `truth`. The atmosphere toggle goes last because
    /// [`TruthSceneMap::set_celestial`] switches the atmosphere on.
    pub fn apply(&self, truth: &mut TruthSceneMap) -> Result<(), MapAdapterError> {
        match &self.celestial {
            Some(celestial) => truth.set_celestial(celestial),
            None => truth.set_sun_direction_ecef(self.sun_direction_ecef)?,
        }
        truth.set_atmosphere(self.atmosphere);
        Ok(())
    }
}
