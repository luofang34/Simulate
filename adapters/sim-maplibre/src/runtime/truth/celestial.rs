//! Host-computed sky (sun, moon, Earth's shadow, twilight, stars) applied to the truth scene.
//!
//! `sim-maplibre` holds no astronomy: the host computes directions and magnitudes and hands
//! them over as plain data. Values `maplibre` rejects (out-of-range radii, colours, or
//! fractions) fall back to "absent" or the default rather than failing the frame.

use glam::DVec3;
use maplibre::celestial::{
    AngularRadius, Disc, EarthShadow, EcefDirection, Star, TwilightBand, VisibleFraction,
};

use super::TruthSceneMap;

/// A body's disc seen from the eye.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CelestialDisc {
    /// ECEF unit direction from the eye to the disc's centre.
    pub direction_ecef: DVec3,
    /// Angular radius in degrees (`0 < r <= 5`).
    pub angular_radius_deg: f64,
}

/// Earth's shadow at the moon's distance, for lunar eclipses.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CelestialEarthShadow {
    /// ECEF unit direction from the eye to the shadow's centre.
    pub axis_ecef: DVec3,
    /// Umbra angular radius in degrees (smaller than the penumbra).
    pub umbra_deg: f64,
    /// Penumbra angular radius in degrees (`<= 5`).
    pub penumbra_deg: f64,
    /// Danjon darkness `0..=4`; `None` selects `maplibre`'s default.
    pub danjon: Option<f32>,
}

/// Solar elevations over which the ground turns from night to day.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CelestialTwilight {
    /// Solar elevation (degrees) where night ends.
    pub night_end_deg: f64,
    /// Solar elevation (degrees, above `night_end_deg`) where full day begins.
    pub day_start_deg: f64,
    /// Night-side brightness relative to day, `0..=1`.
    pub night_brightness: f32,
}

/// One catalogue star.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CelestialStar {
    /// ECEF unit direction towards the star.
    pub direction_ecef: DVec3,
    /// Visual magnitude.
    pub v_mag: f32,
    /// Display colour as an sRGB-encoded triple in `0..=1` (`maplibre` rejects other values).
    pub srgb: [f32; 3],
}

/// One frame's sky as computed by the host's ephemeris.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CelestialSkyInput<'a> {
    /// ECEF unit direction towards the sun.
    pub sun_direction_ecef: DVec3,
    /// Apparent solar angular radius in degrees; `None` draws no solar disc.
    pub sun_angular_radius_deg: Option<f64>,
    /// Share of the solar disc left uncovered by the moon (`1.0` outside an eclipse).
    pub sun_visible_fraction: f32,
    /// Lunar disc, when the moon is drawn.
    pub moon: Option<CelestialDisc>,
    /// Earth's shadow, during a lunar eclipse.
    pub earth_shadow: Option<CelestialEarthShadow>,
    /// Twilight band and night brightness.
    pub twilight: CelestialTwilight,
    /// Replacement star list; `None` keeps the stars already set, so hosts pass them only
    /// when the catalogue or epoch changes and steady frames allocate nothing.
    pub stars: Option<&'a [CelestialStar]>,
}

fn direction(v: DVec3) -> Option<EcefDirection> {
    EcefDirection::new(v.x, v.y, v.z).ok()
}

impl TruthSceneMap {
    /// Applies the host's sky to every truth slot: atmosphere on, sun direction, disc,
    /// eclipse dimming, moon, Earth's shadow, twilight band, and (when given) stars.
    pub fn set_celestial(&mut self, sky: &CelestialSkyInput<'_>) {
        self.map.set_atmosphere(true);
        let sun = direction(sky.sun_direction_ecef);
        if sun.is_some() {
            self.sun_ecef = sky.sun_direction_ecef;
        }
        self.map.set_sun_direction(sun);
        self.map.set_sun_angular_radius(
            sky.sun_angular_radius_deg
                .and_then(|deg| AngularRadius::new(deg).ok()),
        );
        self.map.set_sun_visible_fraction(
            VisibleFraction::new(sky.sun_visible_fraction).unwrap_or_default(),
        );
        self.map.set_moon_disc(sky.moon.and_then(|moon| {
            Disc::new(direction(moon.direction_ecef)?, moon.angular_radius_deg).ok()
        }));
        self.map
            .set_earth_shadow(sky.earth_shadow.and_then(|shadow| {
                EarthShadow::new(
                    direction(shadow.axis_ecef)?,
                    shadow.umbra_deg,
                    shadow.penumbra_deg,
                    shadow.danjon,
                )
                .ok()
            }));
        let band = sky.twilight;
        self.map.set_twilight_band(
            TwilightBand::new(
                band.night_end_deg,
                band.day_start_deg,
                band.night_brightness,
            )
            .unwrap_or_default(),
        );
        if let Some(stars) = sky.stars {
            self.map.set_stars(to_maplibre_stars(stars));
        }
    }
}

/// Converts host stars, dropping any `maplibre` rejects (zero direction, non-finite
/// magnitude, out-of-range colour).
pub(super) fn to_maplibre_stars(stars: &[CelestialStar]) -> Vec<Star> {
    stars
        .iter()
        .filter_map(|star| {
            Star::new(direction(star.direction_ecef)?, star.v_mag, Some(star.srgb)).ok()
        })
        .collect()
}
