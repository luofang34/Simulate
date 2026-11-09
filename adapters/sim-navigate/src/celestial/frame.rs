use glam::DVec3;

/// Solar elevation, degrees, at and below which the sky is fully night.
const TWILIGHT_NIGHT_END_DEG: f64 = -6.0;
/// Solar elevation, degrees, at and above which the sky is fully day.
const TWILIGHT_DAY_START_DEG: f64 = 2.0;
/// Ambient scale of a moonless night relative to day, so terrain stays legible.
const NIGHT_BRIGHTNESS: f32 = 0.22;

/// One catalogue star for the sky renderer.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct StarSkySample {
    /// Unit direction in the observer body's fixed axes.
    pub direction_ecef: DVec3,
    /// Visual magnitude, Johnson V.
    pub v_magnitude: f32,
    /// Black-body colour of the star, linear sRGB scaled so the largest
    /// channel is 1.
    pub linear_rgb: [f32; 3],
}

/// The Moon's disc as seen by the observer.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MoonDiscSample {
    /// Unit topocentric direction to the Moon's centre, body-fixed axes.
    pub direction_ecef: DVec3,
    /// Apparent angular radius, degrees.
    pub angular_radius_deg: f64,
}

/// Earth's shadow cones where they cross the Moon's orbit.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct EarthShadowSample {
    /// Unit direction of the shadow axis (anti-Sun) from the observer,
    /// body-fixed axes.
    pub direction_ecef: DVec3,
    /// Angular radius of the umbra at the Moon's distance, degrees.
    pub umbra_radius_deg: f64,
    /// Angular radius of the penumbra at the Moon's distance, degrees.
    pub penumbra_radius_deg: f64,
    /// Nominal Danjon brightness `L` (0 dark .. 4 bright copper) while the
    /// Moon touches the umbra, else `None`. Geometry alone cannot predict
    /// the real value, which depends on stratospheric aerosol; this one only
    /// darkens with umbral depth.
    pub danjon: Option<f32>,
}

/// Everything the sky renderer needs for one frame.
#[derive(Debug, Clone, PartialEq)]
pub struct CelestialFrame {
    /// The evaluated instant, Unix seconds UTC.
    pub utc_unix_s: f64,
    /// Unit topocentric direction to the Sun, body-fixed axes: ECEF on the
    /// Earth, Moon-fixed on the Moon.
    pub sun_direction_fixed: DVec3,
    /// Apparent solar angular radius, degrees.
    pub sun_angular_radius_deg: Option<f64>,
    /// Fraction of the solar disc area not covered by the Moon, `0..=1`.
    pub sun_visible_fraction: f32,
    /// The Moon's disc, for an observer on the Earth.
    pub moon_disc: Option<MoonDiscSample>,
    /// Earth's shadow at the Moon's distance, for an observer on the Earth.
    pub earth_shadow: Option<EarthShadowSample>,
    /// Solar elevation, degrees, at which night lighting is complete.
    pub twilight_night_end_deg: f64,
    /// Solar elevation, degrees, at which day lighting is complete.
    pub twilight_day_start_deg: f64,
    /// Night ambient scale relative to day.
    pub night_brightness: f32,
    /// A fresh star list when the stars moved enough to re-upload, else `None`.
    pub stars_update: Option<Vec<StarSkySample>>,
}

impl CelestialFrame {
    /// A frame with the given Sun geometry and the shared lighting constants.
    pub fn with_sun(sun_direction_fixed: DVec3, sun_angular_radius_deg: f64) -> Self {
        Self {
            utc_unix_s: 0.0,
            sun_direction_fixed,
            sun_angular_radius_deg: Some(sun_angular_radius_deg),
            sun_visible_fraction: 1.0,
            moon_disc: None,
            earth_shadow: None,
            twilight_night_end_deg: TWILIGHT_NIGHT_END_DEG,
            twilight_day_start_deg: TWILIGHT_DAY_START_DEG,
            night_brightness: NIGHT_BRIGHTNESS,
            stars_update: None,
        }
    }

    /// The sky before any successful evaluation: Sun at the body-fixed pole,
    /// no Moon, no stars.
    pub(super) fn fallback(utc_unix_s: f64) -> Self {
        let mut frame = Self::with_sun(DVec3::Z, 0.0);
        frame.utc_unix_s = utc_unix_s;
        frame.sun_angular_radius_deg = None;
        frame
    }
}
