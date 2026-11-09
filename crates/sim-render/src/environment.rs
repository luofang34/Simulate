//! One frame's light and air, shared by every renderer.
//!
//! The window view, the simulated cameras, and the fallback globe each draw
//! the world on their own, but must agree on what is in it: a camera that
//! sees clear sky while the pilot sees fog would validate visual navigation
//! against a world nobody flew in. So the host builds one [`EnvironmentFrame`]
//! per frame from simulation state, and every renderer draws from it. It is
//! plain data in the observer body's fixed axes, with no GPU types, so any
//! renderer can consume it.

use glam::DVec3;
use sim_geo::{BodyEllipsoid, LocalFrame};

#[cfg(test)]
mod tests;

/// Exo-atmospheric solar illuminance at one astronomical unit, lux.
pub const SOLAR_ILLUMINANCE_1AU_LUX: f64 = 128_000.0;

/// The Sun as one frame sees it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Sunlight {
    /// Unit direction toward the Sun, body-fixed.
    pub direction_fixed: DVec3,
    /// Apparent angular radius, degrees; `None` when the disc is not drawn.
    pub angular_radius_deg: Option<f64>,
    /// Share of the disc not covered by an eclipsing body, `0..=1`.
    pub visible_fraction: f32,
    /// Illuminance on a surface facing the Sun above any atmosphere, lux,
    /// already dimmed by `visible_fraction`.
    pub illuminance_lux: f64,
    /// Colour of that light above any atmosphere, linear RGB.
    pub colour_linear: [f32; 3],
}

impl Sunlight {
    /// An uneclipsed Sun at one astronomical unit toward `direction_fixed`
    /// (normalised here; a zero vector stays zero).
    #[must_use]
    pub fn uneclipsed(direction_fixed: DVec3, angular_radius_deg: Option<f64>) -> Self {
        Self {
            direction_fixed: direction_fixed.normalize_or_zero(),
            angular_radius_deg,
            visible_fraction: 1.0,
            illuminance_lux: SOLAR_ILLUMINANCE_1AU_LUX,
            colour_linear: [1.0; 3],
        }
    }
}

/// Solar elevations over which the scene turns from night to day.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TwilightRamp {
    /// Solar elevation, degrees, at and below which it is full night.
    pub night_end_deg: f64,
    /// Solar elevation, degrees, at and above which it is full day.
    pub day_start_deg: f64,
    /// Ground ambient on the night side relative to day, `0..=1`.
    pub night_brightness: f32,
}

impl TwilightRamp {
    /// How far into day a solar elevation is: `0` at night, `1` in full day,
    /// smoothstepped between.
    #[must_use]
    pub fn daylight(&self, sun_elevation_deg: f64) -> f32 {
        let span = self.day_start_deg - self.night_end_deg;
        if span <= 0.0 {
            return if sun_elevation_deg >= self.day_start_deg {
                1.0
            } else {
                0.0
            };
        }
        let t = ((sun_elevation_deg - self.night_end_deg) / span).clamp(0.0, 1.0);
        (t * t * (3.0 - 2.0 * t)) as f32
    }
}

/// A body's atmosphere: the sky it shows by day and the scattering that
/// makes it, for renderers that model the sky rather than paint it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Atmosphere {
    /// Display-referred colour of the clear day sky, linear RGB.
    pub day_sky_linear_rgb: [f32; 3],
    /// Rayleigh scattering at the surface for red, green, and blue, per metre.
    pub rayleigh_per_m: [f64; 3],
    /// Height over which air density falls by `1/e`, metres.
    pub scale_height_m: f64,
}

impl Atmosphere {
    /// Earth's clear sky. The scattering coefficients are the standard
    /// sea-level Rayleigh values for 680, 550, and 440 nm.
    pub const EARTH: Self = Self {
        day_sky_linear_rgb: [0.4, 0.6, 0.9],
        rayleigh_per_m: [5.802e-6, 13.558e-6, 33.1e-6],
        scale_height_m: 8_000.0,
    };

    /// Share of the surface air column still above `height_m`, `0..=1`.
    /// The thin sky scatters in proportion to the air above the eye, so the
    /// day sky darkens with height and is black from orbit.
    #[must_use]
    pub fn column_above(&self, height_m: f64) -> f64 {
        if self.scale_height_m <= 0.0 {
            return 0.0;
        }
        (-height_m.max(0.0) / self.scale_height_m).exp()
    }

    /// The modelled atmosphere of `body`; bodies with no model are airless.
    #[must_use]
    pub fn of_body(body: &BodyEllipsoid) -> Option<Self> {
        (*body == BodyEllipsoid::WGS84).then_some(Self::EARTH)
    }
}

/// Uniform extinction between the eye and what it sees.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Fog {
    /// Extinction coefficient, per metre; zero is clear air.
    pub extinction_per_m: f64,
    /// Colour that extinguished light is replaced by, linear RGB.
    pub colour_linear: [f32; 3],
}

impl Fog {
    /// No extinction.
    pub const CLEAR: Self = Self {
        extinction_per_m: 0.0,
        colour_linear: [0.0; 3],
    };
}

/// Names a cloud layer held by the host's weather state, so every renderer
/// draws the same clouds.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct CloudLayerHandle(pub u32);

/// Everything a renderer needs to light and fog one frame.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct EnvironmentFrame {
    /// The body the observer is on or near; its fixed axes are the frame's.
    pub body: BodyEllipsoid,
    /// The Sun.
    pub sun: Sunlight,
    /// How sky and ground turn from night to day.
    pub twilight: TwilightRamp,
    /// The body's atmosphere; `None` for an airless body, whose sky is black.
    pub atmosphere: Option<Atmosphere>,
    /// Extinction along every line of sight.
    pub fog: Fog,
    /// The cloud layer to draw, if any.
    pub cloud_layer: Option<CloudLayerHandle>,
    /// Scale from the frame's radiances to the display-linear values a
    /// renderer writes; `1.0` leaves them as given.
    pub exposure: f32,
}

impl EnvironmentFrame {
    /// Clear air on `body` under `sun`, with the body's modelled atmosphere.
    #[must_use]
    pub fn clear(body: BodyEllipsoid, sun: Sunlight, twilight: TwilightRamp) -> Self {
        Self {
            body,
            sun,
            twilight,
            atmosphere: Atmosphere::of_body(&body),
            fog: Fog::CLEAR,
            cloud_layer: None,
            exposure: 1.0,
        }
    }

    /// Solar elevation above the local horizon at `position_fixed`, degrees.
    #[must_use]
    pub fn sun_elevation_deg_at(&self, position_fixed: DVec3) -> f64 {
        let up = LocalFrame::at(&self.body.fixed_to_geodetic(position_fixed)).up();
        let sin = up.dot(self.sun.direction_fixed.normalize_or_zero());
        sin.clamp(-1.0, 1.0).asin().to_degrees()
    }

    /// Colour of the sky seen from `position_fixed`, linear RGB after
    /// exposure: the surface day sky dimmed through twilight, by any
    /// eclipse, and by the air left overhead, and black without an
    /// atmosphere.
    #[must_use]
    pub fn sky_linear_rgb_at(&self, position_fixed: DVec3) -> [f32; 3] {
        let Some(atmosphere) = self.atmosphere else {
            return [0.0; 3];
        };
        let daylight = self
            .twilight
            .daylight(self.sun_elevation_deg_at(position_fixed));
        let height_m = self.body.fixed_to_geodetic(position_fixed).height_m;
        let overhead = atmosphere.column_above(height_m) as f32;
        let scale = daylight * self.sun.visible_fraction.clamp(0.0, 1.0) * overhead * self.exposure;
        atmosphere.day_sky_linear_rgb.map(|c| c * scale)
    }

    /// The fog colour after exposure, linear RGB.
    #[must_use]
    pub fn fog_linear_rgb(&self) -> [f32; 3] {
        self.fog.colour_linear.map(|c| c * self.exposure)
    }
}
