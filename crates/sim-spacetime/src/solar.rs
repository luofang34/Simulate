//! Sunlight anywhere in the solar system: irradiance, the direction it comes
//! from, how much of the solar disc a body hides, and how much an atmosphere
//! lets through to a surface.
//!
//! One query serves everything that feels the Sun. Radiation pressure reads
//! the momentum flux, a solar array the power flux, and a renderer the same
//! direction and lit fraction, so force, power and lighting cannot disagree.

use glam::DVec3;
use serde::{Deserialize, Serialize};

use crate::catalogs::{DerivativeOrder, EphemerisCatalog, EphemerisError};
use crate::ids::NaifId;
use crate::relativity::SPEED_OF_LIGHT_M_S;
use crate::scalar_math::{acos_clamped, asin_clamped, exp, powf, serde_dvec3, sqrt};
use crate::time::TdbEpoch;

/// Nominal total solar irradiance at one astronomical unit, W/m² (IAU 2015
/// Resolution B3).
pub const SOLAR_IRRADIANCE_1AU_W_M2: f64 = 1361.0;

/// The astronomical unit, m (IAU 2012 Resolution B2, exact).
pub const ASTRONOMICAL_UNIT_M: f64 = 149_597_870_700.0;

/// Nominal solar radius, m (IAU 2015 Resolution B3).
pub const SOLAR_RADIUS_M: f64 = 695_700_000.0;

/// A body that can hide the Sun, as a sphere.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct OccultingSphere {
    /// The body.
    pub body: NaifId,
    /// Radius of the sphere that blocks sunlight, m.
    pub radius_m: f64,
}

/// The Sun's output and the bodies that can eclipse it.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct SolarFluxModel {
    /// Total irradiance at one astronomical unit, W/m².
    pub irradiance_1au_w_m2: f64,
    /// Bodies checked for eclipses.
    pub occulters: Vec<OccultingSphere>,
}

impl SolarFluxModel {
    /// Nominal solar output, eclipsed by Earth, the Moon and Mars at their
    /// mean radii.
    #[must_use]
    pub fn standard() -> Self {
        let sphere = |body, radius_m| OccultingSphere { body, radius_m };
        Self {
            irradiance_1au_w_m2: SOLAR_IRRADIANCE_1AU_W_M2,
            occulters: vec![
                sphere(NaifId::EARTH, 6_371_008.8),
                sphere(NaifId::MOON, 1_737_400.0),
                sphere(NaifId::MARS, 3_389_500.0),
            ],
        }
    }

    /// Sunlight at `position_ssb_m` (barycentric, ICRF axes) at `epoch`,
    /// from geometric body positions.
    pub fn sample(
        &self,
        position_ssb_m: DVec3,
        epoch: TdbEpoch,
        ephemeris: &dyn EphemerisCatalog,
    ) -> Result<SolarFluxSample, EphemerisError> {
        let body_at = |body| {
            ephemeris
                .geometric_state_icrf(body, NaifId::SSB, epoch, DerivativeOrder::Position)
                .map(|state| state.pos_m)
        };
        let to_sun = body_at(NaifId::SUN)? - position_ssb_m;
        let sun_distance_m = sqrt(to_sun.length_squared());
        if sun_distance_m.is_nan() || sun_distance_m <= SOLAR_RADIUS_M {
            return Ok(SolarFluxSample::inside_the_sun());
        }
        let to_sun_icrf = to_sun / sun_distance_m;
        let ratio = ASTRONOMICAL_UNIT_M / sun_distance_m;
        let unobstructed_w_m2 = self.irradiance_1au_w_m2 * ratio * ratio;
        let sun_angular_radius = asin_clamped(SOLAR_RADIUS_M / sun_distance_m);

        let mut lit_fraction = 1.0_f64;
        for occulter in &self.occulters {
            let to_body = match body_at(occulter.body) {
                Ok(pos_m) => pos_m - position_ssb_m,
                Err(
                    EphemerisError::UnconnectedBodies { .. }
                    | EphemerisError::MissingBodyCenter { .. },
                ) => continue,
                Err(err) => return Err(err),
            };
            let visible = disc_fraction_visible(
                to_sun_icrf,
                sun_distance_m,
                sun_angular_radius,
                to_body,
                occulter.radius_m,
            );
            lit_fraction = lit_fraction.min(visible);
        }
        Ok(SolarFluxSample {
            to_sun_icrf,
            sun_distance_m,
            unobstructed_w_m2,
            lit_fraction,
        })
    }
}

/// Sunlight at one point and instant.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct SolarFluxSample {
    /// Unit vector towards the Sun's centre, ICRF axes.
    #[serde(with = "serde_dvec3")]
    pub to_sun_icrf: DVec3,
    /// Distance to the Sun's centre, m.
    pub sun_distance_m: f64,
    /// Irradiance with nothing in the way, W/m².
    pub unobstructed_w_m2: f64,
    /// Fraction of the solar disc in view, `0` in an umbra and `1` in full
    /// sunlight. With several occulters it is the smallest any one leaves.
    pub lit_fraction: f64,
}

impl SolarFluxSample {
    const fn inside_the_sun() -> Self {
        Self {
            to_sun_icrf: DVec3::ZERO,
            sun_distance_m: 0.0,
            unobstructed_w_m2: 0.0,
            lit_fraction: 0.0,
        }
    }

    /// Irradiance on a surface facing the Sun, W/m², after eclipses.
    #[must_use]
    pub fn irradiance_w_m2(&self) -> f64 {
        self.unobstructed_w_m2 * self.lit_fraction
    }

    /// Momentum flux of the sunlight, N/m²: the pressure on a perfectly
    /// absorbing surface facing the Sun.
    #[must_use]
    pub fn radiation_pressure_n_m2(&self) -> f64 {
        self.irradiance_w_m2() / SPEED_OF_LIGHT_M_S
    }

    /// Irradiance on a flat surface whose outward unit normal is
    /// `normal_icrf`, W/m²; zero when it faces away from the Sun.
    #[must_use]
    pub fn irradiance_on_w_m2(&self, normal_icrf: DVec3) -> f64 {
        self.irradiance_w_m2() * normal_icrf.dot(self.to_sun_icrf).max(0.0)
    }
}

/// Fraction of the solar disc left in view past a sphere of `radius_m` at
/// `to_body` from the observer.
fn disc_fraction_visible(
    to_sun_unit: DVec3,
    sun_distance_m: f64,
    sun_angular_radius: f64,
    to_body: DVec3,
    radius_m: f64,
) -> f64 {
    let body_distance_m = sqrt(to_body.length_squared());
    if body_distance_m <= radius_m {
        return 0.0;
    }
    if to_body.dot(to_sun_unit) <= 0.0 || body_distance_m >= sun_distance_m {
        return 1.0;
    }
    let body_angular_radius = asin_clamped(radius_m / body_distance_m);
    let separation = acos_clamped(to_body.dot(to_sun_unit) / body_distance_m);
    let (a, b, c) = (sun_angular_radius, body_angular_radius, separation);
    if c >= a + b {
        return 1.0;
    }
    if c <= (a - b).abs() {
        return if b >= a { 0.0 } else { 1.0 - (b * b) / (a * a) };
    }
    let lens = a * a * acos_clamped((c * c + a * a - b * b) / (2.0 * c * a))
        + b * b * acos_clamped((c * c + b * b - a * a) / (2.0 * c * b))
        - 0.5 * sqrt(((-c + a + b) * (c + a - b) * (c - a + b) * (c + a + b)).max(0.0));
    (1.0 - lens / (core::f64::consts::PI * a * a)).clamp(0.0, 1.0)
}

/// How much direct sunlight a clear atmosphere passes to the surface: a
/// single broadband optical depth and the air mass along the slant path.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct AtmosphericTransmission {
    /// Broadband optical depth looking straight up.
    pub zenith_optical_depth: f64,
}

impl AtmosphericTransmission {
    /// No atmosphere.
    pub const VACUUM: Self = Self {
        zenith_optical_depth: 0.0,
    };

    /// Earth at sea level under a clear sky: about three quarters of the
    /// direct beam reaches the ground with the Sun overhead.
    pub const EARTH_CLEAR_SKY: Self = Self {
        zenith_optical_depth: 0.30,
    };

    /// Mars under its usual dust load, outside dust storms.
    pub const MARS_CLEAR_SKY: Self = Self {
        zenith_optical_depth: 0.50,
    };

    /// Fraction of the direct beam that reaches the surface with the Sun
    /// `cos_zenith` from overhead (`1` at the zenith, `0` on the horizon);
    /// zero with the Sun below the horizon. The air mass follows Kasten and
    /// Young (1989), which stays finite at the horizon.
    #[must_use]
    pub fn direct_beam_fraction(&self, cos_zenith: f64) -> f64 {
        if cos_zenith.is_nan() || cos_zenith <= 0.0 {
            return 0.0;
        }
        let zenith_deg = acos_clamped(cos_zenith.min(1.0)).to_degrees();
        let air_mass = 1.0 / (cos_zenith + 0.505_72 * powf(96.079_95 - zenith_deg, -1.636_4));
        exp(-self.zenith_optical_depth.max(0.0) * air_mass)
    }
}

#[cfg(test)]
mod tests;
