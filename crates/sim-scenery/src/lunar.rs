use glam::DVec3;
use sim_geo::{BodyEllipsoid, Geodetic};

use crate::noise::value_noise;
use crate::{ElevationSource, RadiometricSource, SceneryError, SurfaceRadiometry};

mod surface;

use surface::{Ground, ground_at, terrain_height_m};

#[cfg(test)]
mod tests;

/// Latitude of Statio Tranquillitatis, degrees north.
pub const APOLLO_11_SITE_LAT_DEG: f64 = 0.674_16;
/// Longitude of Statio Tranquillitatis, degrees east.
pub const APOLLO_11_SITE_LON_DEG: f64 = 23.473_14;

/// A bowl crater, placed relative to the landing site.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LunarCrater {
    /// Rim-to-rim diameter, metres.
    pub diameter_m: f64,
    /// Centre, metres east of the site.
    pub east_m: f64,
    /// Centre, metres north of the site.
    pub north_m: f64,
}

/// West Crater, the rocky crater Armstrong flew past before landing about
/// 400 m beyond it.
pub const WEST_CRATER: LunarCrater = LunarCrater {
    diameter_m: 180.0,
    east_m: 400.0,
    north_m: 0.0,
};

/// Little West Crater, just east of the Lunar Module.
pub const LITTLE_WEST_CRATER: LunarCrater = LunarCrater {
    diameter_m: 33.0,
    east_m: 60.0,
    north_m: 0.0,
};

/// How far from the site the scenery extends, metres.
const COVERAGE_RADIUS_M: f64 = 4_000.0;
/// Solar constant at the Moon, W/m².
const SOLAR_FLUX_W_M2: f64 = 1_361.0;
/// Temperature of ground out of the Sun, kelvin: lunar regolith cools to
/// around this overnight and in permanent crater shadow.
const SHADOW_TEMPERATURE_K: f32 = 100.0;
/// Baseline for the surface normal, metres: fine enough to see a boulder's
/// flank.
const SLOPE_BASELINE_M: f64 = 0.25;
/// Step and reach of the walk toward the Sun that finds cast shadows.
/// Boulders are left out: their shadows are smaller than the step.
const SHADOW_STEP_M: f64 = 6.0;
const SHADOW_REACH_M: f64 = 420.0;
/// Highest terrain above the walk's start worth checking for, metres.
const SHADOW_CEILING_M: f64 = 60.0;
/// Fine relief of the regolith plain.
const ROLLING_SEED: u64 = 0x00A1_1011;
const ROLLING_AMPLITUDE_M: f64 = 0.8;
const ROLLING_WAVELENGTH_M: f64 = 90.0;

/// Procedural terrain around the Apollo 11 landing site on the IAU Moon.
///
/// A gently rolling mare plain carries West Crater with its boulder-strewn
/// ejecta, Little West Crater, and scattered rocks, around the clear pad the
/// crew landed on. Temperatures follow the Sun: sunlit ground reaches
/// radiative equilibrium with its local slope, and ground in cast shadow is
/// cold. Beyond 4 km of the site everything is `Ok(None)`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LunarTranquilityScenery {
    site: Geodetic,
    to_sun_enu: DVec3,
}

impl LunarTranquilityScenery {
    /// The site with the Sun at `elevation_deg` above the horizon and
    /// `azimuth_deg` clockwise from north.
    pub fn new(azimuth_deg: f64, elevation_deg: f64) -> Self {
        let (sin_el, cos_el) = elevation_deg.to_radians().sin_cos();
        let (sin_az, cos_az) = azimuth_deg.to_radians().sin_cos();
        Self {
            site: Geodetic::from_degrees(APOLLO_11_SITE_LAT_DEG, APOLLO_11_SITE_LON_DEG, 0.0),
            to_sun_enu: DVec3::new(cos_el * sin_az, cos_el * cos_az, sin_el),
        }
    }

    /// The site as the crew saw it: a low morning Sun, 10.8° up in the east.
    pub fn apollo_11_landing() -> Self {
        Self::new(90.0, 10.8)
    }

    /// The position `east_m`, `north_m` from the site, on the sphere.
    pub fn at_local(&self, east_m: f64, north_m: f64) -> Geodetic {
        let radius = BodyEllipsoid::MOON_IAU.semi_major_m;
        Geodetic {
            lat_rad: self.site.lat_rad + north_m / radius,
            lon_rad: self.site.lon_rad + east_m / (radius * self.site.lat_rad.cos()),
            height_m: 0.0,
        }
    }

    /// Metres east and north of the site, or `None` outside the coverage.
    fn local(&self, position: &Geodetic) -> Option<(f64, f64)> {
        let radius = BodyEllipsoid::MOON_IAU.semi_major_m;
        let east = (position.lon_rad - self.site.lon_rad) * radius * self.site.lat_rad.cos();
        let north = (position.lat_rad - self.site.lat_rad) * radius;
        (east.hypot(north) <= COVERAGE_RADIUS_M).then_some((east, north))
    }

    /// Whether the terrain toward the Sun hides it from a surface point.
    fn in_shadow(&self, east: f64, north: f64, height: f64) -> bool {
        let sun = self.to_sun_enu;
        let run = sun.x.hypot(sun.y);
        if run <= 0.0 {
            return false;
        }
        let rise = SHADOW_STEP_M * sun.z / run;
        let step_east = SHADOW_STEP_M * sun.x / run;
        let step_north = SHADOW_STEP_M * sun.y / run;
        let steps = (SHADOW_REACH_M / SHADOW_STEP_M) as u32;
        (1..=steps).any(|step| {
            let travelled = f64::from(step);
            let ray = height + rise * travelled;
            ray < height + SHADOW_CEILING_M
                && terrain_height_m(east + step_east * travelled, north + step_north * travelled)
                    > ray
        })
    }

    /// Upward surface normal at a local point, in east-north-up axes.
    fn surface_normal(&self, east: f64, north: f64) -> DVec3 {
        let h = SLOPE_BASELINE_M;
        let height = |e: f64, n: f64| ground_at(e, n).height_m;
        let slope_east = (height(east + h, north) - height(east - h, north)) / (2.0 * h);
        let slope_north = (height(east, north + h) - height(east, north - h)) / (2.0 * h);
        DVec3::new(-slope_east, -slope_north, 1.0).normalize()
    }

    fn temperature_k(&self, east: f64, north: f64, ground: &Ground) -> f32 {
        let cos_incidence = self.surface_normal(east, north).dot(self.to_sun_enu);
        if cos_incidence <= 0.0 || self.in_shadow(east, north, ground.height_m) {
            return SHADOW_TEMPERATURE_K;
        }
        let absorbed = f64::from(1.0 - ground.albedo) * SOLAR_FLUX_W_M2 * cos_incidence;
        let sigma_epsilon =
            f64::from(SurfaceRadiometry::STEFAN_BOLTZMANN) * f64::from(ground.emissivity);
        ((absorbed / sigma_epsilon).powf(0.25) as f32).max(SHADOW_TEMPERATURE_K)
    }
}

/// Long-wavelength undulation of the plain, metres.
fn rolling_m(east: f64, north: f64) -> f64 {
    let scale = 1.0 / ROLLING_WAVELENGTH_M;
    (value_noise(ROLLING_SEED, east * scale, north * scale) - 0.5) * 2.0 * ROLLING_AMPLITUDE_M
}

impl ElevationSource for LunarTranquilityScenery {
    fn ground_height_m(&self, position: &Geodetic) -> Result<Option<f64>, SceneryError> {
        Ok(self
            .local(position)
            .map(|(east, north)| ground_at(east, north).height_m))
    }
}

impl RadiometricSource for LunarTranquilityScenery {
    fn surface_radiometry(
        &self,
        position: &Geodetic,
    ) -> Result<Option<SurfaceRadiometry>, SceneryError> {
        Ok(self.local(position).map(|(east, north)| {
            let ground = ground_at(east, north);
            SurfaceRadiometry {
                albedo: ground.albedo,
                emissivity: ground.emissivity,
                temperature_k: self.temperature_k(east, north, &ground),
            }
        }))
    }
}
