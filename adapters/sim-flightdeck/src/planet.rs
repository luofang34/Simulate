//! Explicit celestial body ellipsoid, vertical datum, gravity, local ENU geodesy, and sky
//! ephemeris declarations for `sim-flightdeck`.
//!
//! Nothing in `sim-flightdeck` may silently assume Earth or `WGS-84`: callers declare a
//! [`PlanetaryReference`] explicitly, delegating ellipsoid/spheroid geometry, gravity,
//! `EGM96` geoid undulation, and J2000/lunar orientation to `sim-geo` and `navigate-celestial`.

/// Ephemeris-driven Sun, Moon, atmosphere, and Johnson-V starfield evaluation per body.
pub mod sky;

use glam::DVec3;
use sim_geo::{
    BodyEllipsoid, Geodetic, GravityField, LocalFrame, VerticalDatumKind, egm96_undulation_m,
};
use sim_scenery::CatalogVerticalDatum;
use world_core::GeodeticPosition;

pub use sky::{STAR_PACKED_STRIDE_F32, SkyEvaluator, SkySnapshot};

/// Canonical `maplibre` spherical globe radius for `WGS-84` Earth (`m`), matching `sim_maplibre`.
pub const EARTH_RENDER_RADIUS_M: f64 = 6_371_008.8;
/// Highest Earth terrain bound (`m`, Mount Everest) for `maplibre` horizon culling.
pub const EARTH_HIGHEST_GROUND_M: f64 = 8_849.0;
/// Lowest Earth sub-aerial basin bound (`m`, Dead Sea) for `maplibre` far-plane culling.
pub const EARTH_LOWEST_GROUND_M: f64 = -450.0;

/// Canonical `maplibre` spherical globe radius for the `IAU` Moon (`m`), matching `sim_maplibre`.
pub const MOON_RENDER_RADIUS_M: f64 = 1_737_400.0;
/// Highest Lunar terrain bound (`m`, Selenean summit).
pub const MOON_HIGHEST_GROUND_M: f64 = 10_786.0;
/// Lowest Lunar terrain bound (`m`, South Pole-Aitken basin).
pub const MOON_LOWEST_GROUND_M: f64 = -9_120.0;

/// Highest Martian terrain bound (`m`, Olympus Mons).
pub const MARS_HIGHEST_GROUND_M: f64 = 21_287.0;
/// Lowest Martian terrain bound (`m`, Hellas Planitia).
pub const MARS_LOWEST_GROUND_M: f64 = -8_200.0;

/// Explicit identifier for supported planetary reference configurations across FFI and wire APIs.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
#[repr(u8)]
pub enum PlanetaryBodyId {
    /// Earth on the `WGS-84` ellipsoid with orthometric heights above the `EGM96` geoid (`MSL`).
    EarthWgs84Egm96 = 0,
    /// Earth on the `WGS-84` ellipsoid with pure ellipsoidal (`HAE`) heights.
    EarthWgs84Ellipsoid = 1,
    /// The Moon on the `IAU` mean sphere (`R = 1,737.4 km`, `GRAIL` `mu`/`J2`, zero atmosphere).
    MoonIauSphere = 2,
    /// Mars on the `IAU` reference spheroid (`a = 3,396.19 km`, `b = 3,376.20 km`).
    MarsIauSpheroid = 3,
}

impl PlanetaryBodyId {
    /// Decodes an explicit integer code (`0..=3`), returning `None` for unknown codes rather than
    /// silently defaulting to Earth.
    #[must_use]
    pub const fn from_code(code: u8) -> Option<Self> {
        match code {
            0 => Some(Self::EarthWgs84Egm96),
            1 => Some(Self::EarthWgs84Ellipsoid),
            2 => Some(Self::MoonIauSphere),
            3 => Some(Self::MarsIauSpheroid),
            _ => None,
        }
    }

    /// Integer code (`0..=3`) for FFI serialization.
    #[must_use]
    pub const fn code(self) -> u8 {
        self as u8
    }
}

/// Explicit declaration of a celestial body's physical ellipsoid, vertical datum, atmosphere,
/// and spherical render bounds.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PlanetaryReference {
    /// Canonical preset identifier.
    pub id: PlanetaryBodyId,
    /// Physical ellipsoid and `J2` gravity field from `sim-geo`.
    pub ellipsoid: BodyEllipsoid,
    /// Vertical datum from which altitudes are measured.
    pub vertical_datum: VerticalDatumKind,
    /// Mean spherical radius (`m`) passed explicitly to the 3D globe renderer.
    pub render_radius_m: f64,
    /// Highest terrain bound (`m`) above the mean radius for horizon culling.
    pub highest_ground_m: f64,
    /// Lowest terrain bound (`m`) relative to the mean radius.
    pub lowest_ground_m: f64,
    /// Whether the body has an atmosphere that scatters sunlight at the limb.
    pub has_atmosphere: bool,
}

impl PlanetaryReference {
    /// Explicit constructor for Earth (`WGS-84` ellipsoid + `EGM96` `MSL` geoid).
    #[must_use]
    pub const fn earth_wgs84_egm96() -> Self {
        Self {
            id: PlanetaryBodyId::EarthWgs84Egm96,
            ellipsoid: BodyEllipsoid::WGS84,
            vertical_datum: VerticalDatumKind::OrthometricMslEgm96,
            render_radius_m: EARTH_RENDER_RADIUS_M,
            highest_ground_m: EARTH_HIGHEST_GROUND_M,
            lowest_ground_m: EARTH_LOWEST_GROUND_M,
            has_atmosphere: true,
        }
    }

    /// Explicit constructor for Earth (`WGS-84` ellipsoid + ellipsoidal `HAE` vertical datum).
    #[must_use]
    pub const fn earth_wgs84_ellipsoidal() -> Self {
        Self {
            id: PlanetaryBodyId::EarthWgs84Ellipsoid,
            ellipsoid: BodyEllipsoid::WGS84,
            vertical_datum: VerticalDatumKind::EllipsoidWgs84,
            render_radius_m: EARTH_RENDER_RADIUS_M,
            highest_ground_m: EARTH_HIGHEST_GROUND_M,
            lowest_ground_m: EARTH_LOWEST_GROUND_M,
            has_atmosphere: true,
        }
    }

    /// Explicit constructor for the Moon (`IAU` `1,737.4 km` sphere, `GRAIL` gravity, vacuum sky).
    #[must_use]
    pub const fn moon_iau() -> Self {
        Self {
            id: PlanetaryBodyId::MoonIauSphere,
            ellipsoid: BodyEllipsoid::MOON_IAU,
            vertical_datum: VerticalDatumKind::LunarSphereIau,
            render_radius_m: MOON_RENDER_RADIUS_M,
            highest_ground_m: MOON_HIGHEST_GROUND_M,
            lowest_ground_m: MOON_LOWEST_GROUND_M,
            has_atmosphere: false,
        }
    }

    /// Explicit constructor for Mars (`IAU` oblate spheroid, thin atmosphere).
    #[must_use]
    pub const fn mars_iau() -> Self {
        Self {
            id: PlanetaryBodyId::MarsIauSpheroid,
            ellipsoid: BodyEllipsoid::MARS_IAU,
            vertical_datum: VerticalDatumKind::EllipsoidWgs84,
            render_radius_m: BodyEllipsoid::MARS_IAU.mean_radius_m(),
            highest_ground_m: MARS_HIGHEST_GROUND_M,
            lowest_ground_m: MARS_LOWEST_GROUND_M,
            has_atmosphere: true,
        }
    }

    /// Resolves a [`PlanetaryBodyId`] into its full [`PlanetaryReference`].
    #[must_use]
    pub const fn from_id(id: PlanetaryBodyId) -> Self {
        match id {
            PlanetaryBodyId::EarthWgs84Egm96 => Self::earth_wgs84_egm96(),
            PlanetaryBodyId::EarthWgs84Ellipsoid => Self::earth_wgs84_ellipsoidal(),
            PlanetaryBodyId::MoonIauSphere => Self::moon_iau(),
            PlanetaryBodyId::MarsIauSpheroid => Self::mars_iau(),
        }
    }

    /// Concise English aviation/astronautics label declaring both body and datum.
    #[must_use]
    pub const fn label(&self) -> &'static str {
        match self.id {
            PlanetaryBodyId::EarthWgs84Egm96 => "EARTH · WGS-84 / EGM96 MSL",
            PlanetaryBodyId::EarthWgs84Ellipsoid => "EARTH · WGS-84 HAE",
            PlanetaryBodyId::MoonIauSphere => "MOON · IAU 1737.4 KM",
            PlanetaryBodyId::MarsIauSpheroid => "MARS · IAU SPHEROID",
        }
    }

    /// Whether this reference body is `WGS-84` Earth (permitting Earth-specific `WMM2025`,
    /// `FAA NASR`, and `ADS-B` `1090ES` models).
    #[must_use]
    pub fn is_earth_wgs84(&self) -> bool {
        self.ellipsoid == BodyEllipsoid::WGS84
    }

    /// Maps `self.vertical_datum` to `sim-scenery`'s [`CatalogVerticalDatum`] when on `WGS-84`
    /// Earth, or `None` on non-Earth bodies so Earth airport/scenery databases are never assumed.
    #[must_use]
    pub fn catalog_vertical_datum(&self) -> Option<CatalogVerticalDatum> {
        if !self.is_earth_wgs84() {
            return None;
        }
        match self.vertical_datum {
            VerticalDatumKind::OrthometricMslEgm96 => {
                Some(CatalogVerticalDatum::OrthometricMslEgm96)
            }
            VerticalDatumKind::EllipsoidWgs84 | VerticalDatumKind::LunarSphereIau => {
                Some(CatalogVerticalDatum::EllipsoidWgs84)
            }
        }
    }

    /// Position of `(target_lat_deg, target_lon_deg, target_alt_datum_m)` in `eye`'s local
    /// North-East-Down (`NED`) tangent frame on `self.render_radius_m`, matching `maplibre`'s
    /// spherical globe rendering with zero ellipsoid-vs-sphere skew; `None` past the horizon cap.
    #[must_use]
    pub fn render_globe_ned_between(
        &self,
        eye_lat_deg: f64,
        eye_lon_deg: f64,
        eye_alt_datum_m: f64,
        target_lat_deg: f64,
        target_lon_deg: f64,
        target_alt_datum_m: f64,
    ) -> Option<DVec3> {
        if !self.render_radius_m.is_finite() || self.render_radius_m <= 0.0 {
            return None;
        }
        let (elat, elon) = (eye_lat_deg.to_radians(), eye_lon_deg.to_radians());
        let (tlat, tlon) = (target_lat_deg.to_radians(), target_lon_deg.to_radians());
        let eye_u = DVec3::new(
            libm::cos(elat) * libm::sin(elon),
            libm::sin(elat),
            libm::cos(elat) * libm::cos(elon),
        );
        let tgt_u = DVec3::new(
            libm::cos(tlat) * libm::sin(tlon),
            libm::sin(tlat),
            libm::cos(tlat) * libm::cos(tlon),
        );
        if eye_u.dot(tgt_u) <= 0.0 {
            return None;
        }
        let r_eye = self.render_radius_m + eye_alt_datum_m;
        let r_tgt = self.render_radius_m + target_alt_datum_m;
        let delta = tgt_u * r_tgt - eye_u * r_eye;
        let east = DVec3::new(libm::cos(elon), 0.0, -libm::sin(elon));
        let north = eye_u.cross(east);
        Some(DVec3::new(
            delta.dot(north),
            delta.dot(east),
            -delta.dot(eye_u),
        ))
    }

    /// Geoid undulation `N` (`m`) separating the declared vertical datum from the reference
    /// ellipsoid (`h_ellip = H_datum + N`).
    #[must_use]
    pub fn datum_undulation_m(&self, lat_rad: f64, lon_rad: f64) -> f64 {
        if self.is_earth_wgs84() && self.vertical_datum == VerticalDatumKind::OrthometricMslEgm96 {
            egm96_undulation_m(lat_rad, lon_rad)
        } else {
            0.0
        }
    }

    /// Builds a `sim-geo` [`Geodetic`] (whose `height_m` is ellipsoidal) from planetographic
    /// degrees and height above `self.vertical_datum`.
    #[must_use]
    pub fn geodetic_from_datum(&self, lat_deg: f64, lon_deg: f64, height_datum_m: f64) -> Geodetic {
        let lat_rad = lat_deg.to_radians();
        let lon_rad = lon_deg.to_radians();
        let height_ellip_m = height_datum_m + self.datum_undulation_m(lat_rad, lon_rad);
        Geodetic {
            lat_rad,
            lon_rad,
            height_m: height_ellip_m,
        }
    }

    /// Converts a `sim-geo` [`Geodetic`] height back into height above `self.vertical_datum`.
    #[must_use]
    pub fn height_above_datum_m(&self, geo: &Geodetic) -> f64 {
        geo.height_m - self.datum_undulation_m(geo.lat_rad, geo.lon_rad)
    }

    /// Body-fixed (`ECEF` / principal-axis) position vector in metres for `(lat_deg, lon_deg, h_datum_m)`.
    #[must_use]
    pub fn fixed_position_m(&self, lat_deg: f64, lon_deg: f64, height_datum_m: f64) -> DVec3 {
        let geo = self.geodetic_from_datum(lat_deg, lon_deg, height_datum_m);
        self.ellipsoid.geodetic_to_fixed(&geo)
    }

    /// Exact local tangent East-North-Up (`ENU`) displacement in metres from `origin` to `target`
    /// on `self.ellipsoid`.
    #[must_use]
    pub fn enu_between(&self, origin: &Geodetic, target: &Geodetic) -> DVec3 {
        let p0 = self.ellipsoid.geodetic_to_fixed(origin);
        let p1 = self.ellipsoid.geodetic_to_fixed(target);
        LocalFrame::at(origin).ecef_to_enu_vector(p1 - p0)
    }

    /// Displaces `origin` by `enu_m` (`[east_m, north_m, up_m]`) in `origin`'s tangent frame and
    /// projects back onto `self.ellipsoid` via closed-form Heikkinen/spherical inversion.
    #[must_use]
    pub fn offset_geodetic_enu(&self, origin: &Geodetic, enu_m: DVec3) -> Geodetic {
        let p0 = self.ellipsoid.geodetic_to_fixed(origin);
        let p1 = p0 + LocalFrame::at(origin).enu_to_ecef() * enu_m;
        self.ellipsoid.fixed_to_geodetic(p1)
    }

    /// Local gravitational acceleration magnitude (`m/s²`) at `position` from `sim-geo`'s
    /// `J2` [`GravityField`] (`~9.80 m/s²` on Earth, `~1.62 m/s²` on the Moon, `~3.71 m/s²` on Mars).
    #[must_use]
    pub fn gravity_mps2(&self, position: &Geodetic) -> f64 {
        let fixed = self.ellipsoid.geodetic_to_fixed(position);
        let g = self.ellipsoid.attraction_fixed(fixed).length();
        if g.is_finite() && g > 0.01 {
            g
        } else {
            self.ellipsoid.mu_m3_s2 / (self.ellipsoid.semi_major_m * self.ellipsoid.semi_major_m)
        }
    }

    /// Constructs a `world_core::GeodeticPosition` **only** when `self.is_earth_wgs84()` holds,
    /// returning `None` on non-Earth bodies so Earth-only crates (`world-geomag`, `world-notices`,
    /// `world-surveillance`) are never fed lunar or Martian coordinates.
    #[must_use]
    pub fn earth_wgs84_position(
        &self,
        lat_deg: f64,
        lon_deg: f64,
        ellipsoidal_height_m: f64,
    ) -> Option<GeodeticPosition> {
        if !self.is_earth_wgs84() {
            return None;
        }
        let pos = GeodeticPosition::wgs84(lat_deg, lon_deg, ellipsoidal_height_m);
        pos.is_well_formed().then_some(pos)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn planetary_gravity_and_enu_scale_follow_declared_body() {
        let earth = PlanetaryReference::earth_wgs84_egm96();
        let moon = PlanetaryReference::moon_iau();
        let mars = PlanetaryReference::mars_iau();

        let g_earth = earth.gravity_mps2(&earth.geodetic_from_datum(40.0, -74.5, 0.0));
        let g_moon = moon.gravity_mps2(&moon.geodetic_from_datum(0.674, 23.473, 0.0));
        let g_mars = mars.gravity_mps2(&mars.geodetic_from_datum(18.44, 77.45, 0.0));

        assert!((g_earth - 9.81).abs() < 0.05);
        assert!((g_moon - 1.62).abs() < 0.03);
        assert!((g_mars - 3.72).abs() < 0.05);

        // 1 degree of longitude at the equator spans ~111.3 km on Earth vs ~30.3 km on the Moon.
        let e0 = earth.geodetic_from_datum(0.0, 0.0, 0.0);
        let e1 = earth.geodetic_from_datum(0.0, 1.0, 0.0);
        let m0 = moon.geodetic_from_datum(0.0, 0.0, 0.0);
        let m1 = moon.geodetic_from_datum(0.0, 1.0, 0.0);
        assert!((earth.enu_between(&e0, &e1).x - 111_319.0).abs() < 50.0);
        assert!((moon.enu_between(&m0, &m1).x - 30_323.0).abs() < 20.0);
        assert!(moon.earth_wgs84_position(0.0, 0.0, 0.0).is_none());
    }
}
