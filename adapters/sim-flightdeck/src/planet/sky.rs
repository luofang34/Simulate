//! Ephemeris-driven Sun, Moon, atmosphere, and ~5,000 Johnson-V star catalog evaluation
//! across declared planetary bodies (`Earth`, `Moon`, `Mars`).

use glam::{DMat3, DVec3};
use navigate_celestial::{
    AnalyticEphemeris, Correction, Epoch, FrameId, StarSight, State, civil_time, moon_ecef,
    observe_body, stars_ecef, sun_ecef,
};
use sim_geo::{InertialTransform, UtcTime};

use super::{PlanetaryBodyId, PlanetaryReference};

/// Number of `f32` values per packed star entry: `[dir_x, dir_y, dir_z, v_mag, r, g, b]`.
pub const STAR_PACKED_STRIDE_F32: usize = 7;

/// Faintest Johnson-V visual magnitude included in the real-time naked-eye starfield.
const FAINTEST_STAR_MAG: f64 = 6.5;
/// Dark-adapted perceptual exposure shift (`mag`) so 3rd–6th magnitude stars remain clearly visible on LCD/OLED panels.
const DISPLAY_MAG_EXPOSURE_OFFSET: f32 = 1.45;
/// Minimum clock change in seconds before recomputing the ~5,000-star body-fixed rotation.
const STAR_REFRESH_INTERVAL_S: f64 = 4.0;
/// Default B-V color index (solar-type G star) when a catalog entry omits B-V.
const DEFAULT_B_MINUS_V: f64 = 0.65;
const LOCUS_MIN_K: f64 = 1_667.0;
const LOCUS_MAX_K: f64 = 25_000.0;
const NAIF_SUN: i32 = 10;
const NAIF_MOON: i32 = 301;

/// Evaluated body-fixed sky lighting and starfield state for one UTC instant.
#[derive(Clone, Debug, PartialEq)]
pub struct SkySnapshot {
    /// Evaluated Unix timestamp (`s` UTC).
    pub utc_unix_s: f64,
    /// Unit Sun direction in `planet`'s body-fixed axes (`None` if ephemeris unavailable).
    pub sun_direction_fixed: Option<[f64; 3]>,
    /// Apparent solar angular radius (`deg`).
    pub sun_angular_radius_deg: f64,
    /// Topocentric unit Moon direction in `planet`'s body-fixed axes (`None` when standing on the Moon).
    pub moon_direction_fixed: Option<[f64; 3]>,
    /// Apparent lunar angular radius (`deg`).
    pub moon_angular_radius_deg: f64,
    /// Ambient night-side brightness fraction (`0..=1`) preserving chart/relief legibility.
    pub night_brightness: f32,
    /// Whether atmospheric limb scattering should be drawn for `planet`.
    pub atmosphere: bool,
    /// Whether the star buffer changed on this evaluation and should be re-uploaded to GPU.
    pub stars_updated: bool,
}

/// Throttled celestial sky evaluator holding the latest body-fixed star buffer.
#[derive(Clone, Debug, Default)]
pub struct SkyEvaluator {
    last_stars_utc_s: Option<f64>,
    last_stars_body: Option<PlanetaryBodyId>,
    packed_stars: Vec<f32>,
}

impl SkyEvaluator {
    /// Creates an empty [`SkyEvaluator`].
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Packed `[dir_x, dir_y, dir_z, v_mag, r, g, b]` `f32` buffer for the most recent evaluation.
    #[must_use]
    pub fn packed_stars_f32(&self) -> &[f32] {
        &self.packed_stars
    }

    /// Number of stars currently stored in [`Self::packed_stars_f32`].
    #[must_use]
    pub fn star_count(&self) -> u32 {
        (self.packed_stars.len() / STAR_PACKED_STRIDE_F32) as u32
    }

    /// Evaluates Sun, Moon, atmosphere, and body-fixed stars at `utc_unix_s` for an observer at
    /// `(observer_lat_deg, observer_lon_deg, observer_alt_datum_m)` on `planet`.
    pub fn evaluate(
        &mut self,
        planet: &PlanetaryReference,
        utc_unix_s: f64,
        observer_lat_deg: f64,
        observer_lon_deg: f64,
        observer_alt_datum_m: f64,
    ) -> SkySnapshot {
        let safe_utc = if utc_unix_s.is_finite() {
            utc_unix_s
        } else {
            1_788_134_400.0
        };
        let epoch = Epoch::from_unix_seconds(safe_utc);
        let obs_fixed =
            planet.fixed_position_m(observer_lat_deg, observer_lon_deg, observer_alt_datum_m);

        let (sun_dir, sun_rad_deg, moon_dir, moon_rad_deg) = match planet.id {
            PlanetaryBodyId::EarthWgs84Egm96 | PlanetaryBodyId::EarthWgs84Ellipsoid => {
                evaluate_earth_sun_moon(epoch, obs_fixed)
            }
            PlanetaryBodyId::MoonIauSphere => evaluate_non_earth_sun(
                epoch,
                NAIF_MOON,
                InertialTransform::moon_at(UtcTime::from_unix_seconds(safe_utc)),
            ),
            PlanetaryBodyId::MarsIauSpheroid => evaluate_non_earth_sun(
                epoch,
                499,
                InertialTransform::mars_at(UtcTime::from_unix_seconds(safe_utc)),
            ),
        };

        let need_stars = self.last_stars_body != Some(planet.id)
            || self
                .last_stars_utc_s
                .is_none_or(|prev| (safe_utc - prev).abs() >= STAR_REFRESH_INTERVAL_S);
        let mut stars_updated = false;
        if need_stars && let Some(fresh) = compute_packed_stars(planet.id, epoch, safe_utc) {
            self.packed_stars = fresh;
            self.last_stars_utc_s = Some(safe_utc);
            self.last_stars_body = Some(planet.id);
            stars_updated = true;
        }

        let night_brightness = if planet.has_atmosphere { 0.62 } else { 0.20 };
        SkySnapshot {
            utc_unix_s: safe_utc,
            sun_direction_fixed: sun_dir,
            sun_angular_radius_deg: sun_rad_deg,
            moon_direction_fixed: moon_dir,
            moon_angular_radius_deg: moon_rad_deg,
            night_brightness,
            atmosphere: planet.has_atmosphere && sun_dir.is_some(),
            stars_updated,
        }
    }
}

fn evaluate_earth_sun_moon(
    epoch: Epoch,
    obs_ecef_m: DVec3,
) -> (Option<[f64; 3]>, f64, Option<[f64; 3]>, f64) {
    let obs_opt = obs_ecef_m.is_finite().then_some(obs_ecef_m.to_array());
    let (sun_dir, sun_rad) = match sun_ecef(epoch, obs_opt) {
        Ok(s) => (Some(s.direction), s.angular_radius_rad.to_degrees()),
        Err(_) => (None, 0.266),
    };
    let (moon_dir, moon_rad) = match moon_ecef(epoch, obs_opt) {
        Ok(m) => (Some(m.direction), m.angular_radius_rad.to_degrees()),
        Err(_) => (None, 0.259),
    };
    (sun_dir, sun_rad, moon_dir, moon_rad)
}

fn evaluate_non_earth_sun(
    epoch: Epoch,
    center_naif_id: i32,
    inertial: InertialTransform,
) -> (Option<[f64; 3]>, f64, Option<[f64; 3]>, f64) {
    let observer = State {
        position_m: [0.0; 3],
        velocity_m_s: [0.0; 3],
        epoch: civil_time(epoch).terrestrial,
        frame: FrameId {
            center_naif_id,
            orientation_naif_id: 1,
        },
    };
    if let Ok(sight) = observe_body(
        &AnalyticEphemeris,
        observer,
        NAIF_SUN,
        Correction::VacuumReception,
    ) {
        let dir_j2000 = DVec3::from_array(sight.direction);
        let dir_fixed = inertial.j2000_to_fixed(dir_j2000).normalize_or_zero();
        if dir_fixed.length_squared() > 0.5 {
            // IAU 2015 nominal solar photosphere radius (695,700 km).
            let sun_rad_deg =
                libm::asin((695_700_000.0 / sight.coordinate_range_m.max(1.0e9)).clamp(0.0, 1.0))
                    .to_degrees();
            return (Some(dir_fixed.to_array()), sun_rad_deg, None, 0.0);
        }
    }
    (None, 0.266, None, 0.0)
}

fn compute_packed_stars(body: PlanetaryBodyId, epoch: Epoch, utc_unix_s: f64) -> Option<Vec<f32>> {
    let stars = stars_ecef(epoch, FAINTEST_STAR_MAG).ok()?;
    let rotation_from_earth_ecef = match body {
        PlanetaryBodyId::EarthWgs84Egm96 | PlanetaryBodyId::EarthWgs84Ellipsoid => DMat3::IDENTITY,
        PlanetaryBodyId::MoonIauSphere => {
            let ecef_to_j2000 = recover_ecef_to_j2000(epoch)?;
            let moon_tf = InertialTransform::moon_at(UtcTime::from_unix_seconds(utc_unix_s));
            DMat3::from_quat(moon_tf.fixed_to_j2000_rotation().inverse()) * ecef_to_j2000
        }
        PlanetaryBodyId::MarsIauSpheroid => {
            let ecef_to_j2000 = recover_ecef_to_j2000(epoch)?;
            let mars_tf = InertialTransform::mars_at(UtcTime::from_unix_seconds(utc_unix_s));
            DMat3::from_quat(mars_tf.fixed_to_j2000_rotation().inverse()) * ecef_to_j2000
        }
    };

    let mut out = Vec::with_capacity(stars.len().saturating_mul(STAR_PACKED_STRIDE_F32));
    for star in &stars {
        append_packed_star(&mut out, star, &rotation_from_earth_ecef);
    }
    Some(out)
}

fn append_packed_star(out: &mut Vec<f32>, star: &StarSight, rot: &DMat3) {
    let dir = (*rot * DVec3::from_array(star.direction)).normalize_or_zero();
    if dir.length_squared() < 0.5 {
        return;
    }
    let [r, g, b] = star_linear_srgb(star.b_minus_v.unwrap_or(DEFAULT_B_MINUS_V));
    let display_mag = (star.v_magnitude as f32 - DISPLAY_MAG_EXPOSURE_OFFSET).max(-1.6);
    out.extend_from_slice(&[
        dir.x as f32,
        dir.y as f32,
        dir.z as f32,
        display_mag,
        r,
        g,
        b,
    ]);
}

fn recover_ecef_to_j2000(epoch: Epoch) -> Option<DMat3> {
    let sun_fixed = DVec3::from_array(sun_ecef(epoch, None).ok()?.direction);
    let moon_fixed = DVec3::from_array(moon_ecef(epoch, None).ok()?.direction);
    let observer = State {
        position_m: [0.0; 3],
        velocity_m_s: [0.0; 3],
        epoch: civil_time(epoch).terrestrial,
        frame: FrameId {
            center_naif_id: 399,
            orientation_naif_id: 1,
        },
    };
    let sun_j2000 = DVec3::from_array(
        observe_body(
            &AnalyticEphemeris,
            observer,
            NAIF_SUN,
            Correction::VacuumReception,
        )
        .ok()?
        .direction,
    );
    let moon_j2000 = DVec3::from_array(
        observe_body(
            &AnalyticEphemeris,
            observer,
            NAIF_MOON,
            Correction::VacuumReception,
        )
        .ok()?
        .direction,
    );
    let fixed_basis = triad_basis(sun_fixed, moon_fixed)?;
    let j2000_basis = triad_basis(sun_j2000, moon_j2000)?;
    Some(j2000_basis * fixed_basis.transpose())
}

fn triad_basis(primary: DVec3, secondary: DVec3) -> Option<DMat3> {
    let first = primary.try_normalize()?;
    let second = first.cross(secondary).try_normalize()?;
    Some(DMat3::from_cols(first, second, first.cross(second)))
}

fn star_linear_srgb(b_minus_v: f64) -> [f32; 3] {
    let bv = b_minus_v.clamp(-0.4, 2.0);
    let temp_k = (4_600.0 * (1.0 / (0.92 * bv + 1.7) + 1.0 / (0.92 * bv + 0.62)))
        .clamp(LOCUS_MIN_K, LOCUS_MAX_K);
    let [x, y] = planck_cie_xy(temp_k);
    let (big_x, big_z) = (x / y, (1.0 - x - y) / y);
    let r = (3.2406 * big_x - 1.5372 - 0.4986 * big_z).max(0.0);
    let g = (-0.9689 * big_x + 1.8758 + 0.0415 * big_z).max(0.0);
    let b = (0.0557 * big_x - 0.2040 + 1.0570 * big_z).max(0.0);
    let peak = r.max(g).max(b);
    if peak <= 0.0 || !peak.is_finite() {
        return [1.0; 3];
    }
    [(r / peak) as f32, (g / peak) as f32, (b / peak) as f32]
}

fn planck_cie_xy(temp_k: f64) -> [f64; 2] {
    let inv = 1_000.0 / temp_k;
    let inv2 = inv * inv;
    let inv3 = inv2 * inv;
    let x = if temp_k <= 4_000.0 {
        -0.266_123_9 * inv3 - 0.234_358_9 * inv2 + 0.877_695_6 * inv + 0.179_910
    } else {
        -3.025_846_9 * inv3 + 2.107_037_9 * inv2 + 0.222_634_7 * inv + 0.240_390
    };
    let x2 = x * x;
    let x3 = x2 * x;
    let y = if temp_k <= 2_222.0 {
        -1.106_381_4 * x3 - 1.348_110_2 * x2 + 2.185_558_32 * x - 0.202_196_83
    } else if temp_k <= 4_000.0 {
        -0.954_947_6 * x3 - 1.374_185_93 * x2 + 2.091_370_15 * x - 0.167_488_67
    } else {
        3.081_758_0 * x3 - 5.873_386_7 * x2 + 3.751_129_97 * x - 0.370_014_83
    };
    [x, y]
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn evaluates_earth_and_moon_starfields_and_atmosphere_flags() {
        let mut sky = SkyEvaluator::new();
        let earth = PlanetaryReference::earth_wgs84_egm96();
        let snap_earth = sky.evaluate(&earth, 1_788_116_400.0, 40.5, -74.6, 450.0);
        assert!(snap_earth.atmosphere);
        assert!(snap_earth.sun_direction_fixed.is_some());
        assert!(snap_earth.moon_direction_fixed.is_some());
        assert!(snap_earth.stars_updated);
        assert!(sky.star_count() > 4_000);

        let moon = PlanetaryReference::moon_iau();
        let snap_moon = sky.evaluate(&moon, 1_788_116_400.0, 0.674, 23.473, 0.0);
        assert!(!snap_moon.atmosphere);
        assert!(snap_moon.sun_direction_fixed.is_some());
        assert!(snap_moon.moon_direction_fixed.is_none());
        assert!(snap_moon.stars_updated);
        assert!(sky.star_count() > 4_000);
    }
}
