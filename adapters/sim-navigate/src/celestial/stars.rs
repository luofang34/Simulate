use glam::{DMat3, DVec3};
use navigate_celestial::{
    AnalyticEphemeris, Correction, Epoch, FrameId, StarSight, State, civil_time, moon_ecef,
    observe_body, stars_ecef, sun_ecef,
};

use super::FAINTEST_STAR_V_MAGNITUDE;
use super::frame::StarSkySample;
use super::moon::moon_orientation;

/// The catalogue's typical B-V, a G-type star, for stars it gives none.
const DEFAULT_B_MINUS_V: f64 = 0.65;
/// Valid range of the Kim et al. Planck-locus fit, kelvin.
const LOCUS_MIN_K: f64 = 1667.0;
const LOCUS_MAX_K: f64 = 25_000.0;
const SUN: i32 = 10;
const MOON: i32 = 301;
const EARTH_J2000: FrameId = FrameId {
    center_naif_id: 399,
    orientation_naif_id: 1,
};

/// The bright stars in ECEF axes at the civil reading `epoch`.
pub(super) fn earth_stars(epoch: Epoch) -> Option<Vec<StarSkySample>> {
    let stars = stars_ecef(epoch, FAINTEST_STAR_V_MAGNITUDE).ok()?;
    Some(
        stars
            .iter()
            .map(|star| sample(star, DVec3::from_array(star.direction)))
            .collect(),
    )
}

/// The bright stars in Moon-fixed axes. They keep the Earth's annual
/// aberration; the Moon's 1 km/s about the Earth would shift them under an
/// arcsecond.
pub(super) fn moon_stars(epoch: Epoch) -> Option<Vec<StarSkySample>> {
    let ecef_to_j2000 = ecef_to_j2000(epoch)?;
    let to_moon_fixed = moon_orientation(epoch);
    let stars = stars_ecef(epoch, FAINTEST_STAR_V_MAGNITUDE).ok()?;
    Some(
        stars
            .iter()
            .map(|star| {
                let j2000 = ecef_to_j2000 * DVec3::from_array(star.direction);
                sample(star, to_moon_fixed.j2000_to_fixed(j2000).normalize())
            })
            .collect(),
    )
}

/// The rotation Navigate applies from J2000 to ECEF, inverted. Navigate keeps
/// it private, so it is recovered by TRIAD from the geocentric Sun and Moon,
/// which the same observation model gives in both axes; the aberration is
/// identical on both sides and cancels.
pub(super) fn ecef_to_j2000(epoch: Epoch) -> Option<DMat3> {
    let sun_fixed = DVec3::from_array(sun_ecef(epoch, None).ok()?.direction);
    let moon_fixed = DVec3::from_array(moon_ecef(epoch, None).ok()?.direction);
    let observer = State {
        position_m: [0.0; 3],
        velocity_m_s: [0.0; 3],
        epoch: civil_time(epoch).terrestrial,
        frame: EARTH_J2000,
    };
    let inertial = |body| {
        observe_body(
            &AnalyticEphemeris,
            observer,
            body,
            Correction::VacuumReception,
        )
        .ok()
        .map(|sight| DVec3::from_array(sight.direction))
    };
    let fixed_basis = triad_basis(sun_fixed, moon_fixed)?;
    let inertial_basis = triad_basis(inertial(SUN)?, inertial(MOON)?)?;
    Some(inertial_basis * fixed_basis.transpose())
}

/// An orthonormal basis whose first axis is `primary` and whose second lies
/// along `primary × secondary`.
fn triad_basis(primary: DVec3, secondary: DVec3) -> Option<DMat3> {
    let first = primary.try_normalize()?;
    let second = first.cross(secondary).try_normalize()?;
    Some(DMat3::from_cols(first, second, first.cross(second)))
}

fn sample(star: &StarSight, direction_fixed: DVec3) -> StarSkySample {
    StarSkySample {
        direction_ecef: direction_fixed,
        v_magnitude: star.v_magnitude as f32,
        linear_rgb: star_rgb(star.b_minus_v.unwrap_or(DEFAULT_B_MINUS_V)),
    }
}

/// Linear sRGB of a black body at the temperature of a star of colour index
/// `b_minus_v`, scaled so the brightest channel is 1.
pub(super) fn star_rgb(b_minus_v: f64) -> [f32; 3] {
    let [x, y] = planck_chromaticity(ballesteros_temperature_k(b_minus_v));
    let (big_x, big_z) = (x / y, (1.0 - x - y) / y);
    let red = (3.2406 * big_x - 1.5372 - 0.4986 * big_z).max(0.0);
    let green = (-0.9689 * big_x + 1.8758 + 0.0415 * big_z).max(0.0);
    let blue = (0.0557 * big_x - 0.2040 + 1.0570 * big_z).max(0.0);
    let peak = red.max(green).max(blue);
    if peak <= 0.0 || !peak.is_finite() {
        return [1.0; 3];
    }
    [
        (red / peak) as f32,
        (green / peak) as f32,
        (blue / peak) as f32,
    ]
}

/// Ballesteros (2012) effective temperature from B-V, kelvin.
pub(super) fn ballesteros_temperature_k(b_minus_v: f64) -> f64 {
    let bv = b_minus_v.clamp(-0.4, 2.0);
    4600.0 * (1.0 / (0.92 * bv + 1.7) + 1.0 / (0.92 * bv + 0.62))
}

/// CIE 1931 chromaticity of the Planckian locus, Kim et al. (2002) cubic fit.
fn planck_chromaticity(temperature_k: f64) -> [f64; 2] {
    let t = temperature_k.clamp(LOCUS_MIN_K, LOCUS_MAX_K);
    let (t1, t2, t3) = (1e3 / t, 1e6 / (t * t), 1e9 / (t * t * t));
    let x = if t <= 4000.0 {
        -0.266_123_9 * t3 - 0.234_358_9 * t2 + 0.877_695_6 * t1 + 0.179_910
    } else {
        -3.025_846_9 * t3 + 2.107_037_9 * t2 + 0.222_634_7 * t1 + 0.240_390
    };
    let (x2, x3) = (x * x, x * x * x);
    let y = if t <= 2222.0 {
        -1.106_381_4 * x3 - 1.348_110_20 * x2 + 2.185_558_32 * x - 0.202_196_83
    } else if t <= 4000.0 {
        -0.954_947_6 * x3 - 1.374_185_93 * x2 + 2.091_370_15 * x - 0.167_488_67
    } else {
        3.081_758_0 * x3 - 5.873_386_70 * x2 + 3.751_129_97 * x - 0.370_014_83
    };
    [x, y]
}
