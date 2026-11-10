//! Daylight that lights the airframe: the Sun's beam, reddened and dimmed through the air
//! mass it crosses, and a clear sky whose radiance follows the CIE standard clear sky
//! (type 12), bright around the Sun and toward the horizon, blue at the zenith.
//!
//! The sky dome is projected each frame onto nine spherical harmonics and convolved with
//! the cosine lobe, so a shader reads the irradiance on any normal from nine coefficients.
//! The ground below the horizon is a uniform diffuse reflector lit by Sun and sky; its
//! irradiance on a normal is closed-form and stays separate, so the pass can dim it where
//! the ground lies in the airframe's own shadow.
//!
//! Magnitudes are in the shader's units, where `SUN_ZENITH` is the beam irradiance with the
//! Sun overhead. A clear sky gives a horizontal surface about an eighth of that, and less
//! as the Sun sinks: the ratio that sets how dark the shaded side of an airframe reads.

use glam::{DVec3, Vec3};

/// Bytes of the shader's `Sky` uniform block.
pub(crate) const SKY_BYTES: usize = 208;

/// Beam irradiance above the atmosphere in the shader's units: a scale, chosen so the
/// beam of an overhead Sun after [`OPTICAL_DEPTH`] is about 3 (assumed; every other
/// magnitude here is a ratio to it).
const SUN_EXTRATERRESTRIAL: [f32; 3] = [3.55, 3.55, 3.55];
/// Vertical optical depth per channel: Rayleigh scattering at about 610, 550 and 465 nm
/// (0.06, 0.09, 0.19 at sea level) plus a grey aerosol of 0.05 (assumed: clean air,
/// visibility of tens of kilometres).
const OPTICAL_DEPTH: [f32; 3] = [0.11, 0.14, 0.24];
/// Diffuse reflectance of the ground around an airfield: grass, soil and concrete
/// (assumed; typical broadband albedos of those surfaces are 0.15 to 0.3).
const GROUND_ALBEDO: [f32; 3] = [0.20, 0.19, 0.16];
/// Zenith and horizon colour of a clear sky, before the CIE luminance distribution
/// (assumed).
const ZENITH_RGB: [f32; 3] = [0.30, 0.48, 1.0];
const HORIZON_RGB: [f32; 3] = [0.78, 0.86, 1.0];
/// Sky irradiance on a horizontal surface as a share of `SUN_EXTRATERRESTRIAL`: a floor at
/// night (assumed: moon and airglow are far dimmer, but a black airframe is useless to a
/// pilot) and the clear-day value at high Sun (a clear sky's diffuse horizontal
/// irradiance is typically 100 to 150 W/m² against 1361 W/m² above the atmosphere).
const NIGHT_SKY_SHARE: f32 = 0.006;
const DAY_SKY_SHARE: f32 = 0.115;

/// Most an exposure opens up for dusk and night, so a night airframe stays dim (assumed).
const MAX_EXPOSURE_GAIN: f32 = 6.0;
/// Share (as an exponent) of the light's colour cast a camera's or the eye's white
/// balance removes: most of a low Sun's warmth, not all of it (assumed).
const WHITE_BALANCE: f32 = 0.7;
/// Sun elevation of the clear noon the exposure gain is relative to.
const NOON_ELEVATION_DEG: f32 = 70.0;

/// Grid of the numerical sky projection (zenith rings by azimuth sectors).
const RINGS: usize = 24;
const SECTORS: usize = 48;

/// The daylight one slot is lit by, in its anchor East-North-Up frame.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct SkyLight {
    /// Cosine-convolved sky-dome irradiance as nine spherical-harmonic coefficients (rgb).
    pub(crate) irradiance_sh: [Vec3; 9],
    /// Beam irradiance at normal incidence.
    pub(crate) sun: Vec3,
    /// Clear-sky radiance at the zenith, and the CIE normaliser for the shader's lookups.
    pub(crate) zenith: Vec3,
    pub(crate) cie_zenith: f32,
    /// Radiance of the horizon band's colour at the zenith's luminance.
    pub(crate) horizon: Vec3,
    /// Radiance of the sunlit ground below the horizon.
    pub(crate) ground: Vec3,
    /// Exposure relative to a clear noon: a camera, or an eye, opens up as the light on a
    /// subject's sunlit side falls, so a dusk airframe is not drawn black.
    pub(crate) exposure: f32,
    /// Share of [`Self::ground`] left where the ground is shaded from the beam: the sky's
    /// part of the light falling on it.
    pub(crate) shaded_ground: f32,
}

impl SkyLight {
    /// Daylight with the Sun along `to_sun` (unit, ENU) and the pass's `daylight` factor.
    pub(crate) fn new(to_sun: DVec3, daylight: f32) -> Self {
        let s = to_sun.normalize_or(DVec3::Z).as_vec3();
        let elevation = s.z.clamp(-1.0, 1.0).asin();
        let sun = beam(elevation) * daylight;
        let cie_zenith = cie_clear(Vec3::Z, s);
        // The dome's shape at unit zenith radiance, then scaled to the target irradiance.
        let unit = Shape {
            to_sun: s,
            zenith: Vec3::from(ZENITH_RGB),
            horizon: Vec3::from(HORIZON_RGB),
            cie_zenith,
        };
        let raw = project(&unit);
        let horizontal = basis(Vec3::Z)
            .iter()
            .zip(&raw)
            .map(|(y, c)| c.y * y)
            .sum::<f32>();
        let share = sky_share(daylight, elevation.sin());
        let scale = share * SUN_EXTRATERRESTRIAL[1] / horizontal.max(1e-6);
        let irradiance_sh = raw.map(|c| c * scale);
        let sky_on_ground = horizontal * scale;
        let beam_on_ground = sun * s.z.max(0.0);
        let ground = Vec3::from(GROUND_ALBEDO) * (beam_on_ground + Vec3::splat(sky_on_ground))
            / std::f32::consts::PI;
        let global = beam_on_ground.y + sky_on_ground;
        let shaded_ground = sky_on_ground / global.max(1e-6);
        let exposure =
            (noon_card() / card(sun.y, sky_on_ground).max(1e-4)).clamp(1.0, MAX_EXPOSURE_GAIN);
        // The light on the ground, by channel, is what a camera or the eye balances to.
        let sky_rgb = basis(Vec3::Z)
            .iter()
            .zip(&irradiance_sh)
            .map(|(y, c)| *c * *y)
            .sum::<Vec3>();
        let lit = (beam_on_ground + sky_rgb).max(Vec3::splat(1e-6));
        let balance = (Vec3::splat(lit.y) / lit).powf(WHITE_BALANCE);
        Self {
            irradiance_sh: irradiance_sh.map(|c| c * balance),
            sun: sun * balance,
            zenith: unit.zenith * scale * balance,
            cie_zenith,
            horizon: unit.horizon * scale * balance,
            ground: ground * balance,
            exposure,
            shaded_ground,
        }
    }

    /// Sky-dome irradiance on a surface facing `n`, from the coefficients; the shader's
    /// `sky_irradiance` evaluates the same sum.
    #[cfg(test)]
    pub(crate) fn sky_irradiance(&self, n: Vec3) -> Vec3 {
        basis(n)
            .iter()
            .zip(&self.irradiance_sh)
            .map(|(y, c)| *c * *y)
            .sum::<Vec3>()
            .max(Vec3::ZERO)
    }

    /// The shader's `Sky` block, packed little-endian.
    pub(crate) fn block(&self) -> [u8; SKY_BYTES] {
        let rows = self.irradiance_sh.iter().map(|c| c.extend(0.0)).chain([
            self.sun.extend(self.exposure),
            self.zenith.extend(self.cie_zenith),
            self.horizon.extend(0.0),
            self.ground.extend(self.shaded_ground),
        ]);
        let mut out = [0_u8; SKY_BYTES];
        let floats = rows.flat_map(|row| row.to_array());
        for (chunk, value) in out.as_chunks_mut::<4>().0.iter_mut().zip(floats) {
            *chunk = value.to_le_bytes();
        }
        out
    }
}

/// Irradiance (green) on a grey card held facing the Sun, which a photographer meters
/// the lit side of a subject by: the beam at normal incidence and about half the sky
/// (`sky_on_ground` is the whole dome's on a level surface). Exposing for it keeps a
/// sunlit panel at one tone from noon to a low Sun, where the level ground gets far less.
fn card(beam_green: f32, sky_on_ground: f32) -> f32 {
    beam_green + 0.5 * sky_on_ground
}

/// [`card`] at a clear noon.
fn noon_card() -> f32 {
    let elevation = NOON_ELEVATION_DEG.to_radians();
    card(
        beam(elevation).y,
        sky_share(1.0, elevation.sin()) * SUN_EXTRATERRESTRIAL[1],
    )
}

/// Sky irradiance on the ground as a share of `SUN_EXTRATERRESTRIAL`, for a Sun whose
/// elevation has sine `sine`.
fn sky_share(daylight: f32, sine: f32) -> f32 {
    NIGHT_SKY_SHARE + (DAY_SKY_SHARE - NIGHT_SKY_SHARE) * daylight * sine.max(0.0).sqrt().max(0.25)
}

/// Beam irradiance after the air mass of a Sun at `elevation` (rad), Kasten–Young.
fn beam(elevation: f32) -> Vec3 {
    let degrees = elevation.to_degrees();
    if degrees <= -0.5 {
        return Vec3::ZERO;
    }
    let air_mass = 1.0 / (elevation.max(0.0).sin() + 0.505_72 * (degrees + 6.079_95).powf(-1.6364));
    Vec3::from(SUN_EXTRATERRESTRIAL) * Vec3::from(OPTICAL_DEPTH).map(|tau| (-tau * air_mass).exp())
}

/// What the sky's radiance depends on, before scaling.
pub(crate) struct Shape {
    pub(crate) to_sun: Vec3,
    pub(crate) zenith: Vec3,
    pub(crate) horizon: Vec3,
    pub(crate) cie_zenith: f32,
}

impl Shape {
    /// Radiance along `dir` (unit, upper hemisphere). The shader's `sky_radiance` mirrors it.
    pub(crate) fn radiance(&self, dir: Vec3) -> Vec3 {
        let up = dir.z.max(0.0);
        let colour = self.horizon.lerp(self.zenith, up.sqrt());
        colour * (cie_clear(dir, self.to_sun) / self.cie_zenith.max(1e-6))
    }
}

/// CIE standard clear sky (type 12) relative luminance: gradation times indicatrix.
pub(crate) fn cie_clear(dir: Vec3, to_sun: Vec3) -> f32 {
    let cos_zenith = dir.z.max(0.01);
    let gradation = 1.0 - (-0.32 / cos_zenith).exp();
    let chi = dir.dot(to_sun).clamp(-1.0, 1.0).acos();
    let indicatrix = 1.0
        + 10.0 * ((-3.0 * chi).exp() - (-3.0 * std::f32::consts::FRAC_PI_2).exp())
        + 0.45 * chi.cos().powi(2);
    gradation * indicatrix
}

/// Real spherical-harmonic basis to band 2 at unit `n`.
pub(crate) fn basis(n: Vec3) -> [f32; 9] {
    [
        0.282_095,
        0.488_603 * n.y,
        0.488_603 * n.z,
        0.488_603 * n.x,
        1.092_548 * n.x * n.y,
        1.092_548 * n.y * n.z,
        0.315_392 * (3.0 * n.z * n.z - 1.0),
        1.092_548 * n.x * n.z,
        0.546_274 * (n.x * n.x - n.y * n.y),
    ]
}

/// Projects the upper-hemisphere radiance of `shape` and convolves it with the clamped
/// cosine (Ramamoorthi and Hanrahan), giving irradiance coefficients.
fn project(shape: &Shape) -> [Vec3; 9] {
    let mut coefficients = [Vec3::ZERO; 9];
    let d_theta = std::f32::consts::FRAC_PI_2 / RINGS as f32;
    let d_phi = std::f32::consts::TAU / SECTORS as f32;
    for ring in 0..RINGS {
        let theta = (ring as f32 + 0.5) * d_theta;
        let (sin_t, cos_t) = theta.sin_cos();
        let solid = sin_t * d_theta * d_phi;
        for sector in 0..SECTORS {
            let (sin_p, cos_p) = ((sector as f32 + 0.5) * d_phi).sin_cos();
            let dir = Vec3::new(sin_t * cos_p, sin_t * sin_p, cos_t);
            let radiance = shape.radiance(dir) * solid;
            for (c, y) in coefficients.iter_mut().zip(basis(dir)) {
                *c += radiance * y;
            }
        }
    }
    use std::f32::consts::{FRAC_PI_4, PI};
    let third = 2.0 * PI / 3.0;
    let bands = [
        PI, third, third, third, FRAC_PI_4, FRAC_PI_4, FRAC_PI_4, FRAC_PI_4, FRAC_PI_4,
    ];
    for (c, a) in coefficients.iter_mut().zip(bands) {
        *c *= a;
    }
    coefficients
}

#[cfg(test)]
mod tests;
