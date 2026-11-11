//! Sky surface brightness behind a star camera, from the Sun's elevation and
//! the camera's line of sight.
//!
//! The zenith brightness follows the Sun's elevation through a table: the
//! dark night sky below −18°, the V-band twilight brightening of about one
//! magnitude per degree that Patat et al. (2006, A&A 455, 385) measure at
//! the zenith, and a clear daylight sky near 4 V mag/arcsec² (about
//! 3000 cd/m²) once the Sun is well up. Away from the zenith the scattered
//! light grows with the airmass, and while the Sun lights the sky it follows
//! the Rayleigh phase function `1 + cos²ψ` of the angle ψ from the Sun,
//! normalised to the zenith. Moonlight, aerosol, cloud, light pollution and
//! the vehicle's height above the scattering air are not modelled, so the
//! model errs bright at altitude.

use glam::DVec3;
use navigate_celestial::{AnalyticEphemeris, sun_ecef_with};
use sim_core::SimInstant;
use sim_geo::UtcTime;
use sim_render::NIGHT_SKY_V_MAG_PER_ARCSEC2;

use super::TrialError;
use crate::star_camera_epoch;

/// Zenith sky brightness against the Sun's elevation: degrees, then V
/// magnitudes per square arcsecond. Linear in between, flat outside.
const ZENITH_BRIGHTNESS: [(f64, f64); 10] = [
    (-18.0, NIGHT_SKY_V_MAG_PER_ARCSEC2),
    (-15.0, 20.0),
    (-12.0, 17.8),
    (-9.0, 15.2),
    (-6.0, 12.6),
    (-3.0, 10.0),
    (0.0, 7.8),
    (10.0, 5.0),
    (30.0, 4.0),
    (90.0, 3.6),
];
/// Sun elevation below which the sky holds no scattered sunlight, degrees.
const ASTRONOMICAL_NIGHT_DEG: f64 = -18.0;
/// Zenith angle beyond which the airmass stops growing: the plane-parallel
/// `1/cos z` diverges at the horizon.
const MAX_AIRMASS_ZENITH_RAD: f64 = 85.0 * std::f64::consts::PI / 180.0;

/// The sky behind one exposure.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SkySample {
    /// The Sun's elevation above the observer's horizon, degrees.
    pub sun_elevation_deg: f64,
    /// Surface brightness along the line of sight, V mag/arcsec².
    pub v_mag_per_arcsec2: f64,
}

/// The sky along unit `line_of_sight` for an observer whose local vertical
/// is unit `up`, with the Sun along unit `sun`; all in one frame.
pub fn sky_brightness(up: DVec3, sun: DVec3, line_of_sight: DVec3) -> SkySample {
    let sun_elevation_deg = sun.dot(up).clamp(-1.0, 1.0).asin().to_degrees();
    let zenith = zenith_brightness(sun_elevation_deg);
    let zenith_angle = line_of_sight
        .dot(up)
        .clamp(-1.0, 1.0)
        .acos()
        .min(MAX_AIRMASS_ZENITH_RAD);
    let airmass = 1.0 / zenith_angle.cos();
    let phase = if sun_elevation_deg > ASTRONOMICAL_NIGHT_DEG {
        let along = sun.dot(line_of_sight);
        let at_zenith = sun.dot(up);
        (1.0 + along * along) / (1.0 + at_zenith * at_zenith)
    } else {
        1.0
    };
    SkySample {
        sun_elevation_deg,
        v_mag_per_arcsec2: zenith - 2.5 * (airmass * phase).log10(),
    }
}

/// The topocentric Sun direction, ECEF, seen from `position_ecef` at `at` on
/// a run that started at `start`.
pub(super) fn sun_direction_ecef(
    start: UtcTime,
    at: SimInstant,
    position_ecef: DVec3,
) -> Result<DVec3, TrialError> {
    let sun = sun_ecef_with(
        &AnalyticEphemeris,
        star_camera_epoch(start, at),
        Some(position_ecef.to_array()),
    )
    .map_err(|error| TrialError::StarCamera {
        what: format!("Sun ephemeris: {error:?}"),
    })?;
    Ok(DVec3::from_array(sun.direction))
}

fn zenith_brightness(sun_elevation_deg: f64) -> f64 {
    let mut previous = (f64::NEG_INFINITY, NIGHT_SKY_V_MAG_PER_ARCSEC2);
    for (elevation, brightness) in ZENITH_BRIGHTNESS {
        if sun_elevation_deg <= elevation {
            let (low, low_brightness) = previous;
            if !low.is_finite() {
                return brightness;
            }
            let fraction = (sun_elevation_deg - low) / (elevation - low);
            return low_brightness + fraction * (brightness - low_brightness);
        }
        previous = (elevation, brightness);
    }
    previous.1
}

#[cfg(test)]
mod tests {
    use glam::DVec3;

    use super::{NIGHT_SKY_V_MAG_PER_ARCSEC2, sky_brightness};

    fn sun_at(elevation_deg: f64) -> DVec3 {
        let e = elevation_deg.to_radians();
        DVec3::new(e.cos(), 0.0, e.sin())
    }

    #[test]
    fn the_sky_darkens_from_day_through_twilight_to_night() {
        let up = DVec3::Z;
        let brightness: Vec<f64> = [60.0, 5.0, -6.0, -12.0, -20.0]
            .map(|e| sky_brightness(up, sun_at(e), up).v_mag_per_arcsec2)
            .to_vec();
        assert!(brightness.windows(2).all(|w| w[0] < w[1]), "{brightness:?}");
        assert_eq!(brightness[4], NIGHT_SKY_V_MAG_PER_ARCSEC2);
        assert!(brightness[0] < 4.0, "{brightness:?}");
    }

    #[test]
    fn the_sky_brightens_toward_the_horizon_and_the_sun() {
        let up = DVec3::Z;
        let sun = sun_at(30.0);
        let zenith = sky_brightness(up, sun, up).v_mag_per_arcsec2;
        let low = sky_brightness(up, sun, DVec3::new(0.0, 1.0, 0.2).normalize());
        let near_sun = sky_brightness(up, sun, sun_at(35.0));
        assert!(low.v_mag_per_arcsec2 < zenith);
        assert!(near_sun.v_mag_per_arcsec2 < zenith);
    }
}
