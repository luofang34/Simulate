use glam::DVec3;
use navigate_celestial::{
    AnalyticEphemeris, Ephemeris, Epoch, ObservationError, moon_ecef_with, sun_ecef_with,
};

use super::frame::{CelestialFrame, EarthShadowSample, MoonDiscSample};

/// WGS84 equatorial radius, metres.
const EARTH_RADIUS_M: f64 = 6_378_137.0;
/// IAU 2015 nominal solar radius, metres.
const SUN_RADIUS_M: f64 = 695_700_000.0;
/// Danjon's enlargement of the geometric shadow by the opaque lower
/// atmosphere.
const SHADOW_ENLARGEMENT: f64 = 1.02;
/// Nominal Danjon `L` at the umbra's edge and at its centre.
const DANJON_UMBRA_EDGE: f32 = 3.0;
const DANJON_UMBRA_CENTRE: f32 = 1.0;

/// The sky for an observer at `observer_ecef_m`, evaluated with `provider`
/// first and falling back to [`AnalyticEphemeris`] outside `provider`'s coverage.
pub(super) fn frame_with<P: Ephemeris>(
    provider: &P,
    epoch: Epoch,
    observer_ecef_m: DVec3,
) -> Option<CelestialFrame> {
    if let Ok(frame) = frame_from(provider, epoch, observer_ecef_m) {
        return Some(frame);
    }
    let mut fallback_frame = frame_from(&AnalyticEphemeris, epoch, observer_ecef_m).ok()?;
    fallback_frame.used_analytic_fallback = true;
    Some(fallback_frame)
}

/// The sky for an observer at `observer_ecef_m`. Sun and Moon are
/// topocentric: the Moon's parallax reaches about 1°, which decides whether
/// an eclipse is total at a given site.
pub(super) fn frame_from<P: Ephemeris>(
    provider: &P,
    epoch: Epoch,
    observer_ecef_m: DVec3,
) -> Result<CelestialFrame, ObservationError<P::Error>> {
    let observer = Some(observer_ecef_m.to_array());
    let sun = sun_ecef_with(provider, epoch, observer)?;
    let moon = moon_ecef_with(provider, epoch, observer)?;
    let moon_geocentric = moon_ecef_with(provider, epoch, None)?;
    let sun_direction = DVec3::from_array(sun.direction);
    let moon_direction = DVec3::from_array(moon.direction);
    let sun_radius_deg = sun.angular_radius_rad.to_degrees();
    let moon_radius_deg = moon.angular_radius_rad.to_degrees();

    let mut frame = CelestialFrame::with_sun(sun_direction, sun_radius_deg);
    frame.sun_visible_fraction = visible_fraction(
        separation_deg(sun_direction, moon_direction),
        sun_radius_deg,
        moon_radius_deg,
    ) as f32;
    frame.moon_disc = Some(MoonDiscSample {
        direction_ecef: moon_direction,
        angular_radius_deg: moon_radius_deg,
    });
    frame.earth_shadow = Some(earth_shadow(
        -sun_direction,
        moon_direction,
        moon_radius_deg,
        moon_geocentric.distance_m,
        moon.distance_m,
        sun.distance_m,
    ));
    Ok(frame)
}

/// Shadow cone radii at the Moon's geocentric distance, seen from the
/// observer's distance to the Moon.
fn earth_shadow(
    axis: DVec3,
    moon_direction: DVec3,
    moon_radius_deg: f64,
    moon_geocentric_m: f64,
    moon_topocentric_m: f64,
    sun_distance_m: f64,
) -> EarthShadowSample {
    let spread = moon_geocentric_m / sun_distance_m;
    let umbra_m = (EARTH_RADIUS_M - spread * (SUN_RADIUS_M - EARTH_RADIUS_M)) * SHADOW_ENLARGEMENT;
    let penumbra_m =
        (EARTH_RADIUS_M + spread * (SUN_RADIUS_M + EARTH_RADIUS_M)) * SHADOW_ENLARGEMENT;
    let umbra_radius_deg = umbra_m.max(0.0).atan2(moon_topocentric_m).to_degrees();
    let penumbra_radius_deg = penumbra_m.atan2(moon_topocentric_m).to_degrees();
    let offset_deg = separation_deg(axis, moon_direction);
    let danjon =
        (offset_deg < umbra_radius_deg + moon_radius_deg && umbra_radius_deg > 0.0).then(|| {
            let depth = (1.0 - offset_deg / umbra_radius_deg).clamp(0.0, 1.0) as f32;
            DANJON_UMBRA_EDGE + (DANJON_UMBRA_CENTRE - DANJON_UMBRA_EDGE) * depth
        });
    EarthShadowSample {
        direction_ecef: axis,
        umbra_radius_deg,
        penumbra_radius_deg,
        danjon,
    }
}

/// Angle between two unit vectors, degrees, stable for small angles.
pub(super) fn separation_deg(a: DVec3, b: DVec3) -> f64 {
    a.cross(b).length().atan2(a.dot(b)).to_degrees()
}

/// Fraction of a disc of radius `sun` left uncovered by a disc of radius
/// `moon` whose centre is `distance` away. The discs span half a degree, so
/// the flat lens-area formula is exact to far below a percent.
pub(super) fn visible_fraction(distance: f64, sun: f64, moon: f64) -> f64 {
    if sun <= 0.0 || distance >= sun + moon {
        return 1.0;
    }
    if distance <= (sun - moon).abs() {
        return if moon >= sun {
            0.0
        } else {
            1.0 - (moon / sun).powi(2)
        };
    }
    let (d2, s2, m2) = (distance * distance, sun * sun, moon * moon);
    let sun_half_angle = ((d2 + s2 - m2) / (2.0 * distance * sun))
        .clamp(-1.0, 1.0)
        .acos();
    let moon_half_angle = ((d2 + m2 - s2) / (2.0 * distance * moon))
        .clamp(-1.0, 1.0)
        .acos();
    let kite = 0.5
        * ((-distance + sun + moon)
            * (distance + sun - moon)
            * (distance - sun + moon)
            * (distance + sun + moon))
            .max(0.0)
            .sqrt();
    let overlap = s2 * sun_half_angle + m2 * moon_half_angle - kite;
    (1.0 - overlap / (std::f64::consts::PI * s2)).clamp(0.0, 1.0)
}
