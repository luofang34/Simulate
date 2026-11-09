use glam::DVec3;
use sim_geo::{WGS84_A, WGS84_B};

/// Whether the Earth hides a star in `direction` from `eye`, both ECEF.
///
/// The blocking surface is the WGS84 ellipsoid inflated by
/// `atmosphere_height_m`, as a spacecraft's star tracker excludes the limb's
/// air. An eye inside that layer (an aircraft) sees through the air above it,
/// so for it only the solid ellipsoid blocks; an eye below the ellipsoid sees
/// nothing.
pub(super) fn earth_blocks(
    eye_ecef: DVec3,
    direction_ecef: DVec3,
    atmosphere_height_m: f64,
) -> bool {
    match ray_hits_ellipsoid(eye_ecef, direction_ecef, atmosphere_height_m) {
        Some(hit) => hit,
        None => ray_hits_ellipsoid(eye_ecef, direction_ecef, 0.0).unwrap_or(true),
    }
}

/// Whether the ray from `eye` along `direction` meets the WGS84 ellipsoid
/// inflated by `height_m` ahead of the eye; `None` when the eye is on or
/// inside it.
fn ray_hits_ellipsoid(eye: DVec3, direction: DVec3, height_m: f64) -> Option<bool> {
    let equatorial = WGS84_A + height_m;
    let polar = WGS84_B + height_m;
    let scale = DVec3::new(equatorial, equatorial, polar);
    let origin = eye / scale;
    let ray = direction / scale;
    let outside = origin.length_squared() - 1.0;
    if outside <= 0.0 {
        return None;
    }
    // |origin + t ray|² = 1 has positive roots only when the ray heads inward
    // and the discriminant is positive.
    let half_b = origin.dot(ray);
    Some(half_b < 0.0 && half_b * half_b > outside * ray.length_squared())
}

/// Whether a pixel coordinate, pixel centres on integers, falls on a
/// detector `extent` pixels across.
pub(super) fn on_detector(coordinate: f64, extent: u32) -> bool {
    coordinate >= -0.5 && coordinate < f64::from(extent) - 0.5
}
