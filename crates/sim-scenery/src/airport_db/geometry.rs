//! Small-distance 3D tangent-plane geodesy for runways and great-circle
//! distance for the globe index, free of polar `1 / cos(lat)` singularities.

use std::f64::consts::TAU;

use sim_geo::{Geodetic, LocalFrame, ecef_to_geodetic, geodetic_to_ecef};

/// Mean Earth radius, metres (IUGG).
pub(crate) const MEAN_EARTH_RADIUS_M: f64 = 6_371_008.8;

/// Initial true bearing at `from` (radians, `[0, 2π)`) and WGS84 chord
/// distance (metres) to `to` (`(lat_deg, lon_deg)`), accurate to `< 0.05 m`
/// over runway lengths anywhere on Earth including right across `±90°`.
pub(crate) fn bearing_and_distance(from: (f64, f64), to: (f64, f64)) -> (f64, f64) {
    let a = Geodetic::from_degrees(from.0, from.1, 0.0);
    let b = Geodetic::from_degrees(to.0, to.1, 0.0);
    let delta_ecef = geodetic_to_ecef(&b) - geodetic_to_ecef(&a);
    let local = LocalFrame::at(&a);
    let north = delta_ecef.dot(local.north());
    let east = delta_ecef.dot(local.east());
    let bearing = if east.hypot(north) > 0.0 {
        east.atan2(north).rem_euclid(TAU)
    } else {
        0.0
    };
    (bearing, delta_ecef.length())
}

/// The point `(lat_deg, lon_deg)` reached by stepping `distance_m` from
/// `origin` (`(lat_deg, lon_deg)`) along `bearing_rad` in the 3D WGS84
/// tangent plane at `origin`.
pub(crate) fn offset_deg(origin: (f64, f64), bearing_rad: f64, distance_m: f64) -> (f64, f64) {
    let a = Geodetic::from_degrees(origin.0, origin.1, 0.0);
    let local = LocalFrame::at(&a);
    let (sin_b, cos_b) = bearing_rad.sin_cos();
    let step_ecef = (local.north() * cos_b + local.east() * sin_b) * distance_m;
    let moved = ecef_to_geodetic(geodetic_to_ecef(&a) + step_ecef);
    (moved.lat_rad.to_degrees(), moved.lon_rad.to_degrees())
}

/// Central angle between two points on the sphere, radians (haversine).
pub(crate) fn central_angle_rad(a: (f64, f64), b: (f64, f64)) -> f64 {
    let (lat_a, lat_b) = (a.0.to_radians(), b.0.to_radians());
    let d_lat = lat_b - lat_a;
    let d_lon = (b.1 - a.1).to_radians();
    let h = (d_lat * 0.5).sin().powi(2) + lat_a.cos() * lat_b.cos() * (d_lon * 0.5).sin().powi(2);
    2.0 * h.sqrt().clamp(0.0, 1.0).asin()
}
