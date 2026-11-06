//! Short great-circle offsets on a mean-radius sphere, for chart and picker overlays whose
//! extent (a few nautical miles) makes the ellipsoid's difference invisible.
//!
//! Points are `[longitude, latitude]` degrees, the GeoJSON order.

/// Metres in one nautical mile.
pub(crate) const METRES_PER_NM: f64 = 1_852.0;
/// Mean Earth radius, metres.
const EARTH_RADIUS_M: f64 = 6_371_008.8;

/// Initial great-circle bearing from `from` to `to` (`[lon, lat]` degrees), degrees true.
pub(crate) fn initial_bearing_deg([lon1, lat1]: [f64; 2], [lon2, lat2]: [f64; 2]) -> f64 {
    let (phi1, phi2) = (lat1.to_radians(), lat2.to_radians());
    let dlon = (lon2 - lon1).to_radians();
    let y = dlon.sin() * phi2.cos();
    let x = phi1.cos() * phi2.sin() - phi1.sin() * phi2.cos() * dlon.cos();
    y.atan2(x).to_degrees().rem_euclid(360.0)
}

/// Wraps `lon_deg` into `-180.0..=180.0`.
pub(crate) fn wrap_lon_deg(lon_deg: f64) -> f64 {
    (lon_deg + 180.0).rem_euclid(360.0) - 180.0
}

/// The point `distance_m` along the great circle leaving `from` (`[lon, lat]` degrees) at
/// `bearing_deg`, as `[lon, lat]` degrees.
pub(crate) fn destination([lon, lat]: [f64; 2], bearing_deg: f64, distance_m: f64) -> [f64; 2] {
    let delta = distance_m / EARTH_RADIUS_M;
    let theta = bearing_deg.to_radians();
    let phi1 = lat.to_radians();
    let lambda1 = lon.to_radians();
    let sin_phi2 = phi1.sin() * delta.cos() + phi1.cos() * delta.sin() * theta.cos();
    let phi2 = sin_phi2.clamp(-1.0, 1.0).asin();
    let lambda2 = lambda1
        + (theta.sin() * delta.sin())
            .atan2(phi1.cos() * delta.cos() - phi1.sin() * delta.sin() * theta.cos());
    [wrap_lon_deg(lambda2.to_degrees()), phi2.to_degrees()]
}

#[cfg(test)]
mod tests;
