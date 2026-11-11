//! Web Mercator tile arithmetic and corridor tile selection.

use std::collections::BTreeSet;
use std::f64::consts::PI;

use serde::{Deserialize, Serialize};

/// Equatorial circumference of the Web Mercator sphere, metres.
const MERCATOR_CIRCUMFERENCE_M: f64 = 2.0 * PI * 6_378_137.0;
/// Pixels along a tile side.
pub const TILE_PIXELS: u32 = 256;
/// Spacing of the samples along a leg, as a fraction of the corridor radius:
/// close enough that the union of the sample discs covers the corridor.
const SAMPLE_SPACING: f64 = 0.5;

/// One tile of the zoom pyramid.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub struct TileAddress {
    /// Zoom level.
    pub zoom: u8,
    /// Column, west to east.
    pub x: u32,
    /// Row, north to south.
    pub y: u32,
}

/// Fractional tile coordinates of a latitude and longitude at `zoom`.
pub fn tile_xy(lat_deg: f64, lon_deg: f64, zoom: u8) -> [f64; 2] {
    let n = f64::from(1_u32 << zoom.min(30));
    let lat = lat_deg.clamp(-85.051_128, 85.051_128).to_radians();
    let x = (lon_deg + 180.0) / 360.0 * n;
    let y = (1.0 - (lat.tan() + 1.0 / lat.cos()).ln() / PI) / 2.0 * n;
    [x, y]
}

/// Latitude and longitude of fractional tile coordinates at `zoom`.
pub fn tile_lat_lon(x: f64, y: f64, zoom: u8) -> [f64; 2] {
    let n = f64::from(1_u32 << zoom.min(30));
    let lon = x / n * 360.0 - 180.0;
    let lat = (PI * (1.0 - 2.0 * y / n)).sinh().atan().to_degrees();
    [lat, lon]
}

/// Ground sample distance of a tile pixel at `lat_deg`, metres.
pub fn ground_sample_distance_m(lat_deg: f64, zoom: u8) -> f64 {
    MERCATOR_CIRCUMFERENCE_M * lat_deg.to_radians().cos()
        / (f64::from(TILE_PIXELS) * f64::from(1_u32 << zoom.min(30)))
}

/// Every tile at `zoom` that touches the square of half side `radius_m`
/// around a point.
pub fn tiles_around(lat_deg: f64, lon_deg: f64, radius_m: f64, zoom: u8) -> Vec<TileAddress> {
    let span = radius_m / ground_sample_distance_m(lat_deg, zoom) / f64::from(TILE_PIXELS);
    let [x, y] = tile_xy(lat_deg, lon_deg, zoom);
    let last = f64::from((1_u32 << zoom.min(30)).saturating_sub(1));
    let range = |centre: f64| {
        let low = (centre - span).floor().clamp(0.0, last) as u32;
        let high = (centre + span).floor().clamp(0.0, last) as u32;
        low..=high
    };
    range(y)
        .flat_map(|ty| range(x).map(move |tx| TileAddress { zoom, x: tx, y: ty }))
        .collect()
}

/// The tiles at `zoom` within `radius_m` of the polyline through `points`
/// (latitude, longitude in degrees), in address order.
pub fn corridor_tiles(points: &[[f64; 2]], radius_m: f64, zoom: u8) -> Vec<TileAddress> {
    let mut tiles = BTreeSet::new();
    let step_m = (radius_m * SAMPLE_SPACING).max(1.0);
    for pair in points.windows(2) {
        let &[[lat0, lon0], [lat1, lon1]] = pair else {
            continue;
        };
        let length_m = approximate_distance_m([lat0, lon0], [lat1, lon1]);
        let samples = (length_m / step_m).ceil().max(1.0) as u32;
        for i in 0..=samples {
            let t = f64::from(i) / f64::from(samples);
            let lat = lat0 + (lat1 - lat0) * t;
            let lon = lon0 + (lon1 - lon0) * t;
            tiles.extend(tiles_around(lat, lon, radius_m, zoom));
        }
    }
    if let [only] = points {
        tiles.extend(tiles_around(only[0], only[1], radius_m, zoom));
    }
    tiles.into_iter().collect()
}

/// Equirectangular distance, metres: within a fraction of a percent over a
/// corridor leg, which is all tile selection needs.
fn approximate_distance_m([lat0, lon0]: [f64; 2], [lat1, lon1]: [f64; 2]) -> f64 {
    let mean_lat = (0.5 * (lat0 + lat1)).to_radians();
    let east = (lon1 - lon0).to_radians() * mean_lat.cos();
    let north = (lat1 - lat0).to_radians();
    6_371_000.0 * east.hypot(north)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn tile_coordinates_round_trip_and_match_a_known_tile() {
        // The catalog tile imagery/15/9524/12268 lies at about 41.14 N,
        // 75.36 W, on the Delaware Water Gap.
        let [lat, lon] = tile_lat_lon(9524.5, 12268.5, 15);
        assert!(
            (lat - 41.14).abs() < 0.01 && (lon + 75.36).abs() < 0.01,
            "{lat} {lon}"
        );
        let [x, y] = tile_xy(lat, lon, 15);
        assert!((x - 9524.5).abs() < 1e-9 && (y - 12268.5).abs() < 1e-9);
    }

    #[test]
    fn ground_sample_distance_halves_per_zoom() {
        let z15 = ground_sample_distance_m(41.0, 15);
        assert!((z15 - 3.6).abs() < 0.05, "{z15}");
        assert!((ground_sample_distance_m(41.0, 16) * 2.0 - z15).abs() < 1e-12);
    }

    #[test]
    fn a_corridor_covers_every_sample_disc_and_nothing_far_away() {
        let route = [[40.9, -75.1], [40.9, -75.0], [41.0, -75.0]];
        let tiles = corridor_tiles(&route, 1_000.0, 15);
        let inside = |lat: f64, lon: f64| {
            let [x, y] = tile_xy(lat, lon, 15);
            tiles.contains(&TileAddress {
                zoom: 15,
                x: x as u32,
                y: y as u32,
            })
        };
        // On the legs, and 900 m off them.
        assert!(inside(40.9, -75.05) && inside(40.95, -75.0));
        assert!(inside(40.9 + 0.0081, -75.05) && inside(40.95, -75.0 + 0.0107));
        // 5 km off the route.
        assert!(!inside(40.9 - 0.045, -75.05));
        // A z15 tile is about 920 m here: the corridor holds a few per leg width.
        assert!(tiles.len() > 30 && tiles.len() < 200, "{}", tiles.len());
    }
}
