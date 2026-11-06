//! The EGM96 geoid: how far mean sea level sits above the WGS84 ellipsoid.

mod table;

#[cfg(test)]
mod tests;

use table::{COLUMNS, EGM96_DECIMETRES, ROWS, SPACING_DEG};

/// EGM96 geoid undulation `N` at a geodetic latitude and longitude, metres:
/// mean sea level is `N` above the WGS84 ellipsoid, so `h = H + N`.
///
/// Bilinear over a built-in 5° grid of the NGA model, so it needs no I/O and
/// returns identical bits on every target. Longitude wraps across the
/// antimeridian and latitude clamps at the poles. Non-finite input yields
/// zero rather than a non-finite height.
#[must_use]
pub fn egm96_undulation_m(lat_rad: f64, lon_rad: f64) -> f64 {
    if !(lat_rad.is_finite() && lon_rad.is_finite()) {
        return 0.0;
    }
    let last_row = (ROWS - 1) as f64;
    let row = ((90.0 - lat_rad.to_degrees()) / SPACING_DEG).clamp(0.0, last_row);
    let column = lon_rad.to_degrees().rem_euclid(360.0) / SPACING_DEG;
    // `row` is clamped and `column` lies in `0..72`, so both truncations fit.
    let row0 = (row.floor() as usize).min(ROWS - 2);
    let column0 = (column.floor() as usize) % COLUMNS;
    let column1 = (column0 + 1) % COLUMNS;
    let fy = row - row0 as f64;
    let fx = column - column.floor();
    let north = lerp(node(row0, column0), node(row0, column1), fx);
    let south = lerp(node(row0 + 1, column0), node(row0 + 1, column1), fx);
    lerp(north, south, fy)
}

fn node(row: usize, column: usize) -> f64 {
    EGM96_DECIMETRES
        .get(row * COLUMNS + column)
        .map_or(0.0, |&decimetres| f64::from(decimetres) * 0.1)
}

fn lerp(a: f64, b: f64, t: f64) -> f64 {
    a + (b - a) * t
}
