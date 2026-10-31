use std::f64::consts::{PI, TAU};

use sim_geo::Geodetic;

use crate::TileId;

#[cfg(test)]
mod tests;

/// Latitude beyond which Web Mercator has no tiles, degrees.
pub const MAX_MERCATOR_LAT_DEG: f64 = 85.051_128_78;

/// Deepest zoom addressed; `2^zoom` then still fits every tile coordinate
/// well inside `u32` and `f64`'s integer range.
pub const MAX_TILE_ZOOM: u8 = 30;

/// A point inside a Web Mercator tile.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TileCoord {
    /// The tile holding the point.
    pub tile: TileId,
    /// Fraction across the tile, `0.0` at the west edge.
    pub u: f64,
    /// Fraction down the tile, `0.0` at the north edge.
    pub v: f64,
}

impl TileId {
    /// Whether this address exists at its zoom level.
    pub fn is_valid(&self) -> bool {
        self.zoom <= MAX_TILE_ZOOM && self.x < 1 << self.zoom && self.y < 1 << self.zoom
    }
}

/// The tile point under `position` at `zoom`. `None` beyond
/// [`MAX_MERCATOR_LAT_DEG`], where Web Mercator has no tiles, or beyond
/// [`MAX_TILE_ZOOM`]. Longitude wraps.
pub fn project_to_tile(position: &Geodetic, zoom: u8) -> Option<TileCoord> {
    let in_range = position.lat_deg().abs() <= MAX_MERCATOR_LAT_DEG;
    if !in_range || !position.lon_rad.is_finite() || zoom > MAX_TILE_ZOOM {
        return None;
    }
    let tiles = f64::from(1_u32 << zoom);
    let x = (position.lon_rad + PI).rem_euclid(TAU) / TAU;
    // The limit is rounded up, so its own row would fall a hair off the map.
    let y = ((1.0 - position.lat_rad.tan().asinh() / PI) / 2.0).clamp(0.0, 1.0);
    let (column, u) = split_axis(x * tiles, tiles);
    let (row, v) = split_axis(y * tiles, tiles);
    Some(TileCoord {
        tile: TileId {
            zoom,
            x: column,
            y: row,
        },
        u,
        v,
    })
}

/// Splits a tile-space coordinate into tile index and fraction; the far edge
/// of the map belongs to the last tile.
fn split_axis(position: f64, tiles: f64) -> (u32, f64) {
    let index = position.floor().min(tiles - 1.0);
    (index as u32, position - index)
}

/// The position of the tile point `(u, v)`, on the ellipsoid. `None` for an
/// address that does not exist.
pub fn tile_point_to_geodetic(tile: TileId, u: f64, v: f64) -> Option<Geodetic> {
    if !tile.is_valid() {
        return None;
    }
    let tiles = f64::from(1_u32 << tile.zoom);
    let x = (f64::from(tile.x) + u) / tiles;
    let y = (f64::from(tile.y) + v) / tiles;
    Some(Geodetic {
        lat_rad: (PI * (1.0 - 2.0 * y)).sinh().atan(),
        lon_rad: TAU * x - PI,
        height_m: 0.0,
    })
}
