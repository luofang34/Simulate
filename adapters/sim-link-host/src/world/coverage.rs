//! Where the sealed package has imagery, as the outline a map draws: the edges of its
//! deepest imagery tiles that no other of those tiles shares.

use std::collections::BTreeSet;

use super::CameraWorld;
use super::overview::unmercator;

impl CameraWorld {
    /// The outline of the package's imagery at its deepest zoom, as edges between
    /// `[lat, lon]` corners in degrees.
    pub fn coverage_edges(&self) -> Vec<[[f64; 2]; 2]> {
        let scenery = self.scenery();
        let Some(zoom) = scenery.imagery_tiles().map(|tile| tile.zoom).max() else {
            return Vec::new();
        };
        let tiles: BTreeSet<(u32, u32)> = scenery
            .imagery_tiles()
            .filter(|tile| tile.zoom == zoom)
            .map(|tile| (tile.x, tile.y))
            .collect();
        outline_edges(&tiles, zoom)
    }
}

/// The edges of `tiles` (column, row) at `zoom` that no other tile of the set shares.
pub(super) fn outline_edges(tiles: &BTreeSet<(u32, u32)>, zoom: u8) -> Vec<[[f64; 2]; 2]> {
    let size = f64::from(1_u32 << zoom.min(31));
    let corner = |x: u32, y: u32| {
        let (lat, lon) = unmercator(f64::from(x) / size, f64::from(y) / size);
        [lat, lon]
    };
    let has = |x: Option<u32>, y: Option<u32>| match (x, y) {
        (Some(x), Some(y)) => tiles.contains(&(x, y)),
        _ => false,
    };
    let mut edges = Vec::new();
    for &(x, y) in tiles {
        let (east, south) = (x.saturating_add(1), y.saturating_add(1));
        if !has(Some(x), y.checked_sub(1)) {
            edges.push([corner(x, y), corner(east, y)]);
        }
        if !has(Some(x), y.checked_add(1)) {
            edges.push([corner(x, south), corner(east, south)]);
        }
        if !has(x.checked_sub(1), Some(y)) {
            edges.push([corner(x, y), corner(x, south)]);
        }
        if !has(x.checked_add(1), Some(y)) {
            edges.push([corner(east, y), corner(east, south)]);
        }
    }
    edges
}
