//! Laying the runway datum into Terrarium elevation tiles, so terrain drawn from a tile
//! and ground read from it carry the charted runway heights.

use super::RunwayDatum;
use crate::{TerrariumTile, TileId, tile_point_to_geodetic};

/// Coarsest zoom whose tiles take the datum. A pixel there is about 150 m across, already
/// wider than most runways, and coarser tiles only draw distant terrain.
pub const MIN_STAMP_ZOOM: u8 = 10;

impl RunwayDatum {
    /// Whether any runway strip reaches tile `id` at a zoom the datum stamps.
    #[must_use]
    pub fn touches_tile(&self, id: TileId) -> bool {
        if id.zoom < MIN_STAMP_ZOOM {
            return false;
        }
        let (Some(north_west), Some(south_east)) = (
            tile_point_to_geodetic(id, 0.0, 0.0),
            tile_point_to_geodetic(id, 1.0, 1.0),
        ) else {
            return false;
        };
        self.any_strip_in(
            (south_east.lat_deg(), north_west.lat_deg()),
            (north_west.lon_deg(), south_east.lon_deg()),
        )
    }

    /// Lays the datum into `tile`, the raster addressed `id`, blending each pixel at its
    /// centre as [`Self::blend`] does; returns whether any pixel changed.
    pub fn stamp_terrarium(&self, id: TileId, tile: &mut TerrariumTile) -> bool {
        if !self.touches_tile(id) {
            return false;
        }
        let (width, height) = (f64::from(tile.width()), f64::from(tile.height()));
        let mut changed = false;
        tile.map_heights(|x, y, terrain_m| {
            let u = (f64::from(x) + 0.5) / width;
            let v = (f64::from(y) + 0.5) / height;
            let Some(position) = tile_point_to_geodetic(id, u, v) else {
                return terrain_m;
            };
            let stamped = self.blend(&position, terrain_m);
            changed |= stamped != terrain_m;
            stamped
        });
        changed
    }
}
