use std::collections::{BTreeMap, BTreeSet};

use sim_geo::Geodetic;

use crate::terrarium::bilinear;
use crate::{
    ElevationSource, ImageryRaster, ImagerySource, SceneryError, TerrariumTile, TileId,
    project_to_tile,
};

mod manifest;
mod synthetic;

pub use synthetic::SyntheticCorridorSpec;

#[cfg(test)]
mod tests;

/// Resident elevation and orthoimagery tiles, served from memory.
///
/// Anything not resident is `Ok(None)`. Elevation comes from the deepest
/// resident zoom covering the query.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct CatalogScenery {
    elevation: BTreeMap<TileId, TerrariumTile>,
    elevation_zooms: BTreeSet<u8>,
    imagery: BTreeMap<TileId, ImageryRaster>,
}

impl CatalogScenery {
    /// A catalog with nothing resident.
    pub fn new() -> Self {
        Self::default()
    }

    /// Makes an elevation tile resident, replacing any tile at `tile`.
    pub fn insert_elevation(
        &mut self,
        tile: TileId,
        raster: TerrariumTile,
    ) -> Result<(), SceneryError> {
        require_valid(tile)?;
        self.elevation_zooms.insert(tile.zoom);
        self.elevation.insert(tile, raster);
        Ok(())
    }

    /// Makes an imagery tile resident, replacing any tile at `tile`.
    pub fn insert_imagery(
        &mut self,
        tile: TileId,
        raster: ImageryRaster,
    ) -> Result<(), SceneryError> {
        require_valid(tile)?;
        raster.validate()?;
        self.imagery.insert(tile, raster);
        Ok(())
    }

    /// Resident elevation tile addresses, in order.
    pub fn elevation_tiles(&self) -> impl Iterator<Item = TileId> + '_ {
        self.elevation.keys().copied()
    }

    /// Borrows the resident [`TerrariumTile`] at `tile`, if present.
    pub fn elevation_tile(&self, tile: TileId) -> Option<&TerrariumTile> {
        self.elevation.get(&tile)
    }

    /// Resident imagery tile addresses, in order.
    pub fn imagery_tiles(&self) -> impl Iterator<Item = TileId> + '_ {
        self.imagery.keys().copied()
    }

    /// Bilinear height over `home` at its fraction `(u, v)`. Pixels past the
    /// edge come from the adjoining tile when it is resident at the same size,
    /// so adjacent tiles meet without a step; otherwise `home`'s edge holds.
    fn sample(&self, tile: TileId, home: &TerrariumTile, u: f64, v: f64) -> Option<f64> {
        let (width, height) = (home.width(), home.height());
        bilinear(width, height, u, v, |x, y| {
            let edge_x = x.clamp(0, i64::from(width) - 1) as u32;
            let edge_y = y.clamp(0, i64::from(height) - 1) as u32;
            self.neighbour_texel(tile, home, x, y)
                .or_else(|| home.texel(edge_x, edge_y))
        })
    }

    /// The pixel at `(x, y)` relative to `tile`, at most one tile away.
    fn neighbour_texel(&self, tile: TileId, home: &TerrariumTile, x: i64, y: i64) -> Option<f64> {
        let (width, height) = (i64::from(home.width()), i64::from(home.height()));
        let tiles = 1_i64 << tile.zoom;
        let row = i64::from(tile.y) + y.div_euclid(height);
        if !(0..tiles).contains(&row) {
            return None;
        }
        let neighbour = TileId {
            zoom: tile.zoom,
            x: (i64::from(tile.x) + x.div_euclid(width)).rem_euclid(tiles) as u32,
            y: row as u32,
        };
        let raster = self.elevation.get(&neighbour)?;
        if raster.width() != home.width() || raster.height() != home.height() {
            return None;
        }
        raster.texel(x.rem_euclid(width) as u32, y.rem_euclid(height) as u32)
    }
}

fn require_valid(tile: TileId) -> Result<(), SceneryError> {
    if tile.is_valid() {
        return Ok(());
    }
    Err(SceneryError::TileOutOfRange { tile })
}

impl ElevationSource for CatalogScenery {
    fn ground_height_m(&self, position: &Geodetic) -> Result<Option<f64>, SceneryError> {
        for &zoom in self.elevation_zooms.iter().rev() {
            let Some(coord) = project_to_tile(position, zoom) else {
                return Ok(None);
            };
            if let Some(home) = self.elevation.get(&coord.tile) {
                return Ok(self.sample(coord.tile, home, coord.u, coord.v));
            }
        }
        Ok(None)
    }
}

impl ImagerySource for CatalogScenery {
    fn imagery(&self, tile: TileId) -> Result<Option<ImageryRaster>, SceneryError> {
        require_valid(tile)?;
        Ok(self.imagery.get(&tile).cloned())
    }
}
