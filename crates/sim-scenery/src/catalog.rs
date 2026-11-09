use std::collections::{BTreeMap, BTreeSet};

use serde::{Deserialize, Serialize};
use sim_geo::{EllipsoidalHeightM, Geodetic, OrthometricHeightM, egm96_undulation_m};

use crate::terrarium::bilinear;
use crate::{
    ElevationSource, ImageryRaster, ImagerySource, RunwayDatum, SceneryError, TerrariumTile,
    TileId, project_to_tile,
};

mod manifest;
mod synthetic;

pub use synthetic::SyntheticCorridorSpec;

#[cfg(test)]
mod tests;

/// The surface a catalog's elevation tiles measure height from.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default, Deserialize, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum CatalogVerticalDatum {
    /// Mean sea level (the EGM96 geoid), as published DEMs such as Terrarium
    /// are, so a manifest that names no datum means this one.
    #[default]
    OrthometricMslEgm96,
    /// The WGS84 ellipsoid, as GNSS-surveyed or synthetic terrain is.
    EllipsoidWgs84,
}

impl CatalogVerticalDatum {
    /// The ellipsoidal and orthometric heights of a `raw` height in this
    /// datum at a latitude and longitude: `h = H + N`.
    #[must_use]
    pub fn heights(
        self,
        lat_rad: f64,
        lon_rad: f64,
        raw_m: f64,
    ) -> (EllipsoidalHeightM, OrthometricHeightM) {
        let undulation = egm96_undulation_m(lat_rad, lon_rad);
        match self {
            Self::OrthometricMslEgm96 => (
                EllipsoidalHeightM(raw_m + undulation),
                OrthometricHeightM(raw_m),
            ),
            Self::EllipsoidWgs84 => (
                EllipsoidalHeightM(raw_m),
                OrthometricHeightM(raw_m - undulation),
            ),
        }
    }
}

/// Resident elevation and orthoimagery tiles, served from memory.
///
/// Anything not resident is `Ok(None)`. Elevation comes from the deepest
/// resident zoom covering the query. [`ElevationSource::ground_height_m`]
/// returns raster heights as stored, in [`Self::vertical_datum`];
/// [`Self::ellipsoidal_ground_height_m`] and
/// [`Self::orthometric_ground_height_m`] convert them.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct CatalogScenery {
    elevation: BTreeMap<TileId, TerrariumTile>,
    elevation_zooms: BTreeSet<u8>,
    imagery: BTreeMap<TileId, ImageryRaster>,
    imagery_zooms: BTreeSet<u8>,
    vertical_datum: CatalogVerticalDatum,
    version: u64,
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
        self.version = self.version.wrapping_add(1);
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
        self.imagery_zooms.insert(tile.zoom);
        self.imagery.insert(tile, raster);
        self.version = self.version.wrapping_add(1);
        Ok(())
    }

    /// This catalog with `datum` laid into every elevation tile it reaches
    /// ([`RunwayDatum::stamp_terrarium`]), so terrain drawn from the catalog carries the
    /// charted runways. Runways not yet given a profile take it from this catalog's
    /// terrain first.
    #[must_use]
    pub fn with_runway_datum(mut self, datum: &RunwayDatum) -> Self {
        let airports: BTreeSet<_> = self
            .elevation
            .keys()
            .flat_map(|&id| datum.unresolved_airports_reaching(id))
            .collect();
        let terrain = |position: &Geodetic| self.raw_height_and_zoom(position).map(|(h, _)| h);
        for airport in airports {
            datum.resolve_airport(airport, &terrain);
        }
        let mut stamped = false;
        for (&id, tile) in &mut self.elevation {
            stamped |= datum.stamp_terrarium(id, tile);
        }
        if stamped {
            self.version = self.version.wrapping_add(1);
        }
        self
    }

    /// The datum the elevation tiles are measured from.
    #[must_use]
    pub const fn vertical_datum(&self) -> CatalogVerticalDatum {
        self.vertical_datum
    }

    /// This catalog with its elevation read in `datum`.
    #[must_use]
    pub fn with_vertical_datum(mut self, datum: CatalogVerticalDatum) -> Self {
        self.set_vertical_datum(datum);
        self
    }

    /// Reads the elevation tiles in `datum` from now on.
    pub fn set_vertical_datum(&mut self, datum: CatalogVerticalDatum) {
        self.vertical_datum = datum;
        self.version = self.version.wrapping_add(1);
    }

    /// A counter that changes whenever resident data or the datum changes,
    /// so a cache of anything derived from the catalog knows to refresh.
    #[must_use]
    pub const fn version(&self) -> u64 {
        self.version
    }

    /// Height of the ground above the WGS84 ellipsoid at `position`.
    pub fn ellipsoidal_ground_height_m(
        &self,
        position: &Geodetic,
    ) -> Result<Option<EllipsoidalHeightM>, SceneryError> {
        Ok(self.converted(position).map(|(ellipsoidal, _)| ellipsoidal))
    }

    /// Height of the ground above EGM96 mean sea level at `position`.
    pub fn orthometric_ground_height_m(
        &self,
        position: &Geodetic,
    ) -> Result<Option<OrthometricHeightM>, SceneryError> {
        Ok(self.converted(position).map(|(_, orthometric)| orthometric))
    }

    fn converted(&self, position: &Geodetic) -> Option<(EllipsoidalHeightM, OrthometricHeightM)> {
        let (raw, _) = self.raw_height_and_zoom(position)?;
        Some(
            self.vertical_datum
                .heights(position.lat_rad, position.lon_rad, raw),
        )
    }

    /// The stored height at `position` and the zoom it came from.
    pub(crate) fn raw_height_and_zoom(&self, position: &Geodetic) -> Option<(f64, u8)> {
        for &zoom in self.elevation_zooms.iter().rev() {
            let coord = project_to_tile(position, zoom)?;
            if let Some(home) = self.elevation.get(&coord.tile) {
                return self
                    .sample(coord.tile, home, coord.u, coord.v)
                    .map(|height| (height, zoom));
            }
        }
        None
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

    /// Borrows the resident imagery tile at `tile`, if present.
    pub fn imagery_tile(&self, tile: TileId) -> Option<&ImageryRaster> {
        self.imagery.get(&tile)
    }

    /// Bilinear Rec. 601 luma, `0..=255`, of the deepest resident imagery
    /// covering `position`; `None` where no imagery is resident.
    pub fn imagery_luma_at(&self, position: &Geodetic) -> Option<f64> {
        self.imagery_zooms.iter().rev().find_map(|&zoom| {
            let coord = project_to_tile(position, zoom)?;
            self.imagery
                .get(&coord.tile)?
                .luma_bilinear(coord.u, coord.v)
        })
    }

    fn sample(&self, tile: TileId, home: &TerrariumTile, u: f64, v: f64) -> Option<f64> {
        sample_with_neighbours(|id| self.elevation.get(&id), tile, home, u, v)
    }
}

/// Bilinear height over `home`, the tile at `tile`, at its fraction
/// `(u, v)`. Pixels past the edge come from the adjoining tile `lookup`
/// returns when it has the same size, so adjacent tiles meet without a step;
/// otherwise `home`'s edge holds.
pub(crate) fn sample_with_neighbours<'a>(
    lookup: impl Fn(TileId) -> Option<&'a TerrariumTile>,
    tile: TileId,
    home: &TerrariumTile,
    u: f64,
    v: f64,
) -> Option<f64> {
    let (width, height) = (home.width(), home.height());
    bilinear(width, height, u, v, |x, y| {
        let edge_x = x.clamp(0, i64::from(width) - 1) as u32;
        let edge_y = y.clamp(0, i64::from(height) - 1) as u32;
        neighbour_texel(&lookup, tile, home, x, y).or_else(|| home.texel(edge_x, edge_y))
    })
}

/// The pixel at `(x, y)` relative to `tile`, at most one tile away.
fn neighbour_texel<'a>(
    lookup: &impl Fn(TileId) -> Option<&'a TerrariumTile>,
    tile: TileId,
    home: &TerrariumTile,
    x: i64,
    y: i64,
) -> Option<f64> {
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
    let raster = lookup(neighbour)?;
    if raster.width() != home.width() || raster.height() != home.height() {
        return None;
    }
    raster.texel(x.rem_euclid(width) as u32, y.rem_euclid(height) as u32)
}

pub(crate) fn require_valid(tile: TileId) -> Result<(), SceneryError> {
    if tile.is_valid() {
        return Ok(());
    }
    Err(SceneryError::TileOutOfRange { tile })
}

impl ElevationSource for CatalogScenery {
    fn ground_height_m(&self, position: &Geodetic) -> Result<Option<f64>, SceneryError> {
        Ok(self.raw_height_and_zoom(position).map(|(height, _)| height))
    }
}

impl ImagerySource for CatalogScenery {
    fn imagery(&self, tile: TileId) -> Result<Option<ImageryRaster>, SceneryError> {
        require_valid(tile)?;
        Ok(self.imagery.get(&tile).cloned())
    }
}
