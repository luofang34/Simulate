//! Terrarium tiles streamed in while the simulation runs, laid over the
//! catalog so physics stands on the same ground the renderer fetched.

use std::collections::{BTreeMap, BTreeSet};
use std::sync::Arc;

use sim_geo::Geodetic;

use crate::catalog::{require_valid, sample_with_neighbours};
use crate::{CatalogVerticalDatum, SceneryError, TerrariumTile, TileId, project_to_tile};

/// Resident tiles kept unless a caller asks for another bound: a few
/// screens of zoom-12 to zoom-15 tiles around one aircraft.
pub(super) const DEFAULT_LIVE_TILE_CAPACITY: usize = 1_024;

#[derive(Debug, Clone, PartialEq)]
struct LiveTile {
    /// Insertion order, so the bound evicts the stalest tile first.
    sequence: u64,
    raster: Arc<TerrariumTile>,
}

/// The overlay: bounded, zoom-indexed, finest zoom first.
#[derive(Debug, Clone, PartialEq)]
pub(super) struct LiveDem {
    tiles: BTreeMap<TileId, LiveTile>,
    zooms: BTreeSet<u8>,
    datum: CatalogVerticalDatum,
    capacity: usize,
    next_sequence: u64,
}

impl Default for LiveDem {
    fn default() -> Self {
        Self {
            tiles: BTreeMap::new(),
            zooms: BTreeSet::new(),
            // Published Terrarium heights are orthometric.
            datum: CatalogVerticalDatum::OrthometricMslEgm96,
            capacity: DEFAULT_LIVE_TILE_CAPACITY,
            next_sequence: 0,
        }
    }
}

impl LiveDem {
    pub(super) fn datum(&self) -> CatalogVerticalDatum {
        self.datum
    }

    pub(super) fn set_datum(&mut self, datum: CatalogVerticalDatum) {
        self.datum = datum;
    }

    pub(super) fn set_capacity(&mut self, capacity: usize) {
        self.capacity = capacity.max(1);
        self.evict_to_capacity();
    }

    pub(super) fn len(&self) -> usize {
        self.tiles.len()
    }

    pub(super) fn tile_ids(&self) -> impl Iterator<Item = TileId> + '_ {
        self.tiles.keys().copied()
    }

    /// Makes `raster` resident at `id`, replacing any tile there, and evicts
    /// the stalest tiles beyond the bound.
    pub(super) fn insert(
        &mut self,
        id: TileId,
        raster: Arc<TerrariumTile>,
    ) -> Result<(), SceneryError> {
        require_valid(id)?;
        let sequence = self.next_sequence;
        self.next_sequence = sequence.wrapping_add(1);
        self.tiles.insert(id, LiveTile { sequence, raster });
        self.evict_to_capacity();
        Ok(())
    }

    fn evict_to_capacity(&mut self) {
        while self.tiles.len() > self.capacity {
            let stalest = self
                .tiles
                .iter()
                .min_by_key(|(_, tile)| tile.sequence)
                .map(|(id, _)| *id);
            match stalest {
                Some(id) => {
                    self.tiles.remove(&id);
                }
                None => break,
            }
        }
        self.zooms = self.tiles.keys().map(|tile| tile.zoom).collect();
    }

    /// The overlay height at `position`, in the overlay's own datum, from
    /// the finest resident zoom covering it, with that zoom.
    pub(super) fn raw_height_and_zoom(&self, position: &Geodetic) -> Option<(f64, u8)> {
        let lookup = |id: TileId| self.tiles.get(&id).map(|tile| tile.raster.as_ref());
        self.zooms.iter().rev().find_map(|&zoom| {
            let coord = project_to_tile(position, zoom)?;
            let home = lookup(coord.tile)?;
            sample_with_neighbours(lookup, coord.tile, home, coord.u, coord.v)
                .map(|height| (height, zoom))
        })
    }

    /// The overlay height at `position` restated in `target`, the datum the
    /// rest of the surface is read in, so it blends with runways and the
    /// catalog without a geoid-sized step.
    pub(super) fn height_in(
        &self,
        position: &Geodetic,
        target: CatalogVerticalDatum,
    ) -> Option<(f64, u8)> {
        let (raw, zoom) = self.raw_height_and_zoom(position)?;
        let (ellipsoidal, orthometric) =
            self.datum.heights(position.lat_rad, position.lon_rad, raw);
        let height = match target {
            CatalogVerticalDatum::EllipsoidWgs84 => ellipsoidal.0,
            CatalogVerticalDatum::OrthometricMslEgm96 => orthometric.0,
        };
        Some((height, zoom))
    }
}
