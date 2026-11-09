//! Terrarium tiles streamed in while the simulation runs, laid over the
//! catalog so physics stands on the same ground the renderer fetched.

use std::collections::{BTreeMap, BTreeSet};
use std::sync::Arc;

use sim_geo::{Geodetic, geodetic_to_ecef};

use super::{AuthoritativeWorldSurface, Body};
use crate::catalog::{require_valid, sample_with_neighbours};
use crate::{
    CatalogVerticalDatum, SceneryError, TerrariumTile, TileId, project_to_tile,
    tile_point_to_geodetic,
};

/// Resident tiles kept unless a caller asks for another bound: a few
/// screens of zoom-12 to zoom-15 tiles around one aircraft.
pub(super) const DEFAULT_LIVE_TILE_CAPACITY: usize = 1_024;

/// Tiles farther than this from the focus are evicted before any nearer one.
const NEAR_FOCUS_M: f64 = 25_000.0;

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
    /// Where the ground matters most: the ownship.
    focus: Option<Geodetic>,
    /// Tiles a renderer draws, which must stay as long as it draws them.
    pinned: BTreeSet<TileId>,
    /// Tiles evicted since the last [`LiveDem::take_evicted`].
    evicted: Vec<TileId>,
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
            focus: None,
            pinned: BTreeSet::new(),
            evicted: Vec::new(),
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

    /// Evicts until the bound holds, never a pinned tile: first tiles far from the focus,
    /// then the nearer ones, the longest resident of each first. With every tile pinned
    /// the overlay holds more than its bound.
    fn evict_to_capacity(&mut self) {
        while self.tiles.len() > self.capacity {
            let victim = self
                .tiles
                .iter()
                .filter(|(id, _)| !self.pinned.contains(id))
                .min_by_key(|(id, tile)| (!self.is_far(**id), tile.sequence))
                .map(|(id, _)| *id);
            match victim {
                Some(id) => {
                    self.tiles.remove(&id);
                    self.evicted.push(id);
                }
                None => break,
            }
        }
        self.zooms = self.tiles.keys().map(|tile| tile.zoom).collect();
    }

    fn is_far(&self, id: TileId) -> bool {
        self.focus
            .as_ref()
            .and_then(|focus| distance_to_tile_m(focus, id))
            .is_some_and(|distance| distance > NEAR_FOCUS_M)
    }

    pub(super) fn set_focus(&mut self, focus: Geodetic, pinned: BTreeSet<TileId>) {
        self.focus = Some(focus);
        self.pinned = pinned;
        self.evict_to_capacity();
    }

    pub(super) fn take_evicted(&mut self) -> Vec<TileId> {
        std::mem::take(&mut self.evicted)
    }

    pub(super) fn contains(&self, id: TileId) -> bool {
        self.tiles.contains_key(&id)
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

/// Metres from `focus` to the nearest point of tile `id`, on the ellipsoid; zero inside it.
fn distance_to_tile_m(focus: &Geodetic, id: TileId) -> Option<f64> {
    let north_west = tile_point_to_geodetic(id, 0.0, 0.0)?;
    let south_east = tile_point_to_geodetic(id, 1.0, 1.0)?;
    let lat = focus.lat_rad.clamp(south_east.lat_rad, north_west.lat_rad);
    let west = north_west.lon_rad;
    let width = (south_east.lon_rad - west).rem_euclid(std::f64::consts::TAU);
    let past_west = (focus.lon_rad - west).rem_euclid(std::f64::consts::TAU);
    let lon = if past_west <= width {
        focus.lon_rad
    } else if past_west - width < std::f64::consts::TAU - past_west {
        west + width
    } else {
        west
    };
    let nearest = Geodetic {
        lat_rad: lat,
        lon_rad: lon,
        height_m: 0.0,
    };
    let at_ground = Geodetic {
        height_m: 0.0,
        ..*focus
    };
    Some((geodetic_to_ecef(&nearest) - geodetic_to_ecef(&at_ground)).length())
}

impl AuthoritativeWorldSurface {
    /// Keeps the live tiles that matter: those `pinned` (the tiles a renderer draws) are
    /// never evicted, and tiles far from `focus` (the ownship) go before nearer ones.
    /// The world version changes only if a tile had to go. Ignored on the Moon.
    pub fn set_live_dem_focus(&mut self, focus: Geodetic, pinned: BTreeSet<TileId>) {
        if let Body::Earth(earth) = &mut self.body {
            let before = earth.live.len();
            earth.live.set_focus(focus, pinned);
            if earth.live.len() != before {
                self.bump();
            }
        }
    }

    /// The live tiles evicted since the last call, so a loader may fetch them again.
    pub fn take_evicted_live_tiles(&mut self) -> Vec<TileId> {
        match &mut self.body {
            Body::Earth(earth) => earth.live.take_evicted(),
            Body::Moon(_) => Vec::new(),
        }
    }

    /// Whether a live tile is resident at `id`.
    #[must_use]
    pub fn has_live_tile(&self, id: TileId) -> bool {
        match &self.body {
            Body::Earth(earth) => earth.live.contains(id),
            Body::Moon(_) => false,
        }
    }
}
