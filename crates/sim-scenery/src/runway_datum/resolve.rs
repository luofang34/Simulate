//! Giving runways their terrain profile: which airports a tile needs resolved first,
//! which terrain tiles their centrelines cross, and resolving them.

use std::collections::BTreeSet;
use std::sync::atomic::Ordering;

use sim_geo::{Geodetic, ecef_to_geodetic, geodetic_to_ecef};

use super::{RunwayDatum, profile};
use crate::{AirportIdx, TileId, project_to_tile, tile_point_to_geodetic};

/// Spacing of the centreline points whose tiles a profile needs.
const CENTRELINE_STEP_M: f64 = 100.0;

impl RunwayDatum {
    /// The airports whose runways reach tile `id` and have no profile yet: what
    /// [`Self::resolve_airport`] must settle before the tile is stamped, so every tile
    /// carries the same runway. Empty for tiles coarser than [`super::MIN_STAMP_ZOOM`].
    #[must_use]
    pub fn unresolved_airports_reaching(&self, id: TileId) -> Vec<AirportIdx> {
        if !self.touches_tile(id) {
            return Vec::new();
        }
        let (Some(north_west), Some(south_east)) = (
            tile_point_to_geodetic(id, 0.0, 0.0),
            tile_point_to_geodetic(id, 1.0, 1.0),
        ) else {
            return Vec::new();
        };
        let airports: BTreeSet<AirportIdx> = self
            .strips_in(
                (south_east.lat_deg(), north_west.lat_deg()),
                (north_west.lon_deg(), south_east.lon_deg()),
            )
            .map(|strip| strip.runway.airport)
            .filter(|&airport| !self.is_resolved(airport))
            .collect();
        airports.into_iter().collect()
    }

    /// Whether `airport`'s runways have their profiles, from terrain or straight.
    #[must_use]
    pub fn is_resolved(&self, airport: AirportIdx) -> bool {
        self.index
            .profiles
            .get(airport.index())
            .is_none_or(|profiles| profiles.get().is_some())
    }

    /// The tiles at `zoom` that `airport`'s runway centrelines cross.
    #[must_use]
    pub fn centreline_tiles(&self, airport: AirportIdx, zoom: u8) -> BTreeSet<TileId> {
        let Some(record) = self.index.database.get(airport) else {
            return BTreeSet::new();
        };
        record
            .runways
            .iter()
            .flat_map(|runway| {
                let [a, b] = &runway.ends;
                let a = geodetic_to_ecef(&Geodetic::from_degrees(a.lat_deg, a.lon_deg, 0.0));
                let b = geodetic_to_ecef(&Geodetic::from_degrees(b.lat_deg, b.lon_deg, 0.0));
                let steps = ((b - a).length() / CENTRELINE_STEP_M).ceil().max(1.0) as usize;
                (0..=steps).filter_map(move |i| {
                    let point = ecef_to_geodetic(a + (b - a) * (i as f64 / steps as f64));
                    project_to_tile(&point, zoom).map(|coord| coord.tile)
                })
            })
            .collect()
    }

    /// Gives every runway of `airport` its profile from `terrain_msl`, the published
    /// terrain (unstamped) in the runways' datum, sampled along the centrelines; a
    /// runway whose terrain is missing stays straight. The first resolution wins; later
    /// calls change nothing and return `false`.
    pub fn resolve_airport(
        &self,
        airport: AirportIdx,
        terrain_msl: &dyn Fn(&Geodetic) -> Option<f64>,
    ) -> bool {
        let (Some(slot), Some(record)) = (
            self.index.profiles.get(airport.index()),
            self.index.database.get(airport),
        ) else {
            return false;
        };
        if slot.get().is_some() {
            return false;
        }
        let resolved = slot
            .set(profile::airport_profiles(record, terrain_msl))
            .is_ok();
        if resolved {
            self.index.generation.fetch_add(1, Ordering::AcqRel);
        }
        resolved
    }

    /// Counts airports resolved; a changed value means some runway's profile changed.
    #[must_use]
    pub fn generation(&self) -> u64 {
        self.index.generation.load(Ordering::Acquire)
    }
}
