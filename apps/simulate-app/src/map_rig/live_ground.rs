//! Live Terrarium ground for the simulation: drains streamed tiles into the world
//! surface, keeps the tiles the physical scene draws, and fetches again any it lost.

use std::collections::BTreeSet;

use sim_geo::Geodetic;
use sim_maplibre::{HybridTileLoader, SharedMapRuntime, TruthSceneMap};
use sim_scenery::{LiveDemBridge, TileId};

use crate::world_surface::WorldSurfaceHandle;

/// The hand-off from real-world tile loading to the simulation's ground.
pub(super) struct LiveGround {
    bridge: LiveDemBridge,
    /// The transport the maps fetch through, to fetch an evicted tile again.
    refetch: Option<HybridTileLoader>,
    dem_url_template: String,
    /// Tiles fetched again and not yet resident, so each is asked for once.
    requested: BTreeSet<TileId>,
}

impl LiveGround {
    pub(super) fn new(
        bridge: LiveDemBridge,
        refetch: Option<HybridTileLoader>,
        dem_url_template: String,
    ) -> Self {
        Self {
            bridge,
            refetch,
            dem_url_template,
            requested: BTreeSet::new(),
        }
    }

    /// Makes every streamed tile resident, pins the DEM tiles `truth` drew last frame
    /// so the ground under the ownship is never coarser than the terrain drawn there,
    /// and fetches again any drawn tile the surface no longer holds.
    pub(super) fn sync(
        &mut self,
        surface: &WorldSurfaceHandle,
        truth: &TruthSceneMap,
        runtime: &SharedMapRuntime,
        ownship: Geodetic,
    ) {
        let _resident = surface.drain(&self.bridge);
        let drawn = truth.drawn_dem_tiles();
        let missing = surface.keep_live_tiles(ownship, drawn, &self.bridge);
        self.requested.retain(|tile| missing.contains(tile));
        let Some(loader) = &self.refetch else {
            return;
        };
        for tile in missing {
            if self.requested.insert(tile) {
                loader.refetch_terrarium(runtime, &self.dem_url_template, tile);
            }
        }
    }
}
