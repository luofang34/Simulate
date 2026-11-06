//! Where the maps' tiles come from: real tile servers, or the offline regional catalog.

#[cfg(not(target_arch = "wasm32"))]
use std::path::PathBuf;
use std::sync::Arc;

use sim_geo::Geodetic;
use sim_maplibre::{ChartAssetStore, HybridTileLoader, SceneryEndpoints, SharedLoader};
use sim_scenery::{CatalogScenery, LiveDemBridge, TileId};

use crate::error::AppSimError;

/// Everything the rig needs from a [`ScenerySource`] to load and draw tiles.
pub(super) struct SceneryLoader {
    /// Transport every map fetches tiles through.
    pub loader: SharedLoader,
    /// Style URLs of the source's tile servers.
    pub endpoints: SceneryEndpoints,
    /// The offline catalog, supplied to the maps synchronously.
    pub catalog: Option<CatalogScenery>,
    /// Terrarium tiles the transport fetched, waiting for the live Earth surface.
    pub live_dem: Option<LiveDemBridge>,
}

/// Environment variable selecting the offline regional catalog instead of tile servers.
pub const OFFLINE_SCENERY_ENV: &str = "SIMULATE_OFFLINE_SCENERY";

/// Environment variable overriding the native tile cache directory.
#[cfg(not(target_arch = "wasm32"))]
pub const TILE_CACHE_ENV: &str = "SIMULATE_TILE_CACHE_DIR";

/// Tile source of every map in the rig.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum ScenerySource {
    /// Esri World Imagery, AWS Terrarium DEM, and OpenFreeMap vectors over HTTP.
    RealWorld {
        /// Directory keeping downloaded tiles across runs (native only).
        #[cfg(not(target_arch = "wasm32"))]
        cache_dir: Option<PathBuf>,
    },
    /// The synthetic regional catalog, served in memory with no network.
    Catalog,
}

impl ScenerySource {
    /// The offline catalog when `$SIMULATE_OFFLINE_SCENERY` is set, real-world tiles otherwise.
    #[must_use]
    pub fn from_environment() -> Self {
        if std::env::var_os(OFFLINE_SCENERY_ENV).is_some() {
            Self::Catalog
        } else {
            Self::real_world_from_environment()
        }
    }

    /// Real-world tiles, cached under `$SIMULATE_TILE_CACHE_DIR` or the workspace's
    /// `target/simulate-tile-cache`.
    #[must_use]
    pub fn real_world_from_environment() -> Self {
        #[cfg(not(target_arch = "wasm32"))]
        {
            let cache_dir = std::env::var_os(TILE_CACHE_ENV)
                .map(PathBuf::from)
                .unwrap_or_else(|| {
                    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
                        .join("../../target/simulate-tile-cache")
                });
            Self::RealWorld {
                cache_dir: Some(cache_dir),
            }
        }
        #[cfg(target_arch = "wasm32")]
        {
            Self::RealWorld {}
        }
    }

    /// The loader, style endpoints, offline catalog, and live DEM hand-off of this source;
    /// the loader also serves the EFB's approach plates from `charts`.
    pub(super) fn loader(&self, charts: ChartAssetStore) -> Result<SceneryLoader, AppSimError> {
        match self {
            Self::Catalog => {
                let scenery = CatalogScenery::synthetic_regional_network()
                    .map_err(sim_scenario::ScenarioError::from)?;
                let max_dem = scenery
                    .elevation_tiles()
                    .map(|t| t.zoom)
                    .max()
                    .unwrap_or(14);
                let max_ortho = scenery.imagery_tiles().map(|t| t.zoom).max().unwrap_or(15);
                let loader = catalog_transport(Arc::new(scenery.clone()))
                    .with_chart_store(charts)
                    .into_shared_loader();
                Ok(SceneryLoader {
                    loader,
                    endpoints: SceneryEndpoints::catalog(max_dem, max_ortho),
                    catalog: Some(scenery),
                    live_dem: None,
                })
            }
            #[cfg(not(target_arch = "wasm32"))]
            Self::RealWorld { cache_dir } => Ok(real_world_loader(
                HybridTileLoader::new(None, cache_dir.clone()).with_chart_store(charts),
            )),
            #[cfg(target_arch = "wasm32")]
            Self::RealWorld {} => Ok(real_world_loader(
                HybridTileLoader::new(None).with_chart_store(charts),
            )),
        }
    }
}

/// The offline catalog's transport: catalog URLs from memory and nothing over the network,
/// since the catalog's styles name no other tile server.
#[cfg(not(target_arch = "wasm32"))]
fn catalog_transport(scenery: Arc<CatalogScenery>) -> HybridTileLoader {
    HybridTileLoader::new(Some(scenery), None)
}

#[cfg(target_arch = "wasm32")]
fn catalog_transport(scenery: Arc<CatalogScenery>) -> HybridTileLoader {
    HybridTileLoader::new(Some(scenery))
}

fn real_world_loader(transport: HybridTileLoader) -> SceneryLoader {
    let bridge = LiveDemBridge::new();
    SceneryLoader {
        loader: transport
            .with_live_dem_bridge(bridge.clone())
            .into_shared_loader(),
        endpoints: SceneryEndpoints::real_world(),
        catalog: None,
        live_dem: Some(bridge),
    }
}

/// Slippy-map tile containing `position` at `zoom`.
pub(super) fn tile_at(position: &Geodetic, zoom: u8) -> TileId {
    let n = f64::from(1_u32 << zoom.min(30));
    let lat = position.lat_rad.clamp(-1.484_4, 1.484_4);
    let x = ((position.lon_rad.to_degrees() + 180.0) / 360.0 * n).floor();
    let y = ((1.0 - (lat.tan() + 1.0 / lat.cos()).ln() / std::f64::consts::PI) * 0.5 * n).floor();
    TileId {
        zoom,
        x: x.clamp(0.0, n - 1.0) as u32,
        y: y.clamp(0.0, n - 1.0) as u32,
    }
}
