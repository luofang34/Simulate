//! Tile transport serving `https://sim.catalog/` from an in-memory catalog,
//! `https://sim.chart/` from a [`ChartAssetStore`], and every other URL from the platform
//! HTTP client (Reqwest with a disk cache natively, `fetch` in the browser), remembering
//! remote URLs that answered "no such tile".

#[cfg(not(target_arch = "wasm32"))]
use std::path::PathBuf;
use std::sync::{Arc, Mutex};

use maplibre::{
    io::{
        resource_loader::SharedLoader,
        source_client::{ByteRange, HttpClient, SourceFetchError},
    },
    platform::http_client,
};
use sim_scenery::{CatalogScenery, LiveDemBridge, RunwayDatum, TerrariumTile, TileId};

use crate::scenery_loader::encode_rgba_png;
use crate::{CHART_URL_PREFIX, CatalogTileLoader, ChartAssetStore, terrarium_tile_to_rgba_image};

mod negative_cache;
#[cfg(test)]
mod tests;

use negative_cache::{NEGATIVE_CACHE_CAPACITY, NegativeTileCache, fetch_negative_cached};

const CATALOG_PREFIX: &str = "https://sim.catalog/";

#[cfg(not(target_arch = "wasm32"))]
type PlatformClient = http_client::ReqwestHttpClient;
#[cfg(target_arch = "wasm32")]
type PlatformClient = http_client::WebHttpClient;

/// [`HttpClient`] that intercepts catalog URLs and fetches real tile servers otherwise.
#[derive(Clone)]
pub struct HybridTileLoader {
    catalog: Option<CatalogTileLoader>,
    platform: PlatformClient,
    missing: Arc<Mutex<NegativeTileCache>>,
    live_dem: Option<LiveDemBridge>,
    runway_datum: Option<RunwayDatum>,
    charts: ChartAssetStore,
}

fn new_negative_cache() -> Arc<Mutex<NegativeTileCache>> {
    Arc::new(Mutex::new(NegativeTileCache::new(NEGATIVE_CACHE_CAPACITY)))
}

impl HybridTileLoader {
    /// Serves catalog URLs from `catalog` (when given) and fetches the rest over HTTP,
    /// keeping responses under `cache_dir` so tiles download once across runs.
    #[cfg(not(target_arch = "wasm32"))]
    #[must_use]
    pub fn new(catalog: Option<Arc<CatalogScenery>>, cache_dir: Option<PathBuf>) -> Self {
        Self {
            catalog: catalog.map(CatalogTileLoader::new),
            platform: http_client::ReqwestHttpClient::new(cache_dir),
            missing: new_negative_cache(),
            live_dem: None,
            runway_datum: None,
            charts: ChartAssetStore::new(),
        }
    }

    /// Serves catalog URLs from `catalog` (when given) and fetches the rest with `fetch`.
    #[cfg(target_arch = "wasm32")]
    #[must_use]
    pub fn new(catalog: Option<Arc<CatalogScenery>>) -> Self {
        Self {
            catalog: catalog.map(CatalogTileLoader::new),
            platform: http_client::web_http_client(),
            missing: new_negative_cache(),
            live_dem: None,
            runway_datum: None,
            charts: ChartAssetStore::new(),
        }
    }

    /// Also queues every remote Terrarium tile it fetches on `bridge`, so the
    /// simulation's ground matches the terrain the maps draw.
    #[must_use]
    pub fn with_live_dem_bridge(mut self, bridge: LiveDemBridge) -> Self {
        self.live_dem = Some(bridge);
        self
    }

    /// Lays `datum` into every remote Terrarium tile it fetches before any map or the
    /// [`LiveDemBridge`] sees it, so drawn terrain and the simulation's ground both carry
    /// the charted runways. The disk cache keeps the tiles as published.
    #[must_use]
    pub fn with_runway_datum(mut self, datum: RunwayDatum) -> Self {
        self.runway_datum = Some(datum);
        self
    }

    /// Serves `https://sim.chart/` URLs from `charts`, a store the host shares with the
    /// surfaces that register charts.
    #[must_use]
    pub fn with_chart_store(mut self, charts: ChartAssetStore) -> Self {
        self.charts = charts;
        self
    }

    /// The store `https://sim.chart/` URLs are served from.
    #[must_use]
    pub fn chart_store(&self) -> ChartAssetStore {
        self.charts.clone()
    }

    /// Registers (or replaces) chart `chart_id`, served at `https://sim.chart/{chart_id}.png`.
    pub fn put_chart_png(&self, chart_id: &str, bytes: Arc<[u8]>) {
        self.charts.put_chart_png(chart_id, bytes);
    }

    /// Wraps `self` in a coalescing [`SharedLoader`].
    #[must_use]
    pub fn into_shared_loader(self) -> SharedLoader {
        SharedLoader::new(self)
    }

    fn catalog_for(&self, url: &str) -> Option<&CatalogTileLoader> {
        url.starts_with(CATALOG_PREFIX)
            .then_some(self.catalog.as_ref())
            .flatten()
    }

    /// `bytes` with the runway datum laid in, when they are a Terrarium tile it reaches.
    fn stamp_terrarium(&self, url: &str, bytes: Vec<u8>) -> Vec<u8> {
        let (Some(datum), Some(tile)) = (&self.runway_datum, terrarium_tile_id(url)) else {
            return bytes;
        };
        if !datum.touches_tile(tile) {
            return bytes;
        }
        let mut dem = match TerrariumTile::from_png_bytes(&bytes) {
            Ok(dem) => dem,
            Err(error) => {
                tracing::warn!(url, %error, "Terrarium tile served without the runway datum");
                return bytes;
            }
        };
        if !datum.stamp_terrarium(tile, &mut dem) {
            return bytes;
        }
        let rgba = terrarium_tile_to_rgba_image(&dem);
        match encode_rgba_png(rgba.width(), rgba.height(), rgba.as_raw()) {
            Ok(stamped) => stamped,
            Err(error) => {
                tracing::warn!(url, %error, "Terrarium tile served without the runway datum");
                bytes
            }
        }
    }

    fn forward_terrarium(&self, url: &str, bytes: &[u8]) {
        let (Some(bridge), Some(tile)) = (&self.live_dem, terrarium_tile_id(url)) else {
            return;
        };
        if let Err(error) = bridge.ingest_terrarium_png_if_new(tile, bytes) {
            tracing::warn!(url, %error, "Terrarium tile not handed to the simulation ground");
        }
    }
}

/// The tile a Terrarium elevation URL ending `/terrarium/.../{z}/{x}/{y}.png` addresses.
fn terrarium_tile_id(url: &str) -> Option<TileId> {
    let path = url.split(['?', '#']).next()?;
    if !path.contains("/terrarium/") {
        return None;
    }
    let mut parts = path.strip_suffix(".png")?.rsplit('/');
    let y = parts.next()?.parse().ok()?;
    let x = parts.next()?.parse().ok()?;
    let zoom = parts.next()?.parse().ok()?;
    Some(TileId { zoom, x, y })
}

#[cfg_attr(target_arch = "wasm32", async_trait::async_trait(?Send))]
#[cfg_attr(not(target_arch = "wasm32"), async_trait::async_trait)]
impl HttpClient for HybridTileLoader {
    async fn fetch(&self, url: &str) -> Result<Vec<u8>, SourceFetchError> {
        if url.starts_with(CHART_URL_PREFIX) {
            return self
                .charts
                .chart_for_url(url)
                .map(|bytes| bytes.to_vec())
                .ok_or_else(|| SourceFetchError::not_found(url));
        }
        match self.catalog_for(url) {
            Some(catalog) => catalog.fetch(url).await,
            None if url.starts_with(CATALOG_PREFIX) => Err(SourceFetchError::not_found(url)),
            None => {
                let fetched =
                    fetch_negative_cached(&self.missing, url, || self.platform.fetch(url)).await?;
                let bytes = self.stamp_terrarium(url, fetched);
                self.forward_terrarium(url, &bytes);
                Ok(bytes)
            }
        }
    }

    async fn fetch_range(&self, url: &str, range: ByteRange) -> Result<Vec<u8>, SourceFetchError> {
        if url.starts_with(CHART_URL_PREFIX) {
            return Err(SourceFetchError::not_found(url));
        }
        match self.catalog_for(url) {
            Some(catalog) => catalog.fetch_range(url, range).await,
            None if url.starts_with(CATALOG_PREFIX) => Err(SourceFetchError::not_found(url)),
            None => {
                fetch_negative_cached(&self.missing, url, || self.platform.fetch_range(url, range))
                    .await
            }
        }
    }
}
