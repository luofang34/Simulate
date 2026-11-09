//! Tile transport serving `https://sim.catalog/` from an in-memory catalog,
//! `https://sim.chart/` from a [`ChartAssetStore`], and every other URL from the platform
//! HTTP client (Reqwest with a disk cache natively, `fetch` in the browser), remembering
//! remote URLs that answered "no such tile".

use std::collections::BTreeMap;
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
use sim_scenery::{
    CatalogScenery, LiveDemBridge, RunwayDatum, TerrariumTile, TileId, project_to_tile,
};

use crate::scenery_loader::encode_rgba_png;
use crate::{
    CHART_URL_PREFIX, CatalogTileLoader, ChartAssetStore, SharedMapRuntime,
    terrarium_tile_to_rgba_image,
};

mod negative_cache;
#[cfg(test)]
mod tests;

use negative_cache::{NEGATIVE_CACHE_CAPACITY, NegativeTileCache, fetch_negative_cached};

const CATALOG_PREFIX: &str = "https://sim.catalog/";

/// Zoom of the published terrain a runway's profile is read from: pixels of about 38 m
/// at the equator, fine enough for a profile smoothed over 200 m, coarse enough that one
/// or two tiles hold a whole runway.
const PROFILE_ZOOM: u8 = 12;

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

    /// Fetches the Terrarium tile `tile` of `url_template` (`{z}/{x}/{y}`) again on
    /// `runtime`'s tile workers, stamped and handed to the [`LiveDemBridge`] as any fetch
    /// is: for a tile the simulation's ground evicted while a map still draws it. The
    /// disk cache usually answers. Natively only; in the browser nothing is fetched.
    pub fn refetch_terrarium(&self, runtime: &SharedMapRuntime, url_template: &str, tile: TileId) {
        let url = url_template
            .replace("{z}", &tile.zoom.to_string())
            .replace("{x}", &tile.x.to_string())
            .replace("{y}", &tile.y.to_string());
        #[cfg(not(target_arch = "wasm32"))]
        {
            let loader = self.clone();
            runtime.spawn_detached(async move {
                if let Err(error) = loader.fetch(&url).await {
                    tracing::warn!(url, %error, "Terrarium refetch for the simulation ground failed");
                }
            });
        }
        #[cfg(target_arch = "wasm32")]
        {
            let _unfetched = (runtime, url);
        }
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

    /// Gives every runway reaching the Terrarium tile at `url` its terrain profile before
    /// the tile is stamped, from the published zoom-[`PROFILE_ZOOM`] tiles its centrelines
    /// cross. A runway whose terrain cannot be fetched stays straight for good, so every
    /// tile carries the same runway.
    async fn resolve_runway_profiles(&self, url: &str) {
        let (Some(datum), Some(tile)) = (&self.runway_datum, terrarium_tile_id(url)) else {
            return;
        };
        for airport in datum.unresolved_airports_reaching(tile) {
            let mut terrain = BTreeMap::new();
            for profile_tile in datum.centreline_tiles(airport, PROFILE_ZOOM) {
                let Some(profile_url) = terrarium_url_for(url, profile_tile) else {
                    continue;
                };
                let fetched = fetch_negative_cached(&self.missing, &profile_url, || {
                    self.platform.fetch(&profile_url)
                })
                .await;
                match fetched.map(|bytes| TerrariumTile::from_png_bytes(&bytes)) {
                    Ok(Ok(dem)) => {
                        terrain.insert(profile_tile, dem);
                    }
                    Ok(Err(error)) => {
                        tracing::warn!(profile_url, %error, "runway profile terrain unreadable");
                    }
                    Err(error) => {
                        tracing::warn!(profile_url, %error, "runway profile terrain unavailable");
                    }
                }
            }
            datum.resolve_airport(airport, &|position| {
                let coord = project_to_tile(position, PROFILE_ZOOM)?;
                terrain.get(&coord.tile)?.sample_bilinear(coord.u, coord.v)
            });
        }
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

/// The URL of Terrarium tile `tile` on the same server as `url`, another tile's URL.
fn terrarium_url_for(url: &str, tile: TileId) -> Option<String> {
    let (path, query) = url
        .find(['?', '#'])
        .map_or((url, ""), |at| url.split_at(at));
    let mut parts = path.rsplitn(4, '/');
    let (_y, _x, _z) = (parts.next()?, parts.next()?, parts.next()?);
    let prefix = parts.next()?;
    Some(format!(
        "{prefix}/{}/{}/{}.png{query}",
        tile.zoom, tile.x, tile.y
    ))
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
                self.resolve_runway_profiles(url).await;
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
