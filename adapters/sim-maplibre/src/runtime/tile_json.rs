//! TileJSON resolution before a map is assembled, with offline fallback.
//!
//! A source that names only a TileJSON `url` draws nothing until it is resolved.
//! Resolution checks an in-process cache populated by [`prefetch_tile_json`] /
//! [`cache_tile_json`], resolves synchronously on native targets under a timeout, and
//! removes any source still unresolved afterwards so offline sessions still draw.

use std::{
    collections::BTreeMap,
    sync::{Mutex, OnceLock},
};

use maplibre::{
    io::{
        resource_loader::SharedLoader,
        source_client::{HttpClient, SourceClient},
        tile_json::{TileJson, apply_tile_json, apply_tile_json_to_dem},
    },
    platform::http_client,
    style::{Style, source::Source},
};

use super::SharedMapRuntime;
use crate::MapAdapterError;

#[cfg(test)]
mod tests;

#[cfg(not(target_arch = "wasm32"))]
const TILE_JSON_TIMEOUT: std::time::Duration = std::time::Duration::from_secs(6);

static TILE_JSON_CACHE: OnceLock<Mutex<BTreeMap<String, Vec<u8>>>> = OnceLock::new();

fn lock_cache() -> std::sync::MutexGuard<'static, BTreeMap<String, Vec<u8>>> {
    let cache = TILE_JSON_CACHE.get_or_init(|| Mutex::new(BTreeMap::new()));
    match cache.lock() {
        Ok(guard) => guard,
        Err(poisoned) => poisoned.into_inner(),
    }
}

/// Validates raw TileJSON UTF-8 bytes and caches them under `url` so synchronous
/// map assembly (including on `wasm32`) resolves sources without blocking.
pub fn cache_tile_json(url: &str, bytes: &[u8]) -> Result<(), MapAdapterError> {
    let parsed = TileJson::from_json_slice(bytes)
        .map_err(|source| MapAdapterError::StyleParse { source })?;
    let _ = parsed;
    lock_cache().insert(url.to_owned(), bytes.to_vec());
    Ok(())
}

/// Fetches a TileJSON document asynchronously and stores it via [`cache_tile_json`].
///
/// Returns `true` when `url` is cached (already or after fetching), or `false` on
/// network or parse errors so browser callers can still boot offline.
pub async fn prefetch_tile_json(url: &str) -> bool {
    if lock_cache().contains_key(url) {
        return true;
    }
    #[cfg(target_arch = "wasm32")]
    let client = http_client::web_http_client();
    #[cfg(not(target_arch = "wasm32"))]
    let client = http_client::ReqwestHttpClient::new(None::<std::path::PathBuf>);
    match client.fetch(url).await {
        Ok(bytes) => match cache_tile_json(url, &bytes) {
            Ok(()) => {
                tracing::info!(%url, bytes = bytes.len(), "prefetched TileJSON document");
                true
            }
            Err(error) => {
                tracing::warn!(%url, %error, "ignoring invalid prefetched TileJSON document");
                false
            }
        },
        Err(error) => {
            tracing::warn!(%url, %error, "TileJSON prefetch failed");
            false
        }
    }
}

/// Resolves `style`'s TileJSON sources through cached entries and `client`, then strips
/// any still left unresolved.
pub(crate) fn resolve_or_strip_tile_json(
    runtime: &SharedMapRuntime,
    style: &mut Style,
    client: &SourceClient<SharedLoader>,
) {
    if !has_pending(style) {
        return;
    }
    apply_cached_tile_json(style);
    if !has_pending(style) {
        return;
    }
    #[cfg(not(target_arch = "wasm32"))]
    {
        let resolve = maplibre::io::tile_json::resolve_tile_json_sources(style, client);
        let bounded = async { tokio::time::timeout(TILE_JSON_TIMEOUT, resolve).await };
        if runtime.block_on(bounded).is_err() {
            tracing::warn!("TileJSON resolution timed out");
        }
    }
    #[cfg(target_arch = "wasm32")]
    {
        let _unused = (runtime, client);
    }
    strip_unresolved(style);
}

fn apply_cached_tile_json(style: &mut Style) {
    let entries: Vec<(String, String, Vec<u8>)> = {
        let cache = lock_cache();
        style
            .sources
            .iter()
            .filter_map(|(name, source)| {
                let url = pending_url(source)?;
                let bytes = cache.get(url)?;
                Some((name.clone(), url.to_owned(), bytes.clone()))
            })
            .collect()
    };
    for (name, url, bytes) in entries {
        let tile_json = match TileJson::from_json_slice(&bytes) {
            Ok(parsed) => parsed,
            Err(error) => {
                tracing::warn!(source = %name, %url, %error, "cached TileJSON failed to parse");
                continue;
            }
        };
        if let Some(source) = style.sources.get_mut(&name) {
            match source {
                Source::Vector(vector) | Source::Raster(vector) => {
                    apply_tile_json(vector, tile_json);
                }
                Source::RasterDem(dem) => apply_tile_json_to_dem(dem, tile_json),
                Source::GeoJson(_) | Source::Image(_) | Source::Video(_) | Source::Canvas(_) => {}
            }
        }
    }
}

fn pending_url(source: &Source) -> Option<&str> {
    match source {
        Source::Vector(vector) | Source::Raster(vector) => vector
            .tiles
            .is_none()
            .then_some(vector.url.as_deref())
            .flatten(),
        Source::RasterDem(dem) => dem.tiles.is_none().then_some(dem.url.as_deref()).flatten(),
        Source::GeoJson(_) | Source::Image(_) | Source::Video(_) | Source::Canvas(_) => None,
    }
}

fn pending(source: &Source) -> bool {
    pending_url(source).is_some()
}

fn has_pending(style: &Style) -> bool {
    style.sources.values().any(pending)
}

/// Removes unresolved TileJSON sources and every layer drawing from them.
fn strip_unresolved(style: &mut Style) {
    let unresolved: Vec<String> = style
        .sources
        .iter()
        .filter(|(_, source)| pending(source))
        .map(|(name, _)| name.clone())
        .collect();
    for name in &unresolved {
        tracing::warn!(source = %name, "dropping unresolved TileJSON source and its layers");
        style.sources.remove(name);
    }
    style.layers.retain(|layer| {
        layer
            .source
            .as_ref()
            .is_none_or(|source| !unresolved.contains(source))
    });
}
