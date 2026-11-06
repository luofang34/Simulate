//! TileJSON resolution before a map is assembled, with offline fallback.
//!
//! A source that names only a TileJSON `url` draws nothing until it is resolved.
//! Resolution runs once at assembly under a timeout; a source still unresolved
//! afterwards (offline, server down, browser host) is removed with its layers so
//! the rest of the style still draws.

use maplibre::{
    io::{resource_loader::SharedLoader, source_client::SourceClient},
    style::{Style, source::Source},
};

use super::SharedMapRuntime;

#[cfg(not(target_arch = "wasm32"))]
const TILE_JSON_TIMEOUT: std::time::Duration = std::time::Duration::from_secs(6);

/// Resolves `style`'s TileJSON sources through `client`, then strips any left unresolved.
pub(crate) fn resolve_or_strip_tile_json(
    runtime: &SharedMapRuntime,
    style: &mut Style,
    client: &SourceClient<SharedLoader>,
) {
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

fn pending(source: &Source) -> bool {
    match source {
        Source::Vector(vector) | Source::Raster(vector) => {
            vector.tiles.is_none() && vector.url.is_some()
        }
        Source::RasterDem(dem) => dem.tiles.is_none() && dem.url.is_some(),
        Source::GeoJson(_) | Source::Image(_) => false,
    }
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
