//! Unit tests for TileJSON caching and offline fallback stripping.

use maplibre::style::source::Source;
use serde_json::json;

use super::{apply_cached_tile_json, cache_tile_json, strip_unresolved};
use crate::{MapAdapterError, SceneryEndpoints, belief_aviation_chart_style};

#[test]
fn cached_tile_json_resolves_aviation_chart_vector_layers() {
    let url = "https://tiles.example.test/openfreemap-cache-hit.json";
    let doc = br#"{
        "tilejson": "3.0.0",
        "tiles": ["https://tiles.example.test/planet/{z}/{x}/{y}.pbf"],
        "minzoom": 0,
        "maxzoom": 14
    }"#;
    cache_tile_json(url, doc).expect("valid TileJSON caches");

    let mut endpoints = SceneryEndpoints::catalog(0, 0);
    endpoints.vector_tilejson = Some(url.to_owned());
    let mut style = belief_aviation_chart_style(
        &endpoints,
        json!({ "type": "FeatureCollection", "features": [] }),
    )
    .expect("chart style builds");

    apply_cached_tile_json(&mut style);
    strip_unresolved(&mut style);

    let Some(Source::Vector(omt)) = style.sources.get("openmaptiles") else {
        panic!("openmaptiles vector source must remain in style");
    };
    assert_eq!(
        omt.tiles.as_deref(),
        Some(&["https://tiles.example.test/planet/{z}/{x}/{y}.pbf".to_owned()][..])
    );
    assert_eq!(omt.maxzoom, Some(14));
    assert!(
        style
            .layers
            .iter()
            .any(|layer| layer.source.as_deref() == Some("openmaptiles")),
        "basemap vector layers must survive when cached"
    );
}

#[test]
fn uncached_tile_json_source_and_its_layers_are_stripped() {
    let mut endpoints = SceneryEndpoints::catalog(0, 0);
    endpoints.vector_tilejson = Some("https://tiles.example.test/uncached.json".to_owned());
    let mut style = belief_aviation_chart_style(
        &endpoints,
        json!({ "type": "FeatureCollection", "features": [] }),
    )
    .expect("chart style builds");

    apply_cached_tile_json(&mut style);
    strip_unresolved(&mut style);

    assert!(!style.sources.contains_key("openmaptiles"));
    assert!(
        style
            .layers
            .iter()
            .all(|layer| layer.source.as_deref() != Some("openmaptiles")),
        "layers referencing stripped source must be removed"
    );
    assert!(style.sources.contains_key("aerodrome"));
}

#[test]
fn invalid_tile_json_is_rejected_without_panicking() {
    let err = cache_tile_json(
        "https://tiles.example.test/bad.json",
        br#"{"tilejson": "3.0.0"}"#,
    )
    .expect_err("missing tiles array must fail validation");
    assert!(matches!(err, MapAdapterError::StyleParse { .. }));
}
