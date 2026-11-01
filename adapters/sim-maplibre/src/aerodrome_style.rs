//! OpenStreetMap (`aeroway=*`) airport-diagram MapLibre styles for 3D `TruthScene`
//! (OTW + virtual cameras) and 2D `BeliefAvionics` MFD charts.

use maplibre::style::Style;
use serde_json::{Value, json};

use crate::{CATALOG_DEM_URL_TEMPLATE, CATALOG_ORTHO_URL_TEMPLATE, MapAdapterError};

/// Builds the Simulator `TruthScene` globe style with draped OpenStreetMap airport
/// pavement, threshold markings, taxiway centerlines, hold-short bars, and tower/terminal
/// footprints over Terrarium DEM and orthoimagery.
pub fn truth_otw_style_with_aerodromes(
    max_dem_zoom: u8,
    max_ortho_zoom: u8,
    aerodrome_geojson: Value,
) -> Result<Style, MapAdapterError> {
    let spec = json!({
        "version": 8,
        "name": "Simulate Truth OTW + OSM Aerodromes",
        "projection": { "type": "vertical-perspective" },
        "sources": aerodrome_sources(max_dem_zoom, max_ortho_zoom, aerodrome_geojson),
        "terrain": { "source": "dem", "exaggeration": 1.0 },
        "layers": truth_aerodrome_layers(),
    });
    serde_json::from_value(spec).map_err(|source| MapAdapterError::StyleParse { source })
}

/// Builds the MFD `Map2D` belief chart style overlaid with high-contrast OpenStreetMap
/// airport diagrams (runways, taxiways, aprons, hold-short bars, terminals, and towers).
pub fn belief_chart_style_with_aerodromes(
    max_dem_zoom: u8,
    max_ortho_zoom: u8,
    aerodrome_geojson: Value,
) -> Result<Style, MapAdapterError> {
    let spec = json!({
        "version": 8,
        "name": "Indicate Belief Chart 2D + OSM Aerodromes",
        "sources": aerodrome_sources(max_dem_zoom, max_ortho_zoom, aerodrome_geojson),
        "layers": chart_aerodrome_layers(),
    });
    serde_json::from_value(spec).map_err(|source| MapAdapterError::StyleParse { source })
}

fn aerodrome_sources(max_dem_zoom: u8, max_ortho_zoom: u8, aerodrome_geojson: Value) -> Value {
    json!({
        "dem": {
            "type": "raster-dem",
            "tiles": [CATALOG_DEM_URL_TEMPLATE],
            "encoding": "terrarium",
            "tileSize": 256,
            "minzoom": 0,
            "maxzoom": max_dem_zoom
        },
        "ortho": {
            "type": "raster",
            "tiles": [CATALOG_ORTHO_URL_TEMPLATE],
            "tileSize": 256,
            "minzoom": 0,
            "maxzoom": max_ortho_zoom
        },
        "aerodrome": {
            "type": "geojson",
            "data": aerodrome_geojson,
            "maxzoom": 16,
            "buffer": 128
        }
    })
}

fn fill_layer(id: &str, kind: &str, color: &str, opacity: f64) -> Value {
    json!({
        "id": id,
        "type": "fill",
        "source": "aerodrome",
        "filter": ["==", ["get", "kind"], kind],
        "paint": { "fill-color": color, "fill-opacity": opacity }
    })
}

fn line_layer(id: &str, kind: &str, color: &str, width: f64, dash: Option<[f64; 2]>) -> Value {
    let paint = match dash {
        Some([a, b]) => {
            json!({ "line-color": color, "line-width": width, "line-dasharray": [a, b] })
        }
        None => json!({ "line-color": color, "line-width": width }),
    };
    json!({
        "id": id,
        "type": "line",
        "source": "aerodrome",
        "filter": ["==", ["get", "kind"], kind],
        "paint": paint
    })
}

fn truth_aerodrome_layers() -> Vec<Value> {
    vec![
        json!({ "id": "sky-bg", "type": "background", "paint": { "background-color": "#0b1626" } }),
        json!({ "id": "ortho-surface", "type": "raster", "source": "ortho" }),
        fill_layer("aerodrome-ground", "aerodrome_ground", "#2f432b", 0.55),
        fill_layer("aerodrome-apron", "apron", "#555a61", 0.94),
        fill_layer(
            "aerodrome-taxiway-pavement",
            "taxiway_pavement",
            "#434850",
            0.96,
        ),
        fill_layer("aerodrome-stopway", "stopway_pavement", "#383c42", 0.96),
        fill_layer(
            "aerodrome-runway-pavement",
            "runway_pavement",
            "#2b2f35",
            0.98,
        ),
        fill_layer("aerodrome-terminals", "terminal", "#7b8594", 0.94),
        fill_layer("aerodrome-hangars", "hangar", "#636c78", 0.94),
        fill_layer("aerodrome-tower", "control_tower", "#e2e8f0", 0.98),
        line_layer(
            "aerodrome-taxiway-centerline",
            "taxiway_centerline",
            "#eab308",
            1.8,
            None,
        ),
        line_layer(
            "aerodrome-hold-short",
            "hold_short_bar",
            "#facc15",
            3.0,
            None,
        ),
        line_layer(
            "aerodrome-runway-centerline",
            "runway_centerline",
            "#f8fafc",
            2.2,
            Some([4.0, 3.0]),
        ),
        fill_layer("aerodrome-threshold-bar", "threshold_bar", "#f8fafc", 0.98),
    ]
}

fn chart_aerodrome_layers() -> Vec<Value> {
    vec![
        json!({ "id": "chart-bg", "type": "background", "paint": { "background-color": "#101b29" } }),
        json!({ "id": "chart-ortho", "type": "raster", "source": "ortho" }),
        fill_layer(
            "chart-aerodrome-ground",
            "aerodrome_ground",
            "#192b24",
            0.65,
        ),
        fill_layer("chart-apron", "apron", "#334155", 0.88),
        fill_layer(
            "chart-taxiway-pavement",
            "taxiway_pavement",
            "#475569",
            0.92,
        ),
        fill_layer("chart-runway-pavement", "runway_pavement", "#94a3b8", 0.96),
        fill_layer("chart-terminals", "terminal", "#64748b", 0.92),
        fill_layer("chart-hangars", "hangar", "#526071", 0.90),
        fill_layer("chart-tower", "control_tower", "#38bdf8", 0.96),
        line_layer(
            "chart-taxiway-centerline",
            "taxiway_centerline",
            "#facc15",
            1.6,
            None,
        ),
        line_layer("chart-hold-short", "hold_short_bar", "#f59e0b", 2.8, None),
        line_layer(
            "chart-runway-centerline",
            "runway_centerline",
            "#ffffff",
            2.0,
            None,
        ),
        fill_layer("chart-threshold-bar", "threshold_bar", "#ffffff", 0.98),
    ]
}
