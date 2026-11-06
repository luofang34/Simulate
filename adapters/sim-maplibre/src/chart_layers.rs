//! Layer stacks of the dark aeronautical chart: a muted OpenMapTiles basemap and the
//! high-contrast OpenStreetMap `aeroway=*` airport-surface diagram drawn above it.
//!
//! Only `background`, `color-relief`, `hillshade`, `fill`, and `line` layers are used, so
//! the chart needs neither glyphs nor a symbol pipeline.

use serde_json::{Value, json};

fn omt_fill(id: &str, source_layer: &str, filter: Value, color: &str, minzoom: f64) -> Value {
    json!({
        "id": id,
        "type": "fill",
        "source": "openmaptiles",
        "source-layer": source_layer,
        "minzoom": minzoom,
        "filter": filter,
        "paint": { "fill-color": color, "fill-opacity": 1.0 }
    })
}

fn omt_line(id: &str, source_layer: &str, filter: Value, color: &str, widths: [f64; 2]) -> Value {
    let [low, high] = widths;
    json!({
        "id": id,
        "type": "line",
        "source": "openmaptiles",
        "source-layer": source_layer,
        "filter": filter,
        "paint": {
            "line-color": color,
            "line-width": ["interpolate", ["linear"], ["zoom"], 8, low, 16, high]
        }
    })
}

fn class_in(classes: &[&str]) -> Value {
    let mut filter = vec![json!("match"), json!(["get", "class"]), json!(classes)];
    filter.extend([json!(true), json!(false)]);
    Value::Array(filter)
}

/// Muted landcover, landuse, water, roads, rail, and building footprints.
pub(crate) fn basemap_layers() -> Vec<Value> {
    vec![
        omt_fill(
            "omt-wood",
            "landcover",
            class_in(&["wood", "forest"]),
            "#15221d",
            0.0,
        ),
        omt_fill(
            "omt-landuse",
            "landuse",
            class_in(&["residential", "commercial", "industrial"]),
            "#17202b",
            0.0,
        ),
        omt_fill("omt-water", "water", json!(true), "#0a1524", 0.0),
        omt_line(
            "omt-waterway",
            "waterway",
            json!(true),
            "#16304d",
            [0.5, 2.0],
        ),
        omt_line(
            "omt-rail",
            "transportation",
            class_in(&["rail"]),
            "#29384a",
            [0.4, 1.4],
        ),
        omt_line(
            "omt-road-minor",
            "transportation",
            class_in(&["minor", "service", "tertiary"]),
            "#243242",
            [0.3, 2.0],
        ),
        omt_line(
            "omt-road-major",
            "transportation",
            class_in(&["secondary", "primary"]),
            "#2b3b4e",
            [0.6, 3.0],
        ),
        omt_line(
            "omt-road-trunk",
            "transportation",
            class_in(&["trunk", "motorway"]),
            "#33465c",
            [0.9, 4.0],
        ),
        omt_fill("omt-building", "building", json!(true), "#1c2735", 13.0),
    ]
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

fn outline_layer(id: &str, kind: &str, color: &str, width: f64) -> Value {
    json!({
        "id": id,
        "type": "line",
        "source": "aerodrome",
        "filter": ["==", ["get", "kind"], kind],
        "paint": { "line-color": color, "line-width": width }
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

/// Airport-surface diagram: pavement, buildings, centrelines, hold-short bars, thresholds.
pub(crate) fn aerodrome_layers() -> Vec<Value> {
    vec![
        fill_layer("apt-ground", "aerodrome_ground", "#162621", 0.55),
        fill_layer("apt-apron", "apron", "#283546", 0.92),
        fill_layer("apt-taxiway", "taxiway_pavement", "#38485c", 0.95),
        fill_layer("apt-stopway", "stopway_pavement", "#313e50", 0.95),
        fill_layer("apt-runway-turf", "runway_turf", "#1f3a2c", 0.88),
        line_layer(
            "apt-runway-turf-edge",
            "runway_turf",
            "#4ade80",
            1.3,
            Some([3.0, 2.5]),
        ),
        fill_layer("apt-runway", "runway_pavement", "#64748b", 0.98),
        outline_layer("apt-runway-edge", "runway_pavement", "#cbd5e1", 1.6),
        fill_layer("apt-terminal", "terminal", "#475569", 0.95),
        fill_layer("apt-hangar", "hangar", "#3b495b", 0.95),
        fill_layer("apt-tower", "control_tower", "#38bdf8", 0.98),
        line_layer(
            "apt-taxiway-centerline",
            "taxiway_centerline",
            "#eab308",
            1.6,
            None,
        ),
        line_layer("apt-hold-short", "hold_short_bar", "#facc15", 3.0, None),
        line_layer(
            "apt-displaced-centerline",
            "displaced_centerline",
            "#cbd5e1",
            1.3,
            Some([1.5, 2.5]),
        ),
        line_layer(
            "apt-runway-centerline",
            "runway_centerline",
            "#f8fafc",
            1.8,
            Some([4.0, 3.0]),
        ),
        fill_layer("apt-threshold-bar", "threshold_bar", "#ffffff", 0.98),
    ]
}
