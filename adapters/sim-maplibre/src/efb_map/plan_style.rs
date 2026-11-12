//! The plan's layers in the EFB style: a satellite basemap a host may show instead of the
//! chart, outlines of coverage areas, the flight plan's legs in the usual avionics
//! colours, flown tracks and their marks over the legs they follow, and the plan's fixes
//! on top.

use maplibre::style::{Style, source::Source};
use serde_json::{Value, json};

use crate::{MapAdapterError, SceneryEndpoints};

/// Raster source of the satellite basemap.
pub(super) const SATELLITE_SOURCE: &str = "efb-satellite";
/// Raster layer of the satellite basemap, hidden unless chosen.
pub(super) const SATELLITE_LAYER: &str = "efb-satellite-layer";
/// GeoJSON source of the plan's legs and fixes.
pub(super) const ROUTE_SOURCE: &str = "efb-route";
/// GeoJSON source of flown tracks and marks.
pub(super) const TRACKS_SOURCE: &str = "efb-tracks";
/// GeoJSON source of coverage outlines.
pub(super) const OUTLINES_SOURCE: &str = "efb-outlines";

/// The active leg and fix.
pub const ACTIVE_MAGENTA: &str = "#ff40ff";
/// Legs and fixes still to come.
pub const FUTURE_WHITE: &str = "#ffffff";
/// Legs flown and fixes sequenced.
pub const FLOWN_GREY: &str = "#7a8294";
/// Coverage outlines.
pub const OUTLINE_GREEN: &str = "#5fd38d";
/// The dark casing under the active leg and around fixes.
const CASING: &str = "#05070b";

/// Adds the plan's sources and layers to `style`: the satellite layer just under
/// `below`, the rest on top of every layer.
pub(super) fn add_plan_layers(
    style: &mut Style,
    endpoints: &SceneryEndpoints,
    below: Option<&str>,
) -> Result<(), MapAdapterError> {
    let satellite = json!({
        "type": "raster",
        "tiles": [endpoints.ortho_tiles],
        "tileSize": 256,
        "maxzoom": endpoints.ortho_maxzoom,
    });
    add_source(style, SATELLITE_SOURCE, satellite)?;
    for name in [ROUTE_SOURCE, TRACKS_SOURCE, OUTLINES_SOURCE] {
        add_source(style, name, empty_geojson_source())?;
    }
    add_layer(
        style,
        json!({
            "id": SATELLITE_LAYER, "type": "raster", "source": SATELLITE_SOURCE,
            "layout": { "visibility": "none" },
            "paint": { "raster-fade-duration": 0 }
        }),
        below,
    )?;
    for layer in plan_layers() {
        add_layer(style, layer, None)?;
    }
    Ok(())
}

fn plan_layers() -> [Value; 7] {
    let by_state = |active: Value, future: Value, flown: Value| {
        json!([
            "match",
            ["get", "state"],
            "active",
            active,
            "flown",
            flown,
            future
        ])
    };
    // A holding pattern draws as a leg does.
    let leg = json!([
        "any",
        ["==", ["get", "kind"], "leg"],
        ["==", ["get", "kind"], "hold"]
    ]);
    let fix = json!(["==", ["get", "kind"], "fix"]);
    [
        json!({
            "id": "efb-outline-line", "type": "line", "source": OUTLINES_SOURCE,
            "paint": { "line-color": OUTLINE_GREEN, "line-width": 1.5,
                       "line-opacity": 0.85, "line-dasharray": [4, 3] }
        }),
        json!({
            "id": "efb-route-casing", "type": "line", "source": ROUTE_SOURCE,
            "filter": ["all", leg.clone(), ["==", ["get", "state"], "active"]],
            "layout": { "line-join": "round", "line-cap": "round" },
            "paint": { "line-color": CASING, "line-width": 6.0, "line-opacity": 0.8 }
        }),
        json!({
            "id": "efb-route-line", "type": "line", "source": ROUTE_SOURCE,
            "filter": leg,
            "layout": { "line-join": "round", "line-cap": "round" },
            "paint": {
                "line-color": by_state(json!(ACTIVE_MAGENTA), json!(FUTURE_WHITE), json!(FLOWN_GREY)),
                "line-width": by_state(json!(3.0), json!(2.0), json!(2.0))
            }
        }),
        json!({
            "id": "efb-track-line", "type": "line", "source": TRACKS_SOURCE,
            "filter": ["==", ["geometry-type"], "LineString"],
            "layout": { "line-join": "round", "line-cap": "round" },
            "paint": { "line-color": ["get", "colour"], "line-width": ["get", "width"] }
        }),
        json!({
            "id": "efb-track-mark", "type": "circle", "source": TRACKS_SOURCE,
            "filter": ["==", ["geometry-type"], "Point"],
            "paint": {
                "circle-radius": ["case", ["get", "filled"], 2.5, 3.0],
                "circle-color": ["get", "colour"],
                "circle-opacity": ["case", ["get", "filled"], 1.0, 0.0],
                "circle-stroke-color": ["get", "colour"],
                "circle-stroke-width": ["case", ["get", "filled"], 0.0, 1.5]
            }
        }),
        json!({
            "id": "efb-route-fix", "type": "circle", "source": ROUTE_SOURCE,
            "filter": fix.clone(),
            "paint": {
                "circle-radius": 4.5,
                "circle-color": by_state(json!(ACTIVE_MAGENTA), json!(FUTURE_WHITE), json!(FLOWN_GREY)),
                "circle-stroke-color": CASING,
                "circle-stroke-width": 1.5
            }
        }),
        // The fix flown toward, drawn last so a neighbour never covers it.
        json!({
            "id": "efb-route-fix-active", "type": "circle", "source": ROUTE_SOURCE,
            "filter": ["all", fix, ["==", ["get", "state"], "active"]],
            "paint": {
                "circle-radius": 6.0,
                "circle-color": ACTIVE_MAGENTA,
                "circle-stroke-color": CASING,
                "circle-stroke-width": 2.0
            }
        }),
    ]
}

fn empty_geojson_source() -> Value {
    json!({ "type": "geojson", "data": { "type": "FeatureCollection", "features": [] } })
}

fn add_source(style: &mut Style, name: &str, source: Value) -> Result<(), MapAdapterError> {
    let source: Source =
        serde_json::from_value(source).map_err(|source| MapAdapterError::StyleParse { source })?;
    style
        .add_source(name, source)
        .map(|_change| ())
        .map_err(|source| MapAdapterError::StyleMutation {
            target: name.to_owned(),
            source,
        })
}

fn add_layer(style: &mut Style, layer: Value, below: Option<&str>) -> Result<(), MapAdapterError> {
    let target = layer
        .get("id")
        .and_then(Value::as_str)
        .unwrap_or_default()
        .to_owned();
    style
        .add_layer(layer, below)
        .map(|_change| ())
        .map_err(|source| MapAdapterError::StyleMutation { target, source })
}
