//! The airport-selection globe's style: satellite imagery (or a dark ocean with a graticule
//! offline), the selected airport's runways, and tiered aerodrome markers on top.

use maplibre::style::Style;
use serde_json::{Value, json};

use crate::{ESRI_WORLD_IMAGERY_URL_TEMPLATE, MapAdapterError};

/// Inline GeoJSON source of the aerodrome markers.
pub(super) const AIRPORTS_SOURCE: &str = "globe-airports";
/// Inline GeoJSON source of the selected airport's runway geometry.
pub(super) const RUNWAYS_SOURCE: &str = "globe-selected-runways";
/// Initial view: North America and the western Atlantic.
pub(super) const INITIAL_CENTER: [f64; 2] = [-75.0, 39.5];
/// Initial zoom of [`INITIAL_CENTER`].
pub(super) const INITIAL_ZOOM: f64 = 2.6;

const OCEAN: &str = "#0b192c";
const GRATICULE: &str = "#1f3b5c";
/// Zoom from which each marker tier (1 to 4) shows, so the globe is not cluttered.
const TIER_MIN_ZOOM: [f64; 4] = [0.0, 3.5, 5.0, 6.5];

/// The picker's style with empty marker and runway sources.
pub(super) fn picker_style(real_world_tiles: bool) -> Result<Style, MapAdapterError> {
    let mut sources = json!({
        AIRPORTS_SOURCE: empty_geojson_source(),
        RUNWAYS_SOURCE: empty_geojson_source(),
    });
    let mut layers = vec![json!({
        "id": "globe-ocean", "type": "background",
        "paint": { "background-color": OCEAN }
    })];
    if let Some(map) = sources.as_object_mut() {
        if real_world_tiles {
            map.insert(
                "globe-imagery".to_owned(),
                json!({
                    "type": "raster", "tiles": [ESRI_WORLD_IMAGERY_URL_TEMPLATE],
                    "tileSize": 256, "minzoom": 0, "maxzoom": 18
                }),
            );
            layers.push(json!({
                "id": "globe-imagery", "type": "raster", "source": "globe-imagery",
                "paint": { "raster-fade-duration": 0 }
            }));
        } else {
            map.insert(
                "globe-graticule".to_owned(),
                json!({ "type": "geojson", "data": graticule() }),
            );
            layers.push(json!({
                "id": "globe-graticule", "type": "line", "source": "globe-graticule",
                "paint": { "line-color": GRATICULE, "line-width": 0.8 }
            }));
        }
    }
    layers.extend(runway_layers());
    layers.extend(marker_layers());
    serde_json::from_value(json!({
        "version": 8,
        "name": "Globe airport picker",
        "projection": { "type": "vertical-perspective" },
        "center": INITIAL_CENTER,
        "zoom": INITIAL_ZOOM,
        "sources": sources,
        "layers": layers,
    }))
    .map_err(|source| MapAdapterError::StyleParse { source })
}

fn empty_geojson_source() -> Value {
    json!({ "type": "geojson", "data": { "type": "FeatureCollection", "features": [] } })
}

fn runway_layers() -> Vec<Value> {
    let runway_line = |kind: &str, color: &str, width: f64| {
        json!({
            "id": format!("globe-runway-{kind}"), "type": "line", "source": RUNWAYS_SOURCE,
            "filter": ["==", "kind", kind],
            "paint": { "line-color": color, "line-width": width }
        })
    };
    vec![
        runway_line("feather", "#22d3ee", 2.0),
        runway_line("pavement", "#e2e8f0", 4.0),
        runway_line("threshold", "#facc15", 3.0),
    ]
}

/// Untowered and towered markers per tier, then the selected marker's halo and dot.
fn marker_layers() -> Vec<Value> {
    let mut layers = Vec::new();
    for (tier, min_zoom) in (1_u8..).zip(TIER_MIN_ZOOM) {
        for (kind, color, radius) in [("untowered", "#c084fc", 3.4), ("towered", "#38bdf8", 4.2)] {
            layers.push(json!({
                "id": format!("globe-airports-{kind}-tier{tier}"), "type": "circle",
                "source": AIRPORTS_SOURCE, "minzoom": min_zoom,
                "filter": ["all", ["==", "kind", kind], ["==", "tier", tier]],
                "paint": {
                    "circle-color": color, "circle-radius": radius,
                    "circle-stroke-color": OCEAN, "circle-stroke-width": 0.8
                }
            }));
        }
    }
    let selected = ["==", "kind", "selected"];
    layers.push(json!({
        "id": "globe-airports-selected-halo", "type": "circle", "source": AIRPORTS_SOURCE,
        "filter": selected,
        "paint": {
            "circle-color": "#facc15", "circle-opacity": 0.22, "circle-radius": 12.0,
            "circle-stroke-color": "#facc15", "circle-stroke-width": 1.0,
            "circle-stroke-opacity": 0.6
        }
    }));
    layers.push(json!({
        "id": "globe-airports-selected", "type": "circle", "source": AIRPORTS_SOURCE,
        "filter": selected,
        "paint": {
            "circle-color": "#facc15", "circle-radius": 7.0,
            "circle-stroke-color": "#ffffff", "circle-stroke-width": 2.0
        }
    }));
    layers
}

/// Parallels every 15° and meridians every 15°, sampled every 5° so they follow the globe.
fn graticule() -> Value {
    let mut lines = Vec::new();
    for lat in (-75..=75).step_by(15) {
        let line: Vec<[f64; 2]> = (-180..=180)
            .step_by(5)
            .map(|lon| [f64::from(lon), f64::from(lat)])
            .collect();
        lines.push(line);
    }
    for lon in (-180..180).step_by(15) {
        let line: Vec<[f64; 2]> = (-85..=85)
            .step_by(5)
            .map(|lat| [f64::from(lon), f64::from(lat)])
            .collect();
        lines.push(line);
    }
    json!({
        "type": "Feature", "properties": {},
        "geometry": { "type": "MultiLineString", "coordinates": lines }
    })
}
