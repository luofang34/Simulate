//! GeoJSON for the airport-selection globe: one point per aerodrome, and the selected
//! airport's runways with their approach feathers and threshold bars.

use serde_json::{Value, json};

use crate::geodesy::{METRES_PER_NM, destination, initial_bearing_deg};
/// Length of the extended approach centreline drawn before a selected threshold.
const FEATHER_NM: f64 = 5.0;
/// Threshold bar length as a multiple of the runway width, so it overhangs the edges.
const THRESHOLD_BAR_WIDTHS: f64 = 1.6;
/// Latitude cutoff beyond which `geojson-vt` clamps Web Mercator tile coordinates; polar
/// markers and runways beyond this limit draw directly via `OverlayPlugin`.
pub(super) const POLAR_GEOJSON_LIMIT_DEG: f64 = 85.0;

/// Marker entry rendered on the interactive 3D airport-selection globe.
#[derive(Clone, Debug, PartialEq)]
pub struct GlobeAirportMarker {
    /// Primary identifier (`KEWR`, `47N`, `EGLL`).
    pub code: String,
    /// Display label on the globe.
    pub label: String,
    /// Geodetic latitude in degrees.
    pub lat_deg: f64,
    /// Geodetic longitude in degrees.
    pub lon_deg: f64,
    /// Visibility tier (`1` = major international/Class B, `2` = regional, `3` = local paved, `4` = minor).
    pub tier: u8,
    /// Whether the aerodrome has an operating control tower.
    pub towered: bool,
    /// Whether this aerodrome is currently selected.
    pub selected: bool,
}

/// Runway geometry overlay for the currently selected airport on the 3D globe.
#[derive(Clone, Debug, PartialEq)]
pub struct GlobeRunwayEndOverlay {
    /// Runway end designator (`04L`, `22R`).
    pub designator: String,
    /// Threshold latitude in degrees.
    pub threshold_lat_deg: f64,
    /// Threshold longitude in degrees.
    pub threshold_lon_deg: f64,
    /// Opposite end latitude in degrees.
    pub opposite_lat_deg: f64,
    /// Opposite end longitude in degrees.
    pub opposite_lon_deg: f64,
    /// Runway width in metres.
    pub width_m: f32,
    /// Whether this specific runway end is selected for spawn/approach.
    pub selected: bool,
}

/// Marker style class, which picks the circle layer that draws the marker.
pub(super) fn marker_kind(marker: &GlobeAirportMarker) -> &'static str {
    if marker.selected {
        "selected"
    } else if marker.towered {
        "towered"
    } else {
        "untowered"
    }
}

/// Tiers outside `1..=4` are drawn with the minor aerodromes.
pub(super) fn marker_tier(marker: &GlobeAirportMarker) -> u8 {
    marker.tier.clamp(1, 4)
}

/// A feature collection of every finite non-polar marker, selected markers last so they draw
/// on top. Polar markers (`|lat| > 85°`) bypass Web Mercator `geojson-vt` via `OverlayPlugin`.
pub(super) fn airport_features(markers: &[GlobeAirportMarker]) -> Value {
    let mut ordered: Vec<&GlobeAirportMarker> = markers
        .iter()
        .filter(|m| {
            m.lat_deg.is_finite()
                && m.lon_deg.is_finite()
                && m.lat_deg.abs() <= POLAR_GEOJSON_LIMIT_DEG
        })
        .collect();
    ordered.sort_by_key(|m| m.selected);
    let features: Vec<Value> = ordered
        .into_iter()
        .map(|marker| {
            json!({
                "type": "Feature",
                "properties": {
                    "code": marker.code,
                    "label": marker.label,
                    "tier": marker_tier(marker),
                    "kind": marker_kind(marker),
                },
                "geometry": { "type": "Point", "coordinates": [marker.lon_deg, marker.lat_deg] }
            })
        })
        .collect();
    json!({ "type": "FeatureCollection", "features": features })
}

/// Pavement strips for every non-polar runway end, plus the approach feather and threshold
/// bar of each selected end.
pub(super) fn runway_features(runways: &[GlobeRunwayEndOverlay]) -> Value {
    let mut features = Vec::new();
    for runway in runways.iter().filter(|r| {
        is_finite(r)
            && r.threshold_lat_deg.abs() <= POLAR_GEOJSON_LIMIT_DEG
            && r.opposite_lat_deg.abs() <= POLAR_GEOJSON_LIMIT_DEG
    }) {
        let threshold = [runway.threshold_lon_deg, runway.threshold_lat_deg];
        let opposite = [runway.opposite_lon_deg, runway.opposite_lat_deg];
        features.push(line("pavement", &runway.designator, &[threshold, opposite]));
        if !runway.selected {
            continue;
        }
        let course = initial_bearing_deg(threshold, opposite);
        let feather_start = destination(threshold, course + 180.0, FEATHER_NM * METRES_PER_NM);
        features.push(line(
            "feather",
            &runway.designator,
            &[feather_start, threshold],
        ));
        let half_bar = f64::from(runway.width_m.max(10.0)) * THRESHOLD_BAR_WIDTHS * 0.5;
        let bar = [
            destination(threshold, course - 90.0, half_bar),
            destination(threshold, course + 90.0, half_bar),
        ];
        features.push(line("threshold", &runway.designator, &bar));
    }
    json!({ "type": "FeatureCollection", "features": features })
}

fn is_finite(runway: &GlobeRunwayEndOverlay) -> bool {
    [
        runway.threshold_lat_deg,
        runway.threshold_lon_deg,
        runway.opposite_lat_deg,
        runway.opposite_lon_deg,
    ]
    .iter()
    .all(|v| v.is_finite())
}

fn line(kind: &str, designator: &str, coordinates: &[[f64; 2]]) -> Value {
    json!({
        "type": "Feature",
        "properties": { "kind": kind, "designator": designator },
        "geometry": { "type": "LineString", "coordinates": coordinates }
    })
}
