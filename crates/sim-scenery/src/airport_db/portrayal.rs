//! GeoJSON portrayal of an airport's surface: the mapped OpenStreetMap
//! diagram for a built-in aerodrome, else runway pavement, centrelines,
//! displaced-threshold segments and threshold bars from the runway records.

use flightbag::{RunwayDesignator, RunwayEndSpec};
use serde_json::{Value, json};
use world_core::GeodeticPosition;

use super::geometry::{bearing_and_distance, offset_deg};
use super::model::{AirportRecord, RunwayRecord, RunwaySurfaceKind};
use crate::aerodrome::geojson::{linestring_feature, polygon_feature, threshold_bar_feature};

/// Width drawn for a runway whose width is unpublished, metres.
const FALLBACK_WIDTH_M: f64 = 30.0;

impl AirportRecord {
    /// A GeoJSON `FeatureCollection` of the airport surface in the
    /// `kind` / `ref` property schema of [`crate::OsmAerodromeDiagram`]:
    /// `runway_pavement` polygons, `runway_centerline` and
    /// `displaced_centerline` lines and `threshold_bar` polygons.
    #[must_use]
    pub fn to_surface_geojson(&self) -> String {
        if let Some(diagram) = self.osm_diagram() {
            return diagram.portrayal_geojson.to_string();
        }
        let mut features = Vec::new();
        for runway in &self.runways {
            runway_features(runway, &mut features);
        }
        json!({
            "type": "FeatureCollection",
            "features": features,
        })
        .to_string()
    }
}

fn lon_lat(point: (f64, f64)) -> [f64; 2] {
    [point.1, point.0]
}

fn strip_ring(
    start: (f64, f64),
    end: (f64, f64),
    right_start: f64,
    right_end: f64,
    half: f64,
) -> [[f64; 2]; 5] {
    let corner =
        |point: (f64, f64), right: f64, side: f64| lon_lat(offset_deg(point, right, side * half));
    [
        corner(start, right_start, -1.0),
        corner(start, right_start, 1.0),
        corner(end, right_end, 1.0),
        corner(end, right_end, -1.0),
        corner(start, right_start, -1.0),
    ]
}

fn runway_features(runway: &RunwayRecord, features: &mut Vec<Value>) {
    let [a, b] = &runway.ends;
    let (start, end) = ((a.lat_deg, a.lon_deg), (b.lat_deg, b.lon_deg));
    let (bearing_ab, length_m) = bearing_and_distance(start, end);
    let (bearing_ba, _) = bearing_and_distance(end, start);
    let width_m = if runway.width_m > 0.0 {
        f64::from(runway.width_m)
    } else {
        FALLBACK_WIDTH_M
    };
    let half = width_m * 0.5;
    let right_start = bearing_ab + std::f64::consts::FRAC_PI_2;
    let right_end = bearing_ba + std::f64::consts::PI + std::f64::consts::FRAC_PI_2;
    let disp_a = f64::from(a.displaced_threshold_m).clamp(0.0, length_m);
    let disp_b = f64::from(b.displaced_threshold_m).clamp(0.0, (length_m - disp_a).max(0.0));
    let thr_a = offset_deg(start, bearing_ab, disp_a);
    let thr_b = offset_deg(end, bearing_ba, disp_b);
    let label = Some(runway.id.as_str());

    if runway.surface.is_unpaved() {
        let ring = strip_ring(start, end, right_start, right_end, half);
        features.push(polygon_feature("runway_turf", &ring, label));
    } else {
        let displaced_kind = if runway.surface == RunwaySurfaceKind::PavedTurf {
            "runway_turf"
        } else {
            "stopway_pavement"
        };
        if disp_a > 1.0 {
            features.push(polygon_feature(
                displaced_kind,
                &strip_ring(start, thr_a, right_start, right_start, half),
                label,
            ));
        }
        if disp_b > 1.0 {
            features.push(polygon_feature(
                displaced_kind,
                &strip_ring(thr_b, end, right_end, right_end, half),
                label,
            ));
        }
        let pave_start = if disp_a > 1.0 { thr_a } else { start };
        let pave_end = if disp_b > 1.0 { thr_b } else { end };
        features.push(polygon_feature(
            "runway_pavement",
            &strip_ring(pave_start, pave_end, right_start, right_end, half),
            label,
        ));
    }

    if !runway.surface.is_unpaved() && length_m - disp_a - disp_b > 5.0 {
        features.push(linestring_feature(
            "runway_centerline",
            &[lon_lat(thr_a), lon_lat(thr_b)],
            label,
        ));
    }
    let ends = [
        (a, start, thr_a, disp_a, bearing_ab),
        (b, end, thr_b, disp_b, bearing_ba),
    ];
    for (record, origin, threshold, displaced, heading) in ends {
        push_end_features(
            features,
            runway.surface,
            record.designator.as_str(),
            origin,
            threshold,
            displaced,
            heading,
            length_m,
            width_m,
        );
    }
}

#[allow(clippy::too_many_arguments)]
fn push_end_features(
    features: &mut Vec<Value>,
    surface: RunwaySurfaceKind,
    designator_str: &str,
    origin: (f64, f64),
    threshold: (f64, f64),
    displaced: f64,
    heading: f64,
    length_m: f64,
    width_m: f64,
) {
    if displaced > 0.0 && !surface.is_unpaved() {
        features.push(linestring_feature(
            "displaced_centerline",
            &[lon_lat(origin), lon_lat(threshold)],
            Some(designator_str),
        ));
    }
    let Ok(designator) = RunwayDesignator::parse(designator_str) else {
        return;
    };
    features.push(threshold_bar_feature(&RunwayEndSpec {
        designator,
        threshold: GeodeticPosition::wgs84(threshold.0, threshold.1, 0.0),
        true_bearing_rad: heading.rem_euclid(std::f64::consts::TAU),
        length_m: length_m - displaced,
        width_m,
    }));
}
