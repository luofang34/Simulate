use std::f64::consts::FRAC_PI_2;

use flightbag::RunwayEndSpec;
use serde_json::{Value, json};

use super::{RawOsmSnapshot, RawOsmWay, segment_bearing_and_length_m};

const METRES_PER_RAD_LAT: f64 = 6_378_137.0;

pub(super) fn build_portrayal_geojson(
    raw: &RawOsmSnapshot,
    runway_ends: &[RunwayEndSpec],
    default_rwy_width_m: f64,
    default_twy_width_m: f64,
) -> Value {
    let mut features = Vec::new();
    for way in &raw.ways {
        append_way_features(way, default_rwy_width_m, default_twy_width_m, &mut features);
    }
    for end in runway_ends {
        features.push(threshold_bar_feature(end));
    }
    for pt in &raw.points {
        if pt.kind == "holding_position" {
            features.push(hold_short_marker(pt.coord, runway_ends));
        }
    }
    json!({
        "type": "FeatureCollection",
        "features": features,
    })
}

fn append_way_features(
    way: &RawOsmWay,
    default_rwy_width_m: f64,
    default_twy_width_m: f64,
    features: &mut Vec<Value>,
) {
    let is_closed = way.coords.len() >= 4 && way.coords.first() == way.coords.last();
    match way.kind.as_str() {
        "aerodrome" if is_closed => {
            features.push(polygon_feature(
                "aerodrome_ground",
                &way.coords,
                way.r#ref.as_deref(),
            ));
        }
        "apron" | "terminal" | "hangar" | "helipad" | "control_tower" | "stopway" if is_closed => {
            let kind = if way.kind == "stopway" {
                "stopway_pavement"
            } else {
                way.kind.as_str()
            };
            features.push(polygon_feature(kind, &way.coords, way.r#ref.as_deref()));
        }
        "runway" => {
            let width_m = way.width_m.unwrap_or(default_rwy_width_m).max(8.0);
            for poly in buffer_polyline(&way.coords, width_m * 0.5) {
                features.push(polygon_feature(
                    "runway_pavement",
                    &poly,
                    way.r#ref.as_deref(),
                ));
            }
            let cl_kind = if way.runway.as_deref() == Some("displaced_threshold") {
                "displaced_centerline"
            } else {
                "runway_centerline"
            };
            features.push(linestring_feature(
                cl_kind,
                &way.coords,
                way.r#ref.as_deref(),
            ));
        }
        "taxiway" | "taxilane" => {
            if is_closed {
                features.push(polygon_feature(
                    "taxiway_pavement",
                    &way.coords,
                    way.r#ref.as_deref(),
                ));
            } else {
                let fallback = if way.kind == "taxilane" {
                    default_twy_width_m * 0.7
                } else {
                    default_twy_width_m
                };
                let width_m = way.width_m.unwrap_or(fallback).max(5.0);
                for poly in buffer_polyline(&way.coords, width_m * 0.5) {
                    features.push(polygon_feature(
                        "taxiway_pavement",
                        &poly,
                        way.r#ref.as_deref(),
                    ));
                }
                features.push(linestring_feature(
                    "taxiway_centerline",
                    &way.coords,
                    way.r#ref.as_deref(),
                ));
            }
        }
        _ => {}
    }
}

fn threshold_bar_feature(end: &RunwayEndSpec) -> Value {
    let origin = [end.threshold.longitude_deg, end.threshold.latitude_deg];
    let fwd = end.true_bearing_rad;
    let right = fwd + FRAC_PI_2;
    let half_w = (end.width_m * 0.42).max(4.0);
    let depth_m = 8.0;
    let p0 = offset_lonlat(offset_lonlat(origin, fwd, 2.0), right, -half_w);
    let p1 = offset_lonlat(offset_lonlat(origin, fwd, 2.0), right, half_w);
    let p2 = offset_lonlat(offset_lonlat(origin, fwd, 2.0 + depth_m), right, half_w);
    let p3 = offset_lonlat(offset_lonlat(origin, fwd, 2.0 + depth_m), right, -half_w);
    let ring = vec![p0, p1, p2, p3, p0];
    polygon_feature("threshold_bar", &ring, Some(end.designator.as_str()))
}

fn hold_short_marker(coord: [f64; 2], runway_ends: &[RunwayEndSpec]) -> Value {
    let rwy_brg = runway_ends.first().map_or(0.0, |r| r.true_bearing_rad);
    let a = offset_lonlat(coord, rwy_brg, -7.5);
    let b = offset_lonlat(coord, rwy_brg, 7.5);
    linestring_feature("hold_short_bar", &[a, b], None)
}

fn buffer_polyline(coords: &[[f64; 2]], half_width_m: f64) -> Vec<Vec<[f64; 2]>> {
    let mut quads = Vec::new();
    for pair in coords.windows(2) {
        let (Some(&a), Some(&b)) = (pair.first(), pair.last()) else {
            continue;
        };
        let (bearing, len_m) = segment_bearing_and_length_m(a, b);
        if len_m < 0.25 {
            continue;
        }
        let right = bearing + FRAC_PI_2;
        let a_ext = offset_lonlat(a, bearing, -half_width_m * 0.35);
        let b_ext = offset_lonlat(b, bearing, half_width_m * 0.35);
        let p0 = offset_lonlat(a_ext, right, -half_width_m);
        let p1 = offset_lonlat(a_ext, right, half_width_m);
        let p2 = offset_lonlat(b_ext, right, half_width_m);
        let p3 = offset_lonlat(b_ext, right, -half_width_m);
        quads.push(vec![p0, p1, p2, p3, p0]);
    }
    quads
}

fn offset_lonlat(origin: [f64; 2], bearing_rad: f64, dist_m: f64) -> [f64; 2] {
    let (sin_b, cos_b) = bearing_rad.sin_cos();
    let d_north_m = dist_m * cos_b;
    let d_east_m = dist_m * sin_b;
    let lat_rad = origin[1].to_radians();
    let d_lat_deg = (d_north_m / METRES_PER_RAD_LAT).to_degrees();
    let d_lon_deg = (d_east_m / (METRES_PER_RAD_LAT * lat_rad.cos().max(1e-6))).to_degrees();
    [origin[0] + d_lon_deg, origin[1] + d_lat_deg]
}

fn polygon_feature(kind: &str, ring: &[[f64; 2]], label: Option<&str>) -> Value {
    json!({
        "type": "Feature",
        "properties": {
            "kind": kind,
            "ref": label.unwrap_or(""),
        },
        "geometry": {
            "type": "Polygon",
            "coordinates": [ring],
        }
    })
}

fn linestring_feature(kind: &str, coords: &[[f64; 2]], label: Option<&str>) -> Value {
    json!({
        "type": "Feature",
        "properties": {
            "kind": kind,
            "ref": label.unwrap_or(""),
        },
        "geometry": {
            "type": "LineString",
            "coordinates": coords,
        }
    })
}
