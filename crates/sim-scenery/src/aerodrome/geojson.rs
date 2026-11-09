use flightbag::RunwayEndSpec;
use serde_json::{Value, json};
use sim_geo::{Geodetic, LocalFrame, ecef_to_geodetic, geodetic_to_ecef};

use super::{RawOsmSnapshot, RawOsmWay};
use crate::AirportRecord;
use crate::airport_db::portrayal::append_record_runway_features;

pub(super) fn build_portrayal_geojson(
    raw: &RawOsmSnapshot,
    runway_ends: &[RunwayEndSpec],
    record: Option<&AirportRecord>,
    default_rwy_width_m: f64,
    default_twy_width_m: f64,
) -> Value {
    let mut features = Vec::new();
    for way in &raw.ways {
        if record.is_some() && way.kind == "runway" {
            continue;
        }
        append_way_features(way, default_rwy_width_m, default_twy_width_m, &mut features);
    }
    if let Some(rec) = record {
        append_record_runway_features(rec, &mut features);
    } else {
        for end in runway_ends {
            features.push(threshold_bar_feature(end));
        }
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
            let is_displaced = way.runway.as_deref() == Some("displaced_threshold");
            let pave_kind = if is_displaced {
                "stopway_pavement"
            } else {
                "runway_pavement"
            };
            let width_m = way.width_m.unwrap_or(default_rwy_width_m).max(8.0);
            for poly in buffer_polyline_ext(&way.coords, width_m * 0.5, false) {
                features.push(polygon_feature(pave_kind, &poly, way.r#ref.as_deref()));
            }
            let cl_kind = if is_displaced {
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

pub(crate) fn threshold_bar_feature(end: &RunwayEndSpec) -> Value {
    let origin = [end.threshold.longitude_deg, end.threshold.latitude_deg];
    let fwd = end.true_bearing_rad;
    let half_w = (end.width_m * 0.42).max(4.0);
    let depth_m = 8.0;
    let p0 = offset_along_across_lonlat(origin, fwd, 2.0, -half_w);
    let p1 = offset_along_across_lonlat(origin, fwd, 2.0, half_w);
    let p2 = offset_along_across_lonlat(origin, fwd, 2.0 + depth_m, half_w);
    let p3 = offset_along_across_lonlat(origin, fwd, 2.0 + depth_m, -half_w);
    let ring = vec![p0, p1, p2, p3, p0];
    polygon_feature("threshold_bar", &ring, Some(end.designator.as_str()))
}

fn hold_short_marker(coord: [f64; 2], runway_ends: &[RunwayEndSpec]) -> Value {
    let rwy_brg = runway_ends.first().map_or(0.0, |r| r.true_bearing_rad);
    let a = offset_lonlat(coord, rwy_brg, -7.5);
    let b = offset_lonlat(coord, rwy_brg, 7.5);
    linestring_feature("hold_short_bar", &[a, b], None)
}

pub(crate) fn buffer_polyline(coords: &[[f64; 2]], half_width_m: f64) -> Vec<Vec<[f64; 2]>> {
    buffer_polyline_ext(coords, half_width_m, true)
}

fn buffer_polyline_ext(
    coords: &[[f64; 2]],
    half_width_m: f64,
    overlap_ends: bool,
) -> Vec<Vec<[f64; 2]>> {
    let mut quads = Vec::new();
    let pad = if overlap_ends {
        half_width_m * 0.35
    } else {
        0.0
    };
    for pair in coords.windows(2) {
        let (Some(&[lon_a, lat_a]), Some(&[lon_b, lat_b])) = (pair.first(), pair.last()) else {
            continue;
        };
        let a_ecef = geodetic_to_ecef(&Geodetic::from_degrees(lat_a, lon_a, 0.0));
        let b_ecef = geodetic_to_ecef(&Geodetic::from_degrees(lat_b, lon_b, 0.0));
        let chord = b_ecef - a_ecef;
        let len_m = chord.length();
        if len_m < 0.25 {
            continue;
        }
        let fwd = chord / len_m;
        let mid = ecef_to_geodetic(0.5 * (a_ecef + b_ecef));
        let right = fwd.cross(LocalFrame::at(&mid).up()).normalize_or_zero();
        let a_ext = a_ecef - fwd * pad;
        let b_ext = b_ecef + fwd * pad;
        let to_lonlat = |ecef| {
            let g = ecef_to_geodetic(ecef);
            [g.lon_deg(), g.lat_deg()]
        };
        let p0 = to_lonlat(a_ext - right * half_width_m);
        let p1 = to_lonlat(a_ext + right * half_width_m);
        let p2 = to_lonlat(b_ext + right * half_width_m);
        let p3 = to_lonlat(b_ext - right * half_width_m);
        quads.push(vec![p0, p1, p2, p3, p0]);
    }
    quads
}

pub(crate) fn offset_lonlat(origin: [f64; 2], bearing_rad: f64, dist_m: f64) -> [f64; 2] {
    offset_along_across_lonlat(origin, bearing_rad, dist_m, 0.0)
}

fn offset_along_across_lonlat(
    [lon_deg, lat_deg]: [f64; 2],
    fwd_rad: f64,
    along_m: f64,
    right_m: f64,
) -> [f64; 2] {
    let origin = Geodetic::from_degrees(lat_deg, lon_deg, 0.0);
    let local = LocalFrame::at(&origin);
    let (sin_f, cos_f) = fwd_rad.sin_cos();
    let fwd = local.north() * cos_f + local.east() * sin_f;
    let right = local.east() * cos_f - local.north() * sin_f;
    let moved = ecef_to_geodetic(geodetic_to_ecef(&origin) + fwd * along_m + right * right_m);
    [moved.lon_deg(), moved.lat_deg()]
}

pub(crate) fn polygon_feature(kind: &str, ring: &[[f64; 2]], label: Option<&str>) -> Value {
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

pub(crate) fn linestring_feature(kind: &str, coords: &[[f64; 2]], label: Option<&str>) -> Value {
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
