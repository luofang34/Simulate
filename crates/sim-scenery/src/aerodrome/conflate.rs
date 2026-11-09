//! Conflation of a raw OpenStreetMap snapshot into a surface graph: runway
//! ends from runway ways, taxi edges, hold-shorts, stands and the tower.

use std::f64::consts::{PI, TAU};

use flightbag::{
    AirportSurfaceGraph, FacilityCode, HoldShortNode, ParkingKind, ParkingSpot, RunwayDesignator,
    RunwayEndSpec, SurfaceNodeId, TaxiEdgeSpec,
};
use sim_geo::Geodetic;
use world_core::GeodeticPosition;

use super::{AerodromeId, AirspaceControl, OsmAerodromeDiagram, RawOsmSnapshot, geojson};
use crate::{AirportRecord, CatalogVerticalDatum, SceneryError};

#[allow(clippy::too_many_arguments)]
pub(super) fn build_from_snapshot(
    id: AerodromeId,
    name: String,
    control: AirspaceControl,
    field_elevation_m: f64,
    default_tower: Geodetic,
    default_rwy_width_m: f64,
    raw: &RawOsmSnapshot,
    record: Option<&AirportRecord>,
) -> Result<OsmAerodromeDiagram, SceneryError> {
    let facility =
        FacilityCode::parse(&raw.facility).map_err(|_| SceneryError::InvalidCatalog {
            detail: format!("invalid facility code {}", raw.facility),
        })?;
    let tower_viewpoint = detect_osm_tower(raw, field_elevation_m).unwrap_or(default_tower);
    let runway_ends = record
        .map(|rec| record_landing_runway_ends(rec, field_elevation_m))
        .filter(|ends| !ends.is_empty())
        .unwrap_or_else(|| extract_runway_ends(raw, field_elevation_m, default_rwy_width_m));
    let reference_point = record.map_or_else(
        || compute_arp(&runway_ends, field_elevation_m, default_tower),
        |rec| GeodeticPosition::wgs84(rec.lat_deg, rec.lon_deg, field_elevation_m),
    );
    let (taxi_edges, mut next_node) = extract_taxi_edges(raw, id.is_towered());
    let hold_short_nodes =
        extract_hold_shorts(raw, &runway_ends, field_elevation_m, &mut next_node);
    let parking_spots =
        extract_parking_spots(raw, &runway_ends, field_elevation_m, id.is_towered());
    let portrayal_geojson = geojson::build_portrayal_geojson(
        raw,
        &runway_ends,
        record,
        default_rwy_width_m,
        if id.is_towered() { 22.0 } else { 10.5 },
    );

    Ok(OsmAerodromeDiagram {
        id,
        name,
        control,
        field_elevation_m,
        tower_viewpoint,
        surface_graph: AirportSurfaceGraph {
            facility,
            reference_point,
            runway_ends,
            taxi_edges,
            hold_short_nodes,
            parking_spots,
        },
        portrayal_geojson,
    })
}

fn record_landing_runway_ends(
    record: &AirportRecord,
    field_elevation_m: f64,
) -> Vec<RunwayEndSpec> {
    let mut ends = Vec::new();
    for rwy in &record.runways {
        for end in &rwy.ends {
            let Ok(designator) = RunwayDesignator::parse(end.designator.as_str()) else {
                continue;
            };
            if let Some(landing) = record.landing_runway(
                end.designator.as_str(),
                CatalogVerticalDatum::EllipsoidWgs84,
            ) {
                ends.push(RunwayEndSpec {
                    designator,
                    threshold: GeodeticPosition::wgs84(
                        landing.threshold.lat_deg(),
                        landing.threshold.lon_deg(),
                        field_elevation_m,
                    ),
                    true_bearing_rad: landing.true_bearing_rad,
                    length_m: landing.landing_length_m,
                    width_m: landing.width_m,
                });
            }
        }
    }
    ends
}

fn detect_osm_tower(raw: &RawOsmSnapshot, field_elevation_m: f64) -> Option<Geodetic> {
    for way in &raw.ways {
        if way.kind == "control_tower" && !way.coords.is_empty() {
            let count = way.coords.len() as f64;
            let lon = way.coords.iter().map(|c| c[0]).sum::<f64>() / count;
            let lat = way.coords.iter().map(|c| c[1]).sum::<f64>() / count;
            let cab_agl_m = way
                .height
                .as_deref()
                .and_then(|s| s.parse::<f64>().ok())
                .unwrap_or(55.0);
            return Some(Geodetic::from_degrees(
                lat,
                lon,
                field_elevation_m + cab_agl_m,
            ));
        }
    }
    None
}

fn extract_runway_ends(
    raw: &RawOsmSnapshot,
    field_elevation_m: f64,
    default_width_m: f64,
) -> Vec<RunwayEndSpec> {
    let mut ends = Vec::new();
    for way in &raw.ways {
        if way.kind != "runway" || way.runway.as_deref() == Some("displaced_threshold") {
            continue;
        }
        let (Some(&start), Some(&end)) = (way.coords.first(), way.coords.last()) else {
            continue;
        };
        let Some(ref_str) = way.r#ref.as_deref() else {
            continue;
        };
        let mut parts = ref_str.split('/');
        let (Some(a_txt), Some(b_txt)) = (parts.next(), parts.next()) else {
            continue;
        };
        let (Ok(desig_a), Ok(desig_b)) = (
            RunwayDesignator::parse(a_txt),
            RunwayDesignator::parse(b_txt),
        ) else {
            continue;
        };
        let (fwd_bearing, length_m) = segment_bearing_and_length_m(start, end);
        if length_m < 150.0 {
            continue;
        }
        let rev_bearing = (fwd_bearing + PI).rem_euclid(TAU);
        let nominal_a_rad = designator_nominal_rad(desig_a.as_str()).unwrap_or(fwd_bearing);
        let (thr_a, brg_a, thr_b, brg_b) =
            if signed_angle_diff(fwd_bearing, nominal_a_rad).abs() <= PI * 0.5 {
                (start, fwd_bearing, end, rev_bearing)
            } else {
                (end, rev_bearing, start, fwd_bearing)
            };
        let width_m = way.width_m.unwrap_or(default_width_m).max(8.0);
        ends.push(RunwayEndSpec {
            designator: desig_a,
            threshold: GeodeticPosition::wgs84(thr_a[1], thr_a[0], field_elevation_m),
            true_bearing_rad: brg_a,
            length_m,
            width_m,
        });
        ends.push(RunwayEndSpec {
            designator: desig_b,
            threshold: GeodeticPosition::wgs84(thr_b[1], thr_b[0], field_elevation_m),
            true_bearing_rad: brg_b,
            length_m,
            width_m,
        });
    }
    ends
}

fn extract_taxi_edges(raw: &RawOsmSnapshot, towered: bool) -> (Vec<TaxiEdgeSpec>, u32) {
    let default_w = if towered { 22.0 } else { 10.5 };
    let mut edges = Vec::new();
    let mut next_node: u32 = 1;
    for way in &raw.ways {
        if way.kind != "taxiway" && way.kind != "taxilane" {
            continue;
        }
        let from = SurfaceNodeId(next_node);
        next_node = next_node.wrapping_add(1);
        let to = SurfaceNodeId(next_node);
        next_node = next_node.wrapping_add(1);
        edges.push(TaxiEdgeSpec {
            name: way
                .r#ref
                .clone()
                .or_else(|| way.name.clone())
                .unwrap_or_else(|| "TWY".to_owned()),
            from,
            to,
            width_m: way.width_m.unwrap_or(default_w),
        });
    }
    (edges, next_node)
}

fn extract_hold_shorts(
    raw: &RawOsmSnapshot,
    runway_ends: &[RunwayEndSpec],
    field_elevation_m: f64,
    next_node: &mut u32,
) -> Vec<HoldShortNode> {
    let fallback_desig = runway_ends
        .first()
        .map(|r| r.designator)
        .or_else(|| RunwayDesignator::parse("01").ok());
    let Some(default_rwy) = fallback_desig else {
        return Vec::new();
    };
    let mut out = Vec::new();
    for pt in &raw.points {
        if pt.kind != "holding_position" {
            continue;
        }
        let protects = pt
            .r#ref
            .as_deref()
            .and_then(|s| s.split(&['-', '/', ' '][..]).next())
            .and_then(|s| RunwayDesignator::parse(s).ok())
            .unwrap_or(default_rwy);
        let id = SurfaceNodeId(*next_node);
        *next_node = next_node.wrapping_add(1);
        out.push(HoldShortNode {
            node: id,
            position: GeodeticPosition::wgs84(pt.coord[1], pt.coord[0], field_elevation_m),
            protects_runway: protects,
        });
    }
    out
}

fn extract_parking_spots(
    raw: &RawOsmSnapshot,
    runway_ends: &[RunwayEndSpec],
    field_elevation_m: f64,
    towered: bool,
) -> Vec<ParkingSpot> {
    let default_hdg = runway_ends.first().map_or(0.0, |r| r.true_bearing_rad);
    let mut spots = Vec::new();
    let mut idx: u32 = 1;
    for pt in &raw.points {
        if pt.kind != "parking_position" && pt.kind != "gate" {
            continue;
        }
        let kind = if pt.kind == "gate" || towered {
            ParkingKind::AirlineGate
        } else {
            ParkingKind::GeneralAviationRamp
        };
        let label = pt.r#ref.clone().unwrap_or_else(|| format!("RAMP {idx}"));
        idx = idx.wrapping_add(1);
        spots.push(ParkingSpot {
            label,
            kind,
            position: GeodeticPosition::wgs84(pt.coord[1], pt.coord[0], field_elevation_m),
            heading_rad: default_hdg,
            connected_node: None,
        });
    }
    spots
}

fn compute_arp(
    runway_ends: &[RunwayEndSpec],
    field_elevation_m: f64,
    fallback: Geodetic,
) -> GeodeticPosition {
    if runway_ends.is_empty() {
        return GeodeticPosition::wgs84(
            fallback.lat_rad.to_degrees(),
            fallback.lon_rad.to_degrees(),
            field_elevation_m,
        );
    }
    let n = runway_ends.len() as f64;
    let lat = runway_ends
        .iter()
        .map(|r| r.threshold.latitude_deg)
        .sum::<f64>()
        / n;
    let lon = runway_ends
        .iter()
        .map(|r| r.threshold.longitude_deg)
        .sum::<f64>()
        / n;
    GeodeticPosition::wgs84(lat, lon, field_elevation_m)
}

pub(crate) fn segment_bearing_and_length_m(
    [lon_a, lat_a]: [f64; 2],
    [lon_b, lat_b]: [f64; 2],
) -> (f64, f64) {
    crate::airport_db::geometry::bearing_and_distance((lat_a, lon_a), (lat_b, lon_b))
}

fn designator_nominal_rad(designator: &str) -> Option<f64> {
    let digits: String = designator
        .chars()
        .take_while(char::is_ascii_digit)
        .collect();
    let num = digits.parse::<u32>().ok()?;
    Some((f64::from(num) * 10.0).to_radians().rem_euclid(TAU))
}

fn signed_angle_diff(a: f64, b: f64) -> f64 {
    (a - b + PI).rem_euclid(TAU) - PI
}
