//! OpenStreetMap (`aeroway=*`, ODbL 1.0) aerodrome surface diagrams conflated into
//! [`flightbag::AirportSurfaceGraph`] contracts and physical-metre GeoJSON portrayals.

use std::f64::consts::{PI, TAU};

use flightbag::{
    AirportSurfaceGraph, FacilityCode, HoldShortNode, ParkingKind, ParkingSpot, RunwayDesignator,
    RunwayEndSpec, SurfaceNodeId, TaxiEdgeSpec,
};
use serde::Deserialize;
use sim_geo::Geodetic;
use world_core::GeodeticPosition;

use crate::SceneryError;

mod geojson;
#[cfg(test)]
mod tests;

const OSM_47N_JSON: &str = include_str!("aerodrome/data/47n_osm.json");
const OSM_KEWR_JSON: &str = include_str!("aerodrome/data/kewr_osm.json");
const OSM_KDOV_JSON: &str = include_str!("aerodrome/data/kdov_osm.json");

/// Built-in regional aerodromes with real OpenStreetMap surface diagrams.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum AerodromeId {
    /// Central Jersey Regional Airport (`47N`, Manville, NJ — uncontrolled CTAF).
    CentralJersey47N,
    /// Newark Liberty International Airport (`KEWR`, Newark, NJ — Class B towered).
    NewarkKewr,
    /// Dover Air Force Base (`KDOV`, Dover, DE — Class D towered).
    DoverKdov,
}

impl AerodromeId {
    /// All built-in regional aerodromes in cycling order.
    pub const ALL: [Self; 3] = [Self::CentralJersey47N, Self::NewarkKewr, Self::DoverKdov];

    /// ICAO or FAA LID identifier (`"47N"`, `"KEWR"`, `"KDOV"`).
    #[must_use]
    pub const fn code(self) -> &'static str {
        match self {
            Self::CentralJersey47N => "47N",
            Self::NewarkKewr => "KEWR",
            Self::DoverKdov => "KDOV",
        }
    }

    /// Display name for menus and telemetry status bars.
    #[must_use]
    pub const fn title(self) -> &'static str {
        match self {
            Self::CentralJersey47N => "Central Jersey Regional (47N, Uncontrolled)",
            Self::NewarkKewr => "Newark Liberty Intl (KEWR, Towered)",
            Self::DoverKdov => "Dover AFB (KDOV, Delaware)",
        }
    }

    /// Whether this aerodrome operates an active air traffic control tower.
    #[must_use]
    pub const fn is_towered(self) -> bool {
        match self {
            Self::CentralJersey47N => false,
            Self::NewarkKewr | Self::DoverKdov => true,
        }
    }

    /// Primary traffic-pattern runway designator for scenarios and free-flight spawns.
    #[must_use]
    pub const fn primary_runway(self) -> &'static str {
        match self {
            Self::CentralJersey47N => "07",
            Self::NewarkKewr => "22L",
            Self::DoverKdov => "01",
        }
    }

    /// Next aerodrome in [`Self::ALL`].
    #[must_use]
    pub const fn next(self) -> Self {
        match self {
            Self::CentralJersey47N => Self::NewarkKewr,
            Self::NewarkKewr => Self::DoverKdov,
            Self::DoverKdov => Self::CentralJersey47N,
        }
    }
}

/// Airspace and tower/CTAF communications classification for an aerodrome.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum AirspaceControl {
    /// Non-towered airfield operating on a Common Traffic Advisory Frequency (`CTAF`).
    UncontrolledCtaf {
        /// Published CTAF frequency in megahertz (e.g. `122.700`).
        ctaf_mhz: f64,
    },
    /// Towered airfield with dedicated Tower and Ground control frequencies.
    Towered {
        /// ICAO/FAA airspace class (`"B"`, `"C"`, or `"D"`).
        airspace_class: &'static str,
        /// Primary local tower frequency in megahertz.
        tower_mhz: f64,
        /// Primary ground control frequency in megahertz.
        ground_mhz: f64,
    },
}

/// Conflated OpenStreetMap aerodrome diagram: [`AirportSurfaceGraph`] + GeoJSON portrayal.
#[derive(Debug, Clone)]
pub struct OsmAerodromeDiagram {
    /// Aerodrome identifier.
    pub id: AerodromeId,
    /// Full published airport name.
    pub name: String,
    /// Towered vs. uncontrolled CTAF communications rule.
    pub control: AirspaceControl,
    /// Graded airfield elevation above WGS84 (`m` MSL).
    pub field_elevation_m: f64,
    /// Control-tower cab or airfield CTAF camera viewpoint on WGS84.
    pub tower_viewpoint: Geodetic,
    /// Single-copy `Flightbag` surface graph (runway thresholds, taxiways, hold-shorts, stands).
    pub surface_graph: AirportSurfaceGraph,
    /// Physical-metre buffered GeoJSON `FeatureCollection` for 3D OTW and 2D MFD rendering.
    pub portrayal_geojson: serde_json::Value,
}

#[derive(Debug, Deserialize)]
pub(super) struct RawOsmSnapshot {
    pub(super) facility: String,
    pub(super) ways: Vec<RawOsmWay>,
    pub(super) points: Vec<RawOsmPoint>,
}

#[derive(Debug, Deserialize)]
pub(super) struct RawOsmWay {
    pub(super) kind: String,
    pub(super) coords: Vec<[f64; 2]>,
    #[serde(default)]
    pub(super) r#ref: Option<String>,
    #[serde(default)]
    pub(super) name: Option<String>,
    #[serde(default)]
    pub(super) runway: Option<String>,
    #[serde(default)]
    pub(super) width_m: Option<f64>,
    #[serde(default)]
    pub(super) height: Option<String>,
}

#[derive(Debug, Deserialize)]
pub(super) struct RawOsmPoint {
    pub(super) kind: String,
    pub(super) coord: [f64; 2],
    #[serde(default)]
    pub(super) r#ref: Option<String>,
}

impl OsmAerodromeDiagram {
    /// Loads and conflates one of the built-in OpenStreetMap aerodromes.
    pub fn builtin(id: AerodromeId) -> Result<Self, SceneryError> {
        let (json_text, name, control, field_elevation_m, default_tower, default_rwy_w) = match id {
            AerodromeId::CentralJersey47N => (
                OSM_47N_JSON,
                "Central Jersey Regional Airport",
                AirspaceControl::UncontrolledCtaf { ctaf_mhz: 122.700 },
                26.0,
                Geodetic::from_degrees(40.525_620, -74.597_850, 38.0),
                15.2,
            ),
            AerodromeId::NewarkKewr => (
                OSM_KEWR_JSON,
                "Newark Liberty International Airport",
                AirspaceControl::Towered {
                    airspace_class: "B",
                    tower_mhz: 118.300,
                    ground_mhz: 121.800,
                },
                4.5,
                Geodetic::from_degrees(40.692_574, -74.182_761, 64.5),
                45.7,
            ),
            AerodromeId::DoverKdov => (
                OSM_KDOV_JSON,
                "Dover Air Force Base",
                AirspaceControl::Towered {
                    airspace_class: "D",
                    tower_mhz: 126.350,
                    ground_mhz: 118.425,
                },
                8.5,
                Geodetic::from_degrees(39.131_200, -75.469_500, 36.0),
                45.7,
            ),
        };
        let raw: RawOsmSnapshot =
            serde_json::from_str(json_text).map_err(|err| SceneryError::InvalidCatalog {
                detail: format!("OSM aerodrome JSON {}: {err}", id.code()),
            })?;
        build_from_snapshot(
            id,
            name.to_owned(),
            control,
            field_elevation_m,
            default_tower,
            default_rwy_w,
            &raw,
        )
    }

    /// Merges all built-in aerodromes into a single GeoJSON `FeatureCollection`.
    pub fn builtin_all_merged_geojson() -> Result<serde_json::Value, SceneryError> {
        let mut features = Vec::new();
        for id in AerodromeId::ALL {
            let diagram = Self::builtin(id)?;
            if let Some(arr) = diagram
                .portrayal_geojson
                .get("features")
                .and_then(serde_json::Value::as_array)
            {
                features.extend(arr.iter().cloned());
            }
        }
        Ok(serde_json::json!({
            "type": "FeatureCollection",
            "features": features,
        }))
    }

    /// Primary traffic-pattern [`RunwayEndSpec`] for this aerodrome.
    #[must_use]
    pub fn primary_runway_end(&self) -> Option<&RunwayEndSpec> {
        let desig = RunwayDesignator::parse(self.id.primary_runway()).ok()?;
        self.surface_graph.runway_end(desig)
    }
}

fn build_from_snapshot(
    id: AerodromeId,
    name: String,
    control: AirspaceControl,
    field_elevation_m: f64,
    default_tower: Geodetic,
    default_rwy_width_m: f64,
    raw: &RawOsmSnapshot,
) -> Result<OsmAerodromeDiagram, SceneryError> {
    let facility =
        FacilityCode::parse(&raw.facility).map_err(|_| SceneryError::InvalidCatalog {
            detail: format!("invalid facility code {}", raw.facility),
        })?;
    let tower_viewpoint = detect_osm_tower(raw, field_elevation_m).unwrap_or(default_tower);
    let runway_ends = extract_runway_ends(raw, field_elevation_m, default_rwy_width_m);
    let reference_point = compute_arp(&runway_ends, field_elevation_m, default_tower);
    let (taxi_edges, mut next_node) = extract_taxi_edges(raw, id.is_towered());
    let hold_short_nodes =
        extract_hold_shorts(raw, &runway_ends, field_elevation_m, &mut next_node);
    let parking_spots =
        extract_parking_spots(raw, &runway_ends, field_elevation_m, id.is_towered());
    let portrayal_geojson = geojson::build_portrayal_geojson(
        raw,
        &runway_ends,
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

pub(super) fn segment_bearing_and_length_m(a_lonlat: [f64; 2], b_lonlat: [f64; 2]) -> (f64, f64) {
    let mean_lat_rad = 0.5 * (a_lonlat[1] + b_lonlat[1]).to_radians();
    let d_north_m = (b_lonlat[1] - a_lonlat[1]).to_radians() * 6_378_137.0;
    let d_east_m = (b_lonlat[0] - a_lonlat[0]).to_radians() * 6_378_137.0 * mean_lat_rad.cos();
    let bearing = d_east_m.atan2(d_north_m).rem_euclid(TAU);
    (bearing, d_north_m.hypot(d_east_m))
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
