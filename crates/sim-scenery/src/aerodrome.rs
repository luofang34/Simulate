//! OpenStreetMap (`aeroway=*`, ODbL 1.0) aerodrome surface diagrams conflated into
//! [`flightbag::AirportSurfaceGraph`] contracts and physical-metre GeoJSON portrayals.

use flightbag::{AirportSurfaceGraph, RunwayDesignator, RunwayEndSpec};
use serde::Deserialize;
use sim_geo::Geodetic;

use crate::SceneryError;

mod conflate;
mod datum;
pub(crate) mod geojson;
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

    /// Surveyed field elevation above mean sea level, metres, from the FAA
    /// NASR airport record (`ELEV`, feet MSL).
    #[must_use]
    pub const fn field_elevation_msl_m(self) -> f64 {
        match self {
            // 85.9 ft.
            Self::CentralJersey47N => 26.182_32,
            // 17.5 ft.
            Self::NewarkKewr => 5.334,
            // 28.9 ft.
            Self::DoverKdov => 8.808_72,
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
    /// Surveyed field elevation, metres above mean sea level. Every height in
    /// [`Self::surface_graph`] and [`Self::tower_viewpoint`] is charted from it
    /// in the same MSL number; [`Self::to_flightbag_graph_for_datum`] restates
    /// them as WGS84 ellipsoidal heights.
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
                id.field_elevation_msl_m(),
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
                id.field_elevation_msl_m(),
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
                id.field_elevation_msl_m(),
                Geodetic::from_degrees(39.131_200, -75.469_500, 36.0),
                45.7,
            ),
        };
        let raw: RawOsmSnapshot =
            serde_json::from_str(json_text).map_err(|err| SceneryError::InvalidCatalog {
                detail: format!("OSM aerodrome JSON {}: {err}", id.code()),
            })?;
        let record = crate::AirportDatabase::builtin()
            .by_code(id.code())
            .map(|(_, rec)| rec);
        conflate::build_from_snapshot(
            id,
            name.to_owned(),
            control,
            field_elevation_m,
            default_tower,
            default_rwy_w,
            &raw,
            record,
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
