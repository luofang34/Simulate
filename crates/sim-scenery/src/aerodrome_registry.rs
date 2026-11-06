//! An open catalogue of aerodromes a world browser can list, search and
//! spawn at: the built-in OpenStreetMap diagrams plus any surface graph a
//! flight bag, GeoJSON import or scenery pack registers at runtime.

use std::collections::BTreeMap;

use flightbag::{AirportSurfaceGraph, RunwayEndSpec};
use sim_geo::{Geodetic, LocalFrame, ecef_to_geodetic, geodetic_to_ecef};

use crate::{AerodromeId, OsmAerodromeDiagram, SceneryError};

#[cfg(test)]
mod tests;

/// One nautical mile, the conventional short-final distance, metres.
pub const SHORT_FINAL_DISTANCE_M: f64 = 1_852.0;
/// The standard approach path angle, radians (3°).
pub const GLIDESLOPE_RAD: f64 = 3.0 * std::f64::consts::PI / 180.0;
/// Threshold crossing height on the glide path, metres (50 ft).
pub const THRESHOLD_CROSSING_HEIGHT_M: f64 = 15.24;

/// What a [`SpawnAnchor`] puts the aircraft into.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum SpawnKind {
    /// Lined up on the runway at its threshold, on the ground.
    RunwayThreshold,
    /// Airborne on a 3° glide path one nautical mile out.
    ShortFinal,
    /// Parked at a ramp, stand or gate.
    Ramp,
}

/// A pose a vehicle can be placed at.
#[derive(Debug, Clone, PartialEq)]
pub struct SpawnAnchor {
    /// Human-readable label, such as `"RWY 01"` or `"RAMP 2"`.
    pub label: String,
    /// Ground or airborne, and where.
    pub kind: SpawnKind,
    /// Position on WGS84; the height is ellipsoidal, in the datum the
    /// aerodrome's runway thresholds are charted in.
    pub position: Geodetic,
    /// True heading the vehicle faces, radians `[0, 2π)`.
    pub heading_rad: f64,
}

/// One runway end, ready for a spawn menu.
#[derive(Debug, Clone, PartialEq)]
pub struct RunwaySummary {
    /// Designator, such as `"22L"`.
    pub designator: String,
    /// True bearing along the centreline, radians `[0, 2π)`.
    pub true_heading_rad: f64,
    /// Usable length, metres.
    pub length_m: f64,
    /// Paved width, metres.
    pub width_m: f64,
    /// On the ground at the threshold, facing down the runway.
    pub threshold: SpawnAnchor,
    /// On the 3° glide path [`SHORT_FINAL_DISTANCE_M`] before the threshold.
    pub short_final: SpawnAnchor,
}

/// Everything a world browser lists about one aerodrome.
#[derive(Debug, Clone, PartialEq)]
pub struct AerodromeSummary {
    /// ICAO or FAA code.
    pub code: String,
    /// Display name.
    pub name: String,
    /// Region or country tag, such as `"US-NJ"`.
    pub region: String,
    /// Field elevation, metres.
    pub field_elevation_m: f64,
    /// The aerodrome reference point.
    pub reference_point: Geodetic,
    /// Every runway end.
    pub runways: Vec<RunwaySummary>,
    /// Every ramp, stand and gate.
    pub ramps: Vec<SpawnAnchor>,
}

/// One aerodrome in an [`AerodromeRegistry`].
#[derive(Debug, Clone, PartialEq)]
pub struct AerodromeRecord {
    /// ICAO or FAA code; the registry's key, compared without case.
    pub code: String,
    /// Display name.
    pub name: String,
    /// Region or country tag, such as `"US-DE"`.
    pub region: String,
    /// Field elevation, metres.
    pub field_elevation_m: f64,
    /// The built-in diagram this record came from, if any.
    pub builtin: Option<AerodromeId>,
    /// Runways, taxiways and stands.
    pub surface_graph: AirportSurfaceGraph,
}

impl AerodromeRecord {
    /// A record for any surface graph, such as one a flight bag or scenery
    /// pack supplies; its code is the graph's facility.
    #[must_use]
    pub fn from_surface_graph(
        surface_graph: AirportSurfaceGraph,
        name: impl Into<String>,
        region: impl Into<String>,
        field_elevation_m: f64,
    ) -> Self {
        Self {
            code: surface_graph.facility.as_str().to_owned(),
            name: name.into(),
            region: region.into(),
            field_elevation_m,
            builtin: None,
            surface_graph,
        }
    }

    /// A record for a conflated OpenStreetMap diagram.
    #[must_use]
    pub fn from_diagram(diagram: OsmAerodromeDiagram, region: impl Into<String>) -> Self {
        Self {
            code: diagram.id.code().to_owned(),
            name: diagram.name,
            region: region.into(),
            field_elevation_m: diagram.field_elevation_m,
            builtin: Some(diagram.id),
            surface_graph: diagram.surface_graph,
        }
    }

    /// Whether `query`, without case, appears in the code, name or region.
    /// An empty query matches everything.
    #[must_use]
    pub fn matches(&self, query: &str) -> bool {
        let needle = query.trim().to_lowercase();
        [&self.code, &self.name, &self.region]
            .iter()
            .any(|field| field.to_lowercase().contains(&needle))
    }

    /// The browser view of this aerodrome with its spawn anchors.
    #[must_use]
    pub fn summary(&self) -> AerodromeSummary {
        let arp = &self.surface_graph.reference_point;
        AerodromeSummary {
            code: self.code.clone(),
            name: self.name.clone(),
            region: self.region.clone(),
            field_elevation_m: self.field_elevation_m,
            reference_point: Geodetic::from_degrees(
                arp.latitude_deg,
                arp.longitude_deg,
                arp.ellipsoidal_height_m,
            ),
            runways: self
                .surface_graph
                .runway_ends
                .iter()
                .map(runway_summary)
                .collect(),
            ramps: self
                .surface_graph
                .parking_spots
                .iter()
                .map(|spot| SpawnAnchor {
                    label: spot.label.clone(),
                    kind: SpawnKind::Ramp,
                    position: Geodetic::from_degrees(
                        spot.position.latitude_deg,
                        spot.position.longitude_deg,
                        spot.position.ellipsoidal_height_m,
                    ),
                    heading_rad: spot.heading_rad,
                })
                .collect(),
        }
    }
}

fn runway_summary(end: &RunwayEndSpec) -> RunwaySummary {
    let threshold = Geodetic::from_degrees(
        end.threshold.latitude_deg,
        end.threshold.longitude_deg,
        end.threshold.ellipsoidal_height_m,
    );
    let label = format!("RWY {}", end.designator);
    let (sin, cos) = end.true_bearing_rad.sin_cos();
    let frame = LocalFrame::at(&threshold);
    let fwd = frame.east() * sin + frame.north() * cos;
    let back = fwd * -SHORT_FINAL_DISTANCE_M;
    let short_final = Geodetic {
        height_m: threshold.height_m
            + THRESHOLD_CROSSING_HEIGHT_M
            + SHORT_FINAL_DISTANCE_M * GLIDESLOPE_RAD.tan(),
        ..ecef_to_geodetic(geodetic_to_ecef(&threshold) + back)
    };
    let sf_ned = LocalFrame::at(&short_final).ecef_to_ned_vector(fwd);
    let sf_heading_rad = if sf_ned.x.hypot(sf_ned.y) > 1e-9 {
        sf_ned.y.atan2(sf_ned.x).rem_euclid(std::f64::consts::TAU)
    } else {
        end.true_bearing_rad
    };
    RunwaySummary {
        designator: end.designator.to_string(),
        true_heading_rad: end.true_bearing_rad,
        length_m: end.length_m,
        width_m: end.width_m,
        threshold: SpawnAnchor {
            label: format!("{label} threshold"),
            kind: SpawnKind::RunwayThreshold,
            position: threshold,
            heading_rad: end.true_bearing_rad,
        },
        short_final: SpawnAnchor {
            label: format!("{label} short final"),
            kind: SpawnKind::ShortFinal,
            position: short_final,
            heading_rad: sf_heading_rad,
        },
    }
}

/// Region tag of a built-in aerodrome.
const fn builtin_region(id: AerodromeId) -> &'static str {
    match id {
        AerodromeId::CentralJersey47N | AerodromeId::NewarkKewr => "US-NJ",
        AerodromeId::DoverKdov => "US-DE",
    }
}

/// Aerodromes by code, open to registration at runtime.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct AerodromeRegistry {
    records: BTreeMap<String, AerodromeRecord>,
}

impl AerodromeRegistry {
    /// A registry with nothing in it.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// A registry holding every built-in [`OsmAerodromeDiagram`].
    pub fn with_builtins() -> Result<Self, SceneryError> {
        let mut registry = Self::new();
        for id in AerodromeId::ALL {
            let diagram = OsmAerodromeDiagram::builtin(id)?;
            registry.register(AerodromeRecord::from_diagram(diagram, builtin_region(id)));
        }
        Ok(registry)
    }

    /// Adds `record`, returning any record it replaced under the same code.
    pub fn register(&mut self, record: AerodromeRecord) -> Option<AerodromeRecord> {
        self.records.insert(record.code.to_uppercase(), record)
    }

    /// Adds a conflated OpenStreetMap diagram.
    pub fn register_diagram(
        &mut self,
        diagram: OsmAerodromeDiagram,
        region: impl Into<String>,
    ) -> Option<AerodromeRecord> {
        self.register(AerodromeRecord::from_diagram(diagram, region))
    }

    /// The record for `code`, without case.
    #[must_use]
    pub fn get(&self, code: &str) -> Option<&AerodromeRecord> {
        self.records.get(&code.trim().to_uppercase())
    }

    /// Every record, by code.
    pub fn iter(&self) -> impl Iterator<Item = &AerodromeRecord> {
        self.records.values()
    }

    /// How many aerodromes are registered.
    #[must_use]
    pub fn len(&self) -> usize {
        self.records.len()
    }

    /// Whether nothing is registered.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.records.is_empty()
    }

    /// Records whose code, name or region contains `query`, without case,
    /// by code. An empty query lists everything.
    #[must_use]
    pub fn search(&self, query: &str) -> Vec<&AerodromeRecord> {
        self.iter().filter(|record| record.matches(query)).collect()
    }
}
