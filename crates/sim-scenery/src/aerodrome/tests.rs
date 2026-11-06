use std::error::Error;

use flightbag::RunwayDesignator;
use sim_geo::Geodetic;

use super::{AerodromeId, AirspaceControl, OsmAerodromeDiagram};
use crate::{CatalogScenery, ElevationSource};

#[test]
fn kewr_osm_diagram_conflates_all_six_runways_control_tower_and_hold_shorts()
-> Result<(), Box<dyn Error>> {
    let kewr = OsmAerodromeDiagram::builtin(AerodromeId::NewarkKewr)?;
    assert!(matches!(
        kewr.control,
        AirspaceControl::Towered {
            airspace_class: "B",
            ..
        }
    ));
    for desig in ["04L", "22R", "04R", "22L", "11", "29"] {
        let parsed = RunwayDesignator::parse(desig)?;
        let end = kewr.surface_graph.runway_end(parsed);
        assert!(end.is_some(), "KEWR missing runway end {desig}");
    }
    let rwy_22l = kewr.primary_runway_end().ok_or("missing KEWR 22L")?;
    let brg_deg = rwy_22l.true_bearing_rad.to_degrees();
    assert!(
        (205.0..=214.0).contains(&brg_deg),
        "KEWR 22L true bearing {brg_deg}°"
    );
    assert!(rwy_22l.length_m > 2_000.0);
    assert!(rwy_22l.width_m >= 45.0);
    assert!(kewr.surface_graph.taxi_edges.len() > 100);
    assert!(kewr.surface_graph.hold_short_nodes.len() > 80);
    assert!((kewr.tower_viewpoint.lat_rad.to_degrees() - 40.692_574).abs() < 0.002);
    assert!(kewr.tower_viewpoint.height_m > 55.0);
    Ok(())
}

#[test]
fn central_jersey_47n_osm_diagram_conflates_uncontrolled_runway_07_25_and_taxiways()
-> Result<(), Box<dyn Error>> {
    let n47 = OsmAerodromeDiagram::builtin(AerodromeId::CentralJersey47N)?;
    assert!(matches!(
        n47.control,
        AirspaceControl::UncontrolledCtaf { .. }
    ));
    let rwy_07 = n47.primary_runway_end().ok_or("missing 47N Rwy 07")?;
    let brg_deg = rwy_07.true_bearing_rad.to_degrees();
    assert!(
        (48.0..=58.0).contains(&brg_deg),
        "47N Rwy 07 true bearing {brg_deg}°"
    );
    assert!((950.0..=1_200.0).contains(&rwy_07.length_m));
    assert!(!n47.surface_graph.taxi_edges.is_empty());
    assert!(!n47.surface_graph.parking_spots.is_empty());
    Ok(())
}

#[test]
fn regional_network_grades_flat_plateaus_under_47n_and_kewr_and_merges_geojson()
-> Result<(), Box<dyn Error>> {
    let catalog = CatalogScenery::synthetic_regional_network()?;
    let h_47n = catalog
        .ground_height_m(&Geodetic::from_degrees(40.524_5, -74.598_4, 0.0))?
        .ok_or("47N outside synthetic regional network")?;
    let h_kewr = catalog
        .ground_height_m(&Geodetic::from_degrees(40.692_5, -74.168_7, 0.0))?
        .ok_or("KEWR outside synthetic regional network")?;
    assert!((h_47n - 26.18).abs() < 0.05, "47N graded height {h_47n}");
    assert!((h_kewr - 5.33).abs() < 0.05, "KEWR graded height {h_kewr}");

    let merged = OsmAerodromeDiagram::builtin_all_merged_geojson()?;
    let count = merged
        .get("features")
        .and_then(serde_json::Value::as_array)
        .map_or(0, Vec::len);
    assert!(count > 400, "merged OSM portrayal features: {count}");
    Ok(())
}
