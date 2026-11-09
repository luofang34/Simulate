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
    // FAA NASR 47N 07/25 is 1,070 m (3,510 ft) full strip with a 176.8 m RWY 07 displaced
    // landing threshold, giving a surveyed landing length of ~893 m.
    assert!(
        (850.0..=1_100.0).contains(&rwy_07.length_m),
        "47N Rwy 07 landing length {} m",
        rwy_07.length_m,
    );
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

#[test]
fn builtin_diagrams_conflate_authoritative_nasr_runways_and_displaced_thresholds()
-> Result<(), Box<dyn Error>> {
    let db = crate::AirportDatabase::builtin();
    for (id, expected_displaced_rwy, min_disp_m) in [
        (AerodromeId::NewarkKewr, "22L", 500.0),
        (AerodromeId::DoverKdov, "14", 1_200.0),
        (AerodromeId::CentralJersey47N, "07", 150.0),
    ] {
        let diagram = OsmAerodromeDiagram::builtin(id)?;
        let (_, record) = db.by_code(id.code()).ok_or("missing airport record")?;
        let landing = record
            .landing_runway(
                expected_displaced_rwy,
                crate::CatalogVerticalDatum::EllipsoidWgs84,
            )
            .ok_or("missing landing runway")?;
        assert!(
            landing.displaced_threshold_m >= min_disp_m,
            "{} {expected_displaced_rwy} displaced={} m",
            id.code(),
            landing.displaced_threshold_m,
        );
        let desig = RunwayDesignator::parse(expected_displaced_rwy)?;
        let end = diagram
            .surface_graph
            .runway_end(desig)
            .ok_or("missing diagram runway end")?;
        let d_thr_m = (sim_geo::geodetic_to_ecef(&Geodetic::from_degrees(
            end.threshold.latitude_deg,
            end.threshold.longitude_deg,
            0.0,
        )) - sim_geo::geodetic_to_ecef(&Geodetic {
            height_m: 0.0,
            ..landing.threshold
        }))
        .length();
        assert!(
            d_thr_m < 0.05,
            "{} {expected_displaced_rwy} diagram threshold drifted {d_thr_m:.3} m from NASR",
            id.code(),
        );
        assert!((end.length_m - landing.landing_length_m).abs() < 0.05);
        assert!((end.true_bearing_rad - landing.true_bearing_rad).abs() < 1e-6);

        let features = diagram.portrayal_geojson["features"]
            .as_array()
            .ok_or("missing features")?;
        assert!(
            features
                .iter()
                .any(|f| f["properties"]["kind"] == "stopway_pavement"),
            "{} portrayal must separate displaced threshold as stopway_pavement",
            id.code(),
        );
        assert_threshold_bar_matches_landing(features, expected_displaced_rwy, &landing.threshold)?;
    }
    Ok(())
}

fn assert_threshold_bar_matches_landing(
    features: &[serde_json::Value],
    designator: &str,
    landing_thr: &Geodetic,
) -> Result<(), Box<dyn Error>> {
    let bar = features
        .iter()
        .find(|f| {
            f["properties"]["kind"] == "threshold_bar" && f["properties"]["ref"] == designator
        })
        .ok_or("missing threshold_bar for displaced runway")?;
    let ring = bar["geometry"]["coordinates"][0]
        .as_array()
        .ok_or("missing threshold_bar ring")?;
    let c_lon = ring[..4].iter().filter_map(|p| p[0].as_f64()).sum::<f64>() * 0.25;
    let c_lat = ring[..4].iter().filter_map(|p| p[1].as_f64()).sum::<f64>() * 0.25;
    let bar_dist_m = (sim_geo::geodetic_to_ecef(&Geodetic::from_degrees(c_lat, c_lon, 0.0))
        - sim_geo::geodetic_to_ecef(&Geodetic {
            height_m: 0.0,
            ..*landing_thr
        }))
    .length();
    // `threshold_bar_feature` spans `[+2 m, +10 m]` past `landing.threshold` (centroid `+6 m`).
    assert!(
        (bar_dist_m - 6.0).abs() < 0.35,
        "{designator} threshold_bar centroid {bar_dist_m:.2} m from landing threshold",
    );
    Ok(())
}
