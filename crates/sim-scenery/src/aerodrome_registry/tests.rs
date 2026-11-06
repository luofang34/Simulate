use sim_geo::geodetic_to_ecef;

use super::*;

#[test]
fn the_builtins_are_registered_and_searchable() {
    let registry = AerodromeRegistry::with_builtins().unwrap();
    assert_eq!(registry.len(), AerodromeId::ALL.len());
    for id in AerodromeId::ALL {
        let record = registry.get(&id.code().to_lowercase()).unwrap();
        assert_eq!(record.builtin, Some(id));
    }
    let codes = |query: &str| -> Vec<String> {
        registry
            .search(query)
            .iter()
            .map(|record| record.code.clone())
            .collect()
    };
    assert_eq!(codes("us-nj"), vec!["47N", "KEWR"]);
    assert_eq!(codes("dover"), vec!["KDOV"]);
    assert_eq!(codes(""), vec!["47N", "KDOV", "KEWR"]);
    assert!(codes("nowhere").is_empty());
}

#[test]
fn a_runtime_record_joins_and_replaces_by_code() {
    let mut registry = AerodromeRegistry::with_builtins().unwrap();
    let mut graph = OsmAerodromeDiagram::builtin(AerodromeId::DoverKdov)
        .unwrap()
        .surface_graph;
    graph.facility = flightbag::FacilityCode::parse("KXYZ").unwrap();
    let record = AerodromeRecord::from_surface_graph(graph, "Scenery Pack Field", "XX", 12.0);
    assert!(registry.register(record.clone()).is_none());
    assert_eq!(registry.search("scenery pack").len(), 1);
    assert_eq!(
        registry.register(record),
        Some(registry.get("kxyz").unwrap().clone())
    );
    assert_eq!(registry.len(), AerodromeId::ALL.len() + 1);
}

#[test]
fn every_runway_end_has_a_threshold_and_a_three_degree_short_final() {
    let registry = AerodromeRegistry::with_builtins().unwrap();
    for record in registry.iter() {
        let summary = record.summary();
        assert!(!summary.runways.is_empty(), "{}", summary.code);
        for runway in &summary.runways {
            let threshold = &runway.threshold.position;
            let final_fix = &runway.short_final.position;
            let separation = (geodetic_to_ecef(final_fix) - geodetic_to_ecef(threshold)).length();
            let climb = final_fix.height_m - threshold.height_m;
            let expected =
                THRESHOLD_CROSSING_HEIGHT_M + SHORT_FINAL_DISTANCE_M * GLIDESLOPE_RAD.tan();
            assert!((climb - expected).abs() < 1e-6, "{runway:?}");
            assert!((separation.hypot(0.0) - SHORT_FINAL_DISTANCE_M.hypot(climb)).abs() < 1.0);
            assert_eq!(runway.short_final.kind, SpawnKind::ShortFinal);
            assert_eq!(runway.threshold.heading_rad, runway.true_heading_rad);
        }
        assert_eq!(
            summary.ramps.len(),
            record.surface_graph.parking_spots.len()
        );
    }
}
