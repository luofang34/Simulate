//! The global aerodrome catalog: every database airport listed, the mapped three kept as
//! diagrams with surveyed elevations, and ranked code and name searches.

use sim_scenery::{AerodromeId, AirportDatabase, OsmAerodromeDiagram};

use crate::esc_menu::catalog::{AerodromeEntry, AerodromeKind, HubCatalog};

fn first_code(catalog: &HubCatalog, query: &str) -> Option<String> {
    let rows = catalog.filter_aerodromes(query);
    let first = rows.first()?;
    catalog.aerodromes.get(*first).map(|a| a.code.clone())
}

#[test]
fn the_global_catalog_lists_every_database_airport_then_the_corridor_and_the_moon() {
    let catalog = HubCatalog::with_global_airports();
    let database = AirportDatabase::builtin();
    assert!(database.len() > 8_000, "{} airports", database.len());
    assert_eq!(catalog.aerodromes.len(), database.len() + 2);
    let kinds: Vec<AerodromeKind> = catalog
        .aerodromes
        .iter()
        .rev()
        .take(2)
        .map(|a| a.kind)
        .collect();
    assert_eq!(
        kinds,
        [AerodromeKind::Tranquility, AerodromeKind::DelawareCorridor]
    );
    for (i, entry) in catalog.aerodromes.iter().take(database.len()).enumerate() {
        let idx = entry.airport_idx.expect("database row");
        assert_eq!(idx.index(), i, "{} keeps record order", entry.code);
        assert_eq!(catalog.index_of_airport(idx), Some(i));
    }
}

#[test]
fn mapped_airports_keep_their_stands_and_take_surveyed_elevations() {
    let catalog = HubCatalog::with_global_airports();
    for (id, code, elevation_m, elevation_ft) in [
        (AerodromeId::DoverKdov, "KDOV", 8.81, 29.0),
        (AerodromeId::NewarkKewr, "KEWR", 5.33, 17.0),
        (AerodromeId::CentralJersey47N, "47N", 26.18, 86.0),
    ] {
        let entry = catalog
            .aerodromes
            .iter()
            .find(|a| a.kind == AerodromeKind::Osm(id))
            .unwrap_or_else(|| panic!("{code} listed"));
        assert_eq!(entry.code, code);
        let diagram = OsmAerodromeDiagram::builtin(id).expect("diagram");
        let mapped = AerodromeEntry::from_diagram(&diagram);
        assert!(!entry.runways.is_empty(), "{code}");
        assert_eq!(
            (&entry.runways, &entry.stands),
            (&mapped.runways, &mapped.stands),
            "{code}"
        );
        assert!(
            (entry.field_elevation_m - elevation_m).abs() < 0.05,
            "{code} {}",
            entry.field_elevation_m
        );
        assert!(
            (entry.field_elevation_m * 3.280_84 - elevation_ft).abs() < 0.6,
            "{code}"
        );
        assert!(
            entry.location_hint.contains("US-"),
            "{}",
            entry.location_hint
        );
    }
    let kewr = catalog
        .aerodromes
        .iter()
        .find(|a| a.code == "KEWR")
        .expect("KEWR");
    assert!(
        kewr.location_hint.contains("TWR 118.300"),
        "{}",
        kewr.location_hint
    );
}

#[test]
fn searches_rank_exact_codes_first_and_find_names_worldwide() {
    let catalog = HubCatalog::with_global_airports();
    for (query, code) in [
        ("KEWR", "KEWR"),
        ("EWR", "KEWR"),
        ("47N", "47N"),
        ("KDOV", "KDOV"),
        ("KJFK", "KJFK"),
        ("KSFO", "KSFO"),
        ("EGLL", "EGLL"),
        ("RJTT", "RJTT"),
        ("VNLK", "VNLK"),
    ] {
        assert_eq!(
            first_code(&catalog, query).as_deref(),
            Some(code),
            "{query}"
        );
    }
    assert!(catalog.filter_aerodromes("LONDON").len() > 1);
    for query in ["NEWARK", "TOKYO", "LONDON"] {
        let rows = catalog.filter_aerodromes(query);
        assert!(!rows.is_empty(), "{query} finds airports");
        assert!(
            rows.iter().all(|&i| catalog
                .aerodromes
                .get(i)
                .is_some_and(|a| a.airport_idx.is_some())),
            "{query}"
        );
    }
    let empty = catalog.filter_aerodromes("");
    assert!(empty.len() > 10, "the curated list and the specials");
    assert_eq!(
        first_code(&catalog, "tranq").as_deref(),
        Some("TRANQUILITY")
    );
    assert_eq!(
        first_code(&catalog, "DELAWARE").as_deref(),
        Some("DELAWARE")
    );
}

#[test]
fn database_airports_spawn_from_runway_ends_without_a_ramp() {
    let catalog = HubCatalog::with_global_airports();
    let rows = catalog.filter_aerodromes("EGLL");
    let heathrow = rows
        .first()
        .and_then(|&i| catalog.aerodromes.get(i))
        .expect("EGLL");
    assert!(matches!(heathrow.kind, AerodromeKind::Database(_)));
    assert!(heathrow.runways.len() >= 4, "{:?}", heathrow.runways);
    let modes = heathrow.spawn_modes();
    assert!(!modes.is_empty() && !modes.contains(&crate::esc_menu::SpawnMode::Ramp));
    assert!(
        heathrow
            .lat_deg
            .is_some_and(|lat| (51.0..52.0).contains(&lat))
    );
}
