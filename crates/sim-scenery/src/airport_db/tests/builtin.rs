use flightbag::AuthoritySource;
use sim_geo::egm96_undulation_m;

use crate::{
    AerodromeTier, AirportDatabase, AirportIdx, AuthoritativeWorldSurface, CatalogVerticalDatum,
    FrequencyRole, SurfaceSampleSource, WorldSurfaceQuery,
};

fn db() -> &'static AirportDatabase {
    AirportDatabase::builtin()
}

fn code(idx: AirportIdx) -> &'static str {
    db().get(idx).unwrap().ids.primary.as_str()
}

#[test]
fn the_builtin_pack_decodes_thousands_of_airports_worldwide() {
    let fresh = AirportDatabase::decode_builtin().unwrap();
    assert!(fresh.len() >= 6_000, "{} airports", fresh.len());
    assert!(
        fresh.runway_count() >= 9_000,
        "{} runways",
        fresh.runway_count()
    );
    assert_eq!(db().len(), fresh.len());
    let faa = fresh
        .records()
        .iter()
        .filter(|r| r.authority == AuthoritySource::FaaNasr)
        .count();
    assert!(
        faa >= 4_000 && fresh.len() - faa >= 3_000,
        "{faa} FAA of {}",
        fresh.len()
    );
    assert!(
        fresh
            .records()
            .iter()
            .filter(|r| r.authority == AuthoritySource::FaaNasr)
            .all(|r| r.effective_ymd == [2026, 10, 29])
    );
}

#[test]
fn the_builtin_pack_re_encodes_byte_for_byte() {
    assert_eq!(db().to_pack_bytes().unwrap(), super::super::BUILTIN_PACK);
}

#[test]
fn airports_answer_to_icao_faa_and_iata_codes() {
    for (query, primary) in [
        ("KEWR", "KEWR"),
        ("ewr", "KEWR"),
        ("47N", "47N"),
        ("DOV", "KDOV"),
        ("KDOV", "KDOV"),
        ("JFK", "KJFK"),
        ("KSFO", "KSFO"),
        ("ORD", "KORD"),
        ("OSH", "KOSH"),
        ("EGLL", "EGLL"),
        ("LHR", "EGLL"),
        ("RJTT", "RJTT"),
        ("HND", "RJTT"),
        ("LFPG", "LFPG"),
        ("CDG", "LFPG"),
        ("YSSY", "YSSY"),
        ("VNLK", "VNLK"),
        ("LOWI", "LOWI"),
        ("KADW", "KADW"),
        ("PGUA", "PGUA"),
    ] {
        let (_, record) = db()
            .by_code(query)
            .unwrap_or_else(|| panic!("{query} missing"));
        assert_eq!(record.ids.primary.as_str(), primary, "{query}");
    }
    assert!(db().by_code("").is_none());
    assert!(db().by_code("QQQQQ").is_none());
}

#[test]
fn central_jersey_carries_its_surveyed_nasr_runway() {
    let (_, n47) = db().by_code("47N").unwrap();
    assert!(
        (n47.elevation_msl_m - 26.18).abs() < 0.01,
        "{}",
        n47.elevation_msl_m
    );
    assert!((n47.mag_var_deg + 13.0).abs() < 1e-6);
    assert!(!n47.towered);
    assert_eq!(
        n47.primary_frequency_mhz(),
        Some((FrequencyRole::Ctaf, 122.7))
    );
    let runway = n47
        .runways
        .iter()
        .find(|r| r.id.as_str() == "07/25")
        .unwrap();
    assert!((runway.length_m - 3_507.0 * 0.3048).abs() < 0.01);
    let [r07, r25] = &runway.ends;
    assert_eq!(r07.designator.as_str(), "07");
    assert!((r07.elevation_msl_m - 85.3 * 0.3048).abs() < 0.01);
    assert!((r25.elevation_msl_m - 63.5 * 0.3048).abs() < 0.01);
    assert!((r07.displaced_threshold_m - 580.0 * 0.3048).abs() < 0.01);
    // True 56° under 13° W variation is 69° magnetic: runway 07.
    assert!(
        (r07.true_heading_deg - 56.0).abs() < 1.0,
        "{}",
        r07.true_heading_deg
    );
    assert!((r07.magnetic_heading_deg - 69.0).abs() < 1.0);
    assert!((r07.glidepath_deg - 3.4).abs() < 1e-6);
    assert!(r07.has_vgsi && !r07.has_ils);
}

#[test]
fn towered_fields_list_their_tower_first() {
    let (_, kewr) = db().by_code("KEWR").unwrap();
    assert!(kewr.towered);
    assert_eq!(kewr.tier, AerodromeTier::Tier1Major);
    assert_eq!(
        kewr.ids.iata.map(|c| c.as_str().to_owned()).as_deref(),
        Some("EWR")
    );
    assert!((kewr.elevation_msl_m - 5.334).abs() < 0.01);
    assert_eq!(
        kewr.primary_frequency_mhz(),
        Some((FrequencyRole::Tower, 118.3))
    );
    let r04l = kewr
        .runways
        .iter()
        .flat_map(|r| &r.ends)
        .find(|e| e.designator.as_str() == "04L")
        .unwrap();
    assert!(r04l.has_ils && r04l.has_approach_lights);
    let (_, kdov) = db().by_code("KDOV").unwrap();
    assert_eq!(
        kdov.primary_frequency_mhz(),
        Some((FrequencyRole::Tower, 126.35))
    );
    let (_, egll) = db().by_code("EGLL").unwrap();
    assert!(egll.towered && egll.hard_surface);
    assert_eq!(egll.tier, AerodromeTier::Tier1Major);
    assert_eq!(egll.authority, AuthoritySource::RegionalAip);
    assert_eq!(egll.country(), "GB");
    assert_eq!(egll.region_code.as_str(), "GB-ENG");
    // Heathrow's designators sit about 1° W of true.
    assert!(egll.mag_var_deg.abs() < 4.0, "{}", egll.mag_var_deg);
}

#[test]
fn a_world_view_shows_hubs_and_the_built_in_aerodromes() {
    let markers = db().query_globe_visible(40.0, -74.0, 2.0, 200);
    assert!(
        markers.len() <= 200 && markers.len() > 50,
        "{}",
        markers.len()
    );
    let codes: Vec<&str> = markers.iter().map(|&idx| code(idx)).collect();
    for featured in ["47N", "KEWR", "KDOV"] {
        assert!(codes.contains(&featured), "{featured}");
    }
    assert!(codes.contains(&"KJFK") && codes.contains(&"KORD"));
    for &idx in &markers {
        let record = db().get(idx).unwrap();
        let featured = ["47N", "KDOV"].contains(&record.ids.primary.as_str());
        assert!(
            featured || record.tier == AerodromeTier::Tier1Major,
            "{}",
            record.ids.primary
        );
    }
    let few = db().query_globe_visible(40.0, -74.0, 2.0, 5);
    assert_eq!(few.len(), 5);
    assert_eq!(code(few[0]), "47N");
}

#[test]
fn a_zoomed_view_scans_only_nearby_minor_fields() {
    let markers = db().query_globe_visible(40.52, -74.60, 9.0, 500);
    let radius = crate::globe_visible_radius_deg(9.0);
    assert!(radius < 1.0, "{radius}");
    assert!(markers.iter().any(|&idx| code(idx) == "47N"));
    assert!(
        markers
            .iter()
            .any(|&idx| db().get(idx).unwrap().tier >= AerodromeTier::Tier3Local)
    );
    for &idx in &markers {
        let r = db().get(idx).unwrap();
        let featured = ["47N", "KEWR", "KDOV"].contains(&r.ids.primary.as_str());
        assert!(featured || ((r.lat_deg - 40.52).abs() < 1.0 && (r.lon_deg + 74.60).abs() < 1.5));
    }
    let tokyo = db().query_globe_visible(35.55, 139.78, 2.0, 300);
    assert!(tokyo.iter().any(|&idx| code(idx) == "RJTT"));
    assert!(
        !tokyo.iter().any(|&idx| code(idx) == "KEWR"),
        "KEWR is past the horizon"
    );
    let pinned = db().by_code("RJTT").unwrap().0;
    let with_pin = db().query_globe_visible_pinned(35.0, 135.0, 7.0, 3, Some(pinned));
    assert_eq!(with_pin.first(), Some(&pinned));
}

#[test]
fn nearest_picks_the_closest_field_of_a_tier() {
    let n47 = db()
        .nearest(40.5245, -74.5984, 5.0, AerodromeTier::Tier4Minor)
        .unwrap();
    assert_eq!(code(n47), "47N");
    let lhr = db()
        .nearest(51.47, -0.45, 30.0, AerodromeTier::Tier1Major)
        .unwrap();
    assert_eq!(code(lhr), "EGLL");
    assert!(
        db().nearest(0.0, -30.0, 50.0, AerodromeTier::Tier4Minor)
            .is_none()
    );
    let antimeridian = db().nearest(-17.75, 177.45, 50.0, AerodromeTier::Tier2Regional);
    assert_eq!(antimeridian.map(code), Some("NFFN"));
}

#[test]
fn search_ranks_codes_then_names() {
    let curated = db().search("", 5);
    assert_eq!(
        curated.iter().map(|&i| code(i)).collect::<Vec<_>>(),
        ["KEWR", "47N", "KDOV", "KJFK", "KSFO"]
    );
    assert_eq!(code(db().search("EWR", 5)[0]), "KEWR");
    assert_eq!(code(db().search("hnd", 5)[0]), "RJTT");
    assert_eq!(code(db().search("newark", 5)[0]), "KEWR");
    assert_eq!(code(db().search("heathrow", 5)[0]), "EGLL");
    assert_eq!(code(db().search("london", 5)[0]), "EGLL");
    assert_eq!(code(db().search("central jersey", 5)[0]), "47N");
    let prefix = db().search("KS", 40);
    assert!(prefix.iter().any(|&i| code(i) == "KSFO"));
    assert!(db().search("zzzzzzzz", 5).is_empty());
    assert_eq!(db().search("a", 7).len(), 7);
}

#[test]
fn flightbag_graphs_hold_true_ellipsoidal_heights() {
    let (_, kewr) = db().by_code("KEWR").unwrap();
    let graph = kewr.to_flightbag_graph(CatalogVerticalDatum::OrthometricMslEgm96);
    assert_eq!(graph.facility.as_str(), "KEWR");
    assert_eq!(graph.runway_ends.len(), 6);
    assert!(!graph.taxi_edges.is_empty(), "OSM taxiways conflated");
    for end in &graph.runway_ends {
        let record = kewr
            .runways
            .iter()
            .flat_map(|r| &r.ends)
            .find(|e| e.designator.as_str() == end.designator.as_str())
            .unwrap();
        let n = egm96_undulation_m(
            end.threshold.latitude_deg.to_radians(),
            end.threshold.longitude_deg.to_radians(),
        );
        assert!(n < -30.0);
        assert!(
            (end.threshold.ellipsoidal_height_m - (f64::from(record.elevation_msl_m) + n)).abs()
                < 1e-6
        );
    }
    let synthetic = kewr.to_flightbag_graph(CatalogVerticalDatum::EllipsoidWgs84);
    assert!(
        (synthetic.reference_point.ellipsoidal_height_m - f64::from(kewr.elevation_msl_m)).abs()
            < 1e-6
    );

    let (_, egll) = db().by_code("EGLL").unwrap();
    let graph = egll.to_flightbag_graph(CatalogVerticalDatum::OrthometricMslEgm96);
    assert_eq!(graph.facility.as_str(), "EGLL");
    assert_eq!(graph.runway_ends.len(), 4);
    assert!(graph.taxi_edges.is_empty());
    let r09l = graph
        .runway_ends
        .iter()
        .find(|e| e.designator.as_str() == "09L")
        .unwrap();
    assert!((r09l.true_bearing_rad.to_degrees() - 90.0).abs() < 1.0);
    assert!((r09l.length_m - 3_900.0).abs() < 50.0, "{}", r09l.length_m);
}

#[test]
fn surface_geojson_draws_any_airport() {
    let (_, egll) = db().by_code("EGLL").unwrap();
    let value: serde_json::Value = serde_json::from_str(&egll.to_surface_geojson()).unwrap();
    let features = value["features"].as_array().unwrap();
    let count = |kind: &str| {
        features
            .iter()
            .filter(|f| f["properties"]["kind"] == kind)
            .count()
    };
    assert_eq!(count("runway_pavement"), 2);
    assert_eq!(count("runway_centerline"), 2);
    assert_eq!(count("threshold_bar"), 4);
    assert_eq!(count("displaced_centerline"), 2);
    let (_, n47) = db().by_code("47N").unwrap();
    let osm: serde_json::Value = serde_json::from_str(&n47.to_surface_geojson()).unwrap();
    assert!(osm["features"].as_array().unwrap().len() > 50);
}

#[test]
fn towers_stand_beside_the_primary_runway() {
    let (_, egll) = db().by_code("EGLL").unwrap();
    let tower = egll.tower_viewpoint(CatalogVerticalDatum::OrthometricMslEgm96);
    let n = egm96_undulation_m(tower.lat_rad, tower.lon_rad);
    assert!((tower.height_m - (f64::from(egll.elevation_msl_m) + 38.0 + n)).abs() < 1e-3);
    let runway = egll.primary_runway().unwrap();
    let [a, b] = &runway.ends;
    let mid = ((a.lat_deg + b.lat_deg) * 0.5, (a.lon_deg + b.lon_deg) * 0.5);
    let (_, distance) = crate::airport_db::geometry::bearing_and_distance(
        mid,
        (tower.lat_rad.to_degrees(), tower.lon_rad.to_degrees()),
    );
    assert!((distance - 180.0).abs() < 1.0, "{distance}");
    let (_, kdov) = db().by_code("KDOV").unwrap();
    let osm_tower = kdov.tower_viewpoint(CatalogVerticalDatum::EllipsoidWgs84);
    let diagram = crate::OsmAerodromeDiagram::builtin(crate::AerodromeId::DoverKdov).unwrap();
    assert_eq!(osm_tower, diagram.tower_viewpoint);
}

#[test]
fn a_picked_airport_becomes_live_ground() {
    let (_, egll) = db().by_code("EGLL").unwrap();
    let mut surface = AuthoritativeWorldSurface::earth_live_dem();
    surface.set_extra_runways(egll.runway_strips(CatalogVerticalDatum::OrthometricMslEgm96));
    assert_eq!(surface.extra_runway_len(), 4);
    let runway = egll.primary_runway().unwrap();
    let threshold = runway.ends[0].threshold_geodetic(CatalogVerticalDatum::OrthometricMslEgm96);
    let sample = surface.sample_surface(&threshold).unwrap().unwrap();
    assert_eq!(sample.source, SurfaceSampleSource::RunwaySurface);
    assert!((sample.orthometric_msl_m.0 - f64::from(runway.ends[0].elevation_msl_m)).abs() < 1e-3);
    assert!((sample.ellipsoidal_height_m.0 - threshold.height_m).abs() < 1e-3);
}
