use std::sync::Arc;

use sim_geo::Geodetic;

use crate::{
    AdHocAerodrome, AdHocRunwayError, AdHocRunwaySpec, AirportDatabase, AuthoritativeWorldSurface,
    CatalogVerticalDatum, MATHIAS_RUST_1987_WAYPOINTS, RunwayDatum, RunwaySurfaceKind,
    SurfaceSampleSource, TerrariumTile, WorldSurfaceQuery, project_to_tile,
};

#[test]
fn bolshoy_moskvoretsky_bridge_and_red_square_plaza_create_valid_strips() {
    let bridge = AdHocAerodrome::bolshoy_moskvoretsky_bridge().unwrap();
    assert_eq!(bridge.code(), "ZMRS");
    assert_eq!(bridge.airport.runways.len(), 1);
    let rwy = &bridge.airport.runways[0];
    assert_eq!(rwy.id.as_str(), "33/15");
    assert!((rwy.length_m - 456.0).abs() < 25.0, "len={}", rwy.length_m);
    assert_eq!(rwy.width_m, 36.0);
    assert_eq!(rwy.surface, RunwaySurfaceKind::Asphalt);
    assert_eq!(bridge.strips.len(), 2);

    let r33 = bridge
        .airport
        .landing_runway("33", CatalogVerticalDatum::OrthometricMslEgm96)
        .unwrap();
    assert!((r33.threshold_elevation_msl_m - 128.0).abs() < 1e-3);
    assert!((r33.far_elevation_msl_m - 128.0).abs() < 1e-3);

    let plaza = AdHocAerodrome::red_square_plaza().unwrap();
    assert_eq!(plaza.code(), "ZMRQ");
    let prwy = &plaza.airport.runways[0];
    assert_eq!(prwy.id.as_str(), "30/12");
    assert!(
        (prwy.length_m - 294.0).abs() < 15.0,
        "len={}",
        prwy.length_m
    );
    assert!(
        plaza
            .airport
            .landing_runway("30", CatalogVerticalDatum::OrthometricMslEgm96)
            .is_some()
    );

    // Even without an explicit override, a bridge crossing a river gorge (abutments 128 m,
    // river channel 114 m) grades flat across the abutment deck instead of dipping.
    let mut terrain_spec = AdHocRunwaySpec::bolshoy_moskvoretsky_bridge();
    terrain_spec.override_elevation_msl_m = None;
    let from_dem = AdHocAerodrome::from_spec(&terrain_spec, |lat, _| {
        if (lat - 55.749_2).abs() < 0.000_6 {
            Some(114.0)
        } else {
            Some(128.0)
        }
    })
    .unwrap();
    let ends = &from_dem.airport.runways[0].ends;
    assert!((ends[0].elevation_msl_m - 128.0).abs() < 1e-3);
    assert!((ends[1].elevation_msl_m - 128.0).abs() < 1e-3);
}

#[test]
fn helsinki_malmi_efhf_and_mathias_rust_corridor_are_well_formed() {
    let efhf = AdHocAerodrome::helsinki_malmi_efhf().unwrap();
    assert_eq!(efhf.code(), "EFHF");
    assert_eq!(efhf.airport.runways.len(), 2);
    assert!(
        efhf.airport
            .landing_runway("18", CatalogVerticalDatum::OrthometricMslEgm96)
            .is_some()
    );
    assert!(
        efhf.airport
            .landing_runway("36", CatalogVerticalDatum::OrthometricMslEgm96)
            .is_some()
    );
    assert_eq!(MATHIAS_RUST_1987_WAYPOINTS.len(), 9);
    assert_eq!(MATHIAS_RUST_1987_WAYPOINTS[0].0, "KOHTL");
    assert_eq!(MATHIAS_RUST_1987_WAYPOINTS[7].0, "KREML");
    assert_eq!(MATHIAS_RUST_1987_WAYPOINTS[8].0, "ORDNK");
}

#[test]
fn arbitrary_two_point_runway_clamps_excess_terrain_grade_and_validates_bounds() {
    let spec = AdHocRunwaySpec::between(
        "USR1",
        "Grass Ridge Strip",
        [40.6000, -74.6000],
        [40.6060, -74.6000],
    );
    // ~666 m north-south strip over an unrealistic 80 m cliff: clamped to <= 3% (~20 m).
    let built = AdHocAerodrome::from_spec(&spec, |lat, _| {
        if lat < 40.6030 {
            Some(100.0)
        } else {
            Some(180.0)
        }
    })
    .unwrap();
    let [a, b] = &built.airport.runways[0].ends;
    let grade = (f64::from(b.elevation_msl_m - a.elevation_msl_m)).abs()
        / f64::from(built.airport.runways[0].length_m);
    assert!(grade <= 0.030_1, "grade={grade}");

    let too_short = AdHocRunwaySpec::between(
        "TINY",
        "Too Short",
        [40.6000, -74.6000],
        [40.6002, -74.6000],
    );
    assert!(matches!(
        AdHocAerodrome::from_spec(&too_short, |_, _| Some(10.0)),
        Err(AdHocRunwayError::LengthOutOfRange { .. })
    ));
}

#[test]
fn airport_database_upsert_and_live_dem_restamp_agree_on_ad_hoc_ground() {
    let bridge = AdHocAerodrome::bolshoy_moskvoretsky_bridge().unwrap();
    let mut db = (**AirportDatabase::builtin()).clone();
    let idx = db.upsert(bridge.airport.clone()).unwrap();
    assert_eq!(db.by_code("ZMRS").unwrap().0, idx);
    assert!(db.search("Moskvoretsky", 10).contains(&idx));

    let datum = RunwayDatum::new(Arc::new(db));
    let mut surface = AuthoritativeWorldSurface::earth_live_dem();

    // Simulate Moskva River live Terrarium tile (114 m MSL water channel) arriving BEFORE
    // the pilot delineates the ad-hoc bridge runway!
    let mid = Geodetic::from_degrees(55.748_75, 37.624_20, 0.0);
    let tile_id = project_to_tile(&mid, 14).unwrap().tile;
    let river_tile = TerrariumTile::from_heights(256, 256, vec![114.0; 256 * 256]).unwrap();
    surface.insert_terrarium_tile(tile_id, river_tile).unwrap();
    let before = surface.sample_surface(&mid).unwrap().unwrap();
    assert!((before.orthometric_msl_m.0 - 114.0).abs() < 0.05);

    // Register ad-hoc strips into both RunwayDatum and AuthoritativeWorldSurface, then restamp!
    datum.set_ad_hoc_strips(bridge.strips.clone());
    surface.set_extra_runways(bridge.strips.clone());
    let changed = surface.restamp_live_tiles(&datum);
    assert_eq!(changed, 1);

    let after = surface.sample_surface(&mid).unwrap().unwrap();
    assert!(
        (after.orthometric_msl_m.0 - 128.0).abs() < 0.05,
        "bridge deck stamped into resident DEM: {} m MSL (source {:?})",
        after.orthometric_msl_m.0,
        after.source
    );
    assert!(matches!(
        after.source,
        SurfaceSampleSource::ResidentDem { zoom: 14 } | SurfaceSampleSource::RunwaySurface
    ));
}
