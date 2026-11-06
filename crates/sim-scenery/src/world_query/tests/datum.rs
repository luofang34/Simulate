use sim_geo::{Geodetic, egm96_undulation_m};

use crate::{
    AerodromeId, AuthoritativeWorldSurface, CatalogVerticalDatum, LunarTranquilityScenery,
    OsmAerodromeDiagram, RunwayStrip, SurfaceSampleSource, TerrariumTile, WorldSurfaceQuery,
    project_to_tile,
};

fn kdov_threshold() -> (OsmAerodromeDiagram, Geodetic) {
    let diagram = OsmAerodromeDiagram::builtin(AerodromeId::DoverKdov).unwrap();
    let end = *diagram.primary_runway_end().unwrap();
    let (sin, cos) = end.true_bearing_rad.sin_cos();
    let along = 300.0 / 6_371_000.0;
    let lat = end.threshold.latitude_deg.to_radians() + along * cos;
    let lon = end.threshold.longitude_deg.to_radians() + along * sin / lat.cos();
    (
        diagram,
        Geodetic {
            lat_rad: lat,
            lon_rad: lon,
            height_m: 0.0,
        },
    )
}

/// The MSL ground on the KDOV runway with live terrain at `terrain_msl_m`.
fn runway_msl_over_live_terrain(surface: AuthoritativeWorldSurface, terrain_msl_m: f64) -> f64 {
    let (diagram, on_runway) = kdov_threshold();
    let mut surface = surface.with_aerodrome(&diagram);
    let tile = project_to_tile(&on_runway, 13).unwrap().tile;
    surface
        .insert_terrarium_tile(
            tile,
            TerrariumTile::from_heights(4, 4, vec![terrain_msl_m; 16]).unwrap(),
        )
        .unwrap();
    let sample = surface.sample_surface(&on_runway).unwrap().unwrap();
    assert_eq!(sample.source, SurfaceSampleSource::RunwaySurface);
    sample.orthometric_msl_m.0
}

#[test]
fn geoid_undulation_follows_the_surface_datum() {
    let (lat, lon) = (39.13_f64.to_radians(), (-75.47_f64).to_radians());
    let n = egm96_undulation_m(lat, lon);
    assert!(n < -30.0, "Delaware undulation {n}");
    let msl = AuthoritativeWorldSurface::earth(CatalogVerticalDatum::OrthometricMslEgm96);
    assert_eq!(msl.geoid_undulation_m(lat, lon), n);
    assert!((msl.msl_to_ellipsoidal_m(lat, lon, 8.81) - (8.81 + n)).abs() < 1e-12);
    let ellipsoid = AuthoritativeWorldSurface::earth(CatalogVerticalDatum::EllipsoidWgs84);
    assert_eq!(ellipsoid.geoid_undulation_m(lat, lon), 0.0);
    assert_eq!(ellipsoid.msl_to_ellipsoidal_m(lat, lon, 8.81), 8.81);
    let moon = AuthoritativeWorldSurface::moon(LunarTranquilityScenery::apollo_11_landing());
    assert_eq!(moon.geoid_undulation_m(lat, lon), 0.0);
}

#[test]
fn a_runway_matches_unflattened_live_terrain_without_sinking_or_floating() {
    let field = AerodromeId::DoverKdov.field_elevation_msl_m();
    let live = AuthoritativeWorldSurface::earth_live_dem();
    assert!(live.unflattened_visual_dem());
    // Terrarium 3 m above the surveyed threshold: stand on the rendered mesh.
    let raised = runway_msl_over_live_terrain(live.clone(), field + 3.0);
    assert!((raised - (field + 3.0)).abs() < 1e-6, "{raised}");
    // Terrarium 4 m below the surveyed ARP: stand on the rendered mesh rather than floating 4 m up.
    let dipped = runway_msl_over_live_terrain(live, field - 4.0);
    assert!((dipped - (field - 4.0)).abs() < 1e-6, "{dipped}");
    // A flattened world keeps the surveyed height over higher or lower terrain.
    let flattened = AuthoritativeWorldSurface::earth_live_dem().with_unflattened_visual_dem(false);
    let pinned = runway_msl_over_live_terrain(flattened, field + 3.0);
    assert!((pinned - field).abs() < 1e-6, "{pinned}");
}

#[test]
fn the_floor_is_on_for_bare_orthometric_earth_and_off_over_a_catalog() {
    assert!(
        AuthoritativeWorldSurface::earth(CatalogVerticalDatum::OrthometricMslEgm96)
            .unflattened_visual_dem()
    );
    assert!(
        !AuthoritativeWorldSurface::earth(CatalogVerticalDatum::EllipsoidWgs84)
            .unflattened_visual_dem()
    );
    let catalog = crate::CatalogScenery::synthetic_aerodrome(AerodromeId::DoverKdov).unwrap();
    assert!(!AuthoritativeWorldSurface::earth_with_catalog(catalog).unflattened_visual_dem());
}

#[test]
fn extra_runways_join_the_ground_and_move_the_version() {
    let (diagram, on_runway) = kdov_threshold();
    let mut surface = AuthoritativeWorldSurface::earth(CatalogVerticalDatum::OrthometricMslEgm96);
    assert!(surface.sample_surface(&on_runway).unwrap().is_none());
    let version = surface.world_version();
    surface.set_extra_runways(RunwayStrip::from_ends(&diagram.surface_graph.runway_ends));
    assert_ne!(surface.world_version(), version);
    assert_eq!(
        surface.extra_runway_len(),
        diagram.surface_graph.runway_ends.len()
    );
    let sample = surface.sample_surface(&on_runway).unwrap().unwrap();
    assert_eq!(sample.source, SurfaceSampleSource::RunwaySurface);
    assert!((sample.orthometric_msl_m.0 - diagram.field_elevation_m).abs() < 1e-6);
    surface.set_extra_runways(Vec::new());
    assert!(surface.sample_surface(&on_runway).unwrap().is_none());
}

#[test]
fn diagram_heights_restate_as_true_ellipsoidal_heights() {
    let diagram = OsmAerodromeDiagram::builtin(AerodromeId::NewarkKewr).unwrap();
    let graph = diagram.to_flightbag_graph_for_datum(CatalogVerticalDatum::OrthometricMslEgm96);
    for (charted, restated) in diagram
        .surface_graph
        .runway_ends
        .iter()
        .zip(&graph.runway_ends)
    {
        let n = egm96_undulation_m(
            charted.threshold.latitude_deg.to_radians(),
            charted.threshold.longitude_deg.to_radians(),
        );
        let lifted =
            restated.threshold.ellipsoidal_height_m - charted.threshold.ellipsoidal_height_m;
        assert!((lifted - n).abs() < 1e-9);
    }
    let same = diagram.to_flightbag_graph_for_datum(CatalogVerticalDatum::EllipsoidWgs84);
    assert_eq!(same, diagram.surface_graph);
    let tower = diagram.tower_viewpoint_for_datum(CatalogVerticalDatum::OrthometricMslEgm96);
    let n = egm96_undulation_m(tower.lat_rad, tower.lon_rad);
    assert!((tower.height_m - diagram.tower_viewpoint.height_m - n).abs() < 1e-9);
}
