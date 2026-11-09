//! The ground under a runway is the charted runway datum, before terrain streams in and
//! after, and terrain the datum stamps carries it exactly.

use sim_geo::Geodetic;

use crate::{
    AirportDatabase, AuthoritativeWorldSurface, MIN_STAMP_ZOOM, RunwayDatum, TerrariumTile,
    WorldSurfaceQuery, project_to_tile,
};

/// The middle of Colombo's runway 04/22, where the published terrain sits about 2.7 m
/// above the charted runway and the geoid 101.6 m below the ellipsoid.
fn colombo_mid_runway() -> (Geodetic, f64) {
    let (_, vcbi) = AirportDatabase::builtin().by_code("VCBI").unwrap();
    let [a, b] = &vcbi.runways[0].ends;
    let point = Geodetic::from_degrees(
        0.5 * (a.lat_deg + b.lat_deg),
        0.5 * (a.lon_deg + b.lon_deg),
        0.0,
    );
    (
        point,
        0.5 * f64::from(a.elevation_msl_m + b.elevation_msl_m),
    )
}

fn msl_ground(surface: &AuthoritativeWorldSurface, at: &Geodetic) -> f64 {
    surface
        .sample_surface(at)
        .unwrap()
        .unwrap()
        .orthometric_msl_m
        .0
}

fn flat(height_m: f64) -> TerrariumTile {
    TerrariumTile::from_heights(256, 256, vec![height_m; 256 * 256]).unwrap()
}

#[test]
fn runway_ground_is_the_charted_datum_as_terrain_streams_in() {
    let datum = RunwayDatum::builtin();
    let mut surface = AuthoritativeWorldSurface::earth_live_dem().with_runway_datum(datum.clone());
    let (runway, charted) = colombo_mid_runway();
    let terrain = charted + 2.7;
    assert!((msl_ground(&surface, &runway) - charted).abs() < 0.01);

    // A coarse tile the datum does not stamp stands in for the terrain around the field,
    // but the runway keeps its charted height.
    let coarse = project_to_tile(&runway, MIN_STAMP_ZOOM - 1).unwrap().tile;
    surface
        .insert_terrarium_tile(coarse, flat(terrain))
        .unwrap();
    assert!((msl_ground(&surface, &runway) - charted).abs() < 0.01);

    // A fine tile as the maps receive it, stamped with the same datum: the runway reads
    // charted, the terrain beside it reads as published.
    let coord = project_to_tile(&runway, 15).unwrap();
    let mut fine = flat(terrain);
    assert!(datum.stamp_terrarium(coord.tile, &mut fine));
    surface.insert_terrarium_tile(coord.tile, fine).unwrap();
    assert!((msl_ground(&surface, &runway) - charted).abs() < 0.02);
    let beside = crate::tile_point_to_geodetic(coord.tile, 0.02, 0.02).unwrap();
    assert!(datum.height_and_weight(&beside).is_none());
    assert!((msl_ground(&surface, &beside) - terrain).abs() < 0.01);
}
