use sim_geo::Geodetic;
use sim_scenery::{
    CatalogVerticalDatum, LiveDemBridge, TerrariumTile, encode_terrarium, project_to_tile,
};

use super::WorldSurfaceHandle;
use crate::map_rig::ScenerySource;

fn real_world() -> ScenerySource {
    #[cfg(not(target_arch = "wasm32"))]
    {
        ScenerySource::RealWorld { cache_dir: None }
    }
    #[cfg(target_arch = "wasm32")]
    {
        ScenerySource::RealWorld {}
    }
}

fn flat_tile(height_m: f64) -> TerrariumTile {
    let rgb: Vec<u8> = (0..16 * 16)
        .flat_map(|_| encode_terrarium(height_m))
        .collect();
    let image = image::RgbImage::from_raw(16, 16, rgb).unwrap();
    let mut bytes = std::io::Cursor::new(Vec::new());
    image.write_to(&mut bytes, image::ImageFormat::Png).unwrap();
    TerrariumTile::from_png_bytes(&bytes.into_inner()).unwrap()
}

#[test]
fn real_world_ground_comes_from_drained_terrarium_tiles() {
    // Far from the synthetic catalog and every built-in runway.
    let point = Geodetic::from_degrees(46.5, 8.0, 0.0);
    let surface = WorldSurfaceHandle::new();
    surface.configure(&real_world()).unwrap();
    assert_eq!(surface.ellipsoidal_ground_m(&point), None);

    let bridge = LiveDemBridge::new();
    let tile = project_to_tile(&point, 12).unwrap().tile;
    bridge
        .ingest_terrarium_tile(tile, flat_tile(1_500.0))
        .unwrap();
    assert_eq!(surface.drain(&bridge), 1);
    assert_eq!(surface.drain(&bridge), 0);

    let undulation = sim_geo::egm96_undulation_m(point.lat_rad, point.lon_rad);
    let ground = surface.ellipsoidal_ground_m(&point).unwrap();
    assert!((ground - 1_500.0 - undulation).abs() < 0.05, "{ground}");
}

#[test]
fn catalog_source_restores_the_synthetic_regional_ground() {
    let dover = Geodetic::from_degrees(39.08, -75.60, 0.0);
    let surface = WorldSurfaceHandle::new();
    let offline = surface.ellipsoidal_ground_m(&dover).unwrap();
    surface.configure(&real_world()).unwrap();
    assert_eq!(surface.ellipsoidal_ground_m(&dover), None);
    surface.configure(&ScenerySource::Catalog).unwrap();
    assert_eq!(surface.ellipsoidal_ground_m(&dover), Some(offline));
}

#[test]
fn extra_runways_survive_a_scenery_switch_in_either_datum() {
    let db = sim_scenery::AirportDatabase::builtin();
    let (_, egll) = db.by_code("EGLL").unwrap();
    let end = &egll.runways[0].ends[0];
    let threshold = Geodetic::from_degrees(end.lat_deg, end.lon_deg, 0.0);
    let charted = f64::from(end.elevation_msl_m);

    let surface = WorldSurfaceHandle::new();
    assert_eq!(
        surface.vertical_datum(),
        CatalogVerticalDatum::EllipsoidWgs84
    );
    assert_eq!(surface.ellipsoidal_ground_m(&threshold), None);
    surface.set_extra_runways(egll.runway_strips(surface.vertical_datum()));
    let synthetic = surface.ellipsoidal_ground_m(&threshold).unwrap();
    assert!(
        (synthetic - charted).abs() < 0.5,
        "{synthetic} vs {charted}"
    );

    surface.configure(&real_world()).unwrap();
    assert_eq!(
        surface.vertical_datum(),
        CatalogVerticalDatum::OrthometricMslEgm96
    );
    let undulation = surface.geoid_undulation_m(threshold.lat_rad, threshold.lon_rad);
    assert_eq!(
        undulation,
        sim_geo::egm96_undulation_m(threshold.lat_rad, threshold.lon_rad)
    );
    let real = surface.ellipsoidal_ground_m(&threshold).unwrap();
    let expected = surface.msl_to_ellipsoidal_m(threshold.lat_rad, threshold.lon_rad, charted);
    assert!((real - expected).abs() < 0.5, "{real} vs {expected}");
    let msl = surface.orthometric_ground_m(&threshold).unwrap();
    assert!((msl - charted).abs() < 0.5, "{msl} vs {charted}");

    surface.set_extra_runways(Vec::new());
    assert_eq!(surface.ellipsoidal_ground_m(&threshold), None);
    surface.configure(&ScenerySource::Catalog).unwrap();
    assert_eq!(surface.ellipsoidal_ground_m(&threshold), None);
}

#[test]
fn antarctic_polar_plateau_provides_2835m_ground_beyond_web_mercator() {
    // 3 NM on short final to NZSP RWY 02T (across 90°S in the Western Hemisphere),
    // well outside any runway strip and beyond Web Mercator's -85.0511° limit.
    let approach = Geodetic::from_degrees(-89.96, -19.7, 0.0);
    let arctic = Geodetic::from_degrees(89.5, 0.0, 0.0);
    let surface = WorldSurfaceHandle::new();
    assert!((surface.ellipsoidal_ground_m(&approach).unwrap() - 2_835.0).abs() < 0.05);
    assert!(surface.ellipsoidal_ground_m(&arctic).unwrap().abs() < 0.05);

    surface.configure(&real_world()).unwrap();
    assert!((surface.orthometric_ground_m(&approach).unwrap() - 2_835.0).abs() < 0.05);
    assert!(surface.orthometric_ground_m(&arctic).unwrap().abs() < 0.05);
}
