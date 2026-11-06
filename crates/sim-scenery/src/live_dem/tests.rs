use std::io::Cursor;

use glam::DVec3;
use sim_geo::{Geodetic, LocalFrame, egm96_undulation_m, geodetic_to_ecef};

use super::*;
use crate::{
    AerodromeId, CatalogVerticalDatum, OsmAerodromeDiagram, SurfaceSampleSource, WorldSurfaceQuery,
    encode_terrarium, project_to_tile, tile_point_to_geodetic,
};

const SIZE: u32 = 16;

fn tile_over(position: &Geodetic, zoom: u8) -> TileId {
    project_to_tile(position, zoom).unwrap().tile
}

fn dover() -> Geodetic {
    Geodetic::from_degrees(39.08, -75.60, 0.0)
}

/// Terrarium PNG bytes rising `rise_m` per pixel eastward from `base_m`.
fn terrarium_png(base_m: f64, rise_m: f64) -> Vec<u8> {
    let mut rgb = Vec::new();
    for _y in 0..SIZE {
        for x in 0..SIZE {
            rgb.extend_from_slice(&encode_terrarium(base_m + rise_m * f64::from(x)));
        }
    }
    let image = image::RgbImage::from_raw(SIZE, SIZE, rgb).unwrap();
    let mut bytes = Cursor::new(Vec::new());
    image.write_to(&mut bytes, image::ImageFormat::Png).unwrap();
    bytes.into_inner()
}

/// Metres one pixel spans east at `tile`'s centre.
fn pixel_east_m(tile: TileId) -> f64 {
    let west = tile_point_to_geodetic(tile, 0.0, 0.5).unwrap();
    let east = tile_point_to_geodetic(tile, 1.0, 0.5).unwrap();
    (geodetic_to_ecef(&east) - geodetic_to_ecef(&west)).length() / f64::from(SIZE)
}

#[test]
fn png_bytes_drained_from_the_bridge_become_the_ground() {
    let tile = tile_over(&dover(), 12);
    let centre = tile_point_to_geodetic(tile, 0.5, 0.5).unwrap();
    let mut surface = AuthoritativeWorldSurface::earth(CatalogVerticalDatum::EllipsoidWgs84)
        .with_fallback_elevation_m(0.0);
    let before = surface.sample_surface(&centre).unwrap().unwrap();
    assert_eq!(before.source, SurfaceSampleSource::FallbackElevation);
    let version = surface.world_version();

    let bridge = LiveDemBridge::new();
    let loader = bridge.clone();
    let rise_m = 40.0;
    std::thread::spawn(move || loader.ingest_terrarium_png(tile, &terrarium_png(100.0, rise_m)))
        .join()
        .unwrap()
        .unwrap();
    assert_eq!(bridge.generation(), 1);
    assert_eq!(bridge.pending_len_may_be_stale(), 1);
    assert_eq!(bridge.drain_into(&mut surface), 1);
    assert_eq!(bridge.drain_into(&mut surface), 0);
    assert_ne!(surface.world_version(), version);
    assert_eq!(surface.live_dem_tiles(), vec![tile]);

    // Terrarium heights are orthometric; the surface reads ellipsoidal.
    let undulation = egm96_undulation_m(centre.lat_rad, centre.lon_rad);
    let sample = surface.sample_surface(&centre).unwrap().unwrap();
    let expected_msl = 100.0 + rise_m * (f64::from(SIZE) / 2.0 - 0.5);
    assert_eq!(sample.source, SurfaceSampleSource::ResidentDem { zoom: 12 });
    assert!(
        (sample.orthometric_msl_m.0 - expected_msl).abs() < 0.05,
        "{sample:?}"
    );
    assert!((sample.ellipsoidal_height_m.0 - expected_msl - undulation).abs() < 0.05);

    // Rising east tilts the normal west.
    let slope = rise_m / pixel_east_m(tile);
    let expected_normal = DVec3::new(-slope, 0.0, 1.0).normalize();
    assert!(
        (sample.normal_enu - expected_normal).length() < 1e-3,
        "{sample:?}"
    );

    let above = Geodetic {
        height_m: sample.ellipsoidal_height_m.0 + 250.0,
        ..centre
    };
    let agl = surface.agl_clearance_m(&above).unwrap().unwrap();
    assert!((agl.0 - 250.0).abs() < 1e-6, "{agl:?}");

    let from = geodetic_to_ecef(&above);
    let down = LocalFrame::at(&centre).up() * -1_000.0;
    let hit = surface
        .raycast_segment_ecef(from, from + down, 64)
        .unwrap()
        .unwrap();
    assert!((hit.distance_m - 250.0).abs() < 0.5, "{hit:?}");
    assert_eq!(
        hit.surface.source,
        SurfaceSampleSource::ResidentDem { zoom: 12 }
    );
}

#[test]
fn the_finest_live_zoom_wins_over_coarser_tiles_and_the_catalog() {
    let point = dover();
    let mut surface = AuthoritativeWorldSurface::earth(CatalogVerticalDatum::EllipsoidWgs84)
        .with_live_dem_datum(CatalogVerticalDatum::EllipsoidWgs84);
    let flat = |height: f64| TerrariumTile::from_heights(4, 4, vec![height; 16]).unwrap();
    surface
        .insert_terrarium_tile(tile_over(&point, 11), flat(50.0))
        .unwrap();
    surface
        .insert_terrarium_tile(tile_over(&point, 14), flat(75.0))
        .unwrap();
    let sample = surface.sample_surface(&point).unwrap().unwrap();
    assert_eq!(sample.source, SurfaceSampleSource::ResidentDem { zoom: 14 });
    assert!((sample.ellipsoidal_height_m.0 - 75.0).abs() < 1e-9);

    // Outside the zoom-14 tile the zoom-11 tile still answers.
    let elsewhere = tile_point_to_geodetic(tile_over(&point, 11), 0.02, 0.02).unwrap();
    let coarse = surface.sample_surface(&elsewhere).unwrap().unwrap();
    assert_eq!(coarse.source, SurfaceSampleSource::ResidentDem { zoom: 11 });
    assert!((coarse.ellipsoidal_height_m.0 - 50.0).abs() < 1e-9);
}

#[test]
fn a_surveyed_runway_stays_authoritative_over_live_terrain() {
    let diagram = OsmAerodromeDiagram::builtin(AerodromeId::DoverKdov).unwrap();
    let end = *diagram.primary_runway_end().unwrap();
    let threshold =
        Geodetic::from_degrees(end.threshold.latitude_deg, end.threshold.longitude_deg, 0.0);
    let mut surface = AuthoritativeWorldSurface::earth(CatalogVerticalDatum::EllipsoidWgs84)
        .with_aerodrome(&diagram);
    surface
        .insert_terrarium_tile(
            tile_over(&threshold, 13),
            TerrariumTile::from_heights(4, 4, vec![60.0; 16]).unwrap(),
        )
        .unwrap();
    let on_runway = surface.sample_surface(&threshold).unwrap().unwrap();
    assert_eq!(on_runway.source, SurfaceSampleSource::RunwaySurface);
    assert!((on_runway.ellipsoidal_height_m.0 - end.threshold.ellipsoidal_height_m).abs() < 1e-6);
}

#[test]
fn the_overlay_is_bounded_and_refuses_addresses_off_the_pyramid() {
    let mut surface = AuthoritativeWorldSurface::earth(CatalogVerticalDatum::EllipsoidWgs84)
        .with_live_dem_capacity(2);
    let flat = || TerrariumTile::from_heights(2, 2, vec![1.0; 4]).unwrap();
    for x in 0..3 {
        surface
            .insert_terrarium_tile(
                TileId {
                    zoom: 10,
                    x,
                    y: 300,
                },
                flat(),
            )
            .unwrap();
    }
    assert_eq!(
        surface.live_dem_tiles(),
        vec![
            TileId {
                zoom: 10,
                x: 1,
                y: 300
            },
            TileId {
                zoom: 10,
                x: 2,
                y: 300
            }
        ]
    );
    let off = TileId {
        zoom: 2,
        x: 9,
        y: 0,
    };
    assert!(surface.insert_terrarium_tile(off, flat()).is_err());
    let bridge = LiveDemBridge::new();
    assert!(bridge.ingest_terrarium_tile(off, flat()).is_err());
    assert!(
        bridge
            .ingest_terrarium_png(
                TileId {
                    zoom: 3,
                    x: 0,
                    y: 0
                },
                b"not a png"
            )
            .is_err()
    );
    assert_eq!(bridge.generation(), 0);
}

#[test]
fn loaders_on_many_threads_all_reach_the_surface() {
    let bridge = LiveDemBridge::new();
    let handles: Vec<_> = (0..4u32)
        .map(|x| {
            let loader = bridge.clone();
            std::thread::spawn(move || {
                let tile = TerrariumTile::from_heights(2, 2, vec![f64::from(x); 4]).unwrap();
                loader.ingest_terrarium_tile(TileId { zoom: 9, x, y: 100 }, tile)
            })
        })
        .collect();
    for handle in handles {
        handle.join().unwrap().unwrap();
    }
    let mut surface = AuthoritativeWorldSurface::earth(CatalogVerticalDatum::EllipsoidWgs84);
    assert_eq!(bridge.drain_into(&mut surface), 4);
    assert_eq!(surface.live_dem_len(), 4);
    assert_eq!(bridge.generation(), 4);
}
