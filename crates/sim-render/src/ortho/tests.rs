use sim_scenery::{CatalogScenery, ImageryRaster, SyntheticCorridorSpec, tile_point_to_geodetic};

use super::*;

const ZOOM: u8 = 10;

fn tile(x: u32, y: u32) -> TileId {
    TileId { zoom: ZOOM, x, y }
}

fn footprint() -> OrthoFootprint {
    OrthoFootprint::covering([tile(300, 380), tile(301, 381)]).unwrap()
}

#[test]
fn the_footprint_spans_its_tiles() {
    let footprint = footprint();
    assert_eq!((footprint.west, footprint.north), (300, 380));
    assert_eq!(footprint.tiles_wide.get(), 2);
    assert_eq!(footprint.tiles_high.get(), 2);
    assert_eq!(OrthoFootprint::covering([]), None);
}

#[test]
fn coordinates_run_across_the_footprint() {
    let footprint = footprint();
    let corner = tile_point_to_geodetic(tile(300, 380), 0.0, 0.0).unwrap();
    let [u, v] = footprint.uv_at(&corner).unwrap();
    assert!(u.abs() < 1e-5 && v.abs() < 1e-5, "{u} {v}");
    let middle = tile_point_to_geodetic(tile(301, 381), 0.0, 0.0).unwrap();
    let [u, v] = footprint.uv_at(&middle).unwrap();
    assert!((u - 0.5).abs() < 1e-5 && (v - 0.5).abs() < 1e-5, "{u} {v}");
    let beyond = tile_point_to_geodetic(tile(303, 380), 0.5, 0.5).unwrap();
    assert!(footprint.uv_at(&beyond).unwrap()[0] > 1.0);
    let polar = Geodetic::from_degrees(88.0, 0.0, 0.0);
    assert_eq!(footprint.uv_at(&polar), None);
}

#[test]
fn composites_tiles_into_place_and_leaves_holes_transparent() {
    let mut catalog = CatalogScenery::new();
    let opaque = |value: u8| ImageryRaster::new(2, 2, [value, 0, 0, 255].repeat(4)).unwrap();
    catalog.insert_imagery(tile(300, 380), opaque(10)).unwrap();
    catalog.insert_imagery(tile(301, 381), opaque(20)).unwrap();
    let atlas = OrthoAtlas::from_source(&catalog, footprint(), 2).unwrap();
    assert_eq!((atlas.width, atlas.height), (4, 4));
    let pixel = |x: usize, y: usize| &atlas.rgba8[(y * 4 + x) * 4..][..4];
    assert_eq!(pixel(1, 1), [10, 0, 0, 255]);
    assert_eq!(pixel(2, 2), [20, 0, 0, 255]);
    assert_eq!(pixel(3, 0), [0, 0, 0, 0], "north-east tile is missing");
    assert_eq!(pixel(0, 3), [0, 0, 0, 0], "south-west tile is missing");
}

#[test]
fn rejects_tiles_of_the_wrong_size() {
    let mut catalog = CatalogScenery::new();
    let raster = ImageryRaster::new(1, 1, vec![0; 4]).unwrap();
    catalog.insert_imagery(tile(300, 380), raster).unwrap();
    let err = OrthoAtlas::from_source(&catalog, footprint(), 2).unwrap_err();
    assert!(matches!(err, RenderError::AtlasTileSize { .. }), "{err}");
}

#[test]
fn the_synthetic_corridor_composites_with_no_holes() {
    let spec = SyntheticCorridorSpec {
        tile_px: 16,
        ..SyntheticCorridorSpec::delaware()
    };
    let catalog = CatalogScenery::synthetic_corridor(&spec).unwrap();
    let footprint = OrthoFootprint::covering(catalog.imagery_tiles()).unwrap();
    let atlas = OrthoAtlas::from_source(&catalog, footprint, 16).unwrap();
    assert_eq!((atlas.width, atlas.height), (48, 32));
    assert!(
        atlas
            .rgba8
            .as_chunks::<4>()
            .0
            .iter()
            .all(|pixel| pixel[3] == 255)
    );
}
