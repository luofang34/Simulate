use super::*;
use crate::{ElevationSource, ImagerySource};

fn small() -> SyntheticCorridorSpec {
    SyntheticCorridorSpec {
        tile_px: 64,
        ..SyntheticCorridorSpec::delaware()
    }
}

fn origin(spec: &SyntheticCorridorSpec) -> TileId {
    project_to_tile(&spec.south_west, spec.zoom).unwrap().tile
}

#[test]
fn is_identical_on_every_build() {
    let spec = small();
    assert_eq!(
        CatalogScenery::synthetic_corridor(&spec).unwrap(),
        CatalogScenery::synthetic_corridor(&spec).unwrap()
    );
}

#[test]
fn every_tile_in_the_block_is_resident_and_opaque() {
    let spec = small();
    let catalog = CatalogScenery::synthetic_corridor(&spec).unwrap();
    assert_eq!(catalog.imagery_tiles().count(), 6);
    for tile in catalog.imagery_tiles() {
        let raster = catalog.imagery(tile).unwrap().unwrap();
        assert!(
            raster
                .rgba8
                .as_chunks::<4>()
                .0
                .iter()
                .all(|pixel| pixel[3] == 255)
        );
    }
}

#[test]
fn the_seed_changes_the_scenery() {
    let spec = small();
    let other = SyntheticCorridorSpec { seed: 7, ..spec };
    assert_ne!(
        CatalogScenery::synthetic_corridor(&spec).unwrap(),
        CatalogScenery::synthetic_corridor(&other).unwrap()
    );
}

#[test]
fn has_relief_without_leaving_plausible_heights() {
    let spec = small();
    let catalog = CatalogScenery::synthetic_corridor(&spec).unwrap();
    let tile = origin(&spec);
    let heights: Vec<f64> = (0..64)
        .filter_map(|i| {
            let position = tile_point_to_geodetic(tile, f64::from(i) / 64.0, 0.5)?;
            catalog.ground_height_m(&position).unwrap()
        })
        .collect();
    assert_eq!(heights.len(), 64);
    let low = heights.iter().copied().fold(f64::INFINITY, f64::min);
    let high = heights.iter().copied().fold(f64::NEG_INFINITY, f64::max);
    assert!(high - low > 15.0, "relief {low}..{high}");
    assert!(low > 0.0 && high < 200.0, "relief {low}..{high}");
}

#[test]
fn the_ground_outside_the_block_stays_unknown() {
    let spec = small();
    let catalog = CatalogScenery::synthetic_corridor(&spec).unwrap();
    let tile = origin(&spec);
    let beyond = tile_point_to_geodetic(
        TileId {
            x: tile.x + spec.tiles_east + 2,
            ..tile
        },
        0.5,
        0.5,
    )
    .unwrap();
    assert_eq!(catalog.ground_height_m(&beyond), Ok(None));
}

#[test]
fn imagery_has_contrast_at_small_scales() {
    let raster_spec = small();
    let catalog = CatalogScenery::synthetic_corridor(&raster_spec).unwrap();
    let raster = catalog.imagery(origin(&raster_spec)).unwrap().unwrap();
    let luma: Vec<i32> = raster
        .rgba8
        .as_chunks::<4>()
        .0
        .iter()
        .map(|pixel| i32::from(pixel[0]) + i32::from(pixel[1]) + i32::from(pixel[2]))
        .collect();
    let edges = luma.windows(2).filter(|w| (w[0] - w[1]).abs() > 60).count();
    assert!(edges > luma.len() / 50, "{edges} edges");
}

#[test]
fn rejects_a_corridor_with_no_pixels_or_off_the_map() {
    let empty = SyntheticCorridorSpec {
        tile_px: 0,
        ..small()
    };
    assert!(CatalogScenery::synthetic_corridor(&empty).is_err());
    let off_map = SyntheticCorridorSpec {
        south_west: Geodetic::from_degrees(89.0, 0.0, 0.0),
        ..small()
    };
    assert!(CatalogScenery::synthetic_corridor(&off_map).is_err());
    let over_top = SyntheticCorridorSpec {
        south_west: Geodetic::from_degrees(85.05, 0.0, 0.0),
        tiles_north: 5,
        ..small()
    };
    assert!(CatalogScenery::synthetic_corridor(&over_top).is_err());
}
