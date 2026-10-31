use super::*;

const TILE: TileId = TileId {
    zoom: 10,
    x: 300,
    y: 380,
};

fn flat(height_m: f64) -> TerrariumTile {
    TerrariumTile::from_heights(4, 4, vec![height_m; 16]).unwrap()
}

fn point_in(tile: TileId, u: f64, v: f64) -> Geodetic {
    crate::tile_point_to_geodetic(tile, u, v).unwrap()
}

#[test]
fn an_empty_catalog_knows_nothing() {
    let catalog = CatalogScenery::new();
    assert_eq!(catalog.ground_height_m(&point_in(TILE, 0.5, 0.5)), Ok(None));
    assert_eq!(catalog.imagery(TILE), Ok(None));
}

#[test]
fn missing_elevation_is_unknown_never_sea_level() {
    let mut catalog = CatalogScenery::new();
    catalog.insert_elevation(TILE, flat(120.0)).unwrap();
    assert_eq!(
        catalog.ground_height_m(&point_in(TILE, 0.5, 0.5)),
        Ok(Some(120.0))
    );
    let next_door = TileId {
        x: TILE.x + 5,
        ..TILE
    };
    assert_eq!(
        catalog.ground_height_m(&point_in(next_door, 0.5, 0.5)),
        Ok(None)
    );
}

#[test]
fn elevation_beyond_the_mercator_limit_is_unknown() {
    let mut catalog = CatalogScenery::new();
    catalog.insert_elevation(TILE, flat(1.0)).unwrap();
    let pole = Geodetic::from_degrees(89.0, 0.0, 0.0);
    assert_eq!(catalog.ground_height_m(&pole), Ok(None));
}

#[test]
fn imagery_is_served_only_where_resident() {
    let mut catalog = CatalogScenery::new();
    let raster = ImageryRaster::new(1, 1, vec![1, 2, 3, 4]).unwrap();
    catalog.insert_imagery(TILE, raster.clone()).unwrap();
    assert_eq!(catalog.imagery(TILE), Ok(Some(raster)));
    let absent = TileId {
        y: TILE.y + 1,
        ..TILE
    };
    assert_eq!(catalog.imagery(absent), Ok(None));
    assert_eq!(catalog.imagery_tiles().collect::<Vec<_>>(), [TILE]);
}

#[test]
fn malformed_rasters_and_addresses_are_rejected() {
    let mut catalog = CatalogScenery::new();
    let short = ImageryRaster {
        width: 2,
        height: 2,
        rgba8: vec![0; 15],
    };
    assert_eq!(
        catalog.insert_imagery(TILE, short),
        Err(SceneryError::InvalidRaster {
            width: 2,
            height: 2,
            expected: 16,
            actual: 15
        })
    );
    assert!(ImageryRaster::new(1, 1, vec![0; 3]).is_err());
    let beyond = TileId {
        zoom: 1,
        x: 2,
        y: 0,
    };
    assert_eq!(
        catalog.insert_elevation(beyond, flat(0.0)),
        Err(SceneryError::TileOutOfRange { tile: beyond })
    );
    assert_eq!(
        catalog.imagery(beyond),
        Err(SceneryError::TileOutOfRange { tile: beyond })
    );
}

#[test]
fn the_deepest_resident_zoom_wins() {
    let mut catalog = CatalogScenery::new();
    let coarse = TileId {
        zoom: 9,
        x: TILE.x / 2,
        y: TILE.y / 2,
    };
    catalog.insert_elevation(coarse, flat(10.0)).unwrap();
    catalog.insert_elevation(TILE, flat(20.0)).unwrap();
    let inside_fine = point_in(TILE, 0.5, 0.5);
    assert_eq!(catalog.ground_height_m(&inside_fine), Ok(Some(20.0)));
    let sibling = TileId {
        x: TILE.x + 1,
        ..TILE
    };
    let coarse_only = point_in(sibling, 0.5, 0.5);
    assert_eq!(catalog.ground_height_m(&coarse_only), Ok(Some(10.0)));
}

#[test]
fn adjacent_tiles_meet_without_a_step() {
    let east = TileId {
        x: TILE.x + 1,
        ..TILE
    };
    let mut catalog = CatalogScenery::new();
    catalog.insert_elevation(TILE, flat(100.0)).unwrap();
    catalog.insert_elevation(east, flat(200.0)).unwrap();
    let from_west = catalog
        .ground_height_m(&point_in(TILE, 1.0 - 1e-9, 0.5))
        .unwrap()
        .unwrap();
    let from_east = catalog
        .ground_height_m(&point_in(east, 1e-9, 0.5))
        .unwrap()
        .unwrap();
    assert!((from_west - 150.0).abs() < 1e-3, "{from_west}");
    assert!((from_east - 150.0).abs() < 1e-3, "{from_east}");
}

#[test]
fn an_uncovered_edge_holds_rather_than_blending_toward_zero() {
    let mut catalog = CatalogScenery::new();
    catalog.insert_elevation(TILE, flat(100.0)).unwrap();
    let edge = catalog.ground_height_m(&point_in(TILE, 1.0 - 1e-9, 0.5));
    assert_eq!(edge.unwrap().map(f64::round), Some(100.0));
}
