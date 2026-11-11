use navigate_data::{GroundHeight, MemoryStore, TileAddress};
use pollster::block_on;

use crate::fixtures::{CIRCUIT, FakeSource, terrarium_png};
use crate::{
    GROUND_CORRIDOR_M, GROUND_ZOOM, NavdataError, SourceSet, TerrariumGround, ground_tiles,
    read_ground,
};

fn track() -> Vec<[f64; 2]> {
    CIRCUIT.iter().map(|point| [point[0], point[1]]).collect()
}

#[test]
fn the_planning_tiles_are_navigates_zoom_and_corridor() {
    assert_eq!((GROUND_ZOOM, GROUND_CORRIDOR_M), (10, 2_000.0));
    let tiles = ground_tiles(&track()).unwrap();
    assert!(tiles.iter().all(|tile| tile.zoom == 10));
    // A zoom-10 tile is about 29 km wide here; the circuit spans 21 km by
    // 11 km and its corridor crosses a column edge.
    assert!((1..=6).contains(&tiles.len()), "{tiles:?}");
    assert!(tiles.contains(&TileAddress::containing([41.7, -1.05], 10).unwrap()));
}

#[test]
fn a_decoded_tile_gives_its_heights_and_nothing_outside_it() {
    let mut ground = TerrariumGround::default();
    assert!(ground.is_empty());
    let address = TileAddress::containing([41.7, -1.05], GROUND_ZOOM).unwrap();
    ground.insert(address, &terrarium_png(307.5)).unwrap();
    assert_eq!(ground.len(), 1);
    assert_eq!(ground.ground_m([41.7, -1.05]), Some(307.5));
    assert_eq!(ground.ground_m([41.75, -0.95]), Some(307.5));
    assert_eq!(ground.ground_m([10.0, 10.0]), None);
    ground.insert(address, &terrarium_png(-12.25)).unwrap();
    assert_eq!(ground.ground_m([41.7, -1.05]), Some(-12.25));
}

#[test]
fn bytes_that_are_not_a_png_are_refused() {
    let address = TileAddress::new(10, 509, 380).unwrap();
    let error = TerrariumGround::default()
        .insert(address, b"<?xml not a tile")
        .unwrap_err();
    assert!(matches!(error, NavdataError::Decode { tile, .. } if tile == "elevation 10/509/380"));
}

#[test]
fn the_ground_is_read_through_the_cache() {
    let source = FakeSource::new(250.0);
    let sources = SourceSet::default();
    let mut store = MemoryStore::default();
    let ground = block_on(read_ground(&track(), &sources, &source, &mut store)).unwrap();
    let wanted = ground_tiles(&track()).unwrap().len();
    assert_eq!((ground.len(), source.asked()), (wanted, wanted));
    assert!(
        source
            .asked
            .borrow()
            .iter()
            .all(|url| url.contains("/terrarium/10/"))
    );
    assert_eq!(ground.ground_m([41.8, -0.8]), Some(250.0));

    let again = block_on(read_ground(&track(), &sources, &source, &mut store)).unwrap();
    assert_eq!(again, ground);
    assert_eq!(source.asked(), wanted);
}

#[test]
fn a_planning_tile_the_source_lacks_leaves_its_ground_unknown() {
    let mut source = FakeSource::new(250.0);
    source.statuses.insert("/terrarium/10/", 404);
    let mut store = MemoryStore::default();
    let ground = block_on(read_ground(
        &track(),
        &SourceSet::default(),
        &source,
        &mut store,
    ))
    .unwrap();
    assert!(ground.is_empty());
    assert_eq!(ground.ground_m([41.7, -1.05]), None);
}
