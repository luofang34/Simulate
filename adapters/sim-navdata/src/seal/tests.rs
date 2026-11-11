use std::collections::BTreeSet;

use navigate_data::{
    GroundHeight, LayerKind, MemoryStore, Package, TileRead, descriptor_path, manifest_path,
};
use pollster::block_on;

use crate::fixtures::{CIRCUIT, FakeSource, camera};
use crate::{
    NavdataError, PackagePlan, SealedPackage, SourceSet, plan_package, prefetch, seal_package,
};

struct Flat;

impl GroundHeight for Flat {
    fn ground_m(&self, _point: [f64; 2]) -> Option<f64> {
        Some(300.0)
    }
}

fn plan() -> PackagePlan {
    plan_package(&CIRCUIT, camera(), &SourceSet::default(), &Flat).unwrap()
}

fn build(source: &FakeSource, store: &mut MemoryStore) -> Result<SealedPackage, NavdataError> {
    let sources = SourceSet::default();
    let plan = plan();
    let fetched = block_on(prefetch(
        &plan.keys(&sources),
        &sources,
        u64::MAX,
        source,
        store,
        &mut |_| {},
    ))?;
    block_on(seal_package(
        &plan,
        &sources,
        "circuit",
        &fetched.absent,
        store,
    ))
}

#[test]
fn a_sealed_package_is_whole_in_memory_and_in_the_store() {
    let source = FakeSource::new(300.0);
    let mut store = MemoryStore::default();
    let sealed = build(&source, &mut store).unwrap();
    let plan = plan();
    assert_eq!(
        (sealed.summary.tiles, sealed.summary.gaps),
        (plan.tile_count(), 0)
    );

    let package = block_on(Package::open(&sealed.memory, sealed.summary.identity)).unwrap();
    assert_eq!(package.identity(), sealed.summary.identity);
    let layer = package.manifest().layers[0].key;
    let address = *plan.tiles[&LayerKind::Imagery].first().unwrap();
    let tile = block_on(package.tile(&sealed.memory, layer, address)).unwrap();
    assert!(matches!(tile, TileRead::Present(bytes) if bytes.len() == 1_000));
    assert!(sealed.memory_bytes > sealed.summary.bytes);

    for path in [
        manifest_path(sealed.summary.identity),
        descriptor_path(sealed.summary.identity),
    ] {
        assert_eq!(store.get(&path), sealed.memory.get(&path));
        assert!(store.get(&path).is_some(), "{path}");
    }
    block_on(Package::open(&store, sealed.summary.identity)).unwrap();
}

#[test]
fn the_manifest_declares_its_layers_as_the_desktop_package_does() {
    let sealed = build(&FakeSource::new(300.0), &mut MemoryStore::default()).unwrap();
    let manifest = sealed
        .memory
        .get(&manifest_path(sealed.summary.identity))
        .unwrap();
    let manifest = std::str::from_utf8(manifest).unwrap();
    // The layer declarations of the desktop Zaragoza package, byte for byte.
    for declared in [
        r#"{"format":"navigate.data.package","version":1,"body":"earth","ellipsoid":"wgs84","grid":"web_mercator_xyz","layers":[{"key":{"kind":"imagery","source":"esri_world_imagery"},"revision":"unversioned","horizontal_datum":"wgs84","vertical_datum":null,"encoding":"jpeg","tile_pixels":256,"accuracy":{"horizontal_mm":null,"vertical_mm":null,"acquired":null,"age_growth_mm_per_year":null,"surface":"unknown"},"correlation_zoom":16,"index":{"#,
        r#"{"key":{"kind":"elevation","source":"mapzen_terrarium"},"revision":"unversioned","horizontal_datum":"wgs84","vertical_datum":{"kind":"orthometric","geoid":"egm96"},"encoding":"terrarium_png","tile_pixels":256,"accuracy":{"horizontal_mm":null,"vertical_mm":null,"acquired":null,"age_growth_mm_per_year":null,"surface":"unknown"},"correlation_zoom":16,"index":{"#,
    ] {
        assert!(manifest.contains(declared), "{manifest}");
    }
}

#[test]
fn rebuilding_from_the_store_fetches_nothing_and_gives_the_same_identity() {
    let source = FakeSource::new(300.0);
    let mut store = MemoryStore::default();
    let first = build(&source, &mut store).unwrap();
    let asked = source.asked();
    let again = build(&source, &mut store).unwrap();
    assert_eq!(source.asked(), asked);
    assert_eq!(again.summary, first.summary);
    assert_eq!(again.memory, first.memory);
}

#[test]
fn other_content_is_another_package() {
    let first = build(&FakeSource::new(300.0), &mut MemoryStore::default()).unwrap();
    let other = build(&FakeSource::new(301.0), &mut MemoryStore::default()).unwrap();
    assert_ne!(other.summary.identity, first.summary.identity);
}

#[test]
fn absent_tiles_are_gaps_of_the_package() {
    let mut source = FakeSource::new(300.0);
    source.statuses.insert("/terrarium/12/", 404);
    let sealed = build(&source, &mut MemoryStore::default()).unwrap();
    let plan = plan();
    let elevation = plan.tiles[&LayerKind::Elevation].len();
    assert_eq!(sealed.summary.gaps, elevation);
    assert_eq!(sealed.summary.tiles, plan.tile_count() - elevation);
    let package = block_on(Package::open(&sealed.memory, sealed.summary.identity)).unwrap();
    let layer = package.manifest().layers[1].key;
    let address = *plan.tiles[&LayerKind::Elevation].first().unwrap();
    assert_eq!(
        block_on(package.tile(&sealed.memory, layer, address)),
        Ok(TileRead::Gap)
    );
}

#[test]
fn tiles_with_the_same_bytes_are_counted() {
    // Every elevation tile of the fake source is the same flat PNG.
    let sealed = build(&FakeSource::new(300.0), &mut MemoryStore::default()).unwrap();
    assert_eq!(
        sealed.repeated_tiles,
        plan().tiles[&LayerKind::Elevation].len() - 1
    );
}

#[test]
fn sealing_without_the_tiles_is_refused() {
    let sources = SourceSet::default();
    let mut store = MemoryStore::default();
    let error = block_on(seal_package(
        &plan(),
        &sources,
        "circuit",
        &BTreeSet::new(),
        &mut store,
    ))
    .unwrap_err();
    assert!(error.to_string().contains("was not prefetched"), "{error}");
    assert!(store.paths().next().is_none());
}
