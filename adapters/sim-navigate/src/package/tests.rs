use std::io::Cursor;

use super::*;

fn png(image: image::DynamicImage) -> Vec<u8> {
    let mut bytes = Cursor::new(Vec::new());
    image.write_to(&mut bytes, image::ImageFormat::Png).unwrap();
    bytes.into_inner()
}

/// Imagery with a gradient keyed by the tile, and flat 100 m terrain.
fn source(route: &[[f64; 2]], radius_m: f64) -> MemoryTileSource {
    let mut source = MemoryTileSource::default();
    for address in corridor_tiles(route, radius_m, 15) {
        let image = image::RgbImage::from_fn(256, 256, |x, y| {
            image::Rgb([(x + address.x) as u8, (y + address.y) as u8, 7])
        });
        source.tiles.insert(
            (TileLayer::Imagery, address),
            png(image::DynamicImage::ImageRgb8(image)),
        );
    }
    // Terrarium 100 m: 32868 = 128 * 256 + 100.
    for address in corridor_tiles(route, radius_m, 12) {
        let image = image::RgbImage::from_pixel(256, 256, image::Rgb([128, 100, 0]));
        source.tiles.insert(
            (TileLayer::Dem, address),
            png(image::DynamicImage::ImageRgb8(image)),
        );
    }
    source
}

fn spec(route: Vec<[f64; 2]>) -> PackageSpec {
    PackageSpec {
        release_id: "test".into(),
        role: PackageRole::Onboard,
        route,
        corridor_radius_m: 1_500.0,
        imagery_zoom: 15,
        terrain_zoom: 12,
        max_tiles: 1_000,
    }
}

#[test]
fn a_written_package_reads_back_with_its_identity_and_coverage() {
    let route = vec![[40.9, -75.1], [40.9, -75.05]];
    let mut tiles = source(&route, 1_500.0);
    let dir = tempfile::tempdir().unwrap();
    let summary = prepare_package_blocking(&spec(route.clone()), &mut tiles, dir.path()).unwrap();
    assert_eq!(summary.missing_tiles, 0);
    assert!(summary.imagery_tiles > 10);
    let package = MapPackage::open_blocking(dir.path(), Some(&summary.manifest_sha256)).unwrap();
    assert_eq!(
        package.map_revision().manifest_sha256,
        summary.manifest_sha256
    );
    assert_eq!(package.info().role, PackageRole::Onboard);
    let identity = read_identity_blocking(dir.path()).unwrap();
    assert_eq!(identity.manifest_sha256, summary.manifest_sha256);
    // On the route, the window is covered; 20 km off it, nothing is.
    let on = package.scene_around(40.9, -75.075, 500.0).unwrap();
    assert!((on.imagery_coverage - 1.0).abs() < 1e-12);
    let off = package.scene_around(41.1, -75.075, 500.0).unwrap();
    assert_eq!(off.imagery_coverage, 0.0);
    // Heights come back ellipsoidal: 100 m above the geoid, which lies about
    // 33 m below the ellipsoid here.
    let heights = &on.scene.terrain.heights_m;
    let height = f64::from(heights[heights.len() / 2]);
    assert!((height - 67.0).abs() < 3.0, "{height}");
}

#[test]
fn a_tampered_tile_or_an_unexpected_manifest_is_refused() {
    let route = vec![[40.9, -75.1], [40.9, -75.09]];
    let mut tiles = source(&route, 500.0);
    let dir = tempfile::tempdir().unwrap();
    let summary = prepare_package_blocking(
        &PackageSpec {
            corridor_radius_m: 500.0,
            ..spec(route)
        },
        &mut tiles,
        dir.path(),
    )
    .unwrap();
    let wrong = "0".repeat(64);
    assert!(matches!(
        MapPackage::open_blocking(dir.path(), Some(&wrong)),
        Err(PackageError::Integrity { .. })
    ));
    let tile = std::fs::read_dir(dir.path().join("imagery/15"))
        .unwrap()
        .next()
        .unwrap()
        .unwrap()
        .path();
    let first = std::fs::read_dir(&tile)
        .unwrap()
        .next()
        .unwrap()
        .unwrap()
        .path();
    std::fs::write(&first, b"not a tile").unwrap();
    assert!(matches!(
        MapPackage::open_blocking(dir.path(), Some(&summary.manifest_sha256)),
        Err(PackageError::Integrity { .. })
    ));
}

#[test]
fn a_corridor_over_the_tile_limit_is_refused_before_fetching() {
    let route = vec![[40.9, -75.1], [40.9, -74.0]];
    let mut tiles = MemoryTileSource::default();
    let dir = tempfile::tempdir().unwrap();
    let limited = PackageSpec {
        max_tiles: 100,
        ..spec(route)
    };
    let refused = prepare_package_blocking(&limited, &mut tiles, dir.path());
    assert!(matches!(refused, Err(PackageError::Invalid { .. })));
    assert!(!dir.path().join(MANIFEST_NAME).exists());
}
