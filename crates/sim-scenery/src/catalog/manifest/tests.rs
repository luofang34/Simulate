use std::collections::BTreeMap;
use std::io::Cursor;

use image::{ImageFormat, Rgb, RgbImage};

use super::*;
use crate::{ElevationSource, ImagerySource, tile_point_to_geodetic};

const DEM: [u32; 3] = [14, 8192, 8192];

fn png(rgb: [u8; 3]) -> Vec<u8> {
    let mut bytes = Cursor::new(Vec::new());
    RgbImage::from_pixel(4, 4, Rgb(rgb))
        .write_to(&mut bytes, ImageFormat::Png)
        .unwrap();
    bytes.into_inner()
}

fn entry(kind: &str, xyz: [u32; 3], path: &str, bytes: &[u8]) -> serde_json::Value {
    serde_json::json!({
        "kind": kind,
        "xyz": xyz,
        "path": path,
        "sha256": hex_digest(bytes),
        "bytes": bytes.len(),
    })
}

fn manifest(tiles: Vec<serde_json::Value>) -> serde_json::Value {
    serde_json::json!({
        "format": "photo-reference-catalog",
        "version": 1,
        "complete": true,
        "configuration": {"bbox": [-0.01, -0.01, 0.01, 0.01], "dem_zoom": 14, "imagery_zoom": 14},
        "tiles": tiles,
    })
}

fn load(
    manifest: &serde_json::Value,
    files: &BTreeMap<&str, Vec<u8>>,
) -> Result<CatalogScenery, SceneryError> {
    CatalogScenery::from_manifest(&manifest.to_string(), &mut |path| {
        files.get(path).cloned().ok_or_else(|| SceneryError::Read {
            path: path.to_owned(),
            detail: "absent".to_owned(),
        })
    })
}

fn fixture() -> (serde_json::Value, BTreeMap<&'static str, Vec<u8>>) {
    let dem = png([128, 1, 128]);
    let imagery = png([20, 40, 60]);
    let tiles = vec![
        entry("dem", DEM, "dem/a.png", &dem),
        entry("imagery", DEM, "imagery/a.png", &imagery),
    ];
    let files = BTreeMap::from([("dem/a.png", dem), ("imagery/a.png", imagery)]);
    (manifest(tiles), files)
}

#[test]
fn loads_verified_tiles_and_serves_them() {
    let (manifest, files) = fixture();
    let catalog = load(&manifest, &files).unwrap();
    let tile = TileId {
        zoom: 14,
        x: 8192,
        y: 8192,
    };
    let centre = tile_point_to_geodetic(tile, 0.5, 0.5).unwrap();
    assert_eq!(catalog.ground_height_m(&centre), Ok(Some(1.5)));
    let raster = catalog.imagery(tile).unwrap().unwrap();
    assert_eq!((raster.width, raster.height), (4, 4));
    assert_eq!(raster.rgba8.get(..4), Some(&[20, 40, 60, 255][..]));
}

#[test]
fn a_changed_tile_is_rejected() {
    let (manifest, mut files) = fixture();
    let swapped = png([129, 0, 0]);
    let original = files.insert("dem/a.png", swapped.clone()).unwrap();
    assert_eq!(
        swapped.len(),
        original.len(),
        "same size, different content"
    );
    let err = load(&manifest, &files).unwrap_err();
    assert!(
        matches!(&err, SceneryError::CorruptTile { path, detail }
            if path == "dem/a.png" && detail.contains("SHA-256")),
        "{err}"
    );
}

#[test]
fn a_truncated_tile_is_rejected() {
    let (manifest, mut files) = fixture();
    files.get_mut("dem/a.png").unwrap().pop();
    let err = load(&manifest, &files).unwrap_err();
    assert!(matches!(err, SceneryError::CorruptTile { .. }), "{err}");
}

#[test]
fn undecodable_bytes_with_a_valid_digest_are_rejected() {
    let garbage = vec![1, 2, 3, 4];
    let manifest = manifest(vec![entry("dem", DEM, "bad.png", &garbage)]);
    let files = BTreeMap::from([("bad.png", garbage)]);
    let err = load(&manifest, &files).unwrap_err();
    assert!(matches!(err, SceneryError::CorruptTile { .. }), "{err}");
}

#[test]
fn rejects_catalogs_it_cannot_trust() {
    let (good, files) = fixture();
    let tweak = |key: &str, value: serde_json::Value| {
        let mut changed = good.clone();
        changed[key] = value;
        changed
    };
    for bad in [
        tweak("format", "other".into()),
        tweak("version", 2.into()),
        tweak("complete", false.into()),
        tweak("tiles", serde_json::json!([])),
    ] {
        let err = load(&bad, &files).unwrap_err();
        assert!(matches!(err, SceneryError::InvalidCatalog { .. }), "{err}");
    }
    assert!(CatalogScenery::from_manifest("not json", &mut |_| Ok(Vec::new())).is_err());
}

#[test]
fn rejects_tiles_outside_the_pyramid_duplicates_and_unknown_kinds() {
    let bytes = png([128, 0, 0]);
    let files = BTreeMap::from([("a.png", bytes.clone())]);
    let same = entry("dem", DEM, "a.png", &bytes);
    for tiles in [
        vec![entry("dem", [2, 4, 0], "a.png", &bytes)],
        vec![entry("dem", [23, 0, 0], "a.png", &bytes)],
        vec![same.clone(), same],
        vec![entry("lidar", DEM, "a.png", &bytes)],
    ] {
        let err = load(&manifest(tiles), &files).unwrap_err();
        assert!(matches!(err, SceneryError::InvalidCatalog { .. }), "{err}");
    }
}

#[test]
fn tile_paths_cannot_leave_the_catalog_root() {
    let bytes = png([128, 0, 0]);
    for path in ["../secret.png", "/etc/secret.png", "dem/../../x.png"] {
        let manifest = manifest(vec![entry("dem", DEM, path, &bytes)]);
        let mut read_called = false;
        let err = CatalogScenery::from_manifest(&manifest.to_string(), &mut |_| {
            read_called = true;
            Ok(Vec::new())
        })
        .unwrap_err();
        assert!(matches!(err, SceneryError::InvalidCatalog { .. }), "{err}");
        assert!(!read_called, "{path} was read");
    }
}

#[test]
fn loads_a_catalog_directory_from_the_filesystem() {
    let root = tempfile::tempdir().unwrap();
    let (manifest, files) = fixture();
    for (path, bytes) in &files {
        let target = root.path().join(path);
        std::fs::create_dir_all(target.parent().unwrap()).unwrap();
        std::fs::write(target, bytes).unwrap();
    }
    std::fs::write(root.path().join("catalog.json"), manifest.to_string()).unwrap();
    let catalog = CatalogScenery::from_catalog_dir_blocking(root.path()).unwrap();
    assert_eq!(catalog.imagery_tiles().count(), 1);
}

#[test]
fn a_missing_catalog_directory_names_the_path() {
    let root = tempfile::tempdir().unwrap();
    let err = CatalogScenery::from_catalog_dir_blocking(root.path()).unwrap_err();
    assert!(
        matches!(&err, SceneryError::Read { path, .. } if path.ends_with("catalog.json")),
        "{err}"
    );
}

#[test]
fn a_manifest_without_a_datum_is_mean_sea_level_and_one_can_name_the_ellipsoid() {
    let (mut manifest, files) = fixture();
    let catalog = load(&manifest, &files).unwrap();
    assert_eq!(
        catalog.vertical_datum(),
        CatalogVerticalDatum::OrthometricMslEgm96
    );
    let centre = tile_point_to_geodetic(
        TileId {
            zoom: 14,
            x: 8192,
            y: 8192,
        },
        0.5,
        0.5,
    )
    .unwrap();
    let undulation = sim_geo::egm96_undulation_m(centre.lat_rad, centre.lon_rad);
    let ellipsoidal = catalog
        .ellipsoidal_ground_height_m(&centre)
        .unwrap()
        .unwrap();
    assert!((ellipsoidal.0 - (1.5 + undulation)).abs() < 1e-12);

    manifest["vertical_datum"] = serde_json::json!("ellipsoid_wgs84");
    let catalog = load(&manifest, &files).unwrap();
    assert_eq!(
        catalog.vertical_datum(),
        CatalogVerticalDatum::EllipsoidWgs84
    );
    assert_eq!(catalog.ground_height_m(&centre), Ok(Some(1.5)));
    let msl = catalog
        .orthometric_ground_height_m(&centre)
        .unwrap()
        .unwrap();
    assert!((msl.0 - (1.5 - undulation)).abs() < 1e-12);

    manifest["vertical_datum"] = serde_json::json!("navd88");
    assert!(matches!(
        load(&manifest, &files),
        Err(SceneryError::InvalidCatalog { .. })
    ));
}
