use std::path::Path;

use tempfile::TempDir;

use super::*;

#[test]
fn canonical_paths_normalize_terrarium_esri_and_s2cloudless() {
    let root = Path::new("/var/cache/tiles");
    assert_eq!(
        tile_cache_path(
            root,
            "https://s3.amazonaws.com/elevation-tiles-prod/terrarium/12/2036/1523.png"
        ),
        root.join("terrarium/12/2036/1523.png")
    );
    // Esri URLs place {y} before {x}; cache path stores canonical {z}/{x}/{y}.jpg.
    assert_eq!(
        tile_cache_path(
            root,
            "https://services.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/13/3046/4072"
        ),
        root.join("esri_world_imagery/13/4072/3046.jpg")
    );
    assert_eq!(
        tile_cache_path(
            root,
            "https://tiles.maps.eox.at/wmts/1.0.0/s2cloudless-2020_3857/default/g/13/3046/4072.jpg"
        ),
        root.join("s2cloudless/13/4072/3046.jpg")
    );
}

#[cfg(not(target_arch = "wasm32"))]
#[test]
fn disk_cache_populates_atomically_and_serves_offline_deterministically() {
    let dir = TempDir::new().unwrap();
    let terrarium_url = "https://s3.amazonaws.com/elevation-tiles-prod/terrarium/11/1018/761.png";
    let esri_url = "https://services.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/12/1523/2036";

    // Cold offline run must refuse uncached tiles without touching network.
    let cold_offline =
        DiskCachedTransport::new(Some(dir.path().to_path_buf()), CachePolicy::Offline);
    assert!(pollster::block_on(cold_offline.get(terrarium_url)).is_err());
    assert_eq!(cold_offline.stats().network_fetches, 0);

    // Populate cache deterministically via SyntheticFallback.
    let seeder = DiskCachedTransport::new(
        Some(dir.path().to_path_buf()),
        CachePolicy::SyntheticFallback,
    );
    let dem_first = pollster::block_on(seeder.get(terrarium_url)).unwrap();
    let img_first = pollster::block_on(seeder.get(esri_url)).unwrap();
    assert_eq!(dem_first.status, 200);
    assert_eq!(img_first.status, 200);
    let seed_stats = seeder.stats();
    assert_eq!(
        (
            seed_stats.hits,
            seed_stats.misses,
            seed_stats.network_fetches
        ),
        (0, 2, 0)
    );
    assert!(dir.path().join("terrarium/11/1018/761.png").exists());
    assert!(
        dir.path()
            .join("esri_world_imagery/12/2036/1523.jpg")
            .exists()
    );

    // Subsequent --offline transport hits disk 100% with byte-identical payloads.
    let warm_offline =
        DiskCachedTransport::new(Some(dir.path().to_path_buf()), CachePolicy::Offline);
    let dem_second = pollster::block_on(warm_offline.get(terrarium_url)).unwrap();
    let img_second = pollster::block_on(warm_offline.get(esri_url)).unwrap();
    assert_eq!(dem_second.body, dem_first.body);
    assert_eq!(img_second.body, img_first.body);
    let warm_stats = warm_offline.stats();
    assert_eq!(
        (
            warm_stats.hits,
            warm_stats.misses,
            warm_stats.network_fetches
        ),
        (2, 0, 0)
    );
    assert!(warm_stats.bytes_read > 0);
}

#[test]
fn curl_stdout_parser_extracts_status_and_binary_body() {
    let mut raw = vec![0x89, b'P', b'N', b'G', b'\n', 0x42];
    raw.extend_from_slice(b"\n200");
    let ans = split_curl_answer(raw).unwrap();
    assert_eq!(ans.status, 200);
    assert_eq!(ans.body, vec![0x89, b'P', b'N', b'G', b'\n', 0x42]);
    assert_eq!(split_curl_answer(b"\n404".to_vec()).unwrap().status, 404);
    assert!(split_curl_answer(b"missing-newline".to_vec()).is_err());
}
