use std::sync::{
    Mutex,
    atomic::{AtomicUsize, Ordering},
};

use maplibre::io::source_client::SourceFetchError;

use sim_scenery::{LiveDemBridge, TileId, encode_terrarium};

use super::negative_cache::{NegativeTileCache, fetch_negative_cached};
use super::{HybridTileLoader, terrarium_tile_id};

const URL: &str = "https://services.arcgisonline.com/tile/19/1/1";

#[test]
fn repeated_missing_tile_is_answered_without_refetching() {
    let cache = Mutex::new(NegativeTileCache::new(8));
    let calls = AtomicUsize::new(0);
    let fetch = || async {
        calls.fetch_add(1, Ordering::SeqCst);
        Err::<Vec<u8>, _>(SourceFetchError::not_found(URL))
    };
    let first = pollster::block_on(fetch_negative_cached(&cache, URL, fetch));
    let second = pollster::block_on(fetch_negative_cached(&cache, URL, fetch));
    assert!(first.is_err_and(|e| e.is_not_found()));
    assert!(second.is_err_and(|e| e.is_not_found()));
    assert_eq!(calls.load(Ordering::SeqCst), 1);
}

#[test]
fn retryable_failures_are_not_remembered() {
    let cache = Mutex::new(NegativeTileCache::new(8));
    let calls = AtomicUsize::new(0);
    let fetch = || async {
        calls.fetch_add(1, Ordering::SeqCst);
        Err::<Vec<u8>, _>(SourceFetchError::temporary(std::io::Error::other("reset")))
    };
    let _first = pollster::block_on(fetch_negative_cached(&cache, URL, fetch));
    let _second = pollster::block_on(fetch_negative_cached(&cache, URL, fetch));
    assert_eq!(calls.load(Ordering::SeqCst), 2);
    assert!(!cache.lock().unwrap().contains(URL));
}

#[test]
fn successful_fetch_passes_through() {
    let cache = Mutex::new(NegativeTileCache::new(8));
    let body = pollster::block_on(fetch_negative_cached(&cache, URL, || async {
        Ok(vec![1, 2, 3])
    }));
    assert_eq!(body.unwrap(), vec![1, 2, 3]);
    assert_eq!(cache.lock().unwrap().len(), 0);
}

#[test]
fn capacity_evicts_oldest_first() {
    let mut cache = NegativeTileCache::new(2);
    cache.insert("a");
    cache.insert("b");
    cache.insert("a");
    cache.insert("c");
    assert!(!cache.contains("a"));
    assert!(cache.contains("b"));
    assert!(cache.contains("c"));
    assert_eq!(cache.len(), 2);
}

#[test]
fn poisoned_lock_still_serves_the_cache() {
    let cache = Mutex::new(NegativeTileCache::new(8));
    cache.lock().unwrap().insert(URL);
    let _poison = std::panic::catch_unwind(|| {
        let _guard = cache.lock().unwrap();
        panic!("poison");
    });
    assert!(cache.is_poisoned());
    let result = pollster::block_on(fetch_negative_cached(&cache, URL, || async {
        Ok(Vec::new())
    }));
    assert!(result.is_err_and(|e| e.is_not_found()));
}

const TERRARIUM_URL: &str =
    "https://s3.amazonaws.com/elevation-tiles-prod/terrarium/12/1206/1539.png";

fn terrarium_png(height_m: f64) -> Vec<u8> {
    terrarium_png_sized(height_m, 16)
}

fn terrarium_png_sized(height_m: f64, side: u32) -> Vec<u8> {
    let rgb: Vec<u8> = (0..side * side)
        .flat_map(|_| encode_terrarium(height_m))
        .collect();
    let image = image::RgbImage::from_raw(side, side, rgb).unwrap();
    let mut bytes = std::io::Cursor::new(Vec::new());
    image.write_to(&mut bytes, image::ImageFormat::Png).unwrap();
    bytes.into_inner()
}

#[test]
fn terrarium_urls_name_their_tile() {
    let tile = TileId {
        zoom: 12,
        x: 1206,
        y: 1539,
    };
    assert_eq!(terrarium_tile_id(TERRARIUM_URL), Some(tile));
    assert_eq!(
        terrarium_tile_id(&format!("{TERRARIUM_URL}?v=2")),
        Some(tile)
    );
    assert_eq!(
        terrarium_tile_id("https://tiles.example/imagery/12/1/1.png"),
        None
    );
    assert_eq!(
        terrarium_tile_id("https://tiles.example/terrarium/12/x/1.png"),
        None
    );
    assert_eq!(
        terrarium_tile_id("https://tiles.example/terrarium/12/1/1.webp"),
        None
    );
    assert_eq!(
        terrarium_tile_id("https://tiles.example/terrarium/1.png"),
        None
    );
}

#[test]
fn fetched_terrarium_tiles_reach_the_live_dem_bridge() {
    let bridge = LiveDemBridge::new();
    let loader = HybridTileLoader::new(None, None).with_live_dem_bridge(bridge.clone());
    loader.forward_terrarium("https://tiles.example/imagery/12/1206/1539.png", &[1, 2, 3]);
    assert_eq!(bridge.generation(), 0);
    loader.forward_terrarium(TERRARIUM_URL, b"not a png");
    assert_eq!(bridge.generation(), 0);
    loader.forward_terrarium(TERRARIUM_URL, &terrarium_png(120.0));
    assert_eq!(bridge.generation(), 1);
    assert_eq!(bridge.pending_len_may_be_stale(), 1);
}

#[test]
fn chart_urls_are_served_from_the_shared_chart_store() {
    use maplibre::io::source_client::HttpClient;
    let charts = crate::ChartAssetStore::new();
    let loader = HybridTileLoader::new(None, None).with_chart_store(charts.clone());
    charts.put_chart_png("plate", std::sync::Arc::from(vec![9_u8, 8, 7]));
    loader.put_chart_png("other", std::sync::Arc::from(vec![1_u8]));
    let url = crate::chart_url("plate");
    assert_eq!(pollster::block_on(loader.fetch(&url)).unwrap(), [9, 8, 7]);
    assert_eq!(charts.chart_png("other").as_deref(), Some(&[1][..]));
    let missing = pollster::block_on(loader.fetch("https://sim.chart/absent.png"));
    assert!(missing.is_err_and(|e| e.is_not_found()));
}

/// What the maps draw and what the simulation stands on come out of the same stamped
/// bytes: a published Terrarium tile under Colombo's runway leaves the loader carrying the
/// charted runway, while a tile with no runway passes through byte for byte.
#[test]
fn fetched_terrarium_tiles_carry_the_runway_datum_to_maps_and_ground() {
    use sim_geo::Geodetic;
    use sim_scenery::{
        AirportDatabase, AuthoritativeWorldSurface, RunwayDatum, TerrariumTile, WorldSurfaceQuery,
        project_to_tile,
    };

    let (_, vcbi) = AirportDatabase::builtin().by_code("VCBI").unwrap();
    let [a, b] = &vcbi.runways[0].ends;
    let runway = Geodetic::from_degrees(
        0.5 * (a.lat_deg + b.lat_deg),
        0.5 * (a.lon_deg + b.lon_deg),
        0.0,
    );
    let charted = 0.5 * f64::from(a.elevation_msl_m + b.elevation_msl_m);
    let coord = project_to_tile(&runway, 15).unwrap();
    let url = format!(
        "https://s3.amazonaws.com/elevation-tiles-prod/terrarium/15/{}/{}.png",
        coord.tile.x, coord.tile.y
    );

    let bridge = LiveDemBridge::new();
    let loader = HybridTileLoader::new(None, None)
        .with_runway_datum(RunwayDatum::builtin())
        .with_live_dem_bridge(bridge.clone());
    let published = terrarium_png_sized(charted + 2.7, 256);
    let served = loader.stamp_terrarium(&url, published.clone());
    let drawn = TerrariumTile::from_png_bytes(&served).unwrap();
    let height = drawn.sample_bilinear(coord.u, coord.v).unwrap();
    assert!((height - charted).abs() < 0.05, "{height} vs {charted}");

    loader.forward_terrarium(&url, &served);
    let mut ground = AuthoritativeWorldSurface::earth_live_dem();
    assert_eq!(bridge.drain_into(&mut ground), 1);
    let sample = ground.sample_surface(&runway).unwrap().unwrap();
    assert!((sample.orthometric_msl_m.0 - charted).abs() < 0.05);

    assert_eq!(
        loader.stamp_terrarium(TERRARIUM_URL, published.clone()),
        published
    );
}

#[test]
fn profile_terrain_comes_from_the_same_server() {
    let tile = TileId {
        zoom: 12,
        x: 7,
        y: 9,
    };
    assert_eq!(
        super::terrarium_url_for(TERRARIUM_URL, tile).as_deref(),
        Some("https://s3.amazonaws.com/elevation-tiles-prod/terrarium/12/7/9.png")
    );
    assert_eq!(
        super::terrarium_url_for("https://t.example/terrarium/15/1/2.png?key=k", tile).as_deref(),
        Some("https://t.example/terrarium/12/7/9.png?key=k")
    );
}
