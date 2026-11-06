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
    let rgb: Vec<u8> = (0..16 * 16)
        .flat_map(|_| encode_terrarium(height_m))
        .collect();
    let image = image::RgbImage::from_raw(16, 16, rgb).unwrap();
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
