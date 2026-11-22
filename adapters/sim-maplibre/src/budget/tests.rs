use std::time::Duration;

use super::{BROWSER_AVAILABLE_MEMORY_BYTES, CockpitDeviceTier, HANDHELD_AVAILABLE_MEMORY_BYTES};

#[test]
fn browser_and_handheld_tiers_report_bounded_available_memory() {
    assert_eq!(CockpitDeviceTier::Desktop.available_memory_bytes(), None);
    assert_eq!(
        CockpitDeviceTier::Browser.available_memory_bytes(),
        Some(BROWSER_AVAILABLE_MEMORY_BYTES)
    );
    assert_eq!(
        CockpitDeviceTier::Handheld.available_memory_bytes(),
        Some(HANDHELD_AVAILABLE_MEMORY_BYTES)
    );
}

#[test]
fn desktop_drape_budget_keeps_taxi_frames_short() {
    let budget = CockpitDeviceTier::Desktop.otw_drape_budget();
    assert_eq!(budget.per_frame, 6);
    assert_eq!(budget.per_eye_frame, 5);
    assert_eq!(budget.moving_time, Duration::from_micros(3_000));
}

#[test]
fn a_browsers_drape_budget_stays_small_beside_one_window_view() {
    let browser = CockpitDeviceTier::Browser.otw_drape_budget();
    assert_eq!(browser.per_frame, 4);
    assert_eq!(browser.per_eye_frame, 3);
    assert_eq!(browser.moving_time, Duration::from_micros(2_000));

    let handheld = CockpitDeviceTier::Handheld.otw_drape_budget();
    assert_eq!(handheld.per_frame, 2);
    assert_eq!(handheld.per_eye_frame, 1);
    assert_eq!(handheld.moving_time, Duration::from_micros(1_200));
}

#[test]
fn conformal_interval_admits_every_second_60hz_frame() {
    let interval = CockpitDeviceTier::Desktop.conformal_min_interval();
    let two_frames = Duration::from_secs_f64(2.0 / 60.0);
    let one_frame = Duration::from_secs_f64(1.0 / 60.0);
    assert!(two_frames >= interval);
    assert!(one_frame < interval);
    assert_eq!(
        CockpitDeviceTier::Handheld.conformal_min_interval(),
        Duration::from_millis(66)
    );
}

#[test]
fn handheld_tier_scales_down_drapes_caches_shadows_and_dem_capacity() {
    let otw = CockpitDeviceTier::Handheld.truth_scene_settings(wgpu::TextureFormat::Rgba8Unorm);
    assert_eq!(otw.terrain_drape_size, 512);
    assert_eq!(otw.msaa.samples, 1);

    let chart = CockpitDeviceTier::Handheld.belief_chart_settings(wgpu::TextureFormat::Rgba8Unorm);
    assert_eq!(chart.terrain_drape_size, 256);
    assert_eq!(chart.msaa.samples, 1);

    assert_eq!(
        CockpitDeviceTier::Desktop.shared_asset_cache_bytes(),
        128 << 20
    );
    assert_eq!(
        CockpitDeviceTier::Browser.shared_asset_cache_bytes(),
        64 << 20
    );
    assert_eq!(
        CockpitDeviceTier::Handheld.shared_asset_cache_bytes(),
        16 << 20
    );

    assert_eq!(CockpitDeviceTier::Desktop.shadow_map_size(), 2048);
    assert_eq!(CockpitDeviceTier::Browser.shadow_map_size(), 1024);
    assert_eq!(CockpitDeviceTier::Handheld.shadow_map_size(), 512);

    assert_eq!(CockpitDeviceTier::Desktop.live_dem_capacity(), None);
    assert_eq!(CockpitDeviceTier::Browser.live_dem_capacity(), Some(256));
    assert_eq!(CockpitDeviceTier::Handheld.live_dem_capacity(), Some(64));
}
