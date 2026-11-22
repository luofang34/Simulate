use sim_maplibre::CockpitDeviceTier;

use super::{DeviceFacts, choose_tier, is_mobile_user_agent};

const LAPTOP: DeviceFacts = DeviceFacts {
    mobile: Some(false),
    device_memory_gib: Some(16.0),
    fallback_adapter: false,
    max_buffer_bytes: 4 << 30,
    coarse_pointer: false,
    max_touch_points: 0,
    min_viewport_css_px: Some(900.0),
    ios_like_ua: false,
};

#[test]
fn a_desktop_or_laptop_browser_runs_the_browser_tier() {
    let choice = choose_tier(&LAPTOP);
    assert_eq!(choice.tier, CockpitDeviceTier::Browser);
    assert!(
        choice.words().contains("browser tier"),
        "{}",
        choice.words()
    );
    assert_eq!(choice.line(), "Map detail: browser tier");
    // A browser that does not report its memory is not taken for a small one.
    let quiet = DeviceFacts {
        mobile: None,
        device_memory_gib: None,
        ..LAPTOP
    };
    assert_eq!(choose_tier(&quiet).tier, CockpitDeviceTier::Browser);
    // A fine-pointer laptop with a touchscreen above 1024 px also keeps Browser tier.
    let touch_laptop = DeviceFacts {
        coarse_pointer: false,
        max_touch_points: 10,
        min_viewport_css_px: Some(1200.0),
        ..quiet
    };
    assert_eq!(choose_tier(&touch_laptop).tier, CockpitDeviceTier::Browser);
}

#[test]
fn a_phone_a_small_device_or_a_software_gpu_runs_the_handheld_tier() {
    for facts in [
        DeviceFacts {
            mobile: Some(true),
            ..LAPTOP
        },
        DeviceFacts {
            device_memory_gib: Some(4.0),
            ..LAPTOP
        },
        DeviceFacts {
            fallback_adapter: true,
            ..LAPTOP
        },
        DeviceFacts {
            max_buffer_bytes: 256 << 20,
            ..LAPTOP
        },
        DeviceFacts {
            mobile: None,
            device_memory_gib: None,
            ios_like_ua: true,
            ..LAPTOP
        },
        DeviceFacts {
            mobile: None,
            device_memory_gib: None,
            coarse_pointer: true,
            max_touch_points: 5,
            min_viewport_css_px: Some(820.0),
            ..LAPTOP
        },
    ] {
        let choice = choose_tier(&facts);
        assert_eq!(choice.tier, CockpitDeviceTier::Handheld, "{facts:?}");
        assert!(!choice.reasons.is_empty(), "{:?}", choice.reasons);
    }
    let iphone_safari = DeviceFacts {
        mobile: None,
        device_memory_gib: None,
        coarse_pointer: true,
        max_touch_points: 5,
        min_viewport_css_px: Some(390.0),
        ios_like_ua: true,
        ..LAPTOP
    };
    assert_eq!(
        choose_tier(&iphone_safari).tier,
        CockpitDeviceTier::Handheld
    );
    assert!(is_mobile_user_agent(
        "Mozilla/5.0 (iPhone; CPU iPhone OS 18_3 like Mac OS X) AppleWebKit/605.1.15"
    ));
    assert!(!is_mobile_user_agent(
        "Mozilla/5.0 (Macintosh; Intel Mac OS X 10_15_7) AppleWebKit/605.1.15"
    ));
    let all = DeviceFacts {
        mobile: Some(true),
        device_memory_gib: Some(2.0),
        fallback_adapter: true,
        max_buffer_bytes: 128 << 20,
        coarse_pointer: true,
        max_touch_points: 5,
        min_viewport_css_px: Some(390.0),
        ios_like_ua: true,
    };
    assert_eq!(choose_tier(&all).reasons.len(), 7);
}
