use sim_maplibre::CockpitDeviceTier;

use super::{DeviceFacts, choose_tier};

const LAPTOP: DeviceFacts = DeviceFacts {
    mobile: Some(false),
    device_memory_gib: Some(16.0),
    fallback_adapter: false,
    max_buffer_bytes: 4 << 30,
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
    ] {
        let choice = choose_tier(&facts);
        assert_eq!(choice.tier, CockpitDeviceTier::Handheld, "{facts:?}");
        assert_eq!(choice.reasons.len(), 1, "{:?}", choice.reasons);
    }
    let all = DeviceFacts {
        mobile: Some(true),
        device_memory_gib: Some(2.0),
        fallback_adapter: true,
        max_buffer_bytes: 128 << 20,
    };
    assert_eq!(choose_tier(&all).reasons.len(), 4);
}
