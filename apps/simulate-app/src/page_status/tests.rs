use sim_scenario::SimTimeScale;

use super::{EngineState, PAGE_STATUS_LINES, PageStatus};

/// The most characters a status line may take: what a phone's width shows
/// without cutting it.
const MOST_CHARS: usize = 44;

fn running() -> PageStatus<'static> {
    PageStatus {
        view: "OTW",
        height_m: 2159.4,
        speed_m_s: 61.2,
        engine: EngineState::Running(2614.0),
        armed: Some(true),
        phase: Some("Climb"),
        navigation: Some("WAAS [N]"),
        time_scale: SimTimeScale::RealTime,
        real_time_ratio: 1.5,
        fps: 39.0,
    }
}

#[test]
fn every_state_keeps_the_same_short_lines() {
    let paused = PageStatus {
        time_scale: SimTimeScale::Paused,
        real_time_ratio: 0.0,
        ..running()
    };
    let warped = PageStatus {
        time_scale: SimTimeScale::Warp(64.0),
        real_time_ratio: 63.97,
        fps: 120.0,
        ..running()
    };
    let starting = PageStatus {
        view: "Sensor",
        height_m: 12_345.0,
        speed_m_s: 250.0,
        engine: EngineState::Off,
        armed: Some(false),
        phase: None,
        navigation: Some("DEGRADED [N]"),
        ..running()
    };
    let bare = PageStatus {
        engine: EngineState::Absent,
        armed: None,
        phase: None,
        navigation: None,
        ..running()
    };
    for (state, status) in [
        ("running", running()),
        ("paused", paused),
        ("warped", warped),
        ("starting", starting),
        ("bare", bare),
    ] {
        let lines: [String; PAGE_STATUS_LINES] = status.lines();
        for line in &lines {
            assert!(!line.trim().is_empty(), "{state}: an empty line");
            assert!(!line.contains('\n'), "{state}: a line break in {line:?}");
            assert!(
                line.chars().count() <= MOST_CHARS,
                "{state}: {line:?} is longer than {MOST_CHARS} characters"
            );
        }
    }
    assert_eq!(
        running().lines(),
        [
            "OTW · 2159 m · 61 m/s · 2614 rpm",
            "FC armed · Climb · WAAS [N]",
            "Real time · RT 1.50× · 39 fps"
        ]
    );
}
