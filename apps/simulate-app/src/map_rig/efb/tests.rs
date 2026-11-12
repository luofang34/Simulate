//! The EFB's approach plate follows the active runway end of any database airport.

use std::time::Duration;

use sim_maplibre::{
    EfbDisplayMode, EfbPacer, EfbRefreshPolicy, generate_procedural_approach_plate,
};
use sim_scenery::AirportDatabase;
use web_time::Instant;

use super::plate::{plate_spec, runway_end};
use super::{EfbStatus, PLATE_OPACITIES};

#[test]
fn the_plate_spec_follows_the_surveyed_runway_end_and_first_frequency() {
    let (_, kewr) = AirportDatabase::builtin().by_code("KEWR").expect("KEWR");
    let spec = plate_spec(kewr, "22L").expect("22L");
    assert_eq!(spec.airport_code, "KEWR");
    assert_eq!(spec.runway_designator, "22L");
    assert_eq!(
        spec.ils_freq_khz, 118_300,
        "tower, the first frequency to call"
    );
    assert!((spec.threshold_lat_deg - 40.70).abs() < 0.05);
    assert!((spec.threshold_lon_deg + 74.17).abs() < 0.05);
    assert!(
        (200.0..240.0).contains(&spec.true_heading_deg),
        "{}",
        spec.true_heading_deg
    );
    assert!(
        (0.0..40.0).contains(&spec.elevation_ft),
        "{}",
        spec.elevation_ft
    );
    let plate = generate_procedural_approach_plate(&spec).expect("plate");
    assert_eq!(plate.title, "ILS OR LOC RWY 22L");
    let [nw, _, se, _] = plate.corners_lon_lat;
    assert!(nw[1] > spec.threshold_lat_deg - 0.3 && se[1] < spec.threshold_lat_deg + 0.3);
}

#[test]
fn an_unknown_designator_falls_back_to_the_primary_runway() {
    let (_, egll) = AirportDatabase::builtin().by_code("EGLL").expect("EGLL");
    let end = runway_end(egll, "").expect("a primary runway end");
    let primary = egll
        .primary_runway()
        .expect("primary")
        .ends
        .first()
        .copied();
    assert_eq!(Some(*end), primary);
    let spec = plate_spec(egll, "99X").expect("fallback spec");
    assert_eq!(spec.runway_designator, end.designator.as_str());
    assert!(generate_procedural_approach_plate(&spec).is_ok());
}

#[test]
fn the_plate_opacity_cycle_starts_at_three_quarters() {
    assert_eq!(PLATE_OPACITIES, [0.75, 0.45, 0.95]);
    let status = EfbStatus {
        mode: sim_maplibre::EfbDisplayMode::CompactCorner,
        collapsed: false,
        zoom: sim_maplibre::DEFAULT_COMPACT_ZOOM,
        north_up: false,
        custom_bearing: false,
        free_pan: false,
        plate_shown: false,
        plate_opacity_pct: 75,
        diagram_shown: true,
        airport: "KEWR".to_owned(),
        plate_title: None,
        satellite: false,
    };
    assert!((status.zoom - 10.2).abs() < 1e-9);
}

#[test]
fn cockpit_and_standalone_pacers_avoid_vsync_storms_while_keeping_zero_lag_input() {
    let t0 = Instant::now();
    let mut pacer = EfbPacer::new(EfbRefreshPolicy::CockpitAdaptive);
    let compact = EfbDisplayMode::CompactCorner;
    let expanded = EfbDisplayMode::ExpandedEfb;

    assert!(pacer.should_render(t0, Duration::ZERO, compact, false, false, false));
    pacer.record_rendered(t0, Duration::ZERO);

    // Paused with network HTTP requests merely in flight (`has_ready_work == false`): 0 Hz.
    let t8 = t0 + Duration::from_millis(8);
    assert!(!pacer.should_render(t8, Duration::ZERO, compact, true, false, false));

    // Operator command (`dirty`) redraws immediately even 8 ms later.
    assert!(pacer.should_render(t8, Duration::ZERO, compact, true, true, false));

    // Flying at 120 Hz: compact inset coalesces motion and streaming arrivals to 66 ms (15 Hz).
    assert!(!pacer.should_render(t8, Duration::from_millis(8), compact, true, false, false));
    assert!(!pacer.should_render(t8, Duration::ZERO, compact, true, false, true));
    let t66 = t0 + Duration::from_millis(66);
    assert!(pacer.should_render(t66, Duration::ZERO, compact, true, false, true));

    // Expanded cockpit EFB draws arrived tiles immediately (0 ms) and paces motion at 33 ms.
    assert!(pacer.should_render(t8, Duration::ZERO, expanded, true, false, true));
    assert!(!pacer.should_render(t8, Duration::from_millis(8), expanded, true, false, false));
    let t33 = t0 + Duration::from_millis(33);
    assert!(pacer.should_render(t33, Duration::from_millis(33), expanded, true, false, false));

    // Standalone EFB draws arrived tiles immediately in any layout and rewinds cleanly.
    pacer.set_policy(EfbRefreshPolicy::StandaloneInteractive);
    pacer.record_rendered(t33, Duration::from_secs(10));
    assert!(pacer.should_render(
        t33 + Duration::from_millis(4),
        Duration::from_secs(10),
        compact,
        true,
        false,
        true
    ));
    assert!(pacer.should_render(
        t33 + Duration::from_millis(4),
        Duration::from_secs(2),
        compact,
        true,
        false,
        false
    ));
}
