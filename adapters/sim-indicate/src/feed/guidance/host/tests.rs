use indicate_instrument_feeder::{CLOCK_HOST_MONOTONIC, ROLE_NAVIGATION_SOLUTION};
use indicate_instrument_state::{
    AirframeDisplayProfile, FreshnessPolicy, NavFromTo, NavScale, NavSequencing, NavSource,
    PanelData, UnusualAttitudeState, resolve_stateful,
};
use sim_core::{SimInstant, VehicleState};
use sim_geo::{Geodetic, LocalFrame};

use super::*;
use crate::{AltitudeSource, DisplayAxes, FeedConfig, FlightSample, StateFeed};

fn origin() -> Geodetic {
    Geodetic::from_degrees(39.12, -75.47, 10.0)
}

fn feed() -> StateFeed {
    StateFeed::new(FeedConfig {
        body: BodyEllipsoid::WGS84,
        axes: DisplayAxes::AEROSPACE,
        origin: origin(),
        origin_id: 3,
        altitude: AltitudeSource::OriginRelative,
        heading_mode: crate::HeadingMode::default(),
        deviation_scale: DeviationScaleProfile::GNSS_NAVIGATOR,
    })
}

fn vehicle() -> VehicleState {
    let fixed =
        BodyEllipsoid::WGS84.geodetic_to_fixed(&origin()) + LocalFrame::at(&origin()).up() * 300.0;
    VehicleState::level_flight(&BodyEllipsoid::WGS84.fixed_to_geodetic(fixed), 0.0, 40.0)
}

fn stamp(epoch: u32, sequence: u32) -> RawStamp {
    RawStamp {
        role: ROLE_NAVIGATION_SOLUTION,
        integrity: 1,
        source_id: 7,
        incarnation: [3; 16],
        epoch,
        sequence,
        acquired_at_ns: 0,
        clock: CLOCK_HOST_MONOTONIC,
    }
}

/// One nautical mile right of course and 1 000 ft above the profile on a
/// GNSS navigator's terminal scale, whose full scales are one nautical mile
/// and 1 000 ft at two dots: the bar sits two dots left and the vertical
/// pointer two dots up.
fn lane_guidance() -> Guidance {
    Guidance {
        to_ident: IdentStr::new("WPT2").unwrap_or_default(),
        from_ident: IdentStr::new("WPT1").unwrap_or_default(),
        course_rad: 0.5,
        lateral_deviation_m: 1_852.0,
        vertical_deviation_m: 304.8,
        distance_to_waypoint_m: 3_704.0,
        scale: NavScale::Terminal,
        to_from: NavFromTo::To,
        sequencing: NavSequencing::Sequencing,
        leg_index: 1,
        waypoint_count: 3,
        solution_quality: 0,
    }
}

fn lane(epoch: u32, sequence: u32, now_ms: f64) -> HostNavGuidance {
    HostNavGuidance::PilotageLane {
        stamp: stamp(epoch, sequence),
        guidance: lane_guidance(),
        now_ms,
    }
}

fn host(nav: HostNavGuidance, source_epoch: u32) -> HostFlightGuidance {
    HostFlightGuidance {
        selections: PilotSelections {
            heading_rad: 0.5,
            altitude_m: Some(300.0),
            baro_setting_hpa: None,
        },
        nav,
        autopilot: AutopilotModeSummary {
            lateral: IdentStr::new("NAV").ok(),
            vertical: IdentStr::new("ALT").ok(),
            armed_approach: false,
            engaged: true,
        },
        source_epoch,
    }
}

fn panel(feed: &mut StateFeed, guidance: HostFlightGuidance) -> PanelData {
    let sample = FlightSample {
        guidance: Some(guidance),
        ..FlightSample::truth(SimInstant::START, vehicle())
    };
    let state = feed.state(&sample).unwrap();
    resolve_stateful(
        &state,
        &FreshnessPolicy::default(),
        &AirframeDisplayProfile::simulator(),
        &mut UnusualAttitudeState::default(),
    )
}

fn assert_shows_lane(panel: &PanelData) {
    assert!(panel.nav.status.shows_value(), "{:?}", panel.nav.status);
    assert_eq!(panel.nav.data.source, NavSource::Gps);
    assert_eq!(panel.nav.data.fromto, NavFromTo::To);
    assert!((panel.nav.data.cdi_dots + 2.0).abs() < 1e-4);
    assert!((panel.nav.data.vdev_dots.unwrap() - 2.0).abs() < 1e-4);
    assert!((panel.nav.course_rose_rad.value - 0.5).abs() < 1e-4);
}

#[test]
fn no_host_guidance_leaves_the_deviation_scales_absent() {
    let panel = panel(&mut feed(), host(HostNavGuidance::None, 0));
    assert!(!panel.nav.status.shows_value());
    assert!(panel.nav.data.vdev_dots.is_none());
    // Selections still reach the panel; only the course is absent.
    assert!(panel.heading_bug_rose_rad.status.shows_value());
}

#[test]
fn a_pilotage_lane_sample_shows_on_the_hsi() {
    assert_shows_lane(&panel(&mut feed(), host(lane(1, 0, 1_000.0), 0)));
}

#[test]
fn a_new_epoch_is_accepted_while_the_clock_runs_on() {
    let mut feed = feed();
    assert_shows_lane(&panel(&mut feed, host(lane(1, 40, 1_000.0), 0)));
    assert_shows_lane(&panel(&mut feed, host(lane(2, 0, 1_100.0), 1)));
}

#[test]
fn a_rewound_clock_hides_guidance_until_the_feed_resets_it() {
    let mut feed = feed();
    assert_shows_lane(&panel(&mut feed, host(lane(1, 40, 1_000.0), 0)));
    let rewound = host(lane(1_u32.wrapping_add(1), 0, 200.0), 1);
    assert!(!panel(&mut feed, rewound).nav.status.shows_value());
    feed.reset_guidance_clock();
    assert_shows_lane(&panel(&mut feed, rewound));
}

fn lane_age_ms(tracker: &mut NavGuidanceTracker, nav: HostNavGuidance) -> Option<f32> {
    host_nav(
        tracker,
        &nav,
        &DeviationScaleProfile::GNSS_NAVIGATOR,
        &BodyEllipsoid::WGS84,
        DVec3::ZERO,
    )
    .age_ms
}

#[test]
fn only_a_new_sample_restarts_the_guidance_age() {
    let mut tracker = NavGuidanceTracker::new();
    assert_eq!(lane_age_ms(&mut tracker, lane(1, 5, 1_000.0)), Some(0.0));
    assert_eq!(lane_age_ms(&mut tracker, lane(1, 5, 1_400.0)), Some(400.0));
    assert_eq!(lane_age_ms(&mut tracker, lane(2, 0, 1_500.0)), Some(0.0));
}

#[test]
fn a_runway_receiver_keeps_its_ils_scale() {
    let receiver = RunwayNavGuidance {
        threshold: origin(),
        course_rad: 0.0,
        length_m: 2_000.0,
        glide_path_rad: 3.0_f64.to_radians(),
        ident: IdentStr::new("RW01").unwrap_or_default(),
    };
    let panel = panel(
        &mut feed(),
        host(HostNavGuidance::RunwayReceiver(receiver), 0),
    );
    assert!(panel.nav.status.shows_value());
    assert_eq!(panel.nav.data.source, NavSource::Nav1);
    assert_eq!(
        panel.nav.data.scale,
        indicate_instrument_state::NavScale::Approach
    );
}
