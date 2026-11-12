use sim_navigate::trial::{
    EventKind, EventRecord, HoldingEntryName, HoldingFix, HoldingLegName, HoldingPatternRecord,
    HoldingTurnName, ProgressRecord,
};

use super::{RUN_LINES, run_lines, starting_lines, stopped_lines};
use crate::{Monitor, MonitorFeed, Pacer, Slice};

/// The most characters a status line may take: what a phone's width shows
/// without cutting it.
const MOST_CHARS: usize = 44;
const HEAD: &str = "C172 · Pilotage → Aviate · Navigate";

const STARTED: &str = r#"{"type":"started","schema":1,"request_sha256":"00","epoch_utc":"2026-06-15T12:00:00Z","route":{"waypoints":[{"ident":"WP1","lat_deg":43.79,"lon_deg":131.95,"altitude_m":2133.6},{"ident":"WP2","lat_deg":43.98,"lon_deg":131.82,"altitude_m":2133.6},{"ident":"ABCDE","lat_deg":44.04,"lon_deg":131.76,"altitude_m":2133.6}],"planned_duration_s":579.0},"sets":[{"name":"ins.baro.visual","means":["dead_reckoning","visual","barometric"]}],"guidance":"truth"}"#;

fn record(line: &str) -> MonitorFeed {
    MonitorFeed::Progress(ProgressRecord::from_json_line(line).unwrap())
}

fn event(t_s: f64, kind: EventKind) -> MonitorFeed {
    MonitorFeed::Progress(ProgressRecord::Event(EventRecord {
        t_s,
        set: None,
        kind,
    }))
}

fn sequenced(t_s: f64, index: usize, ident: &str, plan_complete: bool) -> MonitorFeed {
    event(
        t_s,
        EventKind::WaypointSequenced {
            waypoint_index: index,
            ident: ident.to_owned(),
            plan_complete,
        },
    )
}

fn pattern() -> HoldingPatternRecord {
    HoldingPatternRecord {
        fix_ident: "ABCDE".to_owned(),
        fix: HoldingFix {
            latitude_rad: 44.04_f64.to_radians(),
            longitude_rad: 131.76_f64.to_radians(),
            altitude_m: 2133.6,
        },
        inbound_course_rad: 315.0_f64.to_radians(),
        turn: HoldingTurnName::Right,
        leg_time_s: 60.0,
        turn_radius_m: 900.0,
        leg_length_m: 3300.0,
    }
}

/// A pacer that has measured a flight dearer than its pace of 64×.
fn held(by_camera: bool) -> Pacer {
    let mut pacer = Pacer::new(Some(64.0));
    for _ in 0..40 {
        let slice = Slice {
            steps: 1,
            finished: false,
            route_s: 1.0,
            wall_s: 0.08,
        };
        pacer.observe(&slice, if by_camera { 0.07 } else { 0.0 });
    }
    pacer
}

/// Every line is one short fact: no line break, never empty, within what a
/// phone shows.
fn assert_lines(state: &str, lines: &[String; RUN_LINES]) {
    for line in lines {
        assert!(
            !line.trim().is_empty(),
            "{state}: an empty line in {lines:?}"
        );
        assert!(!line.contains('\n'), "{state}: a line break in {line:?}");
        assert!(
            line.chars().count() <= MOST_CHARS,
            "{state}: {line:?} is longer than {MOST_CHARS} characters"
        );
    }
}

#[test]
fn every_state_keeps_the_same_short_lines() {
    let unheld = Pacer::new(Some(4.0));
    let mut monitor = Monitor::default();
    assert_lines("starting", &starting_lines(HEAD));
    assert_lines("stopped", &stopped_lines(HEAD));
    assert_lines(
        "before the stream",
        &run_lines(HEAD, &monitor, &unheld, None),
    );

    monitor.apply(record(STARTED));
    monitor.apply(MonitorFeed::Phase("Climb".to_owned()));
    monitor.apply(sequenced(129.0, 0, "WP1", false));
    let running = run_lines(HEAD, &monitor, &held(false), Some(("world view", 0.4)));
    assert_lines("running", &running);
    assert_eq!(running[1], "Route 2:09 / 9:39 · Climb");
    assert_eq!(running[2], "Sequenced WP1 · 1 of 3");
    assert_eq!(running[3], "Hold —");
    assert!(
        running[4].starts_with("Pace ") && running[4].ends_with(" of 64× · held by flight cost"),
        "{}",
        running[4]
    );

    monitor.apply(sequenced(531.0, 2, "ABCDE", true));
    monitor.apply(event(
        532.0,
        EventKind::HoldingStarted { pattern: pattern() },
    ));
    monitor.apply(event(
        540.0,
        EventKind::HoldingEntered {
            entry: HoldingEntryName::Teardrop,
        },
    ));
    monitor.apply(event(
        3700.0,
        EventKind::HoldingLegChanged {
            leg: HoldingLegName::Entry(HoldingEntryName::Parallel),
            circuits: 12,
        },
    ));
    let holding = run_lines(HEAD, &monitor, &held(true), Some(("world view", 1.0)));
    assert_lines("holding", &holding);
    assert_eq!(holding[2], "Plan done at ABCDE · 8:51");
    assert_eq!(holding[3], "Hold ABCDE · parallel entry · 12 circuits");
    assert!(
        holding[4].ends_with("held by world view 100 %"),
        "{}",
        holding[4]
    );

    let held_by_camera = run_lines(HEAD, &monitor, &held(true), None);
    assert!(
        held_by_camera[4].ends_with("held by camera"),
        "{held_by_camera:?}"
    );

    monitor.apply(record(
        r#"{"type":"finished","result_path":null,"wall_time_s":2.0}"#,
    ));
    let ended = run_lines(HEAD, &monitor, &Pacer::new(None), Some(("world view", 0.7)));
    assert_lines("ended", &ended);
    assert!(ended[1].ends_with(" · ended"), "{}", ended[1]);
    assert_eq!(ended[4], "Pace max · world view 70 %");
}
