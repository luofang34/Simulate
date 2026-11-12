//! The plan's progress as the monitor keeps it for the page's map.

use sim_navigate::trial::ProgressRecord;

use super::super::{EVENTS_KEPT, Monitor, MonitorFeed};
use super::{STARTED, record, waypoint};

#[test]
fn the_monitor_keeps_the_holding_pattern_the_host_flies() {
    use sim_navigate::trial::{
        EventKind, EventRecord, HoldingFix, HoldingPatternRecord, HoldingTurnName,
    };
    let mut monitor = Monitor::default();
    monitor.apply(record(STARTED));
    assert!(monitor.hold_pattern().is_none());
    let pattern = HoldingPatternRecord {
        fix_ident: "Z1".to_owned(),
        fix: HoldingFix {
            latitude_rad: 41.7_f64.to_radians(),
            longitude_rad: (-0.8_f64).to_radians(),
            altitude_m: 2000.0,
        },
        inbound_course_rad: 1.0,
        turn: HoldingTurnName::Left,
        leg_time_s: 60.0,
        turn_radius_m: 900.0,
        leg_length_m: 3300.0,
    };
    let event = |kind| {
        MonitorFeed::Progress(ProgressRecord::Event(EventRecord {
            t_s: 10.0,
            set: None,
            kind,
        }))
    };
    monitor.apply(event(EventKind::HoldingStarted {
        pattern: pattern.clone(),
    }));
    monitor.apply(event(EventKind::GuidanceRefused {
        refusal: "x".into(),
    }));
    assert_eq!(monitor.hold_pattern(), Some(&pattern));
}

#[test]
fn the_last_sequenced_waypoint_outlives_the_event_log() {
    let mut monitor = Monitor::default();
    monitor.apply(record(STARTED));
    assert_eq!(monitor.sequenced(), None);
    assert_eq!(monitor.waypoint_altitudes_m(), [2000.0, 2000.0]);
    monitor.apply(record(&waypoint(1.0, 0, "Z0")));
    let outage = r#"{"type":"event","t_s":2.0,"set":null,"kind":"availability_start","sensor":"gnss","schedule":"s","reason":null}"#;
    for _ in 0..EVENTS_KEPT + 3 {
        monitor.apply(record(outage));
    }
    assert_eq!(monitor.last_waypoint(), None, "the log no longer holds it");
    assert_eq!(monitor.sequenced(), Some((0, false)));
}
