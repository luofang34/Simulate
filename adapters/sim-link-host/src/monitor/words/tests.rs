use std::collections::BTreeMap;

use sim_navigate::trial::{EventKind, EventRecord, SensorKind, SensorState};

use super::{clock_words, event_words, state_words, terrain_words};
use crate::{Monitor, MonitorFeed};

fn event(set: Option<&str>, kind: EventKind) -> EventRecord {
    EventRecord {
        t_s: 10.0,
        set: set.map(str::to_owned),
        kind,
    }
}

#[test]
fn route_time_reads_as_a_clock() {
    assert_eq!(clock_words(0.0), "0:00");
    assert_eq!(clock_words(59.9), "0:59");
    assert_eq!(clock_words(1407.7), "23:27");
    assert_eq!(clock_words(3725.0), "1:02:05");
    assert_eq!(clock_words(f64::NAN), "0:00");
    assert_eq!(clock_words(-3.0), "0:00");
}

#[test]
fn events_read_as_what_happened() {
    let waypoint = |plan_complete| EventKind::WaypointSequenced {
        waypoint_index: 1,
        ident: "ZA2".to_owned(),
        plan_complete,
    };
    assert_eq!(event_words(&event(None, waypoint(false))), "sequenced ZA2");
    assert_eq!(
        event_words(&event(None, waypoint(true))),
        "reached ZA2, the last waypoint"
    );
    let withheld = EventKind::AvailabilityStart {
        sensor: SensorKind::Gnss,
        schedule: "out".to_owned(),
        reason: Some("jamming".to_owned()),
    };
    assert_eq!(
        event_words(&event(Some("ins+baro+gnss"), withheld)),
        "ins+baro+gnss: GNSS withheld: jamming"
    );
    let back = EventKind::AvailabilityEnd {
        sensor: SensorKind::TerrainCamera,
        schedule: "out".to_owned(),
        reason: None,
    };
    assert_eq!(
        event_words(&event(None, back)),
        "terrain camera available again"
    );
}

#[test]
fn a_set_without_a_sensor_says_nothing_of_it() {
    assert_eq!(state_words(SensorState::NotInSet), None);
    assert_eq!(state_words(SensorState::Aiding), Some("aiding"));
}

#[test]
fn the_terrain_tally_names_each_reason() {
    let mut monitor = Monitor::default();
    assert_eq!(
        terrain_words(&monitor, "v"),
        "no terrain-camera tally reported yet"
    );
    let fixes: BTreeMap<String, u64> = [
        ("accepted".to_owned(), 3),
        ("unused:outside_coverage:unmatched".to_owned(), 2),
    ]
    .into();
    monitor.apply(MonitorFeed::TerrainFixes {
        set: "v".to_owned(),
        fixes,
    });
    assert_eq!(
        terrain_words(&monitor, "v"),
        "3 accepted; unused: outside_coverage 2"
    );
}
