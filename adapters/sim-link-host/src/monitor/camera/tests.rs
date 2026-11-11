use std::collections::BTreeMap;

use sim_navigate::trial::ProgressRecord;

use super::outcome_words;
use crate::{CameraShot, Monitor, MonitorFeed};

const STARTED: &str = r#"{"type":"started","schema":1,"request_sha256":"00","epoch_utc":"2026-06-15T12:00:00Z","route":{"waypoints":[{"ident":"Z0","lat_deg":41.7,"lon_deg":-1.05,"altitude_m":2000.0},{"ident":"Z1","lat_deg":41.7,"lon_deg":-0.8,"altitude_m":2000.0}],"planned_duration_s":500.0},"sets":[{"name":"ins+baro","means":["dead_reckoning","barometric"]},{"name":"ins+baro+visual","means":["dead_reckoning","visual","barometric"]}],"guidance":"truth"}"#;

const VISUAL: &str = "ins+baro+visual";

fn shot(index: u64, lon_deg: f64) -> MonitorFeed {
    MonitorFeed::Frame(CameraShot {
        index,
        mid_s: index as f64,
        size: (2, 1),
        luma: vec![10, 20],
        at_deg: (41.7, lon_deg),
    })
}

fn tally(entries: &[(&str, u64)]) -> MonitorFeed {
    MonitorFeed::TerrainFixes {
        set: VISUAL.to_owned(),
        fixes: entries
            .iter()
            .map(|(key, count)| ((*key).to_owned(), *count))
            .collect::<BTreeMap<_, _>>(),
    }
}

fn started() -> Monitor {
    let mut monitor = Monitor::default();
    monitor.apply(MonitorFeed::Progress(
        ProgressRecord::from_json_line(STARTED).unwrap(),
    ));
    monitor
}

#[test]
fn a_frame_takes_what_its_set_tallied_after_it() {
    let mut monitor = started();
    monitor.apply(shot(1, -1.0));
    assert_eq!(monitor.camera().latest().unwrap().index, 1);
    assert_eq!(monitor.camera().outcomes(VISUAL), None);
    monitor.apply(tally(&[
        ("context:none", 1),
        ("unused:insufficient_matches:unmatched", 1),
    ]));
    assert_eq!(
        monitor.camera().outcomes(VISUAL).unwrap(),
        ["unused:insufficient_matches:unmatched"]
    );
    assert!(monitor.camera().marks().is_empty());

    monitor.apply(shot(2, -0.9));
    assert_eq!(monitor.camera().outcomes(VISUAL), None);
    monitor.apply(tally(&[
        ("accepted", 1),
        ("context:keyframe", 1),
        ("context:none", 1),
        ("unused:insufficient_matches:unmatched", 1),
    ]));
    assert_eq!(monitor.camera().outcomes(VISUAL).unwrap(), ["accepted"]);
    let marks = monitor.camera().marks();
    assert_eq!(marks.len(), 1);
    assert_eq!((marks[0].set, marks[0].accepted), (1, true));
    assert_eq!(marks[0].at_deg, (41.7, -0.9));

    monitor.apply(shot(3, -0.8));
    monitor.apply(tally(&[
        ("accepted", 1),
        ("context:keyframe", 2),
        ("context:none", 1),
        ("rejected:Innovation", 1),
        ("unused:insufficient_matches:unmatched", 1),
    ]));
    assert_eq!(
        monitor.camera().outcomes(VISUAL).unwrap(),
        ["rejected:Innovation"]
    );
    assert_eq!(monitor.camera().marks().len(), 2);
    assert!(!monitor.camera().marks()[1].accepted);
}

#[test]
fn a_tally_before_any_frame_marks_nothing() {
    let mut monitor = started();
    monitor.apply(tally(&[("accepted", 1)]));
    assert!(monitor.camera().latest().is_none());
    assert!(monitor.camera().marks().is_empty());
}

#[test]
fn outcomes_read_plainly() {
    assert_eq!(outcome_words("accepted"), "accepted");
    assert_eq!(
        outcome_words("rejected:Innovation"),
        "rejected by the filter (Innovation)"
    );
    assert_eq!(
        outcome_words("unused:shared_map_error:matched"),
        "shared_map_error"
    );
}
