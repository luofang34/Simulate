//! The stream's wire form, pinned as literal lines: another program reads
//! these field names, so a rename must fail here, not only round-trip.

use super::{
    EventKind, EventRecord, FinishedRecord, ProgressPosition, ProgressRecord, ProgressRoute,
    ProgressSet, ProgressWaypoint, SampleRecord, SensorStates, StartedRecord,
};
use crate::NavMeans;
use crate::trial::{GuidanceSource, SensorKind, SensorState};

fn line(record: &ProgressRecord) -> String {
    let text = record.to_json_line().unwrap();
    assert_eq!(&ProgressRecord::from_json_line(&text).unwrap(), record);
    text
}

#[test]
fn started_and_finished_lines_are_pinned() {
    let started = ProgressRecord::Started(StartedRecord {
        schema: 1,
        request_sha256: "ab".into(),
        epoch_utc: "2026-03-20T02:00:00Z".into(),
        route: ProgressRoute {
            waypoints: vec![ProgressWaypoint {
                ident: "W0".into(),
                lat_deg: 39.0,
                lon_deg: -75.5,
                altitude_m: 300.0,
            }],
            planned_duration_s: 60.0,
        },
        sets: vec![ProgressSet {
            name: "ins+gnss".into(),
            means: vec![NavMeans::DeadReckoning, NavMeans::Gnss],
        }],
        guidance: GuidanceSource::Estimate,
    });
    assert_eq!(
        line(&started),
        r#"{"type":"started","schema":1,"request_sha256":"ab","epoch_utc":"2026-03-20T02:00:00Z","route":{"waypoints":[{"ident":"W0","lat_deg":39.0,"lon_deg":-75.5,"altitude_m":300.0}],"planned_duration_s":60.0},"sets":[{"name":"ins+gnss","means":["dead_reckoning","gnss"]}],"guidance":"estimate"}"#
    );
    let finished = ProgressRecord::Finished(FinishedRecord {
        result_path: None,
        wall_time_s: 1.5,
    });
    assert_eq!(
        line(&finished),
        r#"{"type":"finished","result_path":null,"wall_time_s":1.5}"#
    );
}

#[test]
fn sample_lines_are_pinned() {
    let at = |lat_deg| ProgressPosition {
        lat_deg,
        lon_deg: 2.0,
        altitude_m: 3.0,
    };
    let sample = ProgressRecord::Sample(SampleRecord {
        t_s: 4.0,
        set: "ins".into(),
        truth: at(1.0),
        estimate: at(1.5),
        horizontal_error_m: 5.0,
        vertical_error_m: 6.0,
        sigma_horizontal_m: 7.0,
        cross_track_true_m: None,
        sensors: SensorStates {
            gnss: SensorState::ScheduledUnavailable,
            baro: SensorState::Aiding,
            stars: SensorState::NotInSet,
            terrain_camera: SensorState::NotInSet,
        },
    });
    assert_eq!(
        line(&sample),
        r#"{"type":"sample","t_s":4.0,"set":"ins","truth":{"lat_deg":1.0,"lon_deg":2.0,"altitude_m":3.0},"estimate":{"lat_deg":1.5,"lon_deg":2.0,"altitude_m":3.0},"horizontal_error_m":5.0,"vertical_error_m":6.0,"sigma_horizontal_m":7.0,"cross_track_true_m":null,"sensors":{"gnss":"scheduled_unavailable","baro":"aiding","stars":"not_in_set","terrain_camera":"not_in_set"}}"#
    );
    let states = [
        (SensorState::PhysicallyUnavailable, "physically_unavailable"),
        (SensorState::Rejected, "rejected"),
    ];
    for (state, name) in states {
        assert_eq!(
            serde_json::to_string(&state).unwrap(),
            format!("\"{name}\"")
        );
    }
}

#[test]
fn event_lines_are_pinned() {
    let event = |set: Option<&str>, kind| {
        line(&ProgressRecord::Event(EventRecord {
            t_s: 8.0,
            set: set.map(str::to_owned),
            kind,
        }))
    };
    let lines = [
        event(
            None,
            EventKind::WaypointSequenced {
                waypoint_index: 1,
                ident: "W1".into(),
                plan_complete: false,
            },
        ),
        event(
            Some("a"),
            EventKind::AvailabilityStart {
                sensor: SensorKind::Gnss,
                schedule: "jam".into(),
                reason: Some("jamming".into()),
            },
        ),
        event(
            Some("a"),
            EventKind::AvailabilityEnd {
                sensor: SensorKind::TerrainCamera,
                schedule: "fog".into(),
                reason: None,
            },
        ),
        event(
            Some("a"),
            EventKind::GuidanceRefused {
                refusal: "IntegrityBelowFloor".into(),
            },
        ),
        event(
            Some("a"),
            EventKind::Terminated {
                rule: "left_corridor".into(),
                detail: "far".into(),
            },
        ),
    ];
    let expected = [
        r#"{"type":"event","t_s":8.0,"set":null,"kind":"waypoint_sequenced","waypoint_index":1,"ident":"W1","plan_complete":false}"#,
        r#"{"type":"event","t_s":8.0,"set":"a","kind":"availability_start","sensor":"gnss","schedule":"jam","reason":"jamming"}"#,
        r#"{"type":"event","t_s":8.0,"set":"a","kind":"availability_end","sensor":"terrain_camera","schedule":"fog","reason":null}"#,
        r#"{"type":"event","t_s":8.0,"set":"a","kind":"guidance_refused","refusal":"IntegrityBelowFloor"}"#,
        r#"{"type":"event","t_s":8.0,"set":"a","kind":"terminated","rule":"left_corridor","detail":"far"}"#,
    ];
    assert_eq!(lines, expected);
}
