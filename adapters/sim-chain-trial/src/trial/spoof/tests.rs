use glam::DVec3;
use sim_geo::{LocalFrame, ecef_to_geodetic, geodetic_to_ecef};
use sim_navigate::trial::{DirectoryStores, EventKind, ProgressRecord, SensorKind};

use super::super::tests::{advanced, spec};
use crate::{ChainTrial, ChainTrialSpec, GnssSpoofSpec, SpoofSpan};

const RATE_M_S: f64 = 5.0;

fn spoofed_from(start_s: f64, end_s: Option<f64>) -> ChainTrialSpec {
    let mut spec = spec(None);
    spec.gnss_spoof = Some(GnssSpoofSpec {
        rate_m_s: RATE_M_S,
        bearing_deg: 90.0,
        spans: vec![SpoofSpan { start_s, end_s }],
    });
    spec
}

/// The receiver's latest fix less the truth, north-east-down metres, when
/// the last cycle delivered one.
fn fix_error_ned_m(trial: &ChainTrial) -> Option<DVec3> {
    let fix = trial.chain().board().last_samples().gnss?;
    let truth = trial.truth().state.position_ecef;
    let frame = LocalFrame::at(&ecef_to_geodetic(truth));
    Some(frame.ecef_to_ned_vector(geodetic_to_ecef(&fix.position) - truth))
}

#[test]
fn the_spoofer_drags_the_receivers_fixes_and_lets_go_at_once() {
    let mut trial = ChainTrial::new(&spoofed_from(10.0, Some(40.0)), &DirectoryStores).unwrap();
    let mut outputs = advanced(trial.step_until(10.0).unwrap());
    let honest = fix_error_ned_m(&trial).map_or(0.0, DVec3::length);
    assert!(honest < 10.0, "an unspoofed fix {honest} m off");
    let mut dragged = Vec::new();
    while trial.now_s() < 40.0 {
        outputs.extend(advanced(trial.step().unwrap()));
        if let Some(error) = fix_error_ned_m(&trial) {
            dragged.push((trial.now_s(), error));
        }
    }
    assert!(dragged.len() > 100, "{} fixes in the span", dragged.len());
    for (t_s, error) in &dragged {
        let expected_east_m = RATE_M_S * (t_s - 10.0);
        assert!(
            (error.y - expected_east_m).abs() < 10.0 && error.x.abs() < 10.0,
            "at {t_s} s the fix is {error} m off, {expected_east_m} m east expected"
        );
    }
    outputs.extend(advanced(trial.step_until(60.0).unwrap()));
    let released = fix_error_ned_m(&trial).map_or(0.0, DVec3::length);
    assert!(released < 10.0, "a released fix {released} m off");

    let events: Vec<_> = outputs
        .records
        .iter()
        .filter_map(|record| match record {
            ProgressRecord::Event(event) => Some((event.t_s, &event.kind)),
            _ => None,
        })
        .filter(|(_, kind)| {
            matches!(
                kind,
                EventKind::SpoofingStart { .. } | EventKind::SpoofingEnd { .. }
            )
        })
        .collect();
    assert!(matches!(
        events.as_slice(),
        [
            (
                start,
                EventKind::SpoofingStart {
                    sensor: SensorKind::Gnss,
                    ..
                }
            ),
            (
                end,
                EventKind::SpoofingEnd {
                    sensor: SensorKind::Gnss
                }
            )
        ] if *start == 10.0 && *end == 40.0
    ));
}

#[test]
fn a_switched_spoofer_flies_the_same_as_the_one_the_spec_states() {
    let mut by_hand = spec(None);
    by_hand.gnss_spoof = Some(GnssSpoofSpec {
        rate_m_s: RATE_M_S,
        bearing_deg: 90.0,
        spans: Vec::new(),
    });
    let mut switched = ChainTrial::new(&by_hand, &DirectoryStores).unwrap();
    let mut first = advanced(switched.step_until(20.0).unwrap());
    assert_eq!(switched.set_gnss_spoof(true), 20.0);
    first.extend(advanced(switched.step_until(30.0).unwrap()));
    assert_eq!(switched.set_gnss_spoof(false), 30.0);
    first.extend(advanced(switched.step_until(40.0).unwrap()));

    let mut stated = spec(None);
    stated.gnss_spoof = Some(switched.gnss_spoof().clone());
    assert_eq!(stated, spoofed_from(20.0, Some(30.0)));
    let mut replay = ChainTrial::new(&stated, &DirectoryStores).unwrap();
    let second = advanced(replay.step_until(40.0).unwrap());
    // The digest of the request differs; everything flown does not.
    assert_eq!(first.records.get(1..), second.records.get(1..));
    assert_eq!(first.mission, second.mission);
    assert_eq!(switched.truth(), replay.truth());
}

#[test]
fn a_spoofer_without_a_drag_is_refused() {
    let mut spec = spoofed_from(0.0, None);
    if let Some(spoof) = spec.gnss_spoof.as_mut() {
        spoof.rate_m_s = 0.0;
    }
    assert!(ChainTrial::new(&spec, &DirectoryStores).is_err());
}
