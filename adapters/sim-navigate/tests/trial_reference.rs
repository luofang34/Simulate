//! The open-loop reference result, guarded portably.
//!
//! A trial is bit-reproducible on one platform but not across platforms:
//! Navigate and this adapter use the platform's floating-point functions,
//! and some targets fuse multiply-adds. So the result is guarded two ways.
//! Within one process, every code path that must not change it (the
//! recorded and the one-call path, progress on or off, a set alone or
//! interleaved with others) is compared byte for byte. Against stored
//! numbers, each set's summary is compared within tolerances that a
//! last-bit difference cannot cross but any change to a model, the filter
//! or the scoring would.

use std::sync::OnceLock;

use sim_navigate::trial::{
    ImuGradeName, ProgressRecord, RecordedTrial, SetReport, TrialReport, TrialRequest, evaluate,
    record_trial, run_trial_with_progress,
};

#[cfg(test)]
#[path = "trial_support/requests.rs"]
mod requests;

/// Relative tolerance for a set with GNSS: its errors are held to the
/// receiver's by every fix, so platform rounding differences stay at the
/// rounding level and cannot grow.
const AIDED_RELATIVE: f64 = 1e-6;
/// Relative tolerance for a set without GNSS: inertial drift integrates
/// and amplifies a last-bit difference over the minute flown, so its
/// errors may differ further, though still far below what any change of
/// model would move them.
const UNAIDED_RELATIVE: f64 = 1e-4;

/// The stored summary of one set: horizontal RMS, p95 and final error,
/// metres; mean position NEES; horizontal error at the end of the GNSS
/// outage, metres, for sets honouring it; and the exact counts: GNSS
/// position and baro observations offered and accepted, GNSS fixes
/// withheld, and star frames solved.
struct Reference {
    set: &'static str,
    horizontal_rms_m: f64,
    horizontal_p95_m: f64,
    horizontal_final_m: f64,
    mean_position_nees: f64,
    outage_end_m: Option<f64>,
    counts: [u64; 6],
}

/// Measured on macOS arm64.
const REFERENCE: [Reference; 6] = [
    Reference {
        set: "gnss+baro+ins",
        horizontal_rms_m: 1.6053039565268463,
        horizontal_p95_m: 1.799634131433854,
        horizontal_final_m: 1.8005993249627634,
        mean_position_nees: 81.7797562381478,
        outage_end_m: None,
        counts: [68, 68, 342, 342, 0, 0],
    },
    Reference {
        set: "ins",
        horizontal_rms_m: 3.347361330664099,
        horizontal_p95_m: 4.465702561986841,
        horizontal_final_m: 4.759341771771265,
        mean_position_nees: 1.283103176367603,
        outage_end_m: None,
        counts: [0, 0, 0, 0, 0, 0],
    },
    Reference {
        set: "ins+baro",
        horizontal_rms_m: 3.431286860734899,
        horizontal_p95_m: 4.510381238079001,
        horizontal_final_m: 4.410614757438282,
        mean_position_nees: 0.41688483790687875,
        outage_end_m: None,
        counts: [0, 0, 342, 342, 0, 0],
    },
    Reference {
        set: "ins+baro+stars",
        horizontal_rms_m: 3.7040039602992074,
        horizontal_p95_m: 8.242053382184928,
        horizontal_final_m: 9.269802124006194,
        mean_position_nees: 0.33363263445849767,
        outage_end_m: None,
        counts: [0, 0, 342, 342, 0, 7],
    },
    Reference {
        set: "gnss-outage+baro+ins",
        horizontal_rms_m: 1.7456875341311218,
        horizontal_p95_m: 2.3764719402643877,
        horizontal_final_m: 1.8279554716664201,
        mean_position_nees: 45.70351118758452,
        outage_end_m: Some(2.6447396414980533),
        counts: [47, 47, 342, 342, 21, 0],
    },
    Reference {
        set: "gnss-outage+baro+ins+stars",
        horizontal_rms_m: 1.7952139827711064,
        horizontal_p95_m: 2.50561212198565,
        horizontal_final_m: 1.8361954377496854,
        mean_position_nees: 46.729106707608665,
        outage_end_m: Some(2.7819512099383283),
        counts: [47, 47, 342, 342, 21, 7],
    },
];

#[cfg(test)]
fn init_tracing() {
    tracing_subscriber::fmt().with_test_writer().try_init().ok();
}

/// One minute east at night with every reference set, star frames every
/// ten seconds, and GNSS jammed from 20 to 40 s for the outage sets.
#[cfg(test)]
fn reference_request() -> TrialRequest {
    let mut request = requests::request(
        requests::straight(1.0),
        ImuGradeName::Tactical,
        requests::reference_sets(),
    );
    request.stars.period_s = 10.0;
    request.availability = vec![requests::gnss_jammed(20.0, 40.0)];
    request
}

/// The reference request's recording and evaluated report, shared by the
/// tests that read them.
#[cfg(test)]
fn reference() -> &'static (TrialRequest, RecordedTrial, TrialReport) {
    static RUN: OnceLock<(TrialRequest, RecordedTrial, TrialReport)> = OnceLock::new();
    RUN.get_or_init(|| {
        let request = reference_request();
        let recorded = record_trial(&request).unwrap();
        let report = evaluate(&request, &recorded).unwrap().report;
        (request, recorded, report)
    })
}

#[cfg(test)]
fn counts(set: &SetReport) -> [u64; 6] {
    let o = &set.observations;
    [
        o.gnss_position.offered,
        o.gnss_position.accepted,
        o.baro.offered,
        o.baro.accepted,
        o.gnss_position.scheduled_unavailable,
        set.stars.map_or(0, |s| s.frames_solved),
    ]
}

#[cfg(test)]
fn outage_end_m(set: &SetReport) -> Option<f64> {
    let interval = set.summary.unavailable.first()?;
    interval.errors.map(|e| e.horizontal_error_at_end_m)
}

/// Prints `sets` as the reference table's source, for retaking it.
#[cfg(test)]
fn print_reference(sets: &[SetReport]) {
    for set in sets {
        let h = &set.summary.horizontal_error_m;
        tracing::info!(
            "Reference {{ set: {:?}, horizontal_rms_m: {:e}, horizontal_p95_m: {:e}, horizontal_final_m: {:e}, mean_position_nees: {:e}, outage_end_m: {:?}, counts: {:?} }},",
            set.name,
            h.rms,
            h.p95,
            h.last,
            set.summary.mean_position_nees,
            outage_end_m(set),
            counts(set)
        );
    }
}

#[cfg(test)]
fn assert_close(set: &str, what: &str, measured: f64, stored: f64, relative: f64) {
    let difference = (measured - stored).abs() / stored.abs().max(f64::MIN_POSITIVE);
    assert!(
        difference <= relative,
        "{set} {what}: measured {measured:e}, stored {stored:e}, relative difference {difference:e} beyond {relative:e}"
    );
}

/// Each set's summary matches the stored numbers: counts exactly, errors
/// within the tolerance of its class.
#[test]
fn the_reference_result_matches_its_stored_numbers() {
    init_tracing();
    let (_, _, report) = reference();
    print_reference(&report.sensor_sets);
    assert_eq!(report.sensor_sets.len(), REFERENCE.len());
    for (set, stored) in report.sensor_sets.iter().zip(&REFERENCE) {
        assert_eq!(set.name, stored.set);
        assert_eq!(counts(set), stored.counts, "{}", set.name);
        let relative = if set.means.contains(&sim_navigate::NavMeans::Gnss) {
            AIDED_RELATIVE
        } else {
            UNAIDED_RELATIVE
        };
        let h = &set.summary.horizontal_error_m;
        let name = set.name.as_str();
        assert_close(
            name,
            "horizontal RMS",
            h.rms,
            stored.horizontal_rms_m,
            relative,
        );
        assert_close(
            name,
            "horizontal p95",
            h.p95,
            stored.horizontal_p95_m,
            relative,
        );
        assert_close(
            name,
            "horizontal final",
            h.last,
            stored.horizontal_final_m,
            relative,
        );
        let nees = set.summary.mean_position_nees;
        assert_close(name, "mean NEES", nees, stored.mean_position_nees, relative);
        match (outage_end_m(set), stored.outage_end_m) {
            (Some(measured), Some(stored)) => {
                assert_close(name, "outage end", measured, stored, UNAIDED_RELATIVE);
            }
            (measured, stored) => assert_eq!(measured, stored, "{name}"),
        }
    }
}

/// The paths that must not change an open-loop result, compared byte for
/// byte in one process: the one-call path with a progress sink taking
/// every record against recording then evaluating, and each set replayed
/// alone over the same recording against the set interleaved with the
/// others.
#[test]
fn every_open_loop_path_writes_the_same_bytes() {
    init_tracing();
    let (request, recorded, evaluated) = reference();
    let mut records = 0_u64;
    let mut sink = |_: &ProgressRecord| -> std::io::Result<()> {
        records = records.wrapping_add(1);
        Ok(())
    };
    let watched = run_trial_with_progress(request, &mut sink).unwrap();
    assert!(records > 60, "{records}");
    assert!(watched.to_json().unwrap() == evaluated.to_json().unwrap());
    let bytes = |set: &SetReport| serde_json::to_string(set).unwrap();
    for (index, interleaved) in evaluated.sensor_sets.iter().enumerate() {
        let mut alone = request.clone();
        alone.sensor_sets = vec![request.sensor_sets[index].clone()];
        let report = evaluate(&alone, recorded).unwrap().report;
        assert!(
            bytes(&report.sensor_sets[0]) == bytes(interleaved),
            "{} alone differs",
            interleaved.name
        );
    }
}
