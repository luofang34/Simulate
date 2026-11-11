//! The trial's estimates come from its sensors alone, its result file is a
//! function of its request, and both its request and result read back as
//! written.

use glam::{DQuat, DVec3};
use sim_geo::{LocalFrame, ecef_to_geodetic};
use sim_navigate::NavMeans;
use sim_navigate::Truth;
use sim_navigate::trial::{
    ImuGradeName, ProgressRecord, TrialReport, TrialRequest, evaluate, evaluate_with_progress,
    record_trial, run_trial,
};

#[cfg(test)]
#[path = "trial_support/requests.rs"]
mod requests;

/// How far the scoring truth is moved, metres.
const OFFSET_M: f64 = 1_000.0;
/// How far the scoring truth is turned, radians.
const TURN_RAD: f64 = 0.05;

#[cfg(test)]
fn init_tracing() {
    tracing_subscriber::fmt().with_test_writer().try_init().ok();
}

/// GNSS with an outage and stars, so every aid and the tracker's prior are
/// exercised, over three straight minutes.
#[cfg(test)]
fn request() -> TrialRequest {
    use NavMeans::{Barometric, Celestial, DeadReckoning, Gnss};
    let sets = vec![
        requests::jammed_set(
            "gnss-outage+baro+ins+stars",
            &[DeadReckoning, Barometric, Gnss, Celestial],
        ),
        requests::set("ins+baro", &[DeadReckoning, Barometric]),
    ];
    let mut request = requests::request(requests::straight(3.0), ImuGradeName::Tactical, sets);
    request.stars.period_s = 20.0;
    request.availability = vec![requests::gnss_jammed(60.0, 120.0)];
    request
}

/// The request's recorded trial, shared by the tests that replay it.
#[cfg(test)]
fn recorded() -> &'static sim_navigate::trial::RecordedTrial {
    static RECORDED: std::sync::OnceLock<sim_navigate::trial::RecordedTrial> =
        std::sync::OnceLock::new();
    RECORDED.get_or_init(|| record_trial(&request()).unwrap())
}

/// Truth the navigator must never see, moved 1 km north and turned 0.05 rad
/// about the body's down axis at every step, against estimates replayed from
/// the same sensor log: the estimates are bit-identical and only the scores
/// move.
#[test]
fn moving_the_truth_changes_scores_and_never_estimates() {
    init_tracing();
    let request = request();
    let recorded = recorded();
    let mut moved = recorded.clone();
    let start = ecef_to_geodetic(recorded.flight.truth[0].get().1.position_ecef);
    let north = LocalFrame::at(&start).north() * OFFSET_M;
    let turn = DQuat::from_axis_angle(DVec3::Z, TURN_RAD);
    for sample in &mut moved.flight.truth {
        let (at, mut state) = *sample.get();
        state.position_ecef += north;
        state.attitude = (state.attitude * turn).normalize();
        *sample = Truth::new((at, state));
    }
    let original = evaluate(&request, recorded).unwrap();
    let perturbed = evaluate(&request, &moved).unwrap();
    assert_eq!(original.estimates, perturbed.estimates);
    for (a, b) in original
        .report
        .sensor_sets
        .iter()
        .zip(&perturbed.report.sensor_sets)
    {
        tracing::info!(
            set = a.name,
            original_m = a.summary.horizontal_error_m.rms,
            moved_m = b.summary.horizontal_error_m.rms,
            original_attitude_mrad = a.summary.attitude_error_rad.rms * 1e3,
            moved_attitude_mrad = b.summary.attitude_error_rad.rms * 1e3,
            "scores against moved truth"
        );
        assert!(b.summary.horizontal_error_m.p50 > 0.9 * OFFSET_M, "{b:?}");
        assert!(b.summary.attitude_error_rad.p50 > 0.9 * TURN_RAD, "{b:?}");
        assert_eq!(a.observations, b.observations);
        assert_eq!(a.stars, b.stars);
    }
    let stars = original.report.sensor_sets[0].stars.unwrap();
    assert!(stars.frames_solved > 1, "{stars:?}");
}

/// The progress stream carries truth beside each estimate, written from the
/// scoring side: with a sink taking every record, moving the truth moves
/// only the stream's truth and the scores, never an estimate.
#[test]
fn progress_carries_truth_without_feeding_it_to_the_navigator() {
    init_tracing();
    // Scored every two seconds against the default one-second progress
    // period: every scored sample is a progress sample.
    let mut request = request();
    request.output.sample_period_s = 2.0;
    let recorded = recorded();
    let mut moved = recorded.clone();
    let start = ecef_to_geodetic(recorded.flight.truth[0].get().1.position_ecef);
    let north = LocalFrame::at(&start).north() * OFFSET_M;
    for sample in &mut moved.flight.truth {
        let (at, mut state) = *sample.get();
        state.position_ecef += north;
        *sample = Truth::new((at, state));
    }
    let watched = |recorded| {
        let mut samples = Vec::new();
        let mut sink = |record: &ProgressRecord| -> std::io::Result<()> {
            if let ProgressRecord::Sample(sample) = record {
                samples.push(sample.clone());
            }
            Ok(())
        };
        let outcome = evaluate_with_progress(&request, recorded, &mut sink).unwrap();
        (outcome, samples)
    };
    let (original, original_samples) = watched(recorded);
    let (perturbed, perturbed_samples) = watched(&moved);
    assert_eq!(original.estimates, perturbed.estimates);
    assert_eq!(
        original.estimates,
        evaluate(&request, recorded).unwrap().estimates
    );
    assert_eq!(original_samples.len(), perturbed_samples.len());
    let scored: usize = original
        .report
        .sensor_sets
        .iter()
        .map(|s| s.series.len())
        .sum();
    assert_eq!(original_samples.len(), scored);
    assert!(original_samples.len() > 150, "{}", original_samples.len());
    for (a, b) in original_samples.iter().zip(&perturbed_samples) {
        assert_eq!((a.t_s, &a.set, a.estimate), (b.t_s, &b.set, b.estimate));
        assert_ne!(a.truth, b.truth);
        assert!(b.horizontal_error_m != a.horizontal_error_m, "{a:?} {b:?}");
    }
}

#[test]
fn the_same_request_writes_the_same_bytes() {
    init_tracing();
    let request = request();
    let first = run_trial(&request).unwrap().to_json().unwrap();
    let second = run_trial(&request).unwrap().to_json().unwrap();
    assert!(first == second, "the result files differ");
}

#[test]
fn the_request_and_result_read_back_as_written() {
    init_tracing();
    let mut request = request();
    request.availability[0].unavailable[0].reason = Some("jamming".into());
    let text = request.to_json().unwrap();
    assert_eq!(TrialRequest::from_json(&text).unwrap(), request);

    let mut short = request.clone();
    short.waypoints = requests::straight(1.0);
    short.availability[0].unavailable.clear();
    let report = run_trial(&short).unwrap();
    let written = report.to_json().unwrap();
    let read = TrialReport::from_json(&written).unwrap();
    assert_eq!(read, report);
    assert_eq!(read.to_json().unwrap(), written);
}

#[test]
fn a_request_with_an_unsimulated_means_is_refused() {
    let mut request = request();
    request.sensor_sets[1].means.push(NavMeans::Visual);
    assert!(request.validate().is_err());
    request.sensor_sets[1].means = vec![NavMeans::Barometric];
    assert!(request.validate().is_err());
}
