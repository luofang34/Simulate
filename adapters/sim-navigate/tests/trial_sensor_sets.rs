//! The trial's sensor sets against truth: GNSS-aided, INS alone, and a GNSS
//! outage, all through Navigate's one inertial filter.

use std::sync::OnceLock;

use sim_navigate::trial::{
    GnssSpec, ImuGradeName, OutageReport, OutageSpec, RecordedTrial, SetReport, SetRun,
    SetSelection, TrialReport, TrialRequest, evaluate, record_trial, trial_inertial_config,
};
use sim_navigate::{AblationSet, NavMeans, compare_with_truth};

#[cfg(test)]
#[path = "trial_support/consistency.rs"]
mod consistency;
#[cfg(test)]
#[path = "trial_support/requests.rs"]
mod requests;

/// The NEES band a GNSS-aided run must average in: the mean over the run's
/// 1 Hz scores of the 3-DOF position NEES, the convention the inertial tests
/// use.
const AIDED_NEES: std::ops::RangeInclusive<f64> = 0.5..=6.0;
/// Time after GNSS returns within which the error must be back at the
/// GNSS-aided level, seconds.
const SETTLING_S: f64 = 60.0;
/// The outage: GNSS denied from t1 to t2 inclusive, seconds of route time.
const OUTAGE: OutageSpec = OutageSpec {
    start_s: 90.0,
    end_s: 240.0,
};
/// Seeds per grade for the unaided ensemble.
const ENSEMBLE: u64 = 4;

#[cfg(test)]
fn init_tracing() {
    tracing_subscriber::fmt().with_test_writer().try_init().ok();
}

#[cfg(test)]
fn log_set(set: &SetReport) {
    let s = &set.summary;
    tracing::info!(
        set = set.name,
        horizontal_rms_m = s.horizontal_error_m.rms,
        horizontal_p95_m = s.horizontal_error_m.p95,
        horizontal_last_m = s.horizontal_error_m.last,
        vertical_rms_m = s.vertical_error_m.rms,
        velocity_rms_m_s = s.velocity_error_m_s.rms,
        attitude_rms_mrad = s.attitude_error_rad.rms * 1e3,
        position_nees = s.mean_position_nees,
        attitude_nees = s.mean_attitude_nees,
        outages = ?s.outages,
        observations = ?set.observations,
        "sensor set"
    );
}

#[cfg(test)]
fn set<'a>(report: &'a TrialReport, name: &str) -> &'a SetReport {
    report.sensor_sets.iter().find(|s| s.name == name).unwrap()
}

/// The GNSS-aided reference and the same sets with the outage, on the
/// tactical grade over the four-waypoint route, with `gnss`.
#[cfg(test)]
fn outage_request(gnss: GnssSpec) -> TrialRequest {
    use NavMeans::{Barometric, DeadReckoning, Gnss};
    let sets = vec![
        requests::set("gnss+baro+ins", &[DeadReckoning, Barometric, Gnss], false),
        requests::set("outage", &[DeadReckoning, Barometric, Gnss], true),
    ];
    let mut request = requests::request(requests::four_waypoints(), ImuGradeName::Tactical, sets);
    request.outages = vec![OUTAGE];
    request.gnss = gnss;
    request
}

/// The default receiver's run, shared by the tests that read it.
#[cfg(test)]
fn default_run() -> &'static (TrialRequest, RecordedTrial, TrialReport) {
    static RUN: OnceLock<(TrialRequest, RecordedTrial, TrialReport)> = OnceLock::new();
    RUN.get_or_init(|| {
        let request = outage_request(GnssSpec::default());
        let recorded = record_trial(&request).unwrap();
        let report = evaluate(&request, &recorded).unwrap().report;
        log_set(set(&report, "gnss+baro+ins"));
        log_set(set(&report, "outage"));
        (request, recorded, report)
    })
}

#[cfg(test)]
fn the_outage(report: &TrialReport) -> OutageReport {
    let outages = &set(report, "outage").summary.outages;
    assert_eq!(outages.len(), 1, "{outages:?}");
    outages[0]
}

#[test]
fn gnss_aiding_holds_the_receiver_error() {
    init_tracing();
    let (_, _, report) = default_run();
    let aided = set(report, "gnss+baro+ins");
    let bound = 2.0 * report.gnss_horizontal_sigma_m;
    let rms = aided.summary.horizontal_error_m.rms;
    assert!(rms <= bound, "{rms} > {bound}");
    assert_eq!(aided.observations.gnss_position.rejected, 0);
}

/// The same runs with the receiver's correlated term set to zero in the
/// request, shared by the tests that read it.
#[cfg(test)]
fn white_run() -> &'static TrialReport {
    static RUN: OnceLock<TrialReport> = OnceLock::new();
    RUN.get_or_init(|| {
        let white = GnssSpec {
            horizontal_correlated_sigma_m: 0.0,
            vertical_correlated_sigma_m: 0.0,
            ..GnssSpec::default()
        };
        let request = outage_request(white);
        let report = evaluate(&request, &record_trial(&request).unwrap())
            .unwrap()
            .report;
        log_set(set(&report, "gnss+baro+ins"));
        log_set(set(&report, "outage"));
        report
    })
}

/// Pipeline validity: with a white-noise receiver, the only error Navigate's
/// filter can represent, it stays consistent with GNSS and across the
/// outage.
#[test]
fn the_pipeline_is_consistent_with_a_white_noise_receiver() {
    init_tracing();
    let report = white_run();
    let nees = set(report, "gnss+baro+ins").summary.mean_position_nees;
    let outage = the_outage(report);
    assert!(AIDED_NEES.contains(&nees), "{nees}");
    assert!(
        AIDED_NEES.contains(&outage.mean_position_nees),
        "{outage:?}"
    );
}

/// The result flags the realistic receiver's overconfidence, which Navigate's
/// filter cannot avoid without a state for the correlated error, and does
/// not flag the white receiver.
#[test]
fn the_consistency_note_flags_the_correlated_receiver_only() {
    init_tracing();
    let realistic = &set(&default_run().2, "gnss+baro+ins").summary.consistency;
    let white = &set(white_run(), "gnss+baro+ins").summary.consistency;
    tracing::info!(?realistic, ?white, "consistency notes");
    assert_eq!(realistic.verdict, "overconfident", "{realistic:?}");
    assert_eq!(white.verdict, "consistent", "{white:?}");
}

/// The ensemble-mean position NEES and RMS final horizontal error of the INS
/// alone over a straight `minutes` on `grade`, the seeds run in parallel.
#[cfg(test)]
fn unaided(grade: ImuGradeName, minutes: f64, seeds: u64) -> (f64, f64, Vec<SetReport>) {
    let request = requests::request(
        requests::straight(minutes),
        grade,
        vec![requests::set("ins", &[NavMeans::DeadReckoning], false)],
    );
    let sets: Vec<SetReport> = std::thread::scope(|scope| {
        let handles: Vec<_> = (0..seeds)
            .map(|seed| {
                let mut request = request.clone();
                request.seeds.imu = 0x3_0000 + seed;
                request.seeds.alignment = seed;
                scope.spawn(move || {
                    let recorded = record_trial(&request).unwrap();
                    let flight_seconds = recorded.flight.truth.len() as f64 / 120.0;
                    assert!(flight_seconds >= 60.0 * minutes, "{flight_seconds}");
                    let report = evaluate(&request, &recorded).unwrap().report;
                    let ins = set(&report, "ins").clone();
                    log_set(&ins);
                    ins
                })
            })
            .collect();
        handles.into_iter().map(|h| h.join().unwrap()).collect()
    });
    let count = sets.len() as f64;
    let mean_nees = sets
        .iter()
        .map(|s| s.summary.mean_position_nees)
        .sum::<f64>()
        / count;
    let rms_final = (sets
        .iter()
        .map(|s| s.summary.horizontal_error_m.last.powi(2))
        .sum::<f64>()
        / count)
        .sqrt();
    tracing::info!(?grade, minutes, mean_nees, rms_final, "INS alone, straight");
    (mean_nees, rms_final, sets)
}

#[test]
fn the_tactical_and_navigation_ins_drift_consistently_for_ten_minutes() {
    init_tracing();
    let (tactical_nees, tactical_final, _) = unaided(ImuGradeName::Tactical, 10.0, ENSEMBLE);
    let (navigation_nees, navigation_final, _) = unaided(ImuGradeName::Navigation, 10.0, ENSEMBLE);
    for nees in [tactical_nees, navigation_nees] {
        assert!(consistency::DEAD_RECKONING_NEES.contains(&nees), "{nees}");
    }
    assert!(navigation_final < tactical_final);
}

/// A consumer part's 0.5 °/s gyro bias turns its attitude by tens of
/// degrees within a minute, out of the filter's small-angle regime, so no
/// NEES band applies; the consistency note must never call such a run
/// consistent.
#[test]
fn a_consumer_ins_past_the_small_angle_limit_is_never_called_consistent() {
    init_tracing();
    let (_, _, sets) = unaided(ImuGradeName::ConsumerMems, 1.0, ENSEMBLE);
    let past_limit: Vec<_> = sets
        .iter()
        .map(|set| &set.summary.consistency)
        .filter(|note| note.max_attitude_error_rad > 0.1)
        .collect();
    assert!(!past_limit.is_empty());
    for note in past_limit {
        assert_ne!(note.verdict, "consistent", "{note:?}");
    }
}

/// Ten unaided minutes turn a consumer part's attitude by radians, outside
/// an error-state filter's regime; the result says so.
#[test]
fn the_result_flags_a_diverged_consumer_ins() {
    init_tracing();
    let (_, _, sets) = unaided(ImuGradeName::ConsumerMems, 10.0, 1);
    let note = &sets[0].summary.consistency;
    tracing::info!(?note, "consumer, ten minutes");
    assert_eq!(note.verdict, "diverged", "{note:?}");
}

/// During the outage the filter runs exactly as an `ins+baro` filter forked
/// from the outage run's own state at t1; its covariance grows; and after
/// GNSS returns the error is back at the aided level.
#[test]
fn an_outage_leaves_the_filter_unaided_until_gnss_returns() {
    init_tracing();
    use NavMeans::{Barometric, DeadReckoning, Gnss};
    let (request, recorded, report) = default_run();
    let outage = the_outage(report);
    let selection = |means: &[NavMeans], outages: Vec<OutageSpec>| SetSelection {
        means: means.iter().fold(AblationSet::EMPTY, |s, m| s.with(*m)),
        outages,
    };
    let config = trial_inertial_config(&request.imu_grade.grade().spec(), &request.baro);
    let mut run = SetRun::new(
        &recorded.log,
        selection(&[DeadReckoning, Barometric, Gnss], vec![OUTAGE]),
        config,
        None,
    )
    .unwrap();
    let step = |seconds: f64| (seconds * 120.0).round() as usize;
    run.advance_to(&recorded.log, step(OUTAGE.start_s)).unwrap();
    let mut fork = run.fork(selection(&[DeadReckoning, Barometric], Vec::new()));
    fork.advance_to(&recorded.log, step(OUTAGE.end_s)).unwrap();
    let solution = fork.solution(&recorded.log).unwrap().unwrap();
    let truth = recorded.flight.truth[step(OUTAGE.end_s)].get().1;
    let forked = compare_with_truth(solution.get(), truth.position_ecef).unwrap();
    let ratio = outage.horizontal_error_at_end_m / forked.horizontal_error_m;
    tracing::info!(
        outage_end_m = outage.horizontal_error_at_end_m,
        forked_end_m = forked.horizontal_error_m,
        ratio,
        "unaided during the outage"
    );
    assert!((0.95..=1.05).contains(&ratio), "{ratio}");

    let series = &set(report, "outage").series;
    let at = |t: f64| series.iter().find(|s| (s.time_s - t).abs() < 1e-6).unwrap();
    let (start, end) = (at(OUTAGE.start_s), at(OUTAGE.end_s));
    assert!(
        end.horizontal_sigma_m > 2.0 * start.horizontal_sigma_m,
        "{start:?} {end:?}"
    );
    let settled = at(OUTAGE.end_s + SETTLING_S);
    tracing::info!(?settled, "after GNSS returns");
    assert!(
        settled.horizontal_error_m <= 2.0 * report.gnss_horizontal_sigma_m,
        "{settled:?}"
    );
}
