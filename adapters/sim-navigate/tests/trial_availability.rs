//! Scheduled unavailability: a schedule withholds a sensor's readings and
//! nothing else, so the filter at the end of an interval is the filter of a
//! run that lost the sensor at the interval's start; a set with every aid
//! withheld all the way is the inertial core alone; and the result counts
//! what was withheld apart from what the physics or the filter refused.

use std::sync::OnceLock;

use sim_navigate::NavMeans;
use sim_navigate::trial::{
    ComparisonRow, ImuGradeName, RecordedTrial, SensorKind, SensorSetSpec, SetAvailability,
    SetSelection, TrialOutcome, TrialRequest, evaluate, record_trial, replay_set,
};

#[cfg(test)]
#[path = "trial_support/requests.rs"]
mod requests;

/// The interval each fork test withholds its sensor over, seconds.
const INTERVAL: (f64, f64) = (60.0, 120.0);
/// Longer than any route here, seconds.
const WHOLE_ROUTE_S: f64 = 1.0e6;

#[cfg(test)]
fn init_tracing() {
    tracing_subscriber::fmt().with_test_writer().try_init().ok();
}

/// Three night minutes on a straight route with every sensor, star frames
/// every ten seconds, and schedules for the fork tests and for a set
/// without any aid all the way.
#[cfg(test)]
fn request() -> TrialRequest {
    use NavMeans::{Barometric, Celestial, DeadReckoning, Gnss};
    let all = [DeadReckoning, Barometric, Gnss, Celestial];
    let withheld_all = [SensorKind::Gnss, SensorKind::Baro, SensorKind::Stars]
        .into_iter()
        .fold(requests::set("withheld-all", &all), |set, sensor| {
            requests::withheld(set, sensor, &format!("no-{}", sensor.name()))
        });
    let sets = vec![
        requests::set("ins", &[DeadReckoning]),
        withheld_all,
        requests::set("ins+baro", &[DeadReckoning, Barometric]),
        requests::set("ins+baro+gnss", &[DeadReckoning, Barometric, Gnss]),
        requests::withheld(
            requests::set("ins+baro+stars", &[DeadReckoning, Barometric, Celestial]),
            SensorKind::Stars,
            "cloud",
        ),
    ];
    let mut request = requests::request(requests::straight(3.0), ImuGradeName::Tactical, sets);
    request.stars.period_s = 10.0;
    request.availability = vec![
        requests::schedule("no-gnss", SensorKind::Gnss, 0.0, WHOLE_ROUTE_S),
        requests::schedule("no-baro", SensorKind::Baro, 0.0, WHOLE_ROUTE_S),
        requests::schedule("no-stars", SensorKind::Stars, 0.0, WHOLE_ROUTE_S),
        requests::schedule("baro-out", SensorKind::Baro, INTERVAL.0, INTERVAL.1),
        requests::schedule("cloud", SensorKind::Stars, INTERVAL.0, INTERVAL.1),
    ];
    request
}

/// The recorded trial and its evaluation, shared by the tests that read
/// them.
#[cfg(test)]
fn run() -> &'static (TrialRequest, RecordedTrial, TrialOutcome) {
    static RUN: OnceLock<(TrialRequest, RecordedTrial, TrialOutcome)> = OnceLock::new();
    RUN.get_or_init(|| {
        let request = request();
        let recorded = record_trial(&request).unwrap();
        let outcome = evaluate(&request, &recorded).unwrap();
        (request, recorded, outcome)
    })
}

#[cfg(test)]
fn step(seconds: f64) -> usize {
    (seconds * 120.0).round() as usize
}

/// Fork equality for `sensor`: `set` run under `schedule` and forked at the
/// interval's start into a run without the sensor's means; at the
/// interval's end both hold the same filter, bit for bit.
#[cfg(test)]
fn assert_fork_equal(set: &SensorSetSpec, sensor: SensorKind, schedule: &str) {
    let (request, recorded, _) = run();
    let mut scheduled_set = set.clone();
    scheduled_set.availability.insert(sensor, schedule.into());
    let mut run = replay_set(request, &scheduled_set, &recorded.log).unwrap();
    run.advance_to(&recorded.log, step(INTERVAL.0)).unwrap();
    let mut fork = run.fork(SetSelection {
        means: set.ablation_set().without(sensor.means()),
        availability: SetAvailability::NONE,
    });
    run.advance_to(&recorded.log, step(INTERVAL.1)).unwrap();
    fork.advance_to(&recorded.log, step(INTERVAL.1)).unwrap();
    let scheduled = run.solution(&recorded.log).unwrap().unwrap();
    let forked = fork.solution(&recorded.log).unwrap().unwrap();
    assert_eq!(
        scheduled, forked,
        "{sensor:?}: the scheduled run and the fork differ"
    );
    let (_, inside) = &run.interval_tallies()[0];
    tracing::info!(?sensor, ?inside, "tallies inside the interval");
    let tally = match sensor {
        SensorKind::Baro => &inside.baro,
        _ => &inside.stars,
    };
    assert!(
        tally.scheduled_unavailable > 0 && tally.offered == 0,
        "{inside:?}"
    );
}

#[test]
fn a_baro_interval_leaves_the_filter_of_a_run_that_lost_baro() {
    init_tracing();
    use NavMeans::{Barometric, DeadReckoning, Gnss};
    let set = requests::set("ins+baro+gnss", &[DeadReckoning, Barometric, Gnss]);
    assert_fork_equal(&set, SensorKind::Baro, "baro-out");
}

#[test]
fn a_star_interval_leaves_the_filter_of_a_run_that_lost_stars() {
    init_tracing();
    use NavMeans::{Barometric, Celestial, DeadReckoning};
    let set = requests::set("ins+baro+stars", &[DeadReckoning, Barometric, Celestial]);
    assert_fork_equal(&set, SensorKind::Stars, "cloud");
}

/// Every aid withheld for the whole route is the inertial core alone, bit
/// for bit, and every withheld reading is counted as scheduled.
#[test]
fn a_set_with_every_aid_withheld_is_the_ins_alone() {
    init_tracing();
    let (_, _, outcome) = run();
    let report = &outcome.report;
    assert_eq!(outcome.estimates[0], outcome.estimates[1]);
    let (ins, withheld) = (&report.sensor_sets[0], &report.sensor_sets[1]);
    assert_eq!(ins.series, withheld.series);
    let tallies = &withheld.observations;
    tracing::info!(?tallies, "every aid withheld");
    for tally in [&tallies.gnss_position, &tallies.baro, &tallies.stars] {
        assert_eq!(tally.offered, 0, "{tallies:?}");
        assert!(tally.scheduled_unavailable > 0, "{tallies:?}");
    }
    assert_eq!(withheld.stars.unwrap().frames_attempted, 0);
}

/// The cloud interval withholds frames the night sky would have solved;
/// outside it the tracker solves again, and the interval's report counts
/// only scheduled frames.
#[test]
fn the_result_reports_each_interval_apart() {
    init_tracing();
    let (_, _, outcome) = run();
    let clouded = &outcome.report.sensor_sets[4];
    let interval = &clouded.summary.unavailable[0];
    tracing::info!(?interval, stars = ?clouded.stars, "cloud interval");
    assert_eq!(interval.sensor, SensorKind::Stars);
    assert_eq!(interval.schedule, "cloud");
    assert!(interval.errors.is_some());
    assert!(
        interval.tallies.stars.scheduled_unavailable >= 5,
        "{interval:?}"
    );
    assert_eq!(interval.tallies.stars.offered, 0);
    assert!(interval.tallies.baro.accepted > 0, "{interval:?}");
    assert!(
        clouded.stars.unwrap().frames_solved > 5,
        "{:?}",
        clouded.stars
    );
}

/// A fused set is compared with the best single-means set it contains that
/// withholds the shared sensors alike; a single-means set with the INS
/// alone; a set whose schedules no candidate shares has no baseline.
#[test]
fn the_comparison_table_picks_like_for_like_baselines() {
    init_tracing();
    let (_, _, outcome) = run();
    let rows = &outcome.report.comparison.rows;
    let row = |set: &str| -> &ComparisonRow { rows.iter().find(|r| r.set == set).unwrap() };
    for r in rows {
        tracing::info!(set = r.set, rms = r.horizontal_rms_m, above_s = r.time_above_threshold_s, versus = ?r.versus, "row");
    }
    assert_eq!(row("ins").versus, None);
    assert_eq!(row("withheld-all").versus, None);
    assert_eq!(row("ins+baro").versus.as_ref().unwrap().set, "ins");
    let fused = row("ins+baro+gnss").versus.as_ref().unwrap();
    assert_eq!(fused.set, "ins+baro");
    let expected = row("ins+baro+gnss").horizontal_rms_m - row("ins+baro").horizontal_rms_m;
    assert_eq!(fused.horizontal_rms_change_m, expected);
    // Clouded stars have no single-means peer clouded alike, so the best
    // like-for-like single-means set is the barometric one.
    let clouded = row("ins+baro+stars").versus.as_ref().unwrap();
    assert_eq!(clouded.set, "ins+baro");
}
