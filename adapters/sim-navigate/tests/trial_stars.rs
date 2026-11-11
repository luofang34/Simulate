//! Star sightings in the trial: Navigate's tracker on the roof camera bounds
//! attitude at night, and the Sun-lit sky leaves it nothing to solve at noon.

use sim_navigate::NavMeans;
use sim_navigate::trial::{
    ImuGradeName, SetReport, TrialOutcome, TrialRequest, evaluate, record_trial,
};

#[cfg(test)]
#[path = "trial_support/consistency.rs"]
mod consistency;
#[cfg(test)]
#[path = "trial_support/requests.rs"]
mod requests;

/// Seconds between star frames: sparse enough for a debug build.
const STAR_PERIOD_S: f64 = 10.0;
/// How much worse star aiding may leave the horizontal error.
const STARS_HORIZONTAL_TOLERANCE: f64 = 1.1;
/// Solved frames at noon that still count as none: one in twenty.
const NOON_SOLVED_FRACTION: f64 = 0.05;

#[cfg(test)]
fn init_tracing() {
    tracing_subscriber::fmt().with_test_writer().try_init().ok();
}

#[cfg(test)]
fn star_request(epoch_utc: &str) -> TrialRequest {
    use NavMeans::{Barometric, Celestial, DeadReckoning};
    let sets = vec![
        requests::set("ins+baro", &[DeadReckoning, Barometric], false),
        requests::set(
            "ins+baro+stars",
            &[DeadReckoning, Barometric, Celestial],
            false,
        ),
    ];
    let mut request = requests::request(requests::four_waypoints(), ImuGradeName::Tactical, sets);
    request.epoch_utc = epoch_utc.into();
    request.stars.period_s = STAR_PERIOD_S;
    request
}

#[cfg(test)]
fn run(request: &TrialRequest) -> (TrialOutcome, Vec<f64>) {
    let recorded = record_trial(request).unwrap();
    let sun_elevations = recorded
        .log
        .stars
        .iter()
        .filter_map(|frame| frame.truth.get().sky.map(|sky| sky.sun_elevation_deg))
        .collect();
    (evaluate(request, &recorded).unwrap(), sun_elevations)
}

#[cfg(test)]
fn log(set: &SetReport, sun_elevations: &[f64]) {
    let s = &set.summary;
    tracing::info!(
        set = set.name,
        horizontal_rms_m = s.horizontal_error_m.rms,
        horizontal_last_m = s.horizontal_error_m.last,
        attitude_rms_mrad = s.attitude_error_rad.rms * 1e3,
        attitude_last_mrad = s.attitude_error_rad.last * 1e3,
        position_nees = s.mean_position_nees,
        attitude_nees = s.mean_attitude_nees,
        stars = ?set.stars,
        star_observations = ?set.observations.stars,
        sun_elevation_first_deg = sun_elevations.first(),
        sun_elevation_last_deg = sun_elevations.last(),
        "star trial"
    );
}

#[test]
fn stars_hold_attitude_at_night() {
    init_tracing();
    let (outcome, sun) = run(&star_request(requests::NIGHT_UTC));
    let [without, with] = &outcome.report.sensor_sets[..] else {
        panic!("two sets");
    };
    log(without, &sun);
    log(with, &sun);
    assert!(sun.iter().all(|e| *e < -18.0), "{sun:?}");
    let stars = with.stars.unwrap();
    assert!(
        stars.frames_solved > 0 && stars.lost_in_space_solved > 0,
        "{stars:?}"
    );
    let attitude = with.summary.attitude_error_rad.rms;
    assert!(attitude < consistency::STAR_ATTITUDE_RAD, "{attitude}");
    let (with_h, without_h) = (
        with.summary.horizontal_error_m.rms,
        without.summary.horizontal_error_m.rms,
    );
    assert!(
        with_h <= STARS_HORIZONTAL_TOLERANCE * without_h,
        "{with_h} vs {without_h}"
    );
}

/// At noon the sky saturates the tracker's frames, so the star set offers
/// no sightings and flies exactly as the set without stars.
#[test]
fn the_noon_sky_leaves_the_tracker_nothing() {
    init_tracing();
    let (outcome, sun) = run(&star_request(requests::NOON_UTC));
    let [without, with] = &outcome.report.sensor_sets[..] else {
        panic!("two sets");
    };
    log(without, &sun);
    log(with, &sun);
    assert!(sun.iter().all(|e| *e > 30.0), "{sun:?}");
    let stars = with.stars.unwrap();
    assert!(stars.frames_attempted > 0, "{stars:?}");
    let solved = stars.frames_solved as f64 / stars.frames_attempted as f64;
    assert!(solved <= NOON_SOLVED_FRACTION, "{stars:?}");
    if stars.frames_solved == 0 {
        assert_eq!(with.observations.stars.offered, 0);
        // Reading the prior publishes a solution, which advances the
        // solution sequence and nothing else; the state is compared.
        let state = |set: usize| -> Vec<_> {
            outcome.estimates[set]
                .iter()
                .map(|e| {
                    let (at, s) = e.get();
                    (
                        *at,
                        s.position,
                        s.velocity,
                        s.attitude,
                        s.position_cov,
                        s.attitude_cov,
                    )
                })
                .collect()
        };
        assert_eq!(state(0), state(1));
        assert_eq!(without.series, with.series);
    }
}

/// A loop of four 85 km legs, about 95 minutes at 60 m/s: more than one
/// Schuler period, all of it in astronomical night.
#[cfg(test)]
fn schuler_loop() -> Vec<sim_navigate::trial::WaypointSpec> {
    let mut points = requests::four_waypoints();
    points.truncate(1);
    let corners = [
        ("L1", 39.0, -74.617),
        ("L2", 39.765, -74.617),
        ("L3", 39.765, -75.6),
        ("L4", 39.0, -75.6),
    ];
    for (ident, lat_deg, lon_deg) in corners {
        let mut point = points[0].clone();
        point.ident = ident.into();
        point.lat_deg = lat_deg;
        point.lon_deg = lon_deg;
        points.push(point);
    }
    points
}

/// Horizontal error with and without stars over more than a Schuler period
/// for the tactical and navigation grades, measured and reported; stars bound
/// tilt, not position, so the assertion is only that they do not make it
/// worse.
#[test]
#[ignore = "about 95 minutes of flight with 1 Hz star frames: run with --release --ignored"]
fn stars_over_a_schuler_period() {
    init_tracing();
    for grade in [ImuGradeName::Tactical, ImuGradeName::Navigation] {
        let started = std::time::Instant::now();
        let mut request = star_request(requests::NIGHT_UTC);
        request.waypoints = schuler_loop();
        request.imu_grade = grade;
        request.stars.period_s = 1.0;
        let (outcome, sun) = run(&request);
        let [without, with] = &outcome.report.sensor_sets[..] else {
            panic!("two sets");
        };
        log(without, &sun);
        log(with, &sun);
        let at_minutes = |set: &SetReport, minutes: f64| {
            set.series
                .iter()
                .find(|s| s.time_s >= minutes * 60.0)
                .map_or(f64::NAN, |s| s.horizontal_error_m)
        };
        for minutes in [10.0, 30.0, 60.0, 84.0, 90.0] {
            tracing::info!(
                ?grade,
                minutes,
                without_m = at_minutes(without, minutes),
                with_m = at_minutes(with, minutes),
                "horizontal error"
            );
        }
        tracing::info!(
            ?grade,
            duration_s = outcome.report.route.duration_s,
            wall_s = started.elapsed().as_secs_f64(),
            "Schuler run"
        );
        assert!(sun.iter().all(|e| *e < -18.0), "{sun:?}");
        let (with_h, without_h) = (
            with.summary.horizontal_error_m.rms,
            without.summary.horizontal_error_m.rms,
        );
        assert!(
            with_h <= STARS_HORIZONTAL_TOLERANCE * without_h,
            "{grade:?}: {with_h} vs {without_h}"
        );
    }
}

/// The same loop in closed loop: each set's own navigator flies it, and the
/// true distance from the final waypoint when the navigator declares
/// arrival is reported with and without stars. Guidance accepts any
/// solution quality and the corridor is wide, so a lost navigator keeps
/// flying rather than being stopped.
#[test]
#[ignore = "about 95 minutes of closed-loop flight per set with 1 Hz star frames: run with --release --ignored"]
fn closed_loop_arrival_over_a_schuler_period() {
    init_tracing();
    for grade in [ImuGradeName::Tactical, ImuGradeName::Navigation] {
        let started = std::time::Instant::now();
        let mut request = star_request(requests::NIGHT_UTC);
        request.waypoints = schuler_loop();
        request.imu_grade = grade;
        request.stars.period_s = 1.0;
        request.vehicle.guidance = sim_navigate::trial::GuidanceSource::Estimate;
        request.vehicle.guidance_min_quality = Some(sim_navigate::trial::QualityFloor::Unusable);
        request.termination = Some(sim_navigate::trial::TerminationSpec {
            corridor_half_width_m: 500_000.0,
            ..Default::default()
        });
        let report = sim_navigate::trial::run_trial(&request).unwrap();
        for set in &report.sensor_sets {
            let closed = set.closed_loop.as_ref().unwrap();
            tracing::info!(
                ?grade,
                set = set.name,
                completed = closed.completed,
                termination = ?closed.termination,
                final_arrival_error_m = closed.final_arrival_error_m,
                max_abs_cross_track_m = closed.max_abs_cross_track_m,
                horizontal_rms_m = set.summary.horizontal_error_m.rms,
                horizontal_last_m = set.summary.horizontal_error_m.last,
                misses = ?closed
                    .waypoints
                    .iter()
                    .map(|w| (w.ident.clone(), w.true_miss_at_sequence_m))
                    .collect::<Vec<_>>(),
                "closed-loop arrival"
            );
        }
        tracing::info!(
            ?grade,
            wall_s = started.elapsed().as_secs_f64(),
            "closed-loop Schuler run"
        );
    }
}
