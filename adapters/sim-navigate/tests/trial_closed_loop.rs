//! Closed loop: each sensor set's own navigator flies the vehicle through
//! Navigate's plan executor and guidance, so navigation error moves the true
//! flight. The pilot sees no truth.

use sim_navigate::NavMeans;
use sim_navigate::trial::{
    Autopilot, ClosedLoopReport, EventKind, GuidanceSource, ImuGradeName, ProgressRecord,
    QualityFloor, SetReport, TerminationSpec, TrialRequest, WaypointSpec, plan_waypoints,
    run_closed_trial, run_trial_blocking, run_trial_with_progress_blocking, trial_director,
};

#[cfg(test)]
#[path = "trial_support/requests.rs"]
mod requests;

/// Cross-track bound on a straight leg after capture with GNSS, before the
/// receiver's error is added, metres.
const STRAIGHT_LEG_CROSS_TRACK_M: f64 = 30.0;
/// GNSS 1-sigma multiples a true miss or deviation may add.
const GNSS_SIGMAS: f64 = 3.0;
/// How closely the true miss at a declared capture must equal the
/// navigation error then.
const LOST_MISS_TOLERANCE: f64 = 0.10;
/// Time after GNSS returns within which true cross-track must be back under
/// the GNSS-aided bound, seconds.
const PULL_BACK_S: f64 = 120.0;

#[cfg(test)]
fn init_tracing() {
    tracing_subscriber::fmt().with_test_writer().try_init().ok();
}

#[cfg(test)]
fn closed(mut request: TrialRequest) -> TrialRequest {
    request.vehicle.guidance = GuidanceSource::Estimate;
    request
}

#[cfg(test)]
fn log(set: &SetReport) {
    let closed = set.closed_loop.as_ref().unwrap();
    tracing::info!(
        set = set.name,
        completed = closed.completed,
        termination = ?closed.termination,
        max_abs_cross_track_m = closed.max_abs_cross_track_m,
        final_arrival_error_m = closed.final_arrival_error_m,
        horizontal_rms_m = set.summary.horizontal_error_m.rms,
        refusals = ?closed.guidance_refusals,
        "closed loop"
    );
    for waypoint in &closed.waypoints {
        tracing::info!(?waypoint, "waypoint");
    }
}

/// Track error under which the vehicle heads along a leg, radians: the
/// open-loop route test's.
const ALONG_TRACK_RAD: f64 = 0.035;

/// Samples of `report` on each leg, by leg index: cross-track and track
/// error.
#[cfg(test)]
fn legs(report: &ClosedLoopReport) -> Vec<(usize, Vec<(f64, f64)>)> {
    let mut legs: Vec<(usize, Vec<(f64, f64)>)> = Vec::new();
    for sample in &report.cross_track {
        let value = (sample.true_cross_track_m, sample.true_track_error_rad);
        match legs.last_mut() {
            Some((index, values)) if *index == sample.leg_index => values.push(value),
            _ => legs.push((sample.leg_index, vec![value])),
        }
    }
    legs
}

/// (a) With GNSS every waypoint is truly passed inside the executor's own
/// threshold plus three GNSS sigmas, and each straight leg, once captured
/// (the first sample both within the bound and heading along the leg, in
/// the leg's first half), stays inside 30 m plus three sigmas.
#[test]
fn gnss_aided_closed_loop_flies_the_route() {
    init_tracing();
    let request = closed(requests::request(
        requests::four_waypoints(),
        ImuGradeName::Tactical,
        vec![requests::reference_sets().remove(0)],
    ));
    let report = run_trial_blocking(&request).unwrap();
    let set = &report.sensor_sets[0];
    log(set);
    let closed = set.closed_loop.as_ref().unwrap();
    assert!(closed.completed && closed.termination.is_none());
    let slack = GNSS_SIGMAS * report.gnss_horizontal_sigma_m;
    for waypoint in &closed.waypoints {
        let miss = waypoint.true_miss_at_sequence_m.unwrap();
        assert!(miss <= waypoint.capture_threshold_m + slack, "{waypoint:?}");
    }
    let bound = STRAIGHT_LEG_CROSS_TRACK_M + slack;
    for (leg, values) in legs(closed) {
        let capture = values
            .iter()
            .position(|(xt, track)| xt.abs() < bound && track.abs() < ALONG_TRACK_RAD)
            .unwrap();
        let worst = values[capture..]
            .iter()
            .map(|(xt, _)| xt.abs())
            .fold(0.0, f64::max);
        tracing::info!(leg, capture_s = capture, worst, bound, "leg after capture");
        assert!(capture < values.len() / 2, "leg {leg}");
        assert!(worst < bound, "leg {leg}: {worst}");
    }
}

/// Fly-over waypoints far enough out for a tactical INS to drift kilometres
/// before it reaches them: 54 km east, 18 km north, 18 km east.
#[cfg(test)]
fn long_fly_over_route() -> Vec<WaypointSpec> {
    let mut points = requests::four_waypoints();
    points.truncate(1);
    for (ident, lat_deg, lon_deg) in [
        ("F1", 39.0, -74.976),
        ("F2", 39.162, -74.976),
        ("F3", 39.162, -74.768),
    ] {
        let mut point = points[0].clone();
        point.ident = ident.into();
        point.lat_deg = lat_deg;
        point.lon_deg = lon_deg;
        point.fly_over = true;
        points.push(point);
    }
    points
}

/// (b) With no position aid the navigator still declares the first
/// fly-over waypoint captured while the vehicle is truly as far from it as
/// the navigator is lost. The jittering estimate then passes the later
/// fly-over fixes outside the capture radius, and the executor sequences
/// each on the perpendicular through the fix, reporting the navigator's
/// cross-track: the flight completes, kilometres from where it believes.
#[test]
fn a_lost_navigator_declares_captures_it_truly_missed() {
    init_tracing();
    use NavMeans::{Barometric, DeadReckoning};
    let mut request = closed(requests::request(
        long_fly_over_route(),
        ImuGradeName::Tactical,
        vec![requests::set("ins+baro", &[DeadReckoning, Barometric])],
    ));
    // Guidance must steer on a solution Navigate rates unusable, and the
    // corridor must hold a vehicle kilometres off its route.
    request.vehicle.guidance_min_quality = Some(QualityFloor::Unusable);
    request.termination = Some(TerminationSpec {
        corridor_half_width_m: 100_000.0,
        ..TerminationSpec::default()
    });
    let report = run_trial_blocking(&request).unwrap();
    let set = &report.sensor_sets[0];
    log(set);
    let closed = set.closed_loop.as_ref().unwrap();
    let first = &closed.waypoints[1];
    let miss = first.true_miss_at_sequence_m.unwrap();
    let lost = first.navigation_error_at_sequence_m.unwrap();
    assert!(lost > first.capture_threshold_m, "{first:?}");
    assert!(
        ((miss - lost) / lost).abs() <= LOST_MISS_TOLERANCE,
        "{first:?}"
    );
    for waypoint in &closed.waypoints {
        tracing::info!(
            ident = waypoint.ident,
            sequenced_at_s = waypoint.sequenced_at_s,
            passed_cross_track_m = waypoint.passed_cross_track_m,
            true_miss_m = waypoint.true_miss_at_sequence_m,
            "sequencing"
        );
    }
    assert!(closed.completed && closed.termination.is_none());
    let capture_m = request.vehicle.capture_radius_m;
    let passed: Vec<f64> = closed
        .waypoints
        .iter()
        .filter_map(|waypoint| waypoint.passed_cross_track_m)
        .collect();
    assert!(!passed.is_empty(), "{:?}", closed.waypoints);
    assert!(passed.iter().all(|xt| xt.abs() > capture_m), "{passed:?}");
    assert!(closed.final_arrival_error_m.unwrap() > first.capture_threshold_m);
}

/// (c) A GNSS outage on a straight leg lets the true track wander, and GNSS
/// pulls it back under the aided bound once it returns.
#[test]
fn an_outage_lets_the_track_wander_and_gnss_pulls_it_back() {
    init_tracing();
    use NavMeans::{Barometric, DeadReckoning, Gnss};
    let outage = requests::gnss_jammed(120.0, 360.0);
    let mut request = closed(requests::request(
        requests::straight(12.0),
        ImuGradeName::Tactical,
        vec![requests::jammed_set(
            "outage",
            &[DeadReckoning, Barometric, Gnss],
        )],
    ));
    let outage_interval = outage.unavailable[0].clone();
    request.availability = vec![outage];
    let report = run_trial_blocking(&request).unwrap();
    let set = &report.sensor_sets[0];
    log(set);
    let closed = set.closed_loop.as_ref().unwrap();
    let samples = &closed.cross_track;
    let worst = |from: f64, to: f64| {
        samples
            .iter()
            .filter(|s| (from..=to).contains(&s.time_s))
            .map(|s| s.true_cross_track_m.abs())
            .fold(0.0, f64::max)
    };
    let before = worst(outage_interval.start_s - 60.0, outage_interval.start_s);
    let during = worst(outage_interval.start_s, outage_interval.end_or_never());
    let bound = STRAIGHT_LEG_CROSS_TRACK_M + GNSS_SIGMAS * report.gnss_horizontal_sigma_m;
    let after = worst(outage_interval.end_or_never() + PULL_BACK_S, f64::INFINITY);
    tracing::info!(
        before,
        during,
        after,
        bound,
        "true cross-track around the outage"
    );
    assert!(during > 2.0 * before && during > bound, "{before} {during}");
    assert!(after < bound, "{after}");
}

/// (d) A closed-loop set's result is a function of its request and its own
/// set alone: flown alone with a pilot trace, and flown interleaved slice by
/// slice with a second set while a sink takes progress, it writes the same
/// bytes. And the pilot is a function of what it reads: replaying its
/// recorded inputs, which hold no truth, through a fresh pilot reproduces
/// every command bit for bit.
#[test]
fn closed_loop_is_deterministic_and_its_pilot_reads_no_truth() {
    init_tracing();
    use NavMeans::{Barometric, Celestial, DeadReckoning, Gnss};
    let mut request = closed(requests::request(
        requests::four_waypoints(),
        ImuGradeName::Tactical,
        vec![requests::jammed_set(
            "gnss-outage+baro+ins+stars",
            &[DeadReckoning, Barometric, Gnss, Celestial],
        )],
    ));
    request.stars.period_s = 30.0;
    request.availability = vec![requests::gnss_jammed(120.0, 200.0)];
    let first = run_closed_trial(&request, true).unwrap();
    let mut paired = request.clone();
    paired.sensor_sets.push(requests::set(
        "gnss+baro+ins",
        &[DeadReckoning, Barometric, Gnss],
    ));
    let mut records = Vec::new();
    let mut sink = |record: &ProgressRecord| -> std::io::Result<()> {
        records.push(record.clone());
        Ok(())
    };
    let second = run_trial_with_progress_blocking(&paired, &mut sink).unwrap();
    check_closed_stream(&records, &paired);
    let bytes = |set: &SetReport| serde_json::to_string(set).unwrap();
    assert!(bytes(&first.report.sensor_sets[0]) == bytes(&second.sensor_sets[0]));

    let waypoints = plan_waypoints(&request.waypoints).unwrap();
    let director = trial_director(&waypoints, &request.vehicle).unwrap();
    let mut pilot = Autopilot::new(director, &request.vehicle);
    let trace = &first.sets[0].trace;
    assert!(trace.len() > 1_000);
    for (input, output) in trace {
        assert_eq!(&pilot.step(input, 1.0 / 120.0), output);
    }
    let stars = first.sets[0].stars;
    assert!(stars.frames_solved > 1, "{stars:?}");
}

/// The closed-loop stream: `started` first, route time never going back
/// though each set flies its own flight, and every set sequencing the
/// whole route in it.
#[cfg(test)]
fn check_closed_stream(records: &[ProgressRecord], request: &TrialRequest) {
    assert!(matches!(records.first(), Some(ProgressRecord::Started(_))));
    let mut last_t = f64::NEG_INFINITY;
    let mut sequenced = std::collections::BTreeMap::<String, usize>::new();
    let mut samples = 0_usize;
    for record in records {
        let t_s = match record {
            ProgressRecord::Sample(sample) => {
                samples = samples.wrapping_add(1);
                sample.t_s
            }
            ProgressRecord::Event(event) => {
                if let EventKind::WaypointSequenced { .. } = event.kind {
                    let set = event.set.clone().unwrap();
                    let count = sequenced.entry(set).or_default();
                    *count = count.wrapping_add(1);
                }
                event.t_s
            }
            _ => continue,
        };
        assert!(t_s >= last_t, "{t_s} after {last_t}");
        last_t = t_s;
    }
    tracing::info!(
        records = records.len(),
        samples,
        ?sequenced,
        "closed-loop stream"
    );
    for set in &request.sensor_sets {
        assert_eq!(sequenced.get(&set.name), Some(&request.waypoints.len()));
    }
}
