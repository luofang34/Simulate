//! Visual terrain navigation in the trial: the simulated vehicle camera over
//! the world imagery, matched by Navigate against the onboard package around
//! the navigator's own estimate. These tests read the Delaware catalogs on
//! disk and are ignored by default; run them with `--release --ignored`.

use sim_navigate::NavMeans;
use sim_navigate::trial::{ImuGradeName, SetReport, TrialReport, TrialRequest, run_trial};

#[cfg(test)]
#[path = "trial_support/visual_fixture.rs"]
mod fixture;
#[cfg(test)]
#[path = "trial_support/requests.rs"]
mod requests;

/// How much smaller the GNSS-denied horizontal RMS must be with visual fixes.
const VISUAL_BENEFIT: f64 = 0.25;

#[cfg(test)]
fn init_tracing() {
    tracing_subscriber::fmt().with_test_writer().try_init().ok();
}

/// A day request over the visual route, GNSS denied throughout.
#[cfg(test)]
fn denied_request(route: Vec<sim_navigate::trial::WaypointSpec>) -> TrialRequest {
    use NavMeans::{Barometric, DeadReckoning, Visual};
    let (_, onboard, world) = fixture::packages();
    let sets = vec![
        requests::set("ins+baro", &[DeadReckoning, Barometric], false),
        requests::set(
            "ins+baro+visual",
            &[DeadReckoning, Barometric, Visual],
            false,
        ),
    ];
    let mut request = requests::request(route, ImuGradeName::Tactical, sets);
    request.epoch_utc = requests::NOON_UTC.into();
    request.vehicle.cruise_speed_m_s = 40.0;
    request.visual = Some(fixture::visual_spec(onboard, world));
    request
}

#[cfg(test)]
fn log(report: &TrialReport) {
    tracing::info!(maps = ?report.visual_maps, "maps");
    for set in &report.sensor_sets {
        let s = &set.summary;
        tracing::info!(
            set = set.name,
            horizontal_rms_m = s.horizontal_error_m.rms,
            horizontal_last_m = s.horizontal_error_m.last,
            horizontal_p95_m = s.horizontal_error_m.p95,
            nees = s.mean_position_nees,
            consistency = s.consistency.statement,
            "set"
        );
        if let Some(visual) = &set.visual {
            tracing::info!(
                tally = ?visual.tally,
                residual = ?visual.accepted_residual_h_m,
                residual_over_sigma = visual.accepted_residual_over_sigma_rms,
                mean_gap_s = visual.mean_s_between_accepted,
                max_gap_s = visual.max_s_between_accepted,
                "visual"
            );
        }
    }
}

#[cfg(test)]
fn set<'a>(report: &'a TrialReport, name: &str) -> &'a SetReport {
    report.sensor_sets.iter().find(|s| s.name == name).unwrap()
}

/// (c) With GNSS denied for the whole route, visual fixes keep the tactical
/// navigator's horizontal error a quarter or less of what it is without.
#[test]
#[ignore = "reads the Delaware catalogs on disk; run with --release --ignored"]
fn visual_fixes_bound_a_gnss_denied_navigator() {
    init_tracing();
    let report = run_trial(&denied_request(fixture::visual_route())).unwrap();
    log(&report);
    let without = set(&report, "ins+baro").summary.horizontal_error_m.rms;
    let with = set(&report, "ins+baro+visual")
        .summary
        .horizontal_error_m
        .rms;
    assert!(
        report.route.duration_s >= 600.0,
        "{}",
        report.route.duration_s
    );
    assert!(with <= VISUAL_BENEFIT * without, "{with} vs {without}");
}

/// The denied request over `route` matched against `onboard`.
#[cfg(test)]
fn with_onboard(
    route: Vec<sim_navigate::trial::WaypointSpec>,
    onboard: &std::path::Path,
) -> TrialRequest {
    let mut request = denied_request(route);
    if let Some(visual) = request.visual.as_mut() {
        visual.package_dir = onboard.display().to_string();
    }
    request
}

/// Mean east residual of the accepted fixes, and their mean declared
/// per-axis sigma.
#[cfg(test)]
fn east_residual(set: &SetReport) -> (f64, f64) {
    let visual = set.visual.as_ref().unwrap();
    let accepted: Vec<_> = visual
        .records
        .iter()
        .filter(|r| r.record.outcome == "accepted")
        .collect();
    assert!(!accepted.is_empty(), "{:?}", visual.tally);
    let n = accepted.len() as f64;
    let east = accepted
        .iter()
        .map(|r| r.residual_ne_m.unwrap()[1])
        .sum::<f64>()
        / n;
    let sigma = accepted
        .iter()
        .map(|r| r.record.fix_sigma_h_m.unwrap() / 2.0_f64.sqrt())
        .sum::<f64>()
        / n;
    (east, sigma)
}

/// East offset of the last estimate of set `index` in `b` from that in `a`,
/// metres.
#[cfg(test)]
fn final_east_offset(
    a: &sim_navigate::trial::TrialOutcome,
    b: &sim_navigate::trial::TrialOutcome,
    index: usize,
) -> f64 {
    let last =
        |o: &sim_navigate::trial::TrialOutcome| o.estimates[index].last().unwrap().get().1.position;
    let (p, q) = (last(a), last(b));
    let geo = |g: navigate_contract::GeodeticPosition| {
        sim_geo::geodetic_to_ecef(&sim_geo::Geodetic {
            lat_rad: g.latitude_rad,
            lon_rad: g.longitude_rad,
            height_m: g.altitude_m,
        })
    };
    let frame = sim_geo::LocalFrame::at(&sim_geo::ecef_to_geodetic(geo(p)));
    frame.ecef_to_ned_vector(geo(q) - geo(p)).y
}

/// (a) The fixes come from the package's pixels: moving the package imagery
/// about 40 m east moves the fixes and the converged estimate east by that,
/// within two of the fixes' own sigmas; and a featureless package gives no
/// accepted fix, leaving the navigator exactly as it is without vision.
#[test]
#[ignore = "reads the Delaware catalogs on disk; run with --release --ignored"]
fn fixes_follow_the_package_pixels_not_truth() {
    init_tracing();
    use sim_navigate::trial::{evaluate, record_trial};
    let (_, onboard, _) = fixture::packages();
    let (_, shifted, featureless) = fixture::altered_packages();
    let route = fixture::short_visual_route();
    let expected = f64::from(fixture::SHIFT_PX)
        * sim_navigate::package::ground_sample_distance_m(route[0].lat_deg, 15);
    let normal_request = with_onboard(route.clone(), onboard);
    let shifted_request = with_onboard(route.clone(), shifted);
    let recorded = record_trial(&normal_request).unwrap();
    let normal = evaluate(&normal_request, &recorded).unwrap();
    let moved = evaluate(&shifted_request, &recorded).unwrap();
    log(&normal.report);
    log(&moved.report);
    let (east_normal, _) = east_residual(set(&normal.report, "ins+baro+visual"));
    let (east_moved, sigma) = east_residual(set(&moved.report, "ins+baro+visual"));
    let fix_shift = east_moved - east_normal;
    let estimate_shift = final_east_offset(&normal, &moved, 1);
    tracing::info!(
        expected,
        fix_shift,
        estimate_shift,
        sigma,
        "package moved east"
    );
    assert!(
        (fix_shift - expected).abs() <= 2.0 * sigma,
        "{fix_shift} vs {expected} ± {sigma}"
    );
    assert!(
        (estimate_shift - expected).abs() <= 2.0 * sigma,
        "{estimate_shift}"
    );

    let blank_request = with_onboard(route, featureless);
    let blank = evaluate(&blank_request, &recorded).unwrap();
    log(&blank.report);
    let visual = set(&blank.report, "ins+baro+visual")
        .visual
        .as_ref()
        .unwrap();
    assert_eq!(visual.tally.filter.offered, 0, "{:?}", visual.tally);
    assert!(visual.tally.frames > 0);
    let state = |set: usize| -> Vec<_> {
        blank.estimates[set]
            .iter()
            .map(|e| {
                let (at, s) = e.get();
                (*at, s.position, s.velocity, s.attitude, s.position_cov)
            })
            .collect()
    };
    assert_eq!(state(0), state(1));
}

/// (b) Truth reaches the visual chain only through the rendered frames:
/// moving the scoring truth 1 km leaves every estimate bit for bit.
#[test]
#[ignore = "reads the Delaware catalogs on disk; run with --release --ignored"]
fn visual_estimates_never_see_scoring_truth() {
    init_tracing();
    use sim_navigate::trial::{evaluate, record_trial};
    let request = denied_request(fixture::short_visual_route());
    let recorded = record_trial(&request).unwrap();
    let mut moved = recorded.clone();
    let start = sim_geo::ecef_to_geodetic(recorded.flight.truth[0].get().1.position_ecef);
    let north = sim_geo::LocalFrame::at(&start).north() * 1_000.0;
    for sample in &mut moved.flight.truth {
        let (at, mut state) = *sample.get();
        state.position_ecef += north;
        *sample = sim_navigate::Truth::new((at, state));
    }
    let original = evaluate(&request, &recorded).unwrap();
    let perturbed = evaluate(&request, &moved).unwrap();
    assert_eq!(original.estimates, perturbed.estimates);
    let residual = |o: &sim_navigate::trial::TrialOutcome| {
        set(&o.report, "ins+baro+visual")
            .visual
            .as_ref()
            .unwrap()
            .accepted_residual_h_m
            .unwrap()
            .p50
    };
    tracing::info!(
        original = residual(&original),
        moved = residual(&perturbed),
        "fix residuals"
    );
    assert!(residual(&perturbed) > 900.0);
}

/// (d) The filter's consistency with visual fixes, reported; the fixes'
/// declared covariance against their residuals, reported.
#[test]
#[ignore = "reads the Delaware catalogs on disk; run with --release --ignored"]
fn visual_consistency_is_reported() {
    init_tracing();
    let report = run_trial(&denied_request(fixture::short_visual_route())).unwrap();
    log(&report);
    let visual_set = set(&report, "ins+baro+visual");
    assert!(visual_set.summary.mean_position_nees.is_finite());
    let visual = visual_set.visual.as_ref().unwrap();
    assert!(visual.accepted_residual_over_sigma_rms.unwrap().is_finite());
}

/// (f) Leaving the package's coverage stops the fixes, with a coverage
/// reason, and the flight carries on.
#[test]
#[ignore = "reads the Delaware catalogs on disk; run with --release --ignored"]
fn fixes_stop_outside_the_package() {
    init_tracing();
    let dir = tempfile::tempdir().unwrap();
    let first_leg = dir.path().join("first-leg");
    fixture::write_package(
        &mut fixture::catalog(fixture::ONBOARD_CATALOG),
        &fixture::short_visual_route(),
        sim_navigate::package::PackageRole::Onboard,
        15,
        12,
        &first_leg,
    );
    let report = run_trial(&with_onboard(fixture::visual_route(), &first_leg)).unwrap();
    log(&report);
    let visual = set(&report, "ins+baro+visual").visual.as_ref().unwrap();
    assert!(visual.tally.outside_coverage > 0, "{:?}", visual.tally);
    assert!(visual.tally.filter.accepted > 0, "{:?}", visual.tally);
    let after = visual
        .records
        .iter()
        .rev()
        .take(5)
        .all(|r| r.record.outcome == "coverage:outside_package");
    assert!(after, "{:?}", visual.records.last());
}

/// (g) The same visual request writes the same bytes, and they read back.
#[test]
#[ignore = "reads the Delaware catalogs on disk; run with --release --ignored"]
fn visual_results_are_deterministic_and_read_back() {
    init_tracing();
    let request = denied_request(fixture::short_visual_route());
    let first = run_trial(&request).unwrap().to_json().unwrap();
    let second = run_trial(&request).unwrap().to_json().unwrap();
    assert!(first == second);
    let read = TrialReport::from_json(&first).unwrap();
    assert_eq!(read.to_json().unwrap(), first);
    assert_eq!(
        TrialRequest::from_json(&request.to_json().unwrap()).unwrap(),
        request
    );
}

/// (e) In closed loop with GNSS denied, visual fixes take the vehicle truly
/// over each waypoint, within the executor's threshold plus three fix
/// sigmas; without them it misses each by about its navigation error: the
/// true miss lies within the threshold of that error.
#[test]
#[ignore = "reads the Delaware catalogs on disk; run with --release --ignored"]
fn visual_fixes_steer_the_closed_loop_onto_the_route() {
    init_tracing();
    let mut request = denied_request(fixture::visual_route());
    request.vehicle.guidance = sim_navigate::trial::GuidanceSource::Estimate;
    request.vehicle.guidance_min_quality = Some(sim_navigate::trial::QualityFloor::Unusable);
    request.termination = Some(sim_navigate::trial::TerminationSpec {
        corridor_half_width_m: 100_000.0,
        ..Default::default()
    });
    let report = run_trial(&request).unwrap();
    log(&report);
    for set in &report.sensor_sets {
        let closed = set.closed_loop.as_ref().unwrap();
        tracing::info!(set = set.name, completed = closed.completed, termination = ?closed.termination,
            arrival = ?closed.final_arrival_error_m, max_xt = closed.max_abs_cross_track_m, "closed loop");
        for w in &closed.waypoints {
            tracing::info!(set = set.name, waypoint = ?w, "waypoint");
        }
    }
    let visual = set(&report, "ins+baro+visual");
    let closed = visual.closed_loop.as_ref().unwrap();
    let sigma = visual
        .visual
        .as_ref()
        .unwrap()
        .records
        .iter()
        .filter_map(|r| r.record.fix_sigma_h_m)
        .fold(0.0, f64::max);
    assert!(closed.completed, "{:?}", closed.termination);
    for w in &closed.waypoints {
        let miss = w.true_miss_at_sequence_m.unwrap();
        assert!(
            miss <= w.capture_threshold_m + 3.0 * sigma,
            "{w:?} sigma {sigma}"
        );
    }
    let blind = set(&report, "ins+baro").closed_loop.as_ref().unwrap();
    for w in blind.waypoints.iter().skip(1) {
        let (Some(miss), Some(lost)) =
            (w.true_miss_at_sequence_m, w.navigation_error_at_sequence_m)
        else {
            continue;
        };
        assert!((miss - lost).abs() <= w.capture_threshold_m, "{w:?}");
    }
}
