//! The terrain camera at the onboard boundary, over a synthetic world: the
//! camera's frames reach the navigator as fixes by day, and the result says
//! what the navigator loaded and matched at.

use std::sync::OnceLock;

use sim_navigate::NavMeans;
use sim_navigate::trial::{
    ImuGradeName, PackageRef, RecordedTrial, TrialOutcome, TrialRequest, evaluate_blocking,
    record_trial_blocking,
};

#[cfg(test)]
#[path = "trial_support/requests.rs"]
mod requests;
#[cfg(test)]
#[path = "trial_support/visual_world.rs"]
mod visual_world;

#[cfg(test)]
fn init_tracing() {
    tracing_subscriber::fmt().with_test_writer().try_init().ok();
}

/// `ins` and `ins+visual` over `package` at `epoch_utc`, flying the
/// synthetic route at 600 m and 60 m/s.
#[cfg(test)]
fn request(epoch_utc: &str, package: &PackageRef) -> TrialRequest {
    use NavMeans::{DeadReckoning, Visual};
    use visual_world::{LAT_DEG, LENGTH_DEG, LON_DEG};
    let waypoints = vec![
        requests::waypoint("V0", LAT_DEG, LON_DEG + 0.005, 600.0),
        requests::waypoint("V1", LAT_DEG, LON_DEG + LENGTH_DEG - 0.005, 600.0),
    ];
    let sets = vec![
        requests::set("ins", &[DeadReckoning]),
        requests::set("ins+visual", &[DeadReckoning, Visual]),
    ];
    let mut request = requests::request(waypoints, ImuGradeName::Tactical, sets);
    request.epoch_utc = epoch_utc.into();
    request.terrain_camera = Some(visual_world::camera());
    request.maps = Some(visual_world::maps(package));
    request.seeds.terrain_camera = Some(11);
    // A poor in-flight position fix: the error the camera has to correct.
    request.alignment.position_sigma_m = 50.0;
    request
}

/// The day run over the textured world, shared by the tests that read it.
#[cfg(test)]
fn day() -> &'static (TrialRequest, RecordedTrial, TrialOutcome) {
    static RUN: OnceLock<(TrialRequest, RecordedTrial, TrialOutcome)> = OnceLock::new();
    RUN.get_or_init(|| {
        let request = request(requests::NOON_UTC, &visual_world::world().textured);
        let recorded = record_trial_blocking(&request).unwrap();
        let outcome = evaluate_blocking(&request, &recorded).unwrap();
        (request, recorded, outcome)
    })
}

/// By day the navigator takes fixes from the camera and holds the error
/// the inertial core alone lets grow; the result names the map it loaded,
/// with the request's assumed accuracy, and the camera it matched at.
#[test]
fn visual_fixes_hold_the_inertial_core_by_day() {
    init_tracing();
    let (_, recorded, outcome) = day();
    let report = &outcome.report;
    let (ins, visual) = (&report.sensor_sets[0], &report.sensor_sets[1]);
    let terrain = visual.terrain.as_ref().unwrap();
    tracing::info!(
        frames = recorded.log.terrain.len(),
        ins_rms_m = ins.summary.horizontal_error_m.rms,
        visual_rms_m = visual.summary.horizontal_error_m.rms,
        fixes = ?terrain.fixes,
        map = ?terrain.map,
        working = ?terrain.working_camera,
        timing = ?outcome.timing,
        "day over the synthetic world"
    );
    assert!(
        terrain.fixes.get("accepted").copied().unwrap_or(0) > 0,
        "{terrain:?}"
    );
    let (ins_h, visual_h) = (
        &ins.summary.horizontal_error_m,
        &visual.summary.horizontal_error_m,
    );
    assert!(visual_h.last < 0.25 * ins_h.last, "{visual_h:?} {ins_h:?}");
    let map = terrain.map.as_ref().unwrap();
    assert_eq!(map.imagery.sigma_from, "assumed");
    assert!(terrain.working_camera.is_some());
    let maps = report.maps.as_ref().unwrap();
    assert!(maps.world_is_onboard, "{maps:?}");
    assert!(ins.terrain.is_none());
}

/// The estimates' state at each scored sample: position, velocity,
/// attitude and their covariances, without the solution sequence, which
/// reading a solution advances.
#[cfg(test)]
fn states(outcome: &TrialOutcome, set: usize) -> Vec<String> {
    outcome.estimates[set]
        .iter()
        .map(|estimate| {
            let (at, s) = estimate.get();
            format!(
                "{at:?} {:?} {:?} {:?} {:?} {:?} {:?}",
                s.position, s.velocity, s.attitude, s.position_cov, s.velocity_cov, s.attitude_cov
            )
        })
        .collect()
}

/// Truth the navigator must never see: moving the recorded truth 1 km north
/// leaves every estimate of the visual set as it was, since the camera's
/// frames were rendered when the flight was sensed.
#[test]
fn moving_the_truth_moves_no_visual_estimate() {
    init_tracing();
    let (request, recorded, outcome) = day();
    let mut moved = recorded.clone();
    let start = sim_geo::ecef_to_geodetic(recorded.flight.truth[0].get().1.position_ecef);
    let north = sim_geo::LocalFrame::at(&start).north() * 1_000.0;
    for sample in &mut moved.flight.truth {
        let (at, mut state) = *sample.get();
        state.position_ecef += north;
        *sample = sim_navigate::Truth::new((at, state));
    }
    let perturbed = evaluate_blocking(request, &moved).unwrap();
    assert_eq!(outcome.estimates, perturbed.estimates);
    let visual = &perturbed.report.sensor_sets[1];
    assert!(
        visual.summary.horizontal_error_m.p50 > 900.0,
        "{:?}",
        visual.summary
    );
}

/// A terrain-camera schedule withholds frames and nothing else: run under
/// it and forked at the interval's start into a run without the camera,
/// the navigator is the same, bit for bit, at the interval's end; and the
/// interval's tallies count every frame inside as scheduled, none offered.
#[test]
fn a_camera_schedule_leaves_the_filter_of_a_run_without_the_camera() {
    init_tracing();
    use sim_navigate::trial::{SensorKind, SetAvailability, SetSelection, replay_set_blocking};
    let (request, recorded, _) = day();
    let (start_s, end_s) = (20.0, 40.0);
    let mut scheduled = request.clone();
    scheduled.availability = vec![requests::schedule(
        "cloud",
        SensorKind::TerrainCamera,
        start_s,
        end_s,
    )];
    scheduled.sensor_sets[1]
        .availability
        .insert(SensorKind::TerrainCamera, "cloud".into());
    let set = &scheduled.sensor_sets[1];
    let step = |seconds: f64| (seconds * 120.0).round() as usize;
    let mut run = replay_set_blocking(&scheduled, set, &recorded.log).unwrap();
    run.advance_to(&recorded.log, step(start_s)).unwrap();
    let mut fork = run.fork(SetSelection {
        means: set.ablation_set().without(NavMeans::Visual),
        availability: SetAvailability::NONE,
    });
    run.advance_to(&recorded.log, step(end_s)).unwrap();
    fork.advance_to(&recorded.log, step(end_s)).unwrap();
    let (a, b) = (
        run.solution(&recorded.log).unwrap().unwrap(),
        fork.solution(&recorded.log).unwrap().unwrap(),
    );
    assert_eq!(a, b, "the scheduled run and the fork differ");
    let (_, inside) = &run.interval_tallies()[0];
    tracing::info!(?inside, "tallies inside the cloud");
    let camera = &inside.terrain_camera;
    assert!(
        camera.scheduled_unavailable >= 19 && camera.offered == 0,
        "{inside:?}"
    );
}

/// The run of `package` at `epoch_utc`, in which the camera can give the
/// navigator nothing usable.
#[cfg(test)]
fn barren(epoch_utc: &str, package: &PackageRef) -> TrialOutcome {
    let request = request(epoch_utc, package);
    let recorded = record_trial_blocking(&request).unwrap();
    evaluate_blocking(&request, &recorded).unwrap()
}

/// Asserts that the visual set of `outcome` took no fix and flew exactly
/// as the inertial core alone.
#[cfg(test)]
fn assert_no_visual_effect(outcome: &TrialOutcome, what: &str) {
    let visual = &outcome.report.sensor_sets[1];
    let terrain = visual.terrain.as_ref().unwrap();
    tracing::info!(what, fixes = ?terrain.fixes, tally = ?visual.observations.terrain_camera, "no visual effect");
    assert_eq!(terrain.fixes.get("accepted"), None, "{what}: {terrain:?}");
    assert_eq!(visual.observations.terrain_camera.offered, 0, "{what}");
    assert_eq!(states(outcome, 0), states(outcome, 1), "{what}");
    assert_eq!(
        outcome.report.sensor_sets[0].series, visual.series,
        "{what}"
    );
}

/// At night the Sun gate leaves the camera only its own noise: no fix, and
/// the visual set flies as the inertial core alone.
#[test]
fn the_sun_gate_leaves_the_camera_nothing_at_night() {
    init_tracing();
    let outcome = barren(requests::NIGHT_UTC, &visual_world::world().textured);
    assert_no_visual_effect(&outcome, "night");
}

/// A featureless map gives the matcher nothing to hold: every frame is
/// refused, and the visual set flies as the inertial core alone.
#[test]
fn a_featureless_world_gives_no_fix() {
    init_tracing();
    let outcome = barren(requests::NOON_UTC, &visual_world::world().featureless);
    assert_no_visual_effect(&outcome, "featureless");
}

/// A host's own store serves as the directory does: the synthetic world
/// copied into a navigate-data memory store, registered by name, gives
/// every set the same result.
#[test]
fn a_store_the_host_keeps_in_memory_serves_like_the_directory() {
    init_tracing();
    use sim_navigate::trial::{MapStore, NamedStores, NoProgress, run_trial_with_stores};
    let world = visual_world::world();
    let mut memory = navigate_data::MemoryStore::default();
    let root = world.dir.path();
    let mut pending = vec![root.to_path_buf()];
    while let Some(dir) = pending.pop() {
        for entry in std::fs::read_dir(dir).unwrap() {
            let path = entry.unwrap().path();
            if path.is_dir() {
                pending.push(path);
            } else {
                let logical = path
                    .strip_prefix(root)
                    .unwrap()
                    .to_str()
                    .unwrap()
                    .to_owned();
                memory.replace(&logical, &std::fs::read(&path).unwrap());
            }
        }
    }
    let mut stores = NamedStores::new();
    stores.insert("memory", MapStore::new(memory));
    let mut in_memory = world.textured.clone();
    in_memory.store = "memory".into();
    let request = request(requests::NOON_UTC, &in_memory);
    let (report, _) = run_trial_with_stores(&request, &stores, &mut NoProgress).unwrap();
    let (_, _, outcome) = day();
    assert_eq!(report.sensor_sets, outcome.report.sensor_sets);
}
