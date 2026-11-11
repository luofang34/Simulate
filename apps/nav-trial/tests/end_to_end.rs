//! The binary's path end to end: parse the command line, read or complete
//! the request, run the trial, stream progress, write the result, and read
//! both back. The full four-waypoint example is checked to parse and
//! validate; running it is the trial tests' job.

#![cfg(not(target_arch = "wasm32"))]

use std::collections::BTreeMap;
use std::path::{Path, PathBuf};

use nav_trial::cli::{CliArgs, ProgressTarget, RequestSource, RouteOverrides, parse_args};
use nav_trial::run::{completed_request_path, run_blocking};
use sim_navigate::trial::{
    GuidanceSource, ImuGradeName, ProgressRecord, SensorState, TrialReport, TrialRequest,
};

#[cfg(test)]
fn request_file(name: &str) -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("requests")
        .join(name)
}

#[cfg(test)]
fn args(list: &[&str]) -> Result<CliArgs, nav_trial::NavTrialError> {
    parse_args(list.iter().map(|s| (*s).to_owned()))
}

#[cfg(test)]
fn path_text(path: &Path) -> String {
    path.display().to_string()
}

#[test]
fn the_command_line_takes_a_request_or_a_route() {
    assert_eq!(
        args(&["--request", "a.json", "--out", "b.json"]).unwrap(),
        CliArgs {
            source: RequestSource::Request("a.json".into()),
            out: "b.json".into(),
            progress: None,
        }
    );
    assert_eq!(
        args(&[
            "--route",
            "r.json",
            "--defaults",
            "d.json",
            "--imu",
            "navigation",
            "--guidance",
            "estimate",
            "--epoch",
            "2026-01-01T00:00:00Z",
            "--out",
            "b.json",
            "--progress",
            "-",
        ])
        .unwrap(),
        CliArgs {
            source: RequestSource::Route {
                route: "r.json".into(),
                defaults: Some("d.json".into()),
                overrides: RouteOverrides {
                    epoch_utc: Some("2026-01-01T00:00:00Z".into()),
                    imu_grade: Some(ImuGradeName::Navigation),
                    guidance: Some(GuidanceSource::Estimate),
                },
            },
            out: "b.json".into(),
            progress: Some(ProgressTarget::Stdout),
        }
    );
    assert!(args(&["--request", "a.json"]).is_err());
    assert!(args(&["--out"]).is_err());
    assert!(args(&["--request", "a", "--request", "b", "--out", "c"]).is_err());
    assert!(args(&["--request", "a", "--route", "b", "--out", "c"]).is_err());
    assert!(args(&["--request", "a", "--imu", "tactical", "--out", "c"]).is_err());
    assert!(args(&["--route", "a", "--imu", "gyro", "--out", "c"]).is_err());
    assert!(args(&["--verbose"]).is_err());
}

#[test]
fn the_four_waypoint_example_is_a_valid_request() {
    let text = std::fs::read_to_string(request_file("four-waypoints.json")).unwrap();
    let request = TrialRequest::from_json(&text).unwrap();
    request.validate().unwrap();
    assert_eq!(request.waypoints.len(), 4);
    assert_eq!(request.sensor_sets.len(), 6);
}

/// Runs the smoke request to `out`, with progress to `progress` when given.
#[cfg(test)]
fn run_smoke(out: &Path, progress: Option<&Path>) -> TrialReport {
    let request = path_text(&request_file("smoke.json"));
    let out = path_text(out);
    let progress = progress.map(path_text);
    let mut list = vec!["--request", &request, "--out", &out];
    if let Some(progress) = progress.as_deref() {
        list.extend(["--progress", progress]);
    }
    run_blocking(&args(&list).unwrap()).unwrap()
}

/// The smoke request end to end, with and without progress: the result
/// files are the same bytes, and the stream obeys its schema: `started`
/// first, `finished` last, route time never going back, every set in every
/// slice, and GNSS `scheduled_unavailable` exactly where the jammed set's
/// schedule says.
#[test]
fn the_smoke_request_runs_end_to_end_with_and_without_progress() {
    tracing_subscriber::fmt().with_test_writer().try_init().ok();
    let dir = tempfile::tempdir().unwrap();
    let (quiet, watched, stream) = (
        dir.path().join("quiet.json"),
        dir.path().join("watched.json"),
        dir.path().join("progress.ndjson"),
    );
    let report = std::thread::scope(|scope| {
        let quiet_run = scope.spawn(|| run_smoke(&quiet, None));
        run_smoke(&watched, Some(&stream));
        quiet_run.join().unwrap()
    });
    let (quiet_bytes, watched_bytes) = (
        std::fs::read(&quiet).unwrap(),
        std::fs::read(&watched).unwrap(),
    );
    assert!(quiet_bytes == watched_bytes, "progress changed the result");
    let read_back = TrialReport::from_json(&String::from_utf8(quiet_bytes).unwrap()).unwrap();
    assert_eq!(read_back, report);
    assert_eq!(report.sensor_sets.len(), 3);
    assert_eq!(report.route.events.len(), 2);
    assert_eq!(report.comparison.rows.len(), 3);
    // Wall-clock timing goes beside the result, never into it.
    let timing = std::fs::read_to_string(nav_trial::run::timing_path(&watched)).unwrap();
    let timing: sim_navigate::trial::TrialTiming = serde_json::from_str(&timing).unwrap();
    assert!(timing.sets.is_empty(), "{timing:?}");

    let text = std::fs::read_to_string(&stream).unwrap();
    let records: Vec<ProgressRecord> = text
        .lines()
        .map(|line| ProgressRecord::from_json_line(line).unwrap())
        .collect();
    check_stream(&records, &report, &watched);
}

#[cfg(test)]
fn check_stream(records: &[ProgressRecord], report: &TrialReport, out: &Path) {
    assert!(matches!(records.first(), Some(ProgressRecord::Started(_))));
    let Some(ProgressRecord::Finished(finished)) = records.last() else {
        panic!("the stream does not end with `finished`");
    };
    assert_eq!(
        finished.result_path.as_deref(),
        Some(path_text(out).as_str())
    );
    let mut last_t = f64::NEG_INFINITY;
    let mut slices: BTreeMap<u64, Vec<String>> = BTreeMap::new();
    let mut jammed = 0_u64;
    for record in records {
        let t_s = match record {
            ProgressRecord::Sample(sample) => {
                slices
                    .entry(sample.t_s.to_bits())
                    .or_default()
                    .push(sample.set.clone());
                let scheduled =
                    sample.set == "gnss-outage+baro+ins" && (40.0..=80.0).contains(&sample.t_s);
                if scheduled {
                    jammed = jammed.wrapping_add(1);
                }
                assert_eq!(
                    sample.sensors.gnss == SensorState::ScheduledUnavailable,
                    scheduled,
                    "{sample:?}"
                );
                sample.t_s
            }
            ProgressRecord::Event(event) => event.t_s,
            _ => continue,
        };
        assert!(t_s >= last_t, "{t_s} after {last_t}");
        last_t = t_s;
    }
    let names: Vec<String> = report.sensor_sets.iter().map(|s| s.name.clone()).collect();
    assert!(slices.len() > 100, "{}", slices.len());
    for sets in slices.values() {
        assert_eq!(sets, &names);
    }
    assert_eq!(jammed, 41);
}

/// A five-waypoint route completed with a defaults file: the completed
/// request written beside the result, run again as a request, gives the
/// same result bytes.
#[test]
fn a_route_completes_to_a_request_that_reruns_identically() {
    tracing_subscriber::fmt().with_test_writer().try_init().ok();
    let dir = tempfile::tempdir().unwrap();
    let from_route = dir.path().join("route.json");
    let from_request = dir.path().join("again.json");
    let route = path_text(&request_file("five-waypoints.route.json"));
    let defaults = path_text(&request_file("quick.defaults.json"));
    let out = path_text(&from_route);
    let routed = args(&["--route", &route, "--defaults", &defaults, "--out", &out]).unwrap();
    run_blocking(&routed).unwrap();
    let completed = completed_request_path(&from_route);
    let request = TrialRequest::from_json(&std::fs::read_to_string(&completed).unwrap()).unwrap();
    assert_eq!(request.waypoints.len(), 5);
    assert_eq!(request.waypoints[1].ident, "WP1");
    assert_eq!(request.sensor_sets.len(), 2);
    let (completed, again) = (path_text(&completed), path_text(&from_request));
    run_blocking(&args(&["--request", &completed, "--out", &again]).unwrap()).unwrap();
    let first = std::fs::read(&from_route).unwrap();
    assert!(first == std::fs::read(&from_request).unwrap());
    let report = TrialReport::from_json(&String::from_utf8(first).unwrap()).unwrap();
    assert_eq!(report.route.events.len(), 5, "{:?}", report.route.events);
}
