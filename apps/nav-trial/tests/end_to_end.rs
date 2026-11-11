//! The binary's path end to end on the shipped smoke request: parse the
//! command line, read the request, run the trial, write the result, and read
//! it back. The full four-waypoint example is checked to parse and validate;
//! running it is the trial tests' job.

#![cfg(not(target_arch = "wasm32"))]

use std::path::{Path, PathBuf};

use nav_trial::cli::{CliCommand, parse_args, run_files_blocking};
use sim_navigate::trial::TrialReport;

#[cfg(test)]
fn request_file(name: &str) -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("requests")
        .join(name)
}

#[test]
fn the_command_line_takes_a_request_and_an_output() {
    let args = |list: &[&str]| parse_args(list.iter().map(|s| (*s).to_owned()));
    let fly = CliCommand::Fly {
        request: "a.json".into(),
        out: "b.json".into(),
    };
    assert_eq!(
        args(&["--request", "a.json", "--out", "b.json"]).unwrap(),
        fly
    );
    assert_eq!(
        args(&["fly", "--out", "b.json", "--request", "a.json"]).unwrap(),
        fly
    );
    assert_eq!(
        args(&[
            "prepare",
            "--request",
            "a.json",
            "--onboard",
            "o",
            "--world",
            "w"
        ])
        .unwrap(),
        CliCommand::Prepare {
            request: "a.json".into(),
            onboard: "o".into(),
            world: "w".into(),
            cache: None,
        }
    );
    assert!(args(&["prepare", "--request", "a.json", "--onboard", "o"]).is_err());
    assert!(args(&["--request", "a.json"]).is_err());
    assert!(args(&["--out"]).is_err());
    assert!(args(&["--request", "a", "--request", "b", "--out", "c"]).is_err());
    assert!(args(&["--verbose"]).is_err());
}

#[test]
fn the_four_waypoint_example_is_a_valid_request() {
    let text = std::fs::read_to_string(request_file("four-waypoints.json")).unwrap();
    let request = sim_navigate::trial::TrialRequest::from_json(&text).unwrap();
    request.validate().unwrap();
    assert_eq!(request.waypoints.len(), 4);
    assert_eq!(request.sensor_sets.len(), 6);
}

#[test]
fn the_smoke_request_runs_end_to_end() {
    tracing_subscriber::fmt().with_test_writer().try_init().ok();
    let dir = tempfile::tempdir().unwrap();
    let out = dir.path().join("result.json");
    let args = parse_args([
        "--request".to_owned(),
        request_file("smoke.json").display().to_string(),
        "--out".to_owned(),
        out.display().to_string(),
    ])
    .unwrap();
    let CliCommand::Fly {
        request,
        out: written,
    } = args
    else {
        panic!("{args:?}");
    };
    let report = run_files_blocking(&request, &written).unwrap();
    let written = std::fs::read_to_string(&out).unwrap();
    let read_back = TrialReport::from_json(&written).unwrap();
    assert_eq!(read_back, report);
    assert_eq!(report.sensor_sets.len(), 3);
    assert_eq!(report.route.events.len(), 2);
    for set in &report.sensor_sets {
        tracing::info!(
            set = set.name,
            horizontal_rms_m = set.summary.horizontal_error_m.rms,
            horizontal_last_m = set.summary.horizontal_error_m.last,
            position_nees = set.summary.mean_position_nees,
            stars = ?set.stars,
            outages = ?set.summary.outages,
            "example set"
        );
    }
}
