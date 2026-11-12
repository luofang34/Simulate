//! End-to-end headless runner tests covering nominal RNP/NEES telemetry,
//! Monte Carlo batching, multi-window GNSS spoofing, and offline disk caching.

#![cfg(not(target_arch = "wasm32"))]

use std::fs;

use link_replay::cache::CachePolicy;
use link_replay::cli::parse_args;
use link_replay::presets::ScenarioFile;
use link_replay::runner::{ReplayReport, run_blocking, run_with_cache_policy};
use sim_chain_trial::{SpoofKind, SpoofWindowSpec, TelemetrySample};
use tempfile::TempDir;

#[test]
fn nominal_and_monte_carlo_runs_emit_ndjson_and_summary_metrics() {
    let dir = TempDir::new().unwrap();
    let trace_path = dir.path().join("trace.ndjson");
    let summary_path = dir.path().join("summary.json");

    let args = parse_args(
        [
            "--preset",
            "zaragoza",
            "--no-camera",
            "--duration-s",
            "10",
            "--sample-hz",
            "2",
            "--seeds",
            "1..=2",
            "--rnp-nm",
            "0.3",
            "--out",
            trace_path.to_str().unwrap(),
            "--summary",
            summary_path.to_str().unwrap(),
        ]
        .map(str::to_owned),
    )
    .unwrap();

    let report = run_blocking(&args).unwrap();
    assert_eq!(report.camera_backend, "none");
    assert_eq!(report.seed_summaries.len(), 2);
    let mc = report.monte_carlo.unwrap();
    assert_eq!(mc.runs, 2);
    assert!(mc.mean_horiz_p95_m < 30.0);
    assert!((mc.mean_rnp_containment - 1.0).abs() < 1e-9);

    let ndjson = fs::read_to_string(&trace_path).unwrap();
    let samples: Vec<TelemetrySample> = ndjson
        .lines()
        .map(|line| serde_json::from_str(line).unwrap())
        .collect();
    assert_eq!(samples.len(), 40);
    let last = samples.last().unwrap();
    assert!(last.inputs.pitot_ok);
    assert!(last.inputs.ias_mps.unwrap() > 35.0);
    let tse = last.guidance_rnp.tse_cross_track_m.unwrap();
    let fte = last.guidance_rnp.fte_cross_track_m.unwrap();
    let nse = last.guidance_rnp.nse_cross_track_m.unwrap();
    assert!((tse - (fte + nse)).abs() < 1e-9);

    let parsed_report: ReplayReport =
        serde_json::from_str(&fs::read_to_string(&summary_path).unwrap()).unwrap();
    assert_eq!(parsed_report.summary.sample_count, 20);
}

#[test]
fn scenario_file_multi_window_spoofing_records_exclusion_and_window_end() {
    let dir = TempDir::new().unwrap();
    let scenario_path = dir.path().join("spoof_scenario.json");
    let trace_path = dir.path().join("spoof.ndjson");

    let scenario = ScenarioFile {
        name: Some("ramp-window-test".to_owned()),
        link: None,
        preset: Some("zaragoza".to_owned()),
        spec: None,
        fly: Some("ins.baro.gnss".to_owned()),
        spoof_windows: vec![SpoofWindowSpec {
            start_s: 5.0,
            end_s: Some(26.0),
            kind: SpoofKind::Ramp,
            offset_enu_m: [140.0, 0.0, 0.0],
            rate_enu_mps: [20.0, 0.0, 0.0],
            velocity_enu_mps: None,
        }],
        outages: Vec::new(),
        hold: None,
        rnp_limit_nm: Some(1.0),
    };
    fs::write(
        &scenario_path,
        serde_json::to_string_pretty(&scenario).unwrap(),
    )
    .unwrap();

    let args = parse_args(
        [
            "--scenario-file",
            scenario_path.to_str().unwrap(),
            "--no-camera",
            "--duration-s",
            "32",
            "--sample-hz",
            "1",
            "--out",
            trace_path.to_str().unwrap(),
        ]
        .map(str::to_owned),
    )
    .unwrap();

    let report = run_blocking(&args).unwrap();
    let samples: Vec<TelemetrySample> = fs::read_to_string(&trace_path)
        .unwrap()
        .lines()
        .map(|l| serde_json::from_str(l).unwrap())
        .collect();

    assert!(!samples[2].inputs.spoof_active);
    assert!(samples[12].inputs.spoof_active);
    assert_eq!(samples[12].inputs.spoof_kind, SpoofKind::Ramp);
    assert!(!samples[29].inputs.spoof_active);

    assert!(report.summary.spoofing.spoof_injected);
    assert!(report.summary.spoofing.time_to_detect_s.is_some());
    assert!(report.summary.spoofing.time_to_exclude_s.is_some());
    assert!(
        report
            .summary
            .spoofing
            .max_error_before_exclusion_m
            .unwrap()
            > 0.0
    );
}

#[test]
fn visual_package_disk_cache_hits_offline_with_bit_identical_flight() {
    let cache_dir = TempDir::new().unwrap();
    let short_visual_link = "v=2&r=IF,A:41.7/-1.05;TF,B:41.7/-1.03&h=2000.0&hd=msl&\
t=20260615T120400Z&veh=c172&seed=1&nav=ins.baro.visual;ins.baro.gnss&fly=ins.baro.gnss&\
cam=160x120,67.4,90,0.5&go=1&mb=200";

    let offline_args = parse_args(
        [
            "--link",
            short_visual_link,
            "--duration-s",
            "4",
            "--sample-hz",
            "1",
            "--camera-backend",
            "cpu",
            "--offline",
            "--cache-dir",
            cache_dir.path().to_str().unwrap(),
        ]
        .map(str::to_owned),
    )
    .unwrap();

    // 1. Cold --offline run must fail without issuing any network requests.
    assert!(run_blocking(&offline_args).is_err());

    // 2. Seed local disk cache deterministically using procedural fallback tiles.
    let seeded = run_with_cache_policy(&offline_args, CachePolicy::SyntheticFallback).unwrap();
    assert_eq!(seeded.tile_cache.network_fetches, 0);
    assert!(seeded.tile_cache.misses > 0);
    assert!(seeded.tile_cache.bytes_written > 0);

    // 3. Re-run in strict --offline mode: 0 misses, 0 network calls, identical flight summary.
    let warm = run_blocking(&offline_args).unwrap();
    assert_eq!(warm.tile_cache.misses, 0);
    assert_eq!(warm.tile_cache.network_fetches, 0);
    assert!(warm.tile_cache.hits > 0);
    assert_eq!(warm.summary, seeded.summary);
}
