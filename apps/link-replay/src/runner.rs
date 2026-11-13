//! Executes single-seed or multi-seed Monte Carlo headless flights, streaming
//! NDJSON [`TelemetrySample`] records and writing run-level [`ReplayReport`].

use std::fs::File;
use std::io::{BufWriter, Write};
use std::path::Path;
use std::sync::Arc;
use std::time::Instant;

use navigate_data::MemoryStore;
use serde::{Deserialize, Serialize};
use sim_chain_trial::{
    ChainTrial, ChainTrialSpec, Outputs, StepOutcome, TelemetryConfig, TelemetryRecorder,
    TelemetrySample, TelemetrySummary,
};
use sim_geo::Geodetic;
use sim_link_host::{CameraWorld, page_stores};
use sim_navigate::trial::{EventKind, ProgressRecord};
use sim_scenery::CatalogScenery;
use tracing::info;

use crate::ReplayError;
use crate::cache::{CachePolicy, CacheStatsSnapshot, DiskCachedTransport};
use crate::cli::ReplayArgs;
use crate::presets::{PreparedScenario, apply_seed};
use crate::seal::seal_blocking;
use crate::wgpu_camera::TerrainCameraRenderer;

/// Complete native flight test report written to `--summary`.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ReplayReport {
    /// Scenario name or source label.
    pub scenario: String,
    /// Sample cadence used during recording, Hz.
    pub sample_hz: f64,
    /// Active terrain camera backend (`"wgpu"`, `"cpu"`, or `"none"`).
    pub camera_backend: String,
    /// Total wall-clock execution time, milliseconds.
    pub wall_time_ms: u64,
    /// Local disk tile cache hit/miss/network statistics.
    pub tile_cache: CacheStatsSnapshot,
    /// Primary run summary (the sole run for single-seed, or first seed in batch).
    pub summary: TelemetrySummary,
    /// Per-seed summaries when `--seeds` runs a Monte Carlo batch (`len > 1`).
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub seed_summaries: Vec<TelemetrySummary>,
    /// Aggregate Monte Carlo statistics across seeds when `seed_summaries` is non-empty.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub monte_carlo: Option<MonteCarloAggregate>,
}

/// Aggregate error and containment statistics across a multi-seed batch.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct MonteCarloAggregate {
    /// Number of seeds executed.
    pub runs: usize,
    /// Mean across seeds of `horizontal_error_m.p95`, metres.
    pub mean_horiz_p95_m: f64,
    /// Worst-case `horizontal_error_m.max` across all seeds, metres.
    pub worst_horiz_max_m: f64,
    /// Mean across seeds of `tse_cross_track_m.p95`, metres.
    pub mean_tse_p95_m: f64,
    /// Mean RNP containment fraction across all seeds (`0.0..=1.0`).
    pub mean_rnp_containment: f64,
    /// Mean 2D horizontal NEES across seeds.
    pub mean_nees_horiz_2d: Option<f64>,
}

/// Runs `args` to completion with the default cache policy (`Offline` when
/// `args.offline`, else `Online`).
///
/// # Errors
///
/// Returns [`ReplayError`] on scenario, cache, package, rendering, trial, or I/O failure.
pub fn run_blocking(args: &ReplayArgs) -> Result<ReplayReport, ReplayError> {
    let policy = if args.offline {
        CachePolicy::Offline
    } else {
        CachePolicy::Online
    };
    run_with_cache_policy(args, policy)
}

/// Runs `args` using an explicit [`CachePolicy`] (allowing offline integration
/// tests to seed a temporary cache via [`CachePolicy::SyntheticFallback`] and
/// re-fly under [`CachePolicy::Offline`]).
///
/// # Errors
///
/// Returns [`ReplayError`] on scenario, cache, package, rendering, trial, or I/O failure.
pub fn run_with_cache_policy(
    args: &ReplayArgs,
    policy: CachePolicy,
) -> Result<ReplayReport, ReplayError> {
    let started_wall = Instant::now();
    let prepared = PreparedScenario::load(&args.source)?;
    let transport = DiskCachedTransport::new(args.cache_dir.clone(), policy);

    let sealed = match &prepared.link_bundle {
        Some((link, hosted)) => seal_blocking(link, hosted, &transport, args.no_camera)?,
        None => None,
    };
    let identity = sealed.as_ref().map(|s| s.summary.identity);
    let memory = sealed.map_or_else(MemoryStore::default, |s| s.memory);
    let base_spec = prepared.build_spec(identity, args.fly.as_deref(), args.no_camera)?;
    let rnp_limit_nm = args.rnp_limit_nm.or(prepared.rnp_limit_nm);

    let mut camera_and_scenery = open_camera_world(&base_spec, &memory, identity, args)?;
    let backend_label = camera_and_scenery
        .as_ref()
        .map_or("none", |(cam, _)| cam.backend_name())
        .to_owned();

    let mut out_writer = open_ndjson_writer(args.out.as_deref())?;
    let seed_list: Vec<Option<u64>> = if args.seeds.is_empty() {
        vec![None]
    } else {
        args.seeds.iter().copied().map(Some).collect()
    };

    let mut summaries = Vec::with_capacity(seed_list.len());
    for seed_opt in seed_list {
        let mut spec = base_spec.clone();
        if let Some(seed) = seed_opt {
            apply_seed(&mut spec, seed);
        }
        let summary = execute_single_flight(
            &spec,
            &memory,
            camera_and_scenery.as_mut(),
            args,
            TelemetryConfig {
                rnp_limit_nm,
                seed: seed_opt,
            },
            out_writer.as_mut(),
        )?;
        summaries.push(summary);
    }

    if let Some((path, writer)) = out_writer.as_mut() {
        writer.flush().map_err(|source| ReplayError::Io {
            path: path.clone(),
            source,
        })?;
    }

    let report = build_report(
        &prepared.label,
        args.sample_hz,
        backend_label,
        started_wall.elapsed().as_millis() as u64,
        transport.stats(),
        summaries,
    )?;
    if let Some(summary_path) = &args.summary {
        write_summary_file(summary_path, &report)?;
    }
    log_completion(&report);
    Ok(report)
}

fn open_camera_world(
    spec: &ChainTrialSpec,
    memory: &MemoryStore,
    identity: Option<navigate_data::Sha256Digest>,
    args: &ReplayArgs,
) -> Result<Option<(TerrainCameraRenderer, Arc<CatalogScenery>)>, ReplayError> {
    if args.no_camera || spec.request.terrain_camera.is_none() {
        return Ok(None);
    }
    let Some(pkg_id) = identity else {
        return Ok(None);
    };
    let world = CameraWorld::open_blocking(memory.clone(), pkg_id)?;
    let scenery = world.scenery();
    let renderer = TerrainCameraRenderer::open(world, args.camera_backend);
    Ok(Some((renderer, scenery)))
}

fn execute_single_flight(
    spec: &ChainTrialSpec,
    memory: &MemoryStore,
    mut camera: Option<&mut (TerrainCameraRenderer, Arc<CatalogScenery>)>,
    args: &ReplayArgs,
    telemetry_cfg: TelemetryConfig,
    mut ndjson_out: Option<&mut (String, BufWriter<File>)>,
) -> Result<TelemetrySummary, ReplayError> {
    let mut trial = ChainTrial::new_inspecting(spec, &page_stores(memory.clone()), true)?;
    let mut recorder = TelemetryRecorder::new(spec, telemetry_cfg);
    let period_s = 1.0 / args.sample_hz;
    let mut step_idx: u64 = 1;
    let mut hold_started_s: Option<f64> = None;

    while trial.now_s() < args.duration_s {
        if hold_started_s.is_some_and(|start| trial.now_s() - start >= args.hold_s) {
            break;
        }
        let target_s = (step_idx as f64 * period_s).min(args.duration_s);
        step_idx = step_idx.wrapping_add(1);
        advance_to_target(
            &mut trial,
            target_s,
            camera.as_deref_mut(),
            &mut hold_started_s,
        )?;

        let scenery_ref = camera.as_ref().map(|(_, s)| Arc::clone(s));
        let sample = recorder.sample_with_terrain(&mut trial, |lat_deg, lon_deg| {
            let sc = scenery_ref.as_ref()?;
            let geo = Geodetic::from_degrees(lat_deg, lon_deg, 0.0);
            sc.orthometric_ground_height_m(&geo)
                .ok()
                .flatten()
                .map(|h| h.0)
        })?;
        if let Some((path, writer)) = ndjson_out.as_deref_mut() {
            write_sample_line(path, writer, &sample)?;
        }
    }
    Ok(recorder.finish())
}

fn advance_to_target(
    trial: &mut ChainTrial,
    target_s: f64,
    mut camera: Option<&mut (TerrainCameraRenderer, Arc<CatalogScenery>)>,
    hold_started_s: &mut Option<f64>,
) -> Result<(), ReplayError> {
    loop {
        match trial.step_until(target_s)? {
            StepOutcome::Advanced(outputs) => {
                note_holding(&outputs, hold_started_s);
                return Ok(());
            }
            StepOutcome::FrameWanted(request, outputs) => {
                note_holding(&outputs, hold_started_s);
                let Some((renderer, _)) = camera.as_deref_mut() else {
                    return Err(sim_link_host::HostError::Render {
                        detail: format!(
                            "camera frame {} requested without camera world",
                            request.index
                        ),
                    }
                    .into());
                };
                let luma = renderer.render_frame(&request)?;
                trial.deliver_frame(request.index, &luma)?;
            }
        }
    }
}

fn note_holding(outputs: &Outputs, hold_started_s: &mut Option<f64>) {
    for rec in &outputs.records {
        if let ProgressRecord::Event(ev) = rec
            && matches!(ev.kind, EventKind::HoldingStarted { .. })
        {
            hold_started_s.get_or_insert(ev.t_s);
        }
    }
}

fn open_ndjson_writer(
    path: Option<&Path>,
) -> Result<Option<(String, BufWriter<File>)>, ReplayError> {
    let Some(p) = path else {
        return Ok(None);
    };
    let display = p.display().to_string();
    if let Some(parent) = p.parent().filter(|par| !par.as_os_str().is_empty()) {
        std::fs::create_dir_all(parent).map_err(|source| ReplayError::Io {
            path: display.clone(),
            source,
        })?;
    }
    let file = File::create(p).map_err(|source| ReplayError::Io {
        path: display.clone(),
        source,
    })?;
    Ok(Some((display, BufWriter::new(file))))
}

fn write_sample_line(
    path: &str,
    writer: &mut BufWriter<File>,
    sample: &TelemetrySample,
) -> Result<(), ReplayError> {
    serde_json::to_writer(&mut *writer, sample).map_err(|source| ReplayError::Json {
        what: path.to_owned(),
        source,
    })?;
    writeln!(writer).map_err(|source| ReplayError::Io {
        path: path.to_owned(),
        source,
    })
}

fn write_summary_file(path: &Path, report: &ReplayReport) -> Result<(), ReplayError> {
    let display = path.display().to_string();
    if let Some(parent) = path.parent().filter(|par| !par.as_os_str().is_empty()) {
        std::fs::create_dir_all(parent).map_err(|source| ReplayError::Io {
            path: display.clone(),
            source,
        })?;
    }
    let json = serde_json::to_string_pretty(report).map_err(|source| ReplayError::Json {
        what: display.clone(),
        source,
    })?;
    std::fs::write(path, json).map_err(|source| ReplayError::Io {
        path: display,
        source,
    })
}

fn build_report(
    scenario: &str,
    sample_hz: f64,
    camera_backend: String,
    wall_time_ms: u64,
    tile_cache: CacheStatsSnapshot,
    summaries: Vec<TelemetrySummary>,
) -> Result<ReplayReport, ReplayError> {
    let primary = summaries
        .first()
        .cloned()
        .ok_or_else(|| ReplayError::Usage("no runs executed".to_owned()))?;
    let (seed_summaries, monte_carlo) = if summaries.len() > 1 {
        let agg = aggregate_monte_carlo(&summaries);
        (summaries, Some(agg))
    } else {
        (Vec::new(), None)
    };
    Ok(ReplayReport {
        scenario: scenario.to_owned(),
        sample_hz,
        camera_backend,
        wall_time_ms,
        tile_cache,
        summary: primary,
        seed_summaries,
        monte_carlo,
    })
}

fn aggregate_monte_carlo(runs: &[TelemetrySummary]) -> MonteCarloAggregate {
    let count = runs.len().max(1) as f64;
    let mean_horiz_p95_m = runs.iter().map(|r| r.horizontal_error_m.p95).sum::<f64>() / count;
    let worst_horiz_max_m = runs
        .iter()
        .map(|r| r.horizontal_error_m.max)
        .fold(0.0_f64, f64::max);
    let mean_tse_p95_m = runs.iter().map(|r| r.tse_cross_track_m.p95).sum::<f64>() / count;
    let mean_rnp_containment = runs.iter().map(|r| r.rnp_containment_fraction).sum::<f64>() / count;
    let nees_vals: Vec<f64> = runs
        .iter()
        .filter_map(|r| r.nees_horiz_2d.map(|n| n.mean))
        .collect();
    let mean_nees_horiz_2d =
        (!nees_vals.is_empty()).then(|| nees_vals.iter().sum::<f64>() / nees_vals.len() as f64);
    MonteCarloAggregate {
        runs: runs.len(),
        mean_horiz_p95_m,
        worst_horiz_max_m,
        mean_tse_p95_m,
        mean_rnp_containment,
        mean_nees_horiz_2d,
    }
}

fn log_completion(report: &ReplayReport) {
    let s = &report.summary;
    info!(
        scenario = %report.scenario,
        steered = %s.steered_set,
        samples = s.sample_count,
        duration_s = s.duration_s,
        horiz_p50_m = s.horizontal_error_m.p50,
        horiz_p95_m = s.horizontal_error_m.p95,
        tse_p95_m = s.tse_cross_track_m.p95,
        rnp_containment = s.rnp_containment_fraction,
        camera_backend = %report.camera_backend,
        cache_hits = report.tile_cache.hits,
        network_fetches = report.tile_cache.network_fetches,
        wall_time_ms = report.wall_time_ms,
        "headless flight complete"
    );
}
