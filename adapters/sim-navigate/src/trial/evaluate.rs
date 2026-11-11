//! A whole trial: fly, sense once, replay each sensor set, score.
//!
//! Every set advances one sample slice at a time in turn, in open loop over
//! the one recorded flight and in closed loop each over its own, so a
//! progress monitor sees every set's curve grow together. Each set owns its
//! navigator, and in closed loop its vehicle and sensors, so the order in
//! which they are stepped changes nothing in any of them.

mod flights;
mod replays;

use std::sync::Arc;

use navigate_contract::NavigationSolution;
use sha2::{Digest, Sha256};
use sim_core::SimInstant;

use super::availability::{ScheduledInterval, SetAvailability};
use super::closed::{ClosedLoopOutcome, ClosedLoopReport};
use super::comparison::comparison;
use super::epoch::parse_utc;
use super::map_store::{DirectoryStores, MapStores};
use super::maps::OpenedPackage;
use super::onboard::trial_navigator_with_stores;
use super::progress::{NoProgress, ProgressSink, started};
use super::record::{SensorLog, record};
use super::replay::{SensorTallies, SetRun, SetSelection, StarAvailability};
use super::report::{
    InputIdentity, REPORT_SCHEMA, RouteReport, SeriesSample, SetReport, TrialReport,
};
use super::route::{RouteFlight, STEP_HZ, fly_route};
use super::score::summarise;
use super::terrain_camera::TerrainCamera;
use super::timing::TrialTiming;
use super::vertical::vertical_plan;
use super::{GuidanceSource, SensorSetSpec, TrialError, TrialRequest, model_statements};
use crate::{Estimate, NavMeans};

/// A flown and sensed trial, before any sensor set has been replayed.
#[derive(Debug, Clone)]
pub struct RecordedTrial {
    /// The true flight.
    pub flight: RouteFlight,
    /// What the sensors reported along it.
    pub log: SensorLog,
    /// Wall-clock seconds spent rendering the terrain camera's frames; not
    /// a function of the request.
    pub terrain_render_s: f64,
}

/// A trial's report with the estimates each set's filter published at the
/// scored samples, in set order.
#[derive(Debug, Clone)]
pub struct TrialOutcome {
    /// The result file's content.
    pub report: TrialReport,
    /// Each set's estimates, by sample.
    pub estimates: Vec<Vec<Estimate<(SimInstant, NavigationSolution)>>>,
    /// Where the wall-clock time went; never part of the report.
    pub timing: TrialTiming,
}

/// A closed-loop trial's report with each set's full outcome, in set order.
#[derive(Debug, Clone)]
pub struct ClosedTrialOutcome {
    /// The result file's content.
    pub report: TrialReport,
    /// Each set's flight, with the pilot trace when asked for.
    pub sets: Vec<ClosedLoopOutcome>,
}

/// What every set of a run shares.
#[derive(Clone, Copy)]
pub(super) struct SetContext {
    pub(super) sample_steps: usize,
}

/// Validates `request`, flies its route and senses the flight; with a
/// visual set it reads the world package from its store to render the
/// terrain camera.
pub fn record_trial_blocking(request: &TrialRequest) -> Result<RecordedTrial, TrialError> {
    record_trial_with_stores(request, &DirectoryStores)
}

/// [`record_trial_blocking`], reading the world package from the store
/// `stores` resolves.
pub fn record_trial_with_stores(
    request: &TrialRequest,
    stores: &dyn MapStores,
) -> Result<RecordedTrial, TrialError> {
    request.validate()?;
    let flight = fly_route(request)?;
    let mut log = record(request, &flight.truth)?;
    let mut terrain_render_s = 0.0;
    if let Some(camera) = terrain_camera(request, stores)? {
        let started = web_time::Instant::now();
        log.terrain = camera.render_along(&flight.truth)?;
        terrain_render_s = started.elapsed().as_secs_f64();
    }
    Ok(RecordedTrial {
        flight,
        log,
        terrain_render_s,
    })
}

/// The request's terrain camera over its world package, when a set takes
/// visual fixes.
pub(super) fn terrain_camera(
    request: &TrialRequest,
    stores: &dyn MapStores,
) -> Result<Option<TerrainCamera>, TrialError> {
    let (true, Some(spec), Some(maps)) = (
        request.uses_visual(),
        request.terrain_camera,
        request.maps.as_ref(),
    ) else {
        return Ok(None);
    };
    let world =
        OpenedPackage::open_blocking(&maps.world, stores, "maps.world")?.scenery_blocking()?;
    let seed = request.seeds.terrain_camera.unwrap_or_default();
    let start = parse_utc(&request.epoch_utc)?;
    Ok(Some(TerrainCamera::new(
        Arc::new(world),
        spec,
        seed,
        start,
    )?))
}

/// Runs `request` end to end, in open or closed loop as it asks.
pub fn run_trial_blocking(request: &TrialRequest) -> Result<TrialReport, TrialError> {
    run_trial_with_progress_blocking(request, &mut NoProgress)
}

/// Runs `request` end to end, handing `sink` a `started` record and then
/// every slice's events and samples. The report is the same whatever the
/// sink does with them.
pub fn run_trial_with_progress_blocking(
    request: &TrialRequest,
    sink: &mut dyn ProgressSink,
) -> Result<TrialReport, TrialError> {
    run_trial_timed_blocking(request, sink).map(|(report, _)| report)
}

/// [`run_trial_with_progress_blocking`], with where the wall-clock time went.
pub fn run_trial_timed_blocking(
    request: &TrialRequest,
    sink: &mut dyn ProgressSink,
) -> Result<(TrialReport, TrialTiming), TrialError> {
    run_trial_with_stores(request, &DirectoryStores, sink)
}

/// Runs `request` end to end for a host that keeps its map packages in the
/// stores `stores` resolves, handing `sink` the progress; returns the
/// report and where the wall-clock time went.
pub fn run_trial_with_stores(
    request: &TrialRequest,
    stores: &dyn MapStores,
    sink: &mut dyn ProgressSink,
) -> Result<(TrialReport, TrialTiming), TrialError> {
    request.validate()?;
    let (legs, _) = vertical_plan(&request.waypoints, &request.vehicle)?;
    let digest = identity(request)?.request_sha256;
    sink.record(&started(request, digest, &legs))
        .map_err(TrialError::Progress)?;
    match request.vehicle.guidance {
        GuidanceSource::Truth => {
            let recorded = record_trial_with_stores(request, stores)?;
            let outcome = replays::open_trial(request, &recorded, stores, sink)?;
            Ok((outcome.report, outcome.timing))
        }
        GuidanceSource::Estimate => Ok((
            flights::closed_trial(request, false, sink)?.report,
            TrialTiming::default(),
        )),
    }
}

/// Flies every sensor set of `request` in closed loop, each on its own
/// truth; `trace` keeps each pilot's inputs and outputs.
pub fn run_closed_trial(
    request: &TrialRequest,
    trace: bool,
) -> Result<ClosedTrialOutcome, TrialError> {
    flights::closed_trial(request, trace, &mut NoProgress)
}

/// Replays every sensor set of `request` over `recorded` and scores each
/// against the recorded flight's truth.
pub fn evaluate_blocking(
    request: &TrialRequest,
    recorded: &RecordedTrial,
) -> Result<TrialOutcome, TrialError> {
    evaluate_with_progress_blocking(request, recorded, &mut NoProgress)
}

/// [`evaluate_blocking`], handing `sink` every slice's events and samples.
pub fn evaluate_with_progress_blocking(
    request: &TrialRequest,
    recorded: &RecordedTrial,
    sink: &mut dyn ProgressSink,
) -> Result<TrialOutcome, TrialError> {
    replays::open_trial(request, recorded, &DirectoryStores, sink)
}

pub(super) fn whole_steps(field: &'static str, seconds: f64) -> Result<usize, TrialError> {
    let steps = seconds * STEP_HZ as f64;
    if !(steps.is_finite() && steps >= 1.0 && (steps - steps.round()).abs() < 1e-9) {
        return Err(TrialError::InvalidRequest {
            field,
            detail: format!("{seconds} s is not a whole number of steps"),
        });
    }
    Ok(steps.round() as usize)
}

/// The scored and progress sample periods in steps. A scored sample is a
/// progress sample when its step is a whole multiple of the progress
/// period, so a progress period finer than the scored one takes every
/// scored sample.
fn sample_periods(request: &TrialRequest) -> Result<(usize, usize), TrialError> {
    let output = &request.output;
    let sample = whole_steps("output.sample_period_s", output.sample_period_s)?;
    let progress = whole_steps("output.progress_period_s", output.progress_period_s)?;
    Ok((sample, progress))
}

/// The navigator of `set` of `request`, aligned from `log`: what a trial
/// runs, for stepping by hand. A visual set's navigator reads its onboard
/// package from the store.
pub fn replay_set_blocking(
    request: &TrialRequest,
    set: &SensorSetSpec,
    log: &SensorLog,
) -> Result<SetRun, TrialError> {
    replay_set_with_stores(request, set, log, &DirectoryStores)
}

/// [`replay_set_blocking`], reading the onboard package from the store
/// `stores` resolves.
pub fn replay_set_with_stores(
    request: &TrialRequest,
    set: &SensorSetSpec,
    log: &SensorLog,
    stores: &dyn MapStores,
) -> Result<SetRun, TrialError> {
    let selection = SetSelection {
        means: set.ablation_set(),
        availability: SetAvailability::resolve(request, set)?,
    };
    SetRun::new(
        log,
        selection,
        trial_navigator_with_stores(request, set, stores)?,
    )
}

/// What a set's run leaves for its report.
pub(super) struct SetResult {
    pub(super) series: Vec<SeriesSample>,
    pub(super) tallies: SensorTallies,
    pub(super) intervals: Vec<(ScheduledInterval, SensorTallies)>,
    pub(super) stars: StarAvailability,
    pub(super) terrain: Option<super::terrain_report::TerrainSetReport>,
    pub(super) terrain_wall: (f64, u64, f64),
    pub(super) closed_loop: Option<ClosedLoopReport>,
}

fn set_report(set: &SensorSetSpec, result: SetResult) -> SetReport {
    let means = set.ablation_set();
    SetReport {
        name: set.name.clone(),
        means: means.means().collect(),
        availability: set.availability.clone(),
        summary: summarise(&result.series, &result.intervals),
        observations: result.tallies,
        stars: means.contains(NavMeans::Celestial).then_some(result.stars),
        series: result.series,
        terrain: result.terrain,
        closed_loop: result.closed_loop,
    }
}

/// The result file of `request` over `route` with each set's `results`.
fn trial_report(
    request: &TrialRequest,
    route: RouteReport,
    results: Vec<SetResult>,
    maps: Option<super::maps::MapsReport>,
) -> Result<TrialReport, TrialError> {
    let sensor_sets: Vec<SetReport> = request
        .sensor_sets
        .iter()
        .zip(results)
        .map(|(set, result)| set_report(set, result))
        .collect();
    Ok(TrialReport {
        schema: REPORT_SCHEMA.into(),
        inputs: identity(request)?,
        models: model_statements(request.vehicle.guidance, request.uses_visual()),
        seeds: request.seeds,
        route,
        availability: request.availability.clone(),
        gnss_horizontal_sigma_m: request.gnss.horizontal_sigma_m(),
        comparison: comparison(&sensor_sets, request.output.horizontal_error_threshold_m),
        maps,
        sensor_sets,
    })
}

fn identity(request: &TrialRequest) -> Result<InputIdentity, TrialError> {
    let compact = serde_json::to_string(request).map_err(|source| TrialError::Json {
        what: "request",
        source,
    })?;
    let digest = Sha256::digest(compact.as_bytes());
    Ok(InputIdentity {
        request_sha256: digest.iter().map(|byte| format!("{byte:02x}")).collect(),
        sim_navigate_version: env!("CARGO_PKG_VERSION").into(),
        epoch_utc: request.epoch_utc.clone(),
    })
}
