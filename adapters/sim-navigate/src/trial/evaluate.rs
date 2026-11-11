//! A whole trial: fly, sense once, replay each sensor set, score.
//!
//! Every set advances one sample slice at a time in turn, in open loop over
//! the one recorded flight and in closed loop each over its own, so a
//! progress monitor sees every set's curve grow together. Each set owns its
//! navigator, and in closed loop its vehicle and sensors, so the order in
//! which they are stepped changes nothing in any of them.

mod flights;
mod replays;

use navigate_contract::NavigationSolution;
use navigate_startrack::PairCatalogue;
use sha2::{Digest, Sha256};
use sim_core::SimInstant;

use super::availability::{ScheduledInterval, SetAvailability};
use super::closed::{ClosedLoopOutcome, ClosedLoopReport};
use super::comparison::comparison;
use super::epoch::parse_utc;
use super::gnss::GnssReceiver;
use super::progress::{NoProgress, ProgressSink, started};
use super::record::{SensorLog, record};
use super::replay::{STAR_SOURCE, SensorTallies, SetRun, SetSelection, StarAvailability};
use super::report::{
    InputIdentity, REPORT_SCHEMA, RouteReport, SeriesSample, SetReport, TrialReport,
};
use super::route::{RouteFlight, STEP_HZ, fly_route};
use super::score::summarise;
use super::stars::sighting::{StarSighting, pair_table};
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
}

/// A trial's report with the estimates each set's filter published at the
/// scored samples, in set order.
#[derive(Debug, Clone)]
pub struct TrialOutcome {
    /// The result file's content.
    pub report: TrialReport,
    /// Each set's estimates, by sample.
    pub estimates: Vec<Vec<Estimate<(SimInstant, NavigationSolution)>>>,
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
pub(super) struct SetContext<'p> {
    pub(super) pairs: Option<&'p PairCatalogue>,
    pub(super) sample_steps: usize,
}

/// Validates `request`, flies its route and senses the flight.
pub fn record_trial(request: &TrialRequest) -> Result<RecordedTrial, TrialError> {
    request.validate()?;
    let flight = fly_route(request)?;
    let log = record(request, &flight.truth)?;
    Ok(RecordedTrial { flight, log })
}

/// Runs `request` end to end, in open or closed loop as it asks.
pub fn run_trial(request: &TrialRequest) -> Result<TrialReport, TrialError> {
    run_trial_with_progress(request, &mut NoProgress)
}

/// Runs `request` end to end, handing `sink` a `started` record and then
/// every slice's events and samples. The report is the same whatever the
/// sink does with them.
pub fn run_trial_with_progress(
    request: &TrialRequest,
    sink: &mut dyn ProgressSink,
) -> Result<TrialReport, TrialError> {
    request.validate()?;
    let (legs, _) = vertical_plan(&request.waypoints, &request.vehicle)?;
    let digest = identity(request)?.request_sha256;
    sink.record(&started(request, digest, &legs))
        .map_err(TrialError::Progress)?;
    match request.vehicle.guidance {
        GuidanceSource::Truth => {
            let recorded = record_trial(request)?;
            Ok(evaluate_with_progress(request, &recorded, sink)?.report)
        }
        GuidanceSource::Estimate => Ok(flights::closed_trial(request, false, sink)?.report),
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
pub fn evaluate(
    request: &TrialRequest,
    recorded: &RecordedTrial,
) -> Result<TrialOutcome, TrialError> {
    evaluate_with_progress(request, recorded, &mut NoProgress)
}

/// [`evaluate`], handing `sink` every slice's events and samples.
pub fn evaluate_with_progress(
    request: &TrialRequest,
    recorded: &RecordedTrial,
    sink: &mut dyn ProgressSink,
) -> Result<TrialOutcome, TrialError> {
    replays::open_trial(request, recorded, sink)
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

/// The lost-in-space pair table when any set of `request` takes stars.
pub fn star_pair_table(request: &TrialRequest) -> Result<Option<PairCatalogue>, TrialError> {
    if request.uses_stars() {
        Ok(Some(pair_table(parse_utc(&request.epoch_utc)?)?))
    } else {
        Ok(None)
    }
}

/// The navigator `set` of `request` replays `log` with, aligned, its star
/// tracker on `pairs` when the set takes stars: what a trial runs, for
/// stepping by hand.
pub fn replay_set<'p>(
    request: &TrialRequest,
    set: &SensorSetSpec,
    pairs: Option<&'p PairCatalogue>,
    log: &SensorLog,
) -> Result<SetRun<'p>, TrialError> {
    let context = SetContext {
        pairs,
        sample_steps: 1,
    };
    set_run(request, set, context, log)
}

/// The navigator of `set`, aligned from `log`.
pub(super) fn set_run<'p>(
    request: &TrialRequest,
    set: &SensorSetSpec,
    context: SetContext<'p>,
    log: &SensorLog,
) -> Result<SetRun<'p>, TrialError> {
    let means = set.ablation_set();
    let sighting = match context
        .pairs
        .filter(|_| means.contains(NavMeans::Celestial))
    {
        Some(pairs) => Some(StarSighting::new(
            pairs,
            parse_utc(&request.epoch_utc)?,
            STAR_SOURCE,
            request.stars.mount_alignment_sigma_rad,
        )?),
        None => None,
    };
    let selection = SetSelection {
        means,
        availability: SetAvailability::resolve(request, set)?,
    };
    let config = super::trial_inertial_config(&request.imu_grade.grade().spec(), &request.baro);
    SetRun::new(log, selection, config, sighting)
}

/// What a set's run leaves for its report.
pub(super) struct SetResult {
    pub(super) series: Vec<SeriesSample>,
    pub(super) tallies: SensorTallies,
    pub(super) intervals: Vec<(ScheduledInterval, SensorTallies)>,
    pub(super) stars: StarAvailability,
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
        closed_loop: result.closed_loop,
    }
}

/// The result file of `request` over `route` with each set's `results`.
fn trial_report(
    request: &TrialRequest,
    route: RouteReport,
    results: Vec<SetResult>,
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
        models: model_statements(request.vehicle.guidance),
        seeds: request.seeds,
        route,
        availability: request.availability.clone(),
        gnss_horizontal_sigma_m: GnssReceiver::horizontal_sigma_m(&request.gnss),
        comparison: comparison(&sensor_sets, request.output.horizontal_error_threshold_m),
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
