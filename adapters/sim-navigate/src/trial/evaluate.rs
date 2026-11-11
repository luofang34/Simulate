//! A whole trial: fly, sense once, replay each sensor set, score.

use navigate_contract::NavigationSolution;
use navigate_startrack::PairCatalogue;
use sha2::{Digest, Sha256};
use sim_core::SimInstant;
use sim_geo::ecef_to_geodetic;

use std::sync::Arc;

use super::closed::{ClosedLoopOutcome, fly_closed_loop};
use super::contribution::{ablation_origin, fill_contributions};
use super::epoch::parse_utc;
use super::gnss::GnssReceiver;
use super::record::{SensorLog, record};
use super::replay::{STAR_SOURCE, SetRun, SetSelection, trial_inertial_config};
use super::report::{
    InputIdentity, REPORT_SCHEMA, RouteReport, SetReport, TrackPoint, TrialReport,
};
use super::route::{RouteFlight, STEP_HZ, fly_route};
use super::score::{score_sample, summarise};
use super::stars::sighting::{StarSighting, pair_table};
use super::visual::{
    FrameTiming, load_onboard_package, maps_report, mean_ground_distance_m, visual_aid_for,
    visual_report,
};
use super::{GuidanceSource, SensorSetSpec, TrialError, TrialRequest, model_statements};
use crate::package::MapPackage;
use crate::{AblationEvaluation, Estimate, NavMeans};

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
    /// Each visual set's per-frame wall times, by set name; kept out of the
    /// report so that it stays a function of the request.
    pub visual_timings: Vec<(String, Vec<FrameTiming>)>,
}

/// What every set of a run shares.
#[derive(Clone, Copy)]
pub(super) struct SetContext<'p> {
    pub(super) pairs: Option<&'p PairCatalogue>,
    pub(super) package: Option<&'p Arc<MapPackage>>,
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
    match request.vehicle.guidance {
        GuidanceSource::Truth => {
            let recorded = record_trial(request)?;
            Ok(evaluate(request, &recorded)?.report)
        }
        GuidanceSource::Estimate => Ok(run_closed_trial(request, false)?.report),
    }
}

/// Per-frame wall times of each visual set, by set name.
pub type SetTimings = Vec<(String, Vec<FrameTiming>)>;

/// Runs `request` end to end and returns the per-frame wall times of each
/// visual set beside the report, which stays free of them.
pub fn run_trial_with_timings(
    request: &TrialRequest,
) -> Result<(TrialReport, SetTimings), TrialError> {
    match request.vehicle.guidance {
        GuidanceSource::Truth => {
            let recorded = record_trial(request)?;
            let outcome = evaluate(request, &recorded)?;
            Ok((outcome.report, outcome.visual_timings))
        }
        GuidanceSource::Estimate => {
            let outcome = run_closed_trial(request, false)?;
            Ok((outcome.report, outcome.visual_timings))
        }
    }
}

/// A closed-loop trial's report with each set's full outcome, in set order.
#[derive(Debug, Clone)]
pub struct ClosedTrialOutcome {
    /// The result file's content.
    pub report: TrialReport,
    /// Each set's flight, with the pilot trace when asked for.
    pub sets: Vec<ClosedLoopOutcome>,
    /// Each visual set's per-frame wall times, by set name.
    pub visual_timings: Vec<(String, Vec<FrameTiming>)>,
}

/// Flies every sensor set of `request` in closed loop, each on its own
/// truth; `trace` keeps each pilot's inputs and outputs.
pub fn run_closed_trial(
    request: &TrialRequest,
    trace: bool,
) -> Result<ClosedTrialOutcome, TrialError> {
    request.validate()?;
    let sample_steps = whole_steps("output.sample_period_s", request.output.sample_period_s)?;
    let pairs = if request.uses_stars() {
        Some(pair_table(parse_utc(&request.epoch_utc)?)?)
    } else {
        None
    };
    let package = load_onboard_package(request)?;
    let context = SetContext {
        pairs: pairs.as_ref(),
        package: package.as_ref(),
        sample_steps,
    };
    let mut sets = Vec::new();
    let mut reports = Vec::new();
    let mut visual_timings = Vec::new();
    let mut ground = Vec::new();
    for set in &request.sensor_sets {
        let mut outcome = fly_closed_loop(request, set, context, trace)?;
        if !outcome.visual_timings.is_empty() {
            visual_timings.push((
                set.name.clone(),
                std::mem::take(&mut outcome.visual_timings),
            ));
        }
        ground.extend(outcome.mean_ground_distance_m);
        reports.push(SetReport {
            name: set.name.clone(),
            means: set.ablation_set().means().collect(),
            gnss_outages: set.gnss_outages,
            summary: summarise(&outcome.series, &set_outages(request, set)),
            observations: outcome.tallies.clone(),
            stars: set
                .ablation_set()
                .contains(NavMeans::Celestial)
                .then_some(outcome.stars),
            series: outcome.series.clone(),
            closed_loop: Some(outcome.report.clone()),
            visual: outcome.visual.clone(),
            contribution: None,
        });
        sets.push(outcome);
    }
    fill_contributions(&request.sensor_sets, &mut reports);
    let duration_s = sets
        .iter()
        .map(|set| set.report.duration_s)
        .fold(0.0, f64::max);
    let report = TrialReport {
        schema: REPORT_SCHEMA.into(),
        inputs: identity(request)?,
        models: model_statements(request.vehicle.guidance),
        seeds: request.seeds,
        route: RouteReport {
            waypoints: request.waypoints.clone(),
            events: Vec::new(),
            duration_s,
            track: Vec::new(),
        },
        outages: request.outages.clone(),
        gnss_horizontal_sigma_m: GnssReceiver::horizontal_sigma_m(&request.gnss),
        sensor_sets: reports,
        visual_maps: maps_report(
            request,
            (!ground.is_empty()).then(|| ground.iter().sum::<f64>() / ground.len() as f64),
        )?,
    };
    Ok(ClosedTrialOutcome {
        report,
        sets,
        visual_timings,
    })
}

fn set_outages(request: &TrialRequest, set: &SensorSetSpec) -> Vec<super::OutageSpec> {
    if set.gnss_outages {
        request.outages.clone()
    } else {
        Vec::new()
    }
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

/// Replays every sensor set of `request` over `recorded` and scores each
/// against the recorded flight's truth.
pub fn evaluate(
    request: &TrialRequest,
    recorded: &RecordedTrial,
) -> Result<TrialOutcome, TrialError> {
    let sample_steps = whole_steps("output.sample_period_s", request.output.sample_period_s)?;
    let pairs = if request.uses_stars() {
        Some(pair_table(parse_utc(&request.epoch_utc)?)?)
    } else {
        None
    };
    let package = load_onboard_package(request)?;
    let context = SetContext {
        pairs: pairs.as_ref(),
        package: package.as_ref(),
        sample_steps,
    };
    let mut reports = Vec::new();
    let mut estimates = Vec::new();
    let mut visual_timings = Vec::new();
    for set in &request.sensor_sets {
        let (report, series, timings) = evaluate_set(request, recorded, set, context)?;
        reports.push(report);
        estimates.push(series);
        if !timings.is_empty() {
            visual_timings.push((set.name.clone(), timings));
        }
    }
    fill_contributions(&request.sensor_sets, &mut reports);
    let report = TrialReport {
        schema: REPORT_SCHEMA.into(),
        inputs: identity(request)?,
        models: model_statements(request.vehicle.guidance),
        seeds: request.seeds,
        route: route_report(request, &recorded.flight)?,
        outages: request.outages.clone(),
        gnss_horizontal_sigma_m: GnssReceiver::horizontal_sigma_m(&request.gnss),
        sensor_sets: reports,
        visual_maps: maps_report(request, mean_ground_distance_m(&recorded.log.camera_frames))?,
    };
    Ok(TrialOutcome {
        report,
        estimates,
        visual_timings,
    })
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
        outages: if set.gnss_outages {
            request.outages.clone()
        } else {
            Vec::new()
        },
    };
    let config = trial_inertial_config(&request.imu_grade.grade().spec(), &request.baro);
    Ok(
        SetRun::new(log, selection, config, sighting)?.with_visual(visual_aid_for(
            request,
            set,
            context.package,
        )),
    )
}

type SetOutcome = (
    SetReport,
    Vec<Estimate<(SimInstant, NavigationSolution)>>,
    Vec<FrameTiming>,
);

fn evaluate_set(
    request: &TrialRequest,
    recorded: &RecordedTrial,
    set: &SensorSetSpec,
    context: SetContext<'_>,
) -> Result<SetOutcome, TrialError> {
    let sample_steps = context.sample_steps;
    let means = set.ablation_set();
    let outages = set_outages(request, set);
    let mut run = set_run(request, set, context, &recorded.log)?;
    let origin = ablation_origin(means);
    let mut evaluation = AblationEvaluation::default();
    let truth = &recorded.flight.truth;
    let last = truth.len().saturating_sub(1);
    let mut steps: Vec<usize> = (sample_steps..=last).step_by(sample_steps).collect();
    if steps.last() != Some(&last) && last > 0 {
        steps.push(last);
    }
    let mut series = Vec::with_capacity(steps.len());
    let mut published = Vec::with_capacity(steps.len());
    for step in steps {
        run.advance_to(&recorded.log, step)?;
        let Some(estimate) = run.solution(&recorded.log)? else {
            continue;
        };
        evaluation.expect(origin);
        let Some(sample_truth) = truth.get(step) else {
            continue;
        };
        series.push(score_sample(
            &mut evaluation,
            origin,
            sample_truth,
            &estimate,
        )?);
        published.push(Estimate::new((sample_truth.get().0, *estimate.get())));
    }
    let report = SetReport {
        name: set.name.clone(),
        means: means.means().collect(),
        gnss_outages: set.gnss_outages,
        summary: summarise(&series, &outages),
        observations: run.tallies().clone(),
        stars: means
            .contains(NavMeans::Celestial)
            .then(|| run.star_availability()),
        series,
        closed_loop: None,
        visual: run.visual().map(|aid| {
            visual_report(aid, |step| {
                truth
                    .get(step)
                    .map(|sample| (sample.get().0, sample.get().1.position_ecef))
            })
        }),
        contribution: None,
    };
    let timings = run
        .visual()
        .map(|aid| aid.timings().to_vec())
        .unwrap_or_default();
    Ok((report, published, timings))
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

fn route_report(request: &TrialRequest, flight: &RouteFlight) -> Result<RouteReport, TrialError> {
    let track_steps = whole_steps("output.track_period_s", request.output.track_period_s)?;
    let point = |(at, state): &(SimInstant, sim_core::VehicleState)| {
        let geo = ecef_to_geodetic(state.position_ecef);
        TrackPoint {
            time_s: at.duration_since(SimInstant::START).as_seconds(),
            lat_deg: geo.lat_rad.to_degrees(),
            lon_deg: geo.lon_rad.to_degrees(),
            height_m: geo.height_m,
        }
    };
    let mut track: Vec<TrackPoint> = flight
        .truth
        .iter()
        .step_by(track_steps)
        .map(|sample| point(sample.get()))
        .collect();
    if let Some(last) = flight.truth.last()
        && track.last().map(|p| p.time_s) != Some(point(last.get()).time_s)
    {
        track.push(point(last.get()));
    }
    Ok(RouteReport {
        waypoints: request.waypoints.clone(),
        events: flight.events.clone(),
        duration_s: track.last().map_or(0.0, |p| p.time_s),
        track,
    })
}
