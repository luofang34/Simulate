//! Open loop: every set replays the one recorded flight.

use navigate_contract::{GeodeticPosition, NavigationSolution};
use navigate_geodesy::{BodyEllipsoid, cross_track_m};
use sim_core::{SimInstant, VehicleState};
use sim_geo::ecef_to_geodetic;

use super::{
    RecordedTrial, SetResult, TrialOutcome, replay_set_blocking, sample_periods, trial_report,
    whole_steps,
};
use crate::trial::progress::{EventKind, EventRecord, ProgressSink, Scored, Slices};
use crate::trial::record::SensorLog;
use crate::trial::replay::SetRun;
use crate::trial::report::{RouteReport, SeriesSample, TrackPoint};
use crate::trial::route::{RouteFlight, step_seconds};
use crate::trial::score::score_sample;
use crate::trial::timing::{SetTiming, TrialTiming};
use crate::trial::vertical::{VerticalReport, flown_profile, vertical_plan};
use crate::trial::{SensorSetSpec, TrialError, TrialRequest};
use crate::{AblationEvaluation, Estimate, NavigationOrigin};

/// Replays every set over `recorded` slice by slice, handing `sink` each
/// slice's events and samples.
pub(super) fn open_trial(
    request: &TrialRequest,
    recorded: &RecordedTrial,
    sink: &mut dyn ProgressSink,
) -> Result<TrialOutcome, TrialError> {
    let (sample_steps, progress_steps) = sample_periods(request)?;
    let mut replays = request
        .sensor_sets
        .iter()
        .map(|set| OpenSet::new(request, set, &recorded.log))
        .collect::<Result<Vec<_>, _>>()?;
    let known = waypoint_events(&recorded.flight);
    let mut slices = Slices::new(sink, request, progress_steps, known)?;
    let steps = scored_steps(recorded.flight.truth.len(), sample_steps);
    let last = steps.last().copied();
    for step in steps {
        let due = slices.due(step, Some(step) == last);
        let cross_track = cross_track_at(&recorded.flight, step);
        for (set, replay) in request.sensor_sets.iter().zip(&mut replays) {
            if let Some(record) =
                replay.sample(recorded, step, due.then_some(&set.name), cross_track)?
            {
                slices.push(record);
            }
        }
        if due {
            slices.flush(step_seconds(step))?;
        }
    }
    let mut results = Vec::new();
    let mut estimates = Vec::new();
    let frames = recorded.log.terrain.len() as u64;
    let mut timing = TrialTiming {
        terrain_frames_rendered: frames,
        terrain_render_s_per_frame: recorded.terrain_render_s / frames.max(1) as f64,
        sets: Vec::new(),
    };
    for (set, replay) in request.sensor_sets.iter().zip(replays) {
        let (result, published) = replay.finish();
        let (wall_s, handed, slowest_s) = result.terrain_wall;
        if handed > 0 {
            timing.sets.push(SetTiming {
                set: set.name.clone(),
                terrain_frames: handed,
                navigator_s_per_frame: wall_s / handed as f64,
                navigator_s_slowest: slowest_s,
            });
        }
        results.push(result);
        estimates.push(published);
    }
    let report = trial_report(request, route_report(request, &recorded.flight)?, results)?;
    Ok(TrialOutcome {
        report,
        estimates,
        timing,
    })
}

/// The scored steps of a flight of `len` steps: every `period` steps, and
/// the last.
fn scored_steps(len: usize, period: usize) -> Vec<usize> {
    let last = len.saturating_sub(1);
    let mut steps: Vec<usize> = (period..=last).step_by(period.max(1)).collect();
    if steps.last() != Some(&last) && last > 0 {
        steps.push(last);
    }
    steps
}

fn waypoint_events(flight: &RouteFlight) -> Vec<EventRecord> {
    flight
        .events
        .iter()
        .map(|event| EventRecord {
            t_s: event.time_s,
            set: None,
            kind: EventKind::WaypointSequenced {
                waypoint_index: event.waypoint_index,
                ident: event.ident.clone(),
                plan_complete: event.plan_complete,
            },
        })
        .collect()
}

/// True cross-track at `step` from the leg the executor was flying.
fn cross_track_at(flight: &RouteFlight, step: usize) -> Option<f64> {
    let active = flight.events.iter().filter(|e| e.step <= step).count();
    let from = flight.waypoints.get(active.checked_sub(1)?)?;
    let to = flight.waypoints.get(active)?;
    let geo = ecef_to_geodetic(flight.truth.get(step)?.get().1.position_ecef);
    let position = GeodeticPosition::new(geo.lat_rad, geo.lon_rad, geo.height_m);
    cross_track_m(
        &position,
        &from.position,
        &to.position,
        BodyEllipsoid::WGS84,
    )
    .ok()
}

/// One set's replay of a recorded flight and its scores so far.
struct OpenSet {
    run: SetRun,
    origin: NavigationOrigin,
    evaluation: AblationEvaluation,
    series: Vec<SeriesSample>,
    published: Vec<Estimate<(SimInstant, NavigationSolution)>>,
}

impl OpenSet {
    fn new(
        request: &TrialRequest,
        set: &SensorSetSpec,
        log: &SensorLog,
    ) -> Result<Self, TrialError> {
        Ok(Self {
            run: replay_set_blocking(request, set, log)?,
            origin: NavigationOrigin::Fused(set.ablation_set()),
            evaluation: AblationEvaluation::default(),
            series: Vec::new(),
            published: Vec::new(),
        })
    }

    /// Replays up to `step` and scores the solution there against truth;
    /// with a set name, also its progress sample.
    fn sample(
        &mut self,
        recorded: &RecordedTrial,
        step: usize,
        progress: Option<&String>,
        cross_track: Option<f64>,
    ) -> Result<Option<crate::trial::progress::ProgressRecord>, TrialError> {
        self.run.advance_to(&recorded.log, step)?;
        let Some(estimate) = self.run.solution(&recorded.log)? else {
            return Ok(None);
        };
        self.evaluation.expect(self.origin);
        let Some(sample_truth) = recorded.flight.truth.get(step) else {
            return Ok(None);
        };
        let sample = score_sample(&mut self.evaluation, self.origin, sample_truth, &estimate)?;
        self.series.push(sample);
        let (at, truth): (SimInstant, VehicleState) = *sample_truth.get();
        self.published.push(Estimate::new((at, *estimate.get())));
        Ok(progress.map(|name| {
            let scored = Scored {
                truth: &truth,
                estimate: estimate.get(),
                sample: &sample,
            };
            scored.record(name, &self.run, cross_track)
        }))
    }

    fn finish(self) -> (SetResult, Vec<Estimate<(SimInstant, NavigationSolution)>>) {
        let result = SetResult {
            series: self.series,
            tallies: self.run.tallies().clone(),
            intervals: self.run.interval_tallies().to_vec(),
            stars: self.run.star_availability(),
            terrain: self.run.terrain_report(),
            terrain_wall: self.run.terrain_wall(),
            closed_loop: None,
        };
        (result, self.published)
    }
}

fn route_report(request: &TrialRequest, flight: &RouteFlight) -> Result<RouteReport, TrialError> {
    let track_steps = whole_steps("output.track_period_s", request.output.track_period_s)?;
    let point = |(at, state): &(SimInstant, VehicleState)| {
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
    let (legs, planned) = vertical_plan(&request.waypoints, &request.vehicle)?;
    Ok(RouteReport {
        waypoints: request.waypoints.clone(),
        events: flight.events.clone(),
        duration_s: track.last().map_or(0.0, |p| p.time_s),
        track,
        vertical: VerticalReport {
            legs,
            planned,
            flown: flown_profile(&flight.truth, track_steps),
        },
    })
}
