//! Closed loop: every set flies its own truth, all of them one sample
//! slice at a time in turn.

use super::{ClosedTrialOutcome, SetContext, SetResult, sample_periods, trial_report};
use crate::trial::closed::ClosedSet;
use crate::trial::progress::{ProgressSink, Slices};
use crate::trial::report::RouteReport;
use crate::trial::route::{plan_waypoints, step_seconds};
use crate::trial::vertical::{VerticalReport, vertical_plan};
use crate::trial::{TrialError, TrialRequest};

/// Flies every set of `request` in closed loop, handing `sink` each
/// slice's events and samples; `trace` keeps each pilot's inputs and
/// outputs.
pub(super) fn closed_trial(
    request: &TrialRequest,
    trace: bool,
    sink: &mut dyn ProgressSink,
) -> Result<ClosedTrialOutcome, TrialError> {
    request.validate()?;
    let (sample_steps, progress_steps) = sample_periods(request)?;
    let context = SetContext { sample_steps };
    let waypoints = plan_waypoints(&request.waypoints)?;
    let mut slices = Slices::new(sink, request, progress_steps, Vec::new())?;
    let mut flying = request
        .sensor_sets
        .iter()
        .map(|set| ClosedSet::new(request, set, &waypoints, context, progress_steps, trace))
        .collect::<Result<Vec<_>, _>>()?;
    let mut target = sample_steps;
    while flying.iter().any(|set| !set.done()) {
        for set in &mut flying {
            set.advance_through(target)?;
            for record in set.take_progress() {
                slices.push(record);
            }
        }
        slices.flush(step_seconds(target))?;
        target = target.saturating_add(sample_steps);
    }
    let mut sets = Vec::new();
    let mut results = Vec::new();
    for flown in flying {
        let outcome = flown.finish()?;
        results.push(SetResult {
            series: outcome.series.clone(),
            tallies: outcome.tallies.clone(),
            intervals: outcome.interval_tallies.clone(),
            stars: outcome.stars,
            terrain: None,
            terrain_wall: (0.0, 0, 0.0),
            closed_loop: Some(outcome.report.clone()),
        });
        sets.push(outcome);
    }
    let duration_s = sets
        .iter()
        .map(|set| set.report.duration_s)
        .fold(0.0, f64::max);
    let (legs, planned) = vertical_plan(&request.waypoints, &request.vehicle)?;
    let route = RouteReport {
        waypoints: request.waypoints.clone(),
        events: Vec::new(),
        duration_s,
        track: Vec::new(),
        vertical: VerticalReport {
            legs,
            planned,
            flown: Vec::new(),
        },
    };
    let report = trial_report(request, route, results)?;
    Ok(ClosedTrialOutcome { report, sets })
}
