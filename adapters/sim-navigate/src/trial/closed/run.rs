//! One sensor set's closed-loop flight, stepped so that a trial can fly
//! every set side by side in route time. Each set owns its vehicle, sensors
//! and navigator, so the order in which sets are stepped changes nothing in
//! any of them.

use std::collections::BTreeMap;

use glam::DVec3;
use navigate_contract::{NavigationSolution, Waypoint};
use sim_core::{SimInstant, VehicleState};

use super::flight::Flight;
use super::sensors::LiveSensors;
use super::{Autopilot, ClosedLoopOutcome, PilotInput, PilotOutput};
use crate::trial::evaluate::{SetContext, replay_set};
use crate::trial::onboard::trial_director;
use crate::trial::progress::{EventKind, EventRecord, ProgressRecord, Scored};
use crate::trial::record::{SensorLog, initial_alignment};
use crate::trial::replay::SetRun;
use crate::trial::route::{start_state, step_seconds, step_span};
use crate::trial::score::score_sample;
use crate::trial::{SensorSetSpec, TrialError, TrialRequest};
use crate::{AblationEvaluation, Estimate, MasterClockBridge, NavMeans, NavigationOrigin, Truth};

/// One set flying the request's route in closed loop: the set's own
/// navigator steers, so the set flies its own truth.
pub(in crate::trial) struct ClosedSet<'a> {
    request: &'a TrialRequest,
    name: &'a str,
    waypoints: &'a [Waypoint],
    pilot: Autopilot,
    flight: Flight<'a>,
    sensors: LiveSensors,
    log: SensorLog,
    run: SetRun,
    scorer: Scorer,
    clock: MasterClockBridge,
    sample_steps: usize,
    rate: DVec3,
    trace: Option<Vec<(PilotInput, PilotOutput)>>,
    done: bool,
    progress: Progress,
}

/// What a closed set has for the progress stream.
struct Progress {
    period_steps: usize,
    records: Vec<ProgressRecord>,
    refusal_counts: BTreeMap<String, u64>,
    refusing: bool,
}

impl<'a> ClosedSet<'a> {
    /// `set` at the start of `waypoints`, its navigator aligned, with a
    /// progress sample every `progress_steps`; `trace` keeps every step's
    /// pilot input and output.
    pub(in crate::trial) fn new(
        request: &'a TrialRequest,
        set: &'a SensorSetSpec,
        waypoints: &'a [Waypoint],
        context: SetContext,
        progress_steps: usize,
        trace: bool,
    ) -> Result<Self, TrialError> {
        let clock = MasterClockBridge::default();
        let director = trial_director(waypoints, &request.vehicle)?;
        let pilot = Autopilot::new(director, &request.vehicle);
        let start = start_state(waypoints, &request.vehicle)?;
        let means = set.ablation_set();
        let mut sensors = LiveSensors::new(request, means.contains(NavMeans::Celestial))?;
        sensors.start(SimInstant::START, &start)?;
        let log = empty_log(request, &start);
        let run = replay_set(request, set, &log)?;
        Ok(Self {
            request,
            name: &set.name,
            waypoints,
            pilot,
            flight: Flight::new(request, waypoints, start),
            sensors,
            log,
            run,
            scorer: Scorer::new(NavigationOrigin::Fused(means), context.sample_steps),
            clock,
            sample_steps: context.sample_steps,
            rate: DVec3::ZERO,
            trace: trace.then(Vec::new),
            done: false,
            progress: Progress {
                period_steps: progress_steps.max(1),
                records: Vec::new(),
                refusal_counts: BTreeMap::new(),
                refusing: false,
            },
        })
    }

    /// Whether the flight has completed or been terminated.
    pub(in crate::trial) const fn done(&self) -> bool {
        self.done
    }

    /// The progress records since the last call.
    pub(in crate::trial) fn take_progress(&mut self) -> Vec<ProgressRecord> {
        std::mem::take(&mut self.progress.records)
    }

    /// Flies until the pilot has read `target`, or the flight ends.
    pub(in crate::trial) fn advance_through(&mut self, target: usize) -> Result<(), TrialError> {
        while !self.done && self.run.step() <= target {
            self.advance()?;
        }
        Ok(())
    }

    /// One step: the pilot reads the navigator, the step is judged and
    /// scored on truth, and unless the flight ended the vehicle flies on
    /// and the sensors and navigator take the next step.
    fn advance(&mut self) -> Result<(), TrialError> {
        let step = self.run.step();
        let at = self.flight.instant(step)?;
        let estimate = self.run.solution(&self.log)?;
        let input = PilotInput {
            estimate,
            sensed_rate_body: self.rate,
            now: self.clock.to_monotonic(at)?,
        };
        let output = self.pilot.step(&input, step_span().as_seconds());
        self.flight
            .observe(step, &input, &output, self.sample_steps);
        self.done = self.flight.finished();
        let scored =
            self.scorer
                .sample(step, self.done, &self.flight.truth, input.estimate.as_ref())?;
        self.report_progress(step, &output, scored.as_ref(), input.estimate.as_ref());
        if let Some(trace) = self.trace.as_mut() {
            trace.push((input, output));
        }
        if self.done {
            return Ok(());
        }
        self.flight.fly(&output)?;
        let next = step.wrapping_add(1);
        self.rate = self
            .sensors
            .sense(&self.flight.truth, next, &mut self.log)?;
        self.run.advance_to(&self.log, next)
    }

    /// The step's events, and its progress sample when one is due.
    fn report_progress(
        &mut self,
        step: usize,
        output: &PilotOutput,
        scored: Option<&crate::trial::SeriesSample>,
        estimate: Option<&Estimate<NavigationSolution>>,
    ) {
        let t_s = step_seconds(step);
        let event = |kind| {
            ProgressRecord::Event(EventRecord {
                t_s,
                set: Some(self.name.to_owned()),
                kind,
            })
        };
        let mut records = Vec::new();
        if let Some(sequenced) = output.event {
            let ident = self
                .waypoints
                .get(sequenced.waypoint_index)
                .map_or_else(String::new, |w| w.ident.as_str().to_owned());
            records.push(event(EventKind::WaypointSequenced {
                waypoint_index: sequenced.waypoint_index,
                ident,
                plan_complete: sequenced.plan_complete,
            }));
        }
        if let Some(refusal) = self.refusal_onset() {
            records.push(event(EventKind::GuidanceRefused { refusal }));
        }
        if let Some(termination) = self.flight.termination().filter(|_| self.done) {
            records.push(event(EventKind::Terminated {
                rule: termination.rule.clone(),
                detail: termination.detail.clone(),
            }));
        }
        let due = self.done || step.is_multiple_of(self.progress.period_steps);
        if let (Some(sample), Some(estimate), Some(truth), true) =
            (scored, estimate, self.flight.truth.get(step), due)
        {
            let scored = Scored {
                truth: &truth.get().1,
                estimate: estimate.get(),
                sample,
            };
            records.push(scored.record(self.name, &self.run, self.flight.cross_track_at(t_s)));
        }
        self.progress.records.extend(records);
    }

    /// The refusal kind when guidance refused this step after accepting
    /// the step before.
    fn refusal_onset(&mut self) -> Option<String> {
        let refusals = self.pilot.refusals();
        let counted = &self.progress.refusal_counts;
        let refused = refusals
            .iter()
            .find(|(kind, count)| counted.get(*kind) != Some(*count))
            .map(|(kind, _)| kind.clone());
        let refusing = refused.is_some();
        let onset = refused.filter(|_| !self.progress.refusing);
        self.progress.refusing = refusing;
        if self.progress.refusing {
            self.progress.refusal_counts = refusals.clone();
        }
        onset
    }

    /// The flight's results.
    pub(in crate::trial) fn finish(self) -> Result<ClosedLoopOutcome, TrialError> {
        let refusals = self.pilot.refusals().clone();
        Ok(ClosedLoopOutcome {
            report: self.flight.into_report(self.request, refusals)?,
            series: self.scorer.series,
            estimates: self.scorer.estimates,
            tallies: self.run.tallies().clone(),
            interval_tallies: self.run.interval_tallies().to_vec(),
            stars: self.run.star_availability(),
            trace: self.trace.unwrap_or_default(),
        })
    }
}

fn empty_log(request: &TrialRequest, start: &VehicleState) -> SensorLog {
    SensorLog {
        instants: vec![SimInstant::START],
        alignment: initial_alignment(request).measure(start),
        imu: Vec::new(),
        gnss: Vec::new(),
        baro: Vec::new(),
        stars: Vec::new(),
        star_mid_steps: Vec::new(),
        terrain: Vec::new(),
    }
}

/// Scores the navigator against truth at the sample steps and at the end.
struct Scorer {
    origin: NavigationOrigin,
    sample_steps: usize,
    evaluation: AblationEvaluation,
    series: Vec<crate::trial::SeriesSample>,
    estimates: Vec<Estimate<(SimInstant, NavigationSolution)>>,
}

impl Scorer {
    fn new(origin: NavigationOrigin, sample_steps: usize) -> Self {
        Self {
            origin,
            sample_steps: sample_steps.max(1),
            evaluation: AblationEvaluation::default(),
            series: Vec::new(),
            estimates: Vec::new(),
        }
    }

    fn sample(
        &mut self,
        step: usize,
        last: bool,
        truth: &[Truth<(SimInstant, VehicleState)>],
        estimate: Option<&Estimate<NavigationSolution>>,
    ) -> Result<Option<crate::trial::SeriesSample>, TrialError> {
        let due = step > 0 && (step.is_multiple_of(self.sample_steps) || last);
        let (Some(estimate), Some(truth), true) = (estimate, truth.get(step), due) else {
            return Ok(None);
        };
        self.evaluation.expect(self.origin);
        let sample = score_sample(&mut self.evaluation, self.origin, truth, estimate)?;
        self.series.push(sample);
        self.estimates
            .push(Estimate::new((truth.get().0, *estimate.get())));
        Ok(Some(sample))
    }
}
