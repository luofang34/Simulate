//! The true flight of one closed-loop set: the vehicle, and everything
//! judged on its truth — closest approaches, the pilot's sequencing,
//! cross-track, and the termination rules.

use std::collections::BTreeMap;

use navigate_contract::{GeodeticPosition, NavigationSolution, Waypoint};
use navigate_geodesy::{BodyEllipsoid, cross_track_m, distance_m, initial_bearing_rad};
use sim_core::{Block, KinematicInput, KinematicModel, SimInstant, VehicleState};
use sim_geo::ecef_to_geodetic;

use super::{
    ClosedLoopReport, CrossTrackSample, PilotInput, PilotOutput, TerminationReport, WaypointOutcome,
};
use crate::trial::report::TrackPoint;
use crate::trial::route::{capture_threshold_m, geodetic, leg_speed, step_span};
use crate::trial::vertical::flown_profile;
use crate::trial::{TerminationSpec, TrialError, TrialRequest};
use crate::{Truth, compare_with_truth};

/// Steps between checks of the corridor rule: 10 Hz.
const CORRIDOR_CHECK_STEPS: usize = 12;

/// The true flight: the vehicle, and everything judged on its truth.
pub(super) struct Flight<'w> {
    waypoints: &'w [Waypoint],
    vehicle: crate::trial::VehicleSpec,
    limits: TerminationSpec,
    model: KinematicModel,
    pub(super) truth: Vec<Truth<(SimInstant, VehicleState)>>,
    outcomes: Vec<WaypointOutcome>,
    cross_track: Vec<CrossTrackSample>,
    termination: Option<TerminationReport>,
    last_event_step: usize,
    final_arrival_error_m: Option<f64>,
    completed: bool,
}

impl<'w> Flight<'w> {
    pub(super) fn new(
        request: &TrialRequest,
        waypoints: &'w [Waypoint],
        start: VehicleState,
    ) -> Self {
        let outcomes = waypoints
            .iter()
            .enumerate()
            .map(|(index, waypoint)| WaypointOutcome {
                index,
                ident: waypoint.ident.as_str().to_owned(),
                capture_threshold_m: capture_threshold_m(waypoints, index, &request.vehicle),
                sequenced_at_s: None,
                true_miss_at_sequence_m: None,
                estimated_distance_at_sequence_m: None,
                navigation_error_at_sequence_m: None,
                truly_within_threshold: None,
                closest_true_approach_m: f64::INFINITY,
            })
            .collect();
        Self {
            waypoints,
            vehicle: request.vehicle,
            limits: request.termination.unwrap_or_default(),
            model: KinematicModel::new(start),
            truth: vec![Truth::new((SimInstant::START, start))],
            outcomes,
            cross_track: Vec::new(),
            termination: None,
            last_event_step: 0,
            final_arrival_error_m: None,
            completed: false,
        }
    }

    pub(super) fn instant(&self, step: usize) -> Result<SimInstant, TrialError> {
        self.truth
            .get(step)
            .map(|sample| sample.get().0)
            .ok_or_else(|| TrialError::InvalidRequest {
                field: "closed loop",
                detail: format!("no truth at step {step}"),
            })
    }

    /// The termination rule that fired, if one has.
    pub(super) const fn termination(&self) -> Option<&TerminationReport> {
        self.termination.as_ref()
    }

    /// True cross-track recorded at `time_s`, when a sample was taken then.
    pub(super) fn cross_track_at(&self, time_s: f64) -> Option<f64> {
        self.cross_track
            .last()
            .filter(|sample| sample.time_s == time_s)
            .map(|sample| sample.true_cross_track_m)
    }

    pub(super) fn finished(&self) -> bool {
        self.completed || self.termination.is_some()
    }

    pub(super) fn fly(&mut self, output: &PilotOutput) -> Result<(), TrialError> {
        let at = self
            .truth
            .last()
            .map_or(SimInstant::START, |sample| sample.get().0);
        self.model
            .step(&KinematicInput {
                controls: output.controls,
            })
            .map_err(TrialError::Vehicle)?;
        self.truth
            .push(Truth::new((at.advanced(step_span()), self.model.output())));
        Ok(())
    }

    /// Judges `step` on truth: closest approaches, the pilot's sequencing,
    /// cross-track, and the termination rules.
    pub(super) fn observe(
        &mut self,
        step: usize,
        input: &PilotInput,
        output: &PilotOutput,
        sample: usize,
    ) {
        let Some((at, state)) = self.truth.get(step).map(|s| *s.get()) else {
            return;
        };
        let time_s = at.duration_since(SimInstant::START).as_seconds();
        let position = geodetic(&state);
        for (outcome, waypoint) in self.outcomes.iter_mut().zip(self.waypoints) {
            let distance = horizontal_m(&position, &waypoint.position);
            outcome.closest_true_approach_m = outcome.closest_true_approach_m.min(distance);
        }
        if let (Some(event), Some(estimate)) = (output.event, input.estimate.as_ref()) {
            self.sequenced(event, estimate.get(), &state, &position, time_s, step);
        }
        let leg = output.active_index;
        let check_corridor = step.is_multiple_of(CORRIDOR_CHECK_STEPS);
        if let Some(index) = leg
            && (step.is_multiple_of(sample.max(1)) || check_corridor)
            && let Some((xt, course)) = self.true_cross_track(index, &position)
        {
            if step.is_multiple_of(sample.max(1)) {
                let frame = sim_geo::LocalFrame::at(&ecef_to_geodetic(state.position_ecef));
                let v = state.velocity_ecef;
                let track = v.dot(frame.east()).atan2(v.dot(frame.north()));
                let error = (track - course + std::f64::consts::PI)
                    .rem_euclid(std::f64::consts::TAU)
                    - std::f64::consts::PI;
                self.cross_track.push(CrossTrackSample {
                    time_s,
                    leg_index: index,
                    true_cross_track_m: xt,
                    true_track_error_rad: error,
                });
            }
            if xt.abs() > self.limits.corridor_half_width_m {
                self.terminate(
                    "left_corridor",
                    time_s,
                    format!("{xt:.1} m off leg {index}"),
                );
            }
        }
        self.check_height(position.altitude_m, time_s);
        if let Some(index) = leg {
            self.check_leg_time(index, step, time_s);
        }
    }

    fn sequenced(
        &mut self,
        event: super::PilotEvent,
        estimate: &NavigationSolution,
        state: &VehicleState,
        position: &GeodeticPosition,
        time_s: f64,
        step: usize,
    ) {
        let index = event.waypoint_index;
        let (Some(outcome), Some(waypoint)) =
            (self.outcomes.get_mut(index), self.waypoints.get(index))
        else {
            return;
        };
        let miss = horizontal_m(position, &waypoint.position);
        outcome.sequenced_at_s = Some(time_s);
        outcome.true_miss_at_sequence_m = Some(miss);
        outcome.estimated_distance_at_sequence_m =
            Some(horizontal_m(&estimate.position, &waypoint.position));
        outcome.navigation_error_at_sequence_m = compare_with_truth(estimate, state.position_ecef)
            .ok()
            .map(|c| c.horizontal_error_m);
        outcome.truly_within_threshold = Some(miss <= outcome.capture_threshold_m);
        self.last_event_step = step;
        if event.plan_complete {
            self.completed = true;
            self.final_arrival_error_m = Some(miss);
        }
    }

    /// True cross-track from leg `index`, and the leg's course.
    fn true_cross_track(&self, index: usize, position: &GeodeticPosition) -> Option<(f64, f64)> {
        let from = self.waypoints.get(index.checked_sub(1)?)?;
        let to = self.waypoints.get(index)?;
        let xt =
            cross_track_m(position, &from.position, &to.position, BodyEllipsoid::WGS84).ok()?;
        Some((xt, initial_bearing_rad(&from.position, &to.position)))
    }

    fn check_height(&mut self, height_m: f64, time_s: f64) {
        let (low, high) = (self.limits.min_height_m, self.limits.max_height_m);
        if !(low..=high).contains(&height_m) {
            let detail = format!("{height_m:.1} m outside {low}..{high} m");
            self.terminate("altitude_limit", time_s, detail);
        }
    }

    fn check_leg_time(&mut self, index: usize, step: usize, time_s: f64) {
        let planned_s = match (
            index.checked_sub(1).and_then(|i| self.waypoints.get(i)),
            self.waypoints.get(index),
        ) {
            (Some(from), Some(to)) => {
                distance_m(&from.position, &to.position, BodyEllipsoid::WGS84)
                    / leg_speed(self.waypoints, index, &self.vehicle)
            }
            _ => 0.0,
        };
        let limit_s = self.limits.leg_time_factor * planned_s + self.limits.leg_time_margin_s;
        let elapsed_s = step.saturating_sub(self.last_event_step) as f64 * step_span().as_seconds();
        if elapsed_s > limit_s {
            let detail = format!("waypoint {index} not sequenced within {limit_s:.0} s");
            self.terminate("sequence_timeout", time_s, detail);
        }
    }

    fn terminate(&mut self, rule: &str, time_s: f64, detail: String) {
        if self.termination.is_none() && !self.completed {
            self.termination = Some(TerminationReport {
                rule: rule.into(),
                time_s,
                detail,
            });
        }
    }

    pub(super) fn into_report(
        self,
        request: &TrialRequest,
        guidance_refusals: BTreeMap<String, u64>,
    ) -> Result<ClosedLoopReport, TrialError> {
        let track_steps = crate::trial::evaluate::whole_steps(
            "output.track_period_s",
            request.output.track_period_s,
        )?;
        let point = |(at, state): &(SimInstant, VehicleState)| {
            let p = geodetic(state);
            TrackPoint {
                time_s: at.duration_since(SimInstant::START).as_seconds(),
                lat_deg: p.latitude_rad.to_degrees(),
                lon_deg: p.longitude_rad.to_degrees(),
                height_m: p.altitude_m,
            }
        };
        let mut track: Vec<TrackPoint> = self
            .truth
            .iter()
            .step_by(track_steps)
            .map(|s| point(s.get()))
            .collect();
        let last = self.truth.last().map(|s| point(s.get()));
        if let Some(last) = last
            && track.last().map(|p| p.time_s) != Some(last.time_s)
        {
            track.push(last);
        }
        let max_abs_cross_track_m = self
            .cross_track
            .iter()
            .map(|s| s.true_cross_track_m.abs())
            .fold(0.0, f64::max);
        Ok(ClosedLoopReport {
            completed: self.completed,
            termination: self.termination,
            duration_s: last.map_or(0.0, |p| p.time_s),
            vertical_profile: flown_profile(&self.truth, track_steps),
            track,
            cross_track: self.cross_track,
            max_abs_cross_track_m,
            waypoints: self.outcomes,
            final_arrival_error_m: self.final_arrival_error_m,
            guidance_refusals,
        })
    }
}

/// Horizontal distance from `a` to `b` on the WGS84 ellipsoid, metres: in
/// `a`'s north-east plane, the metric the navigation error is scored in, so
/// a true miss and a navigation error compare directly.
fn horizontal_m(a: &GeodeticPosition, b: &GeodeticPosition) -> f64 {
    let ecef = |p: &GeodeticPosition| {
        sim_geo::geodetic_to_ecef(&sim_geo::Geodetic {
            lat_rad: p.latitude_rad,
            lon_rad: p.longitude_rad,
            height_m: p.altitude_m,
        })
    };
    let (from, to) = (ecef(a), ecef(b));
    let d = sim_geo::LocalFrame::at(&ecef_to_geodetic(from)).ecef_to_ned_vector(to - from);
    d.x.hypot(d.y)
}
