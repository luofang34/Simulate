//! What a page shows while a link flies: every set's error against the
//! simulator's truth, the mission phase, what became of the terrain
//! camera's frames, and which sensors each set has at the moment.
//!
//! The monitor is fed, never asks: the flight hands it the trial's progress
//! records as they are made, the mission phase in the mission host's own
//! words, and each set's running tally of terrain-camera fixes. It derives
//! nothing about the flight that the feed does not say.

mod camera;
mod hold;
mod matching;
mod pacer;
mod replay;
mod sensor_controls;
mod status_lines;
mod thinning;
mod trace;
mod words;

#[cfg(test)]
mod tests;

use std::collections::BTreeMap;

use sim_navigate::NavMeans;
use sim_navigate::trial::{
    EventKind, EventRecord, HoldingPatternRecord, ProgressRecord, SensorKind, SensorStates,
};

pub use camera::{CameraLog, CameraShot, FixMark, outcome_words};
pub use hold::{HoldState, entry_words, leg_words, pattern_words};
pub use matching::{GreyImage, MatchLine, MatchLog, MatchShot, OfferedPosition, SolvedFit};
pub use pacer::{
    Clock, FLIGHT_SHARE, FlightStepper, MAX_FRAME_S, PaceLimit, Pacer, SLICE_BUDGET_S, Slice,
    StepState, run_slice,
};
pub use replay::Replay;
pub use sensor_controls::{SensorControl, SensorFault, SpoofDrag, sensor_controls};
pub use status_lines::{ABSENT, RUN_LINES, run_lines, starting_lines, stopped_lines};
pub use thinning::SAMPLES_KEPT;
pub use trace::{Column, SetTrace, TracePoint};
pub use words::{clock_words, event_words, sensor_words, state_words, terrain_words};

/// The flight's picture when the plan completed, which a page keeps
/// showing while the vehicle flies on in its hold.
#[derive(Debug, Clone, PartialEq)]
pub struct RouteEnd {
    /// Route time of the plan's completion, seconds.
    pub t_s: f64,
    /// The last waypoint, the one that completed the plan.
    pub ident: String,
    /// Each set's latest sample then, in the run's order of sets.
    pub samples: Vec<Option<TracePoint>>,
}

/// A spoofer holding a sensor of the flight.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Spoofing {
    /// Route time it captured the sensor, seconds.
    pub since_s: f64,
    /// The rate the sensor's reported position leaves the true one, m/s.
    pub rate_m_s: f64,
    /// The direction of the drag, degrees from true north.
    pub bearing_deg: f64,
}

/// Events the monitor keeps for its log; older ones drop off.
pub const EVENTS_KEPT: usize = 12;

/// One thing the flight tells the monitor.
#[derive(Debug, Clone, PartialEq)]
pub enum MonitorFeed {
    /// A record of the trial's progress stream.
    Progress(ProgressRecord),
    /// The mission phase, as the mission host names it.
    Phase(String),
    /// Something the mission host reported, in its own words.
    Mission {
        /// Route time, seconds.
        t_s: f64,
        /// What it said.
        event: String,
    },
    /// A set's running tally of terrain-camera fixes, keyed as the trial
    /// result keys them: `accepted`, `rejected:<reason>`,
    /// `unused:<reason>:matched` or `:unmatched`, `context:<kind>`.
    TerrainFixes {
        /// The set.
        set: String,
        /// The tally so far.
        fixes: BTreeMap<String, u64>,
    },
    /// A terrain-camera frame the flight delivered.
    Frame(CameraShot),
    /// A visual set's latest terrain match.
    Match(Box<MatchShot>),
}

/// A sensor a visitor can switch, because some set of the run uses it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MeansSwitch {
    /// The sensor.
    pub sensor: SensorKind,
    /// The means it serves.
    pub means: NavMeans,
}

/// The monitor's picture of the flight so far.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Monitor {
    traces: Vec<SetTrace>,
    truth: thinning::Thinned<(f64, (f64, f64))>,
    waypoints: Vec<(String, (f64, f64))>,
    altitudes_m: Vec<f64>,
    sequenced: Option<(usize, bool)>,
    hold_pattern: Option<HoldingPatternRecord>,
    camera: CameraLog,
    matches: MatchLog,
    sensors: BTreeMap<String, SensorStates>,
    fixes: BTreeMap<String, BTreeMap<String, u64>>,
    phase: Option<String>,
    events: Vec<EventRecord>,
    mission: Vec<(f64, String)>,
    planned_duration_s: Option<f64>,
    sim_time_s: f64,
    finished: bool,
    route_end: Option<RouteEnd>,
    hold: Option<HoldState>,
    gnss_spoofing: Option<Spoofing>,
}

impl Monitor {
    /// Takes one thing the flight said.
    pub fn apply(&mut self, feed: MonitorFeed) {
        match feed {
            MonitorFeed::Progress(record) => self.progress(record),
            MonitorFeed::Phase(phase) => self.phase = Some(phase),
            MonitorFeed::Mission { t_s, event } => {
                self.mission.push((t_s, event));
                let over = self.mission.len().saturating_sub(EVENTS_KEPT);
                self.mission.drain(..over);
            }
            MonitorFeed::TerrainFixes { set, fixes } => {
                let index = self.traces.iter().position(|t| t.name == set);
                if let Some(index) = index {
                    self.camera
                        .tallied((&set, index), self.fixes.get(&set), &fixes);
                }
                self.fixes.insert(set, fixes);
            }
            MonitorFeed::Frame(shot) => self.camera.deliver(shot),
            MonitorFeed::Match(shot) => self.matches.take(*shot),
        }
    }

    fn progress(&mut self, record: ProgressRecord) {
        match record {
            ProgressRecord::Started(started) => {
                let altitudes_m = started.route.waypoints.iter().map(|w| w.altitude_m);
                *self = Self {
                    altitudes_m: altitudes_m.collect(),
                    traces: started.sets.into_iter().map(SetTrace::new).collect(),
                    waypoints: started
                        .route
                        .waypoints
                        .into_iter()
                        .map(|w| (w.ident, (w.lat_deg, w.lon_deg)))
                        .collect(),
                    planned_duration_s: Some(started.route.planned_duration_s),
                    ..Self::default()
                };
            }
            ProgressRecord::Sample(sample) => {
                self.sim_time_s = self.sim_time_s.max(sample.t_s);
                self.sensors.insert(sample.set.clone(), sample.sensors);
                let truth = (sample.truth.lat_deg, sample.truth.lon_deg);
                if self.truth.latest().is_none_or(|(t_s, _)| sample.t_s > *t_s) {
                    self.truth.push((sample.t_s, truth));
                }
                if let Some(trace) = self.traces.iter_mut().find(|t| t.name == sample.set) {
                    trace.push(TracePoint {
                        t_s: sample.t_s,
                        horizontal_error_m: sample.horizontal_error_m,
                        sigma_horizontal_m: sample.sigma_horizontal_m,
                        estimate_deg: (sample.estimate.lat_deg, sample.estimate.lon_deg),
                    });
                }
            }
            ProgressRecord::Event(event) => {
                self.sim_time_s = self.sim_time_s.max(event.t_s);
                self.note_route_end(&event);
                self.hold = HoldState::after(self.hold.take(), &event.kind);
                match &event.kind {
                    EventKind::WaypointSequenced {
                        waypoint_index,
                        plan_complete,
                        ..
                    } => self.sequenced = Some((*waypoint_index, *plan_complete)),
                    EventKind::HoldingStarted { pattern } => {
                        self.hold_pattern = Some(pattern.clone());
                    }
                    EventKind::SpoofingStart {
                        sensor: SensorKind::Gnss,
                        rate_m_s,
                        bearing_deg,
                    } => {
                        self.gnss_spoofing = Some(Spoofing {
                            since_s: event.t_s,
                            rate_m_s: *rate_m_s,
                            bearing_deg: *bearing_deg,
                        });
                    }
                    EventKind::SpoofingEnd {
                        sensor: SensorKind::Gnss,
                    } => self.gnss_spoofing = None,
                    _ => {}
                }
                self.events.push(event);
                let over = self.events.len().saturating_sub(EVENTS_KEPT);
                self.events.drain(..over);
            }
            ProgressRecord::Finished(_) => self.finished = true,
        }
    }

    /// The plan's completion as `event` reports it, once.
    fn note_route_end(&mut self, event: &EventRecord) {
        let EventKind::WaypointSequenced {
            ident,
            plan_complete: true,
            ..
        } = &event.kind
        else {
            return;
        };
        if self.route_end.is_none() {
            self.route_end = Some(RouteEnd {
                t_s: event.t_s,
                ident: ident.clone(),
                samples: self
                    .traces
                    .iter()
                    .map(|trace| trace.latest().copied())
                    .collect(),
            });
        }
    }

    /// Where the vehicle is in the holding pattern it flies after its plan.
    pub const fn hold(&self) -> Option<&HoldState> {
        self.hold.as_ref()
    }

    /// The flight's picture when the plan completed, once it has.
    pub const fn route_end(&self) -> Option<&RouteEnd> {
        self.route_end.as_ref()
    }

    /// Every set's trace, in the run's order.
    pub fn traces(&self) -> &[SetTrace] {
        &self.traces
    }

    /// TRUTH: the simulator's latitude and longitude over route time,
    /// degrees, one point per sampled instant.
    pub fn truth(&self) -> &[(f64, (f64, f64))] {
        self.truth.items()
    }

    /// The route's waypoints in fly order, latitude and longitude in
    /// degrees.
    pub fn waypoints(&self) -> &[(String, (f64, f64))] {
        &self.waypoints
    }

    /// Each waypoint's height above the WGS84 ellipsoid, metres, in fly order.
    pub fn waypoint_altitudes_m(&self) -> &[f64] {
        &self.altitudes_m
    }

    /// The waypoint the mission host last sequenced, its index in fly order, and
    /// whether that completed the plan; kept however many events follow.
    pub const fn sequenced(&self) -> Option<(usize, bool)> {
        self.sequenced
    }

    /// The holding pattern the mission host flies, once it holds.
    pub const fn hold_pattern(&self) -> Option<&HoldingPatternRecord> {
        self.hold_pattern.as_ref()
    }

    /// Each visual set's latest terrain matches.
    pub const fn matches(&self) -> &MatchLog {
        &self.matches
    }

    /// The terrain camera's latest frame, its outcomes and the fix marks.
    pub const fn camera(&self) -> &CameraLog {
        &self.camera
    }

    /// The sensors of `set` at its latest sample.
    pub fn sensors(&self, set: &str) -> Option<&SensorStates> {
        self.sensors.get(set)
    }

    /// The spoofer holding the GNSS receiver now, if one does.
    pub const fn gnss_spoofing(&self) -> Option<&Spoofing> {
        self.gnss_spoofing.as_ref()
    }

    /// The mission phase, once the mission host has named one.
    pub fn phase(&self) -> Option<&str> {
        self.phase.as_deref()
    }

    /// The latest events, oldest first.
    pub fn events(&self) -> &[EventRecord] {
        &self.events
    }

    /// The mission host's latest reports with their route times, oldest
    /// first.
    pub fn mission_events(&self) -> &[(f64, String)] {
        &self.mission
    }

    /// The waypoint the flight last sequenced, with its index in fly order.
    pub fn last_waypoint(&self) -> Option<(usize, &str)> {
        self.events
            .iter()
            .rev()
            .find_map(|event| match &event.kind {
                EventKind::WaypointSequenced {
                    waypoint_index,
                    ident,
                    ..
                } => Some((*waypoint_index, ident.as_str())),
                _ => None,
            })
    }

    /// Route time of the latest record, seconds.
    pub const fn sim_time_s(&self) -> f64 {
        self.sim_time_s
    }

    /// The planned flight time, seconds, once the run has started.
    pub const fn planned_duration_s(&self) -> Option<f64> {
        self.planned_duration_s
    }

    /// Whether the run has ended.
    pub const fn finished(&self) -> bool {
        self.finished
    }

    /// Why the terrain camera's frames of `set` went unused, each reason
    /// with its count, matched and unmatched frames together, most frequent
    /// first; and the accepted count.
    pub fn unused_reasons(&self, set: &str) -> Option<(u64, Vec<(String, u64)>)> {
        let fixes = self.fixes.get(set)?;
        let mut reasons: BTreeMap<&str, u64> = BTreeMap::new();
        for (key, count) in fixes {
            if let Some(rest) = key.strip_prefix("unused:") {
                let reason = rest.split(':').next().unwrap_or(rest);
                let entry = reasons.entry(reason).or_default();
                *entry = entry.saturating_add(*count);
            }
        }
        let mut reasons: Vec<(String, u64)> = reasons
            .into_iter()
            .map(|(reason, count)| (reason.to_owned(), count))
            .collect();
        reasons.sort_by(|a, b| b.1.cmp(&a.1).then_with(|| a.0.cmp(&b.0)));
        let accepted = fixes.get("accepted").copied().unwrap_or(0);
        Some((accepted, reasons))
    }

    /// The sensors a visitor can switch: those whose means some set of the
    /// run uses. A means the build does not link is in no set, so it has
    /// no switch.
    pub fn switches(&self) -> Vec<MeansSwitch> {
        SensorKind::ALL
            .into_iter()
            .map(|sensor| MeansSwitch {
                sensor,
                means: sensor.means(),
            })
            .filter(|switch| {
                self.traces
                    .iter()
                    .any(|trace| trace.means.contains(&switch.means))
            })
            .collect()
    }
}
