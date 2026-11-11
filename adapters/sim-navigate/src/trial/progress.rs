//! The progress stream: records a running trial hands to a caller-supplied
//! sink so a monitor can watch every set's error grow in route time.
//!
//! The stream is for evaluation, so its samples carry the simulator's
//! truth beside each set's estimate. It is written from the scoring side,
//! after the navigator has produced its estimate; nothing in it flows back
//! into a navigator, and the result file is the same with or without it.
//!
//! A trial emits `started`, then per time slice the `event`s up to the
//! slice and every set's `sample` there, ordered by route time; the caller
//! emits `finished`, because only it knows where the result went and how
//! long the run took on the wall clock.

mod slices;
#[cfg(test)]
mod tests;

use std::io;

use serde::{Deserialize, Serialize};

pub(super) use slices::{Scored, Slices, started};

use super::GuidanceSource;
use super::availability::{SensorKind, SensorState};
use crate::NavMeans;

/// The schema number of [`ProgressRecord`].
pub const PROGRESS_SCHEMA: u32 = 1;

/// One line of the progress stream.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum ProgressRecord {
    /// The run began.
    Started(StartedRecord),
    /// One set at one instant.
    Sample(SampleRecord),
    /// Something happened.
    Event(EventRecord),
    /// The run ended.
    Finished(FinishedRecord),
}

impl ProgressRecord {
    /// The record as one line of JSON, without the line end.
    pub fn to_json_line(&self) -> Result<String, super::TrialError> {
        serde_json::to_string(self).map_err(|source| super::TrialError::Json {
            what: "progress record",
            source,
        })
    }

    /// Parses one line of the stream.
    pub fn from_json_line(line: &str) -> Result<Self, super::TrialError> {
        serde_json::from_str(line).map_err(|source| super::TrialError::Json {
            what: "progress record",
            source,
        })
    }
}

/// What the run is.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct StartedRecord {
    /// [`PROGRESS_SCHEMA`].
    pub schema: u32,
    /// The result's request digest.
    pub request_sha256: String,
    /// The trial epoch, UTC.
    pub epoch_utc: String,
    /// The route.
    pub route: ProgressRoute,
    /// Every set, in request order.
    pub sets: Vec<ProgressSet>,
    /// What guidance steers on.
    pub guidance: GuidanceSource,
}

/// The route as planned.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ProgressRoute {
    /// The waypoints in fly order.
    pub waypoints: Vec<ProgressWaypoint>,
    /// Sum of each leg's length over its speed, seconds; turns and climbs
    /// make the flight a little longer.
    pub planned_duration_s: f64,
}

/// One waypoint.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ProgressWaypoint {
    /// Identifier.
    pub ident: String,
    /// Latitude, degrees.
    pub lat_deg: f64,
    /// Longitude, degrees east.
    pub lon_deg: f64,
    /// Height above the ellipsoid, metres.
    pub altitude_m: f64,
}

/// One set.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ProgressSet {
    /// Its name.
    pub name: String,
    /// Its means.
    pub means: Vec<NavMeans>,
}

/// One set at one instant: the simulator's truth, the set's estimate, and
/// what each sensor was doing.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SampleRecord {
    /// Route time, seconds.
    pub t_s: f64,
    /// The set's name.
    pub set: String,
    /// TRUTH: the simulator's true position, for evaluation only.
    pub truth: ProgressPosition,
    /// The set's navigation estimate.
    pub estimate: ProgressPosition,
    /// Horizontal error of the estimate against truth, metres.
    pub horizontal_error_m: f64,
    /// Vertical error of the estimate against truth, metres.
    pub vertical_error_m: f64,
    /// The estimate's own horizontal 1-sigma, metres.
    pub sigma_horizontal_m: f64,
    /// TRUTH: the true distance from the leg being flown, right positive,
    /// metres; `null` with no leg active.
    pub cross_track_true_m: Option<f64>,
    /// What each sensor was doing.
    pub sensors: SensorStates,
}

/// A position.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ProgressPosition {
    /// Latitude, degrees.
    pub lat_deg: f64,
    /// Longitude, degrees east.
    pub lon_deg: f64,
    /// Height above the ellipsoid, metres.
    pub altitude_m: f64,
}

/// What each sensor was doing for a set.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SensorStates {
    /// The GNSS receiver.
    pub gnss: SensorState,
    /// The barometric altimeter.
    pub baro: SensorState,
    /// The star camera.
    pub stars: SensorState,
}

/// Something that happened at an instant.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct EventRecord {
    /// Route time, seconds.
    pub t_s: f64,
    /// The set it happened to; `null` for the shared open-loop flight.
    pub set: Option<String>,
    /// What happened.
    #[serde(flatten)]
    pub kind: EventKind,
}

/// What happened.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum EventKind {
    /// Navigate's executor sequenced a waypoint.
    WaypointSequenced {
        /// Its index in fly order.
        waypoint_index: usize,
        /// Its identifier.
        ident: String,
        /// Whether it completed the plan.
        plan_complete: bool,
    },
    /// A scheduled interval began.
    AvailabilityStart {
        /// The sensor withheld.
        sensor: SensorKind,
        /// The schedule.
        schedule: String,
        /// The schedule's reason.
        reason: Option<String>,
    },
    /// A scheduled interval ended.
    AvailabilityEnd {
        /// The sensor withheld.
        sensor: SensorKind,
        /// The schedule.
        schedule: String,
        /// The schedule's reason.
        reason: Option<String>,
    },
    /// Closed-loop guidance began refusing the set's solution; one event per
    /// run of consecutive refused steps.
    GuidanceRefused {
        /// Navigate's refusal kind.
        refusal: String,
    },
    /// A closed-loop flight was ended by a termination rule.
    Terminated {
        /// The rule.
        rule: String,
        /// The values that tripped it.
        detail: String,
    },
}

/// The run ended.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct FinishedRecord {
    /// Where the result was written; `null` when it was not.
    pub result_path: Option<String>,
    /// Wall-clock duration of the run, seconds. It appears only here, never
    /// in the result.
    pub wall_time_s: f64,
}

/// Where progress records go.
pub trait ProgressSink {
    /// Takes one record; an error stops the trial.
    fn record(&mut self, record: &ProgressRecord) -> io::Result<()>;
}

impl<F: FnMut(&ProgressRecord) -> io::Result<()>> ProgressSink for F {
    fn record(&mut self, record: &ProgressRecord) -> io::Result<()> {
        self(record)
    }
}

/// A sink that drops every record.
#[derive(Debug, Clone, Copy, Default)]
pub struct NoProgress;

impl ProgressSink for NoProgress {
    fn record(&mut self, _record: &ProgressRecord) -> io::Result<()> {
        Ok(())
    }
}
