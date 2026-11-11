use std::collections::BTreeMap;

use serde::{Deserialize, Serialize};

use crate::trial::TrackPoint;

/// One sensor set's closed-loop flight: where its navigator truly took the
/// vehicle.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ClosedLoopReport {
    /// The navigator sequenced the final waypoint.
    pub completed: bool,
    /// The termination rule that ended the flight, if one fired.
    pub termination: Option<TerminationReport>,
    /// Route time flown, seconds.
    pub duration_s: f64,
    /// The true ground track, downsampled.
    pub track: Vec<TrackPoint>,
    /// True cross-track deviation from the leg being flown, per sample.
    pub cross_track: Vec<CrossTrackSample>,
    /// Largest absolute true cross-track deviation, metres.
    pub max_abs_cross_track_m: f64,
    /// Each waypoint's true outcome.
    pub waypoints: Vec<WaypointOutcome>,
    /// True distance to the final waypoint when the navigator declared the
    /// plan complete, metres.
    pub final_arrival_error_m: Option<f64>,
    /// Guidance refusals by Navigate's refusal kind; each held the present
    /// track and height for one step.
    pub guidance_refusals: BTreeMap<String, u64>,
}

/// Which termination rule fired.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct TerminationReport {
    /// `left_corridor`, `altitude_limit` or `sequence_timeout`.
    pub rule: String,
    /// Route time, seconds.
    pub time_s: f64,
    /// The values that tripped it.
    pub detail: String,
}

/// True cross-track deviation at one sample.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct CrossTrackSample {
    /// Route time, seconds.
    pub time_s: f64,
    /// The waypoint the navigator was flying to.
    pub leg_index: usize,
    /// True deviation from that leg's track, right positive, metres.
    pub true_cross_track_m: f64,
    /// True track minus the leg's course, radians in `[-π, π)`.
    pub true_track_error_rad: f64,
}

/// One waypoint in closed loop.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct WaypointOutcome {
    /// Index in fly order.
    pub index: usize,
    /// Identifier.
    pub ident: String,
    /// The distance inside which the executor sequences it, metres.
    pub capture_threshold_m: f64,
    /// When the navigator sequenced it, route seconds.
    pub sequenced_at_s: Option<f64>,
    /// True horizontal distance to the fix at that moment, metres, on the
    /// ellipsoid in the true position's north-east plane.
    pub true_miss_at_sequence_m: Option<f64>,
    /// The navigator's own distance to the fix at that moment, metres.
    pub estimated_distance_at_sequence_m: Option<f64>,
    /// The navigator's horizontal error at that moment, metres.
    pub navigation_error_at_sequence_m: Option<f64>,
    /// Whether the true position was inside the capture threshold then.
    pub truly_within_threshold: Option<bool>,
    /// Closest true approach over the whole flight, metres.
    pub closest_true_approach_m: f64,
}
