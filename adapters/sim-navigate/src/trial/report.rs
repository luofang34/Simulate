//! The trial's result file.

use serde::{Deserialize, Serialize};

use super::replay::{SensorTallies, StarAvailability};
use super::{OutageSpec, RouteEvent, SeedSpec, TrialError, WaypointSpec};
use crate::NavMeans;

/// The schema identifier of [`TrialReport`].
pub const REPORT_SCHEMA: &str = "simulate.nav-trial.report/1";

/// Everything one trial measured, per sensor set, over one truth.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct TrialReport {
    /// [`REPORT_SCHEMA`].
    pub schema: String,
    /// What identifies the inputs.
    pub inputs: InputIdentity,
    /// Which components are Navigate's and which are simulated, and how.
    pub models: Vec<ModelStatement>,
    /// The seeds of every draw.
    pub seeds: SeedSpec,
    /// The flown route.
    pub route: RouteReport,
    /// GNSS outages as requested.
    pub outages: Vec<OutageSpec>,
    /// Horizontal 1-sigma of the simulated GNSS position error, metres.
    pub gnss_horizontal_sigma_m: f64,
    /// One entry per sensor set, in request order.
    pub sensor_sets: Vec<SetReport>,
    /// The onboard map and the world imagery, when a set navigates
    /// visually.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub visual_maps: Option<super::visual::VisualMapsReport>,
}

/// What identifies a run's inputs.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct InputIdentity {
    /// SHA-256 of the request's compact JSON, hex.
    pub request_sha256: String,
    /// The `sim-navigate` crate version that ran it.
    pub sim_navigate_version: String,
    /// The trial epoch, UTC.
    pub epoch_utc: String,
}

/// One component and what it is.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ModelStatement {
    /// The component.
    pub component: String,
    /// `navigate` for Navigate's own code, `simulated` for Simulate's model.
    pub origin: String,
    /// What it is and what it leaves out.
    pub statement: String,
}

/// The route as flown.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RouteReport {
    /// The waypoints as requested.
    pub waypoints: Vec<WaypointSpec>,
    /// Navigate's sequencing events.
    pub events: Vec<RouteEvent>,
    /// Route time from start to completion, seconds.
    pub duration_s: f64,
    /// The true track, downsampled.
    pub track: Vec<TrackPoint>,
}

/// One point of the true track.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct TrackPoint {
    /// Route time, seconds.
    pub time_s: f64,
    /// Latitude, degrees.
    pub lat_deg: f64,
    /// Longitude, degrees.
    pub lon_deg: f64,
    /// Height above the ellipsoid, metres.
    pub height_m: f64,
}

/// One sensor set's result.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SetReport {
    /// The set's name.
    pub name: String,
    /// Its means.
    pub means: Vec<NavMeans>,
    /// Whether the GNSS outages applied.
    pub gnss_outages: bool,
    /// Summary numbers.
    pub summary: SetSummary,
    /// Observations offered, accepted and refused, per sensor.
    pub observations: SensorTallies,
    /// The star tracker's availability; `None` without stars.
    pub stars: Option<StarAvailability>,
    /// The scored samples.
    pub series: Vec<SeriesSample>,
    /// The set's own flight, in closed loop only.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub closed_loop: Option<super::closed::ClosedLoopReport>,
    /// The set's visual navigation, when it has it.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub visual: Option<super::visual::VisualReport>,
    /// What the set's aiding adds over its baseline, when it has aiding.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub contribution: Option<super::Contribution>,
}

/// One scored sample: truth against the filter's estimate and its own
/// claimed 1-sigma.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SeriesSample {
    /// Route time, seconds.
    pub time_s: f64,
    /// Horizontal position error, metres.
    pub horizontal_error_m: f64,
    /// Vertical position error, metres.
    pub vertical_error_m: f64,
    /// Velocity error magnitude, m/s.
    pub velocity_error_m_s: f64,
    /// Attitude error angle, radians.
    pub attitude_error_rad: f64,
    /// Claimed horizontal 1-sigma, `√(P_nn + P_ee)`, metres.
    pub horizontal_sigma_m: f64,
    /// Claimed vertical 1-sigma, metres.
    pub vertical_sigma_m: f64,
    /// Claimed velocity 1-sigma, `√tr P_v`, m/s.
    pub velocity_sigma_m_s: f64,
    /// Claimed attitude 1-sigma, `√tr P_θ`, radians.
    pub attitude_sigma_rad: f64,
    /// Position NEES, 3 degrees of freedom.
    pub position_nees: f64,
    /// Attitude NEES, 3 degrees of freedom.
    pub attitude_nees: f64,
}

/// Distribution of one error over the run.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ErrorStats {
    /// Root mean square.
    pub rms: f64,
    /// Median.
    pub p50: f64,
    /// 95th percentile, nearest rank.
    pub p95: f64,
    /// Largest.
    pub max: f64,
    /// At the last sample.
    pub last: f64,
}

/// One GNSS outage seen from a set that honoured it.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct OutageReport {
    /// First second without GNSS.
    pub start_s: f64,
    /// Last second without GNSS.
    pub end_s: f64,
    /// Horizontal error at the start, metres.
    pub horizontal_error_at_start_m: f64,
    /// Horizontal error at the end, the unaided error, metres.
    pub horizontal_error_at_end_m: f64,
    /// Claimed horizontal 1-sigma at the end, metres.
    pub horizontal_sigma_at_end_m: f64,
    /// Mean horizontal error growth over the outage, m/s.
    pub horizontal_growth_m_s: f64,
    /// Mean position NEES over the outage.
    pub mean_position_nees: f64,
}

/// A set's summary numbers.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SetSummary {
    /// Samples scored.
    pub samples: u64,
    /// Horizontal position error, metres.
    pub horizontal_error_m: ErrorStats,
    /// Vertical position error, metres.
    pub vertical_error_m: ErrorStats,
    /// Velocity error, m/s.
    pub velocity_error_m_s: ErrorStats,
    /// Attitude error, radians.
    pub attitude_error_rad: ErrorStats,
    /// Mean position NEES; consistent outputs average 3.
    pub mean_position_nees: f64,
    /// Mean attitude NEES; consistent outputs average 3.
    pub mean_attitude_nees: f64,
    /// Each honoured outage.
    pub outages: Vec<OutageReport>,
    /// Whether the filter's claimed covariance matched its errors.
    pub consistency: ConsistencyNote,
}

/// The filter's consistency over a run, judged from the scored samples.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ConsistencyNote {
    /// `consistent`, `overconfident`, `conservative` or `diverged`.
    pub verdict: String,
    /// Mean 3-DOF position NEES over the samples.
    pub mean_position_nees: f64,
    /// The band a consistent filter's mean NEES lies in.
    pub band: [f64; 2],
    /// Largest attitude error, radians.
    pub max_attitude_error_rad: f64,
    /// The verdict in words.
    pub statement: String,
}

impl TrialReport {
    /// The report as pretty JSON. Every number must be finite: JSON has no
    /// infinity or NaN, and writing one as `null` would not read back.
    pub fn to_json(&self) -> Result<String, TrialError> {
        let text = serde_json::to_string_pretty(self).map_err(|source| TrialError::Json {
            what: "result",
            source,
        })?;
        if self.has_non_finite() {
            return Err(TrialError::NonFinite { what: "result" });
        }
        Ok(text)
    }

    /// Parses a report from JSON.
    pub fn from_json(text: &str) -> Result<Self, TrialError> {
        serde_json::from_str(text).map_err(|source| TrialError::Json {
            what: "result",
            source,
        })
    }

    fn has_non_finite(&self) -> bool {
        self.sensor_sets.iter().any(|set| {
            let m = &set.summary;
            let stats = [
                m.horizontal_error_m,
                m.vertical_error_m,
                m.velocity_error_m_s,
                m.attitude_error_rad,
            ];
            let summary_finite = stats.iter().all(|e| {
                [e.rms, e.p50, e.p95, e.max, e.last]
                    .iter()
                    .all(|v| v.is_finite())
            }) && m.mean_position_nees.is_finite()
                && m.mean_attitude_nees.is_finite();
            !summary_finite
                || set.series.iter().any(|s| {
                    ![
                        s.horizontal_error_m,
                        s.vertical_error_m,
                        s.velocity_error_m_s,
                        s.attitude_error_rad,
                        s.horizontal_sigma_m,
                        s.vertical_sigma_m,
                        s.velocity_sigma_m_s,
                        s.attitude_sigma_rad,
                        s.position_nees,
                        s.attitude_nees,
                    ]
                    .iter()
                    .all(|v| v.is_finite())
                })
        })
    }
}
