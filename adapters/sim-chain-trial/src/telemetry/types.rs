//! Serializable ML/RL-training-grade and RNP telemetry schemas.

use std::collections::BTreeMap;

use serde::{Deserialize, Serialize};

use crate::SpoofKind;

/// Current schema version for [`TelemetrySample`] and [`TelemetrySummary`].
pub const TELEMETRY_SCHEMA: u32 = 1;

/// Configuration for [`super::TelemetryRecorder`].
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct TelemetryConfig {
    /// Optional explicit RNP containment limit in nautical miles; `None`
    /// selects the standard ICAO PBN phase limit (`1.0` NM enroute/terminal,
    /// `0.3` NM approach).
    pub rnp_limit_nm: Option<f64>,
    /// Optional Monte Carlo seed identifier stamped onto recorded samples.
    pub seed: Option<u64>,
}

/// One synchronized truth, sensor-input, multi-navigator, and RNP sample.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct TelemetrySample {
    /// Telemetry schema version ([`TELEMETRY_SCHEMA`]).
    pub schema: u32,
    /// Optional Monte Carlo seed tag for multi-run dataset generation.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub seed: Option<u64>,
    /// Exact 6-DoF aircraft truth and ambient atmosphere state.
    pub truth: TruthTelemetry,
    /// Sensor health, physical measurements, camera metrics, and ground-truth
    /// adversarial GNSS spoofing vectors.
    pub inputs: InputTelemetry,
    /// Estimates, covariances, NEES consistency, and integrity per navigator.
    pub navigators: Vec<NavigatorTelemetry>,
    /// Active leg modes and ICAO RNP (`NSE` / `FTE` / `TSE`) decomposition.
    pub guidance_rnp: GuidanceRnpTelemetry,
}

/// Exact 6-DoF simulation truth at one instant.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct TruthTelemetry {
    /// Route time since `SimInstant::START`, seconds.
    pub t_s: f64,
    /// Exact master-clock tick count.
    pub tick: u128,
    /// True WGS84 geodetic latitude, degrees.
    pub lat_deg: f64,
    /// True WGS84 longitude, degrees east.
    pub lon_deg: f64,
    /// True orthometric altitude above EGM96 mean sea level, metres.
    pub alt_msl_m: f64,
    /// True altitude above local terrain (`alt_msl_m - terrain_elev_m`), metres.
    pub alt_agl_m: f64,
    /// Local terrain orthometric elevation above MSL, metres.
    pub terrain_elev_m: f64,
    /// True north velocity, m/s.
    pub vn_mps: f64,
    /// True east velocity, m/s.
    pub ve_mps: f64,
    /// True down velocity, m/s.
    pub vd_mps: f64,
    /// True horizontal ground speed, m/s.
    pub ground_speed_mps: f64,
    /// True ground track clockwise from true north in `[0, 360)`, degrees.
    pub true_track_deg: f64,
    /// True Euler bank angle (right wing down positive), degrees.
    pub roll_deg: f64,
    /// True Euler pitch angle (nose up positive), degrees.
    pub pitch_deg: f64,
    /// True Euler heading clockwise from true north in `[0, 360)`, degrees.
    pub yaw_deg: f64,
    /// Body roll rate `p` relative to Earth, rad/s.
    pub p_rps: f64,
    /// Body pitch rate `q` relative to Earth, rad/s.
    pub q_rps: f64,
    /// Body yaw rate `r` relative to Earth, rad/s.
    pub r_rps: f64,
    /// True airspeed relative to the local airmass, m/s.
    pub tas_mps: f64,
    /// Local ambient wind `[north, east, down]`, m/s.
    pub wind_ned_mps: [f64; 3],
}

/// Physical sensor measurements, availability flags, spoofing vectors, and
/// terrain-camera frame diagnostics.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct InputTelemetry {
    /// Whether the GNSS receiver is in service and tracking satellites.
    pub gnss_available: bool,
    /// Whether the strapdown IMU delivered an increment on the latest cycle.
    pub imu_ok: bool,
    /// Whether the static-pressure barometer is in service and delivering data.
    pub baro_ok: bool,
    /// Whether the tri-axis magnetometer has delivered a valid field reading.
    pub mag_ok: bool,
    /// Whether the pitot differential-pressure channel is active and fresh.
    pub pitot_ok: bool,
    /// Whether the terrain camera is configured and not in a device outage.
    pub camera_ok: bool,
    /// Sensed pitot impact (differential) pressure, pascals.
    pub pitot_diff_pressure_pa: Option<f64>,
    /// Sensed indicated airspeed derived from `pitot_diff_pressure_pa`, m/s.
    pub ias_mps: Option<f64>,
    /// Sensed barometric pressure altitude (ISA), metres.
    pub baro_alt_msl_m: Option<f64>,
    /// Sensed body-frame magnetic field `[x, y, z]`, microtesla.
    pub mag_body_ut: Option<[f64; 3]>,
    /// Tilt-compensated magnetic heading in `[0, 360)`, degrees.
    pub mag_heading_deg: Option<f64>,
    /// Ground-truth flag indicating whether adversarial GNSS spoofing is active.
    pub spoof_active: bool,
    /// Ground-truth active spoofing profile kind.
    pub spoof_kind: SpoofKind,
    /// Ground-truth injected GNSS position offset `[east, north, up]`, metres.
    pub spoof_offset_enu_m: [f64; 3],
    /// Ground-truth GNSS spoofing pull rate `[east, north, up]`, m/s.
    pub spoof_rate_enu_mps: [f64; 3],
    /// Horizontal magnitude of `spoof_offset_enu_m`, metres.
    pub spoof_horizontal_mag_m: f64,
    /// Latest raw GNSS receiver fix `[lat_deg, lon_deg, alt_msl_m]`.
    pub gnss_measured_lla: Option<[f64; 3]>,
    /// True antenna `[lat_deg, lon_deg, alt_msl_m]` at that GNSS fix instant.
    pub gnss_true_lla: Option<[f64; 3]>,
    /// Latest terrain-camera frame diagnostics when a camera is active.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub camera_frame: Option<CameraFrameTelemetry>,
}

/// Diagnostics for the latest rendered terrain-camera exposure and visual match.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct CameraFrameTelemetry {
    /// Monotonic camera exposure index (1-based).
    pub frame_index: u64,
    /// Off-nadir obliquity angle of the camera optical axis, degrees.
    pub nadir_obliquity_deg: f64,
    /// Normalized RMS neighbor luminance contrast in `[0.0, 1.0]`.
    pub local_contrast: f64,
    /// Visual matcher verdict (`accepted`, `faint_refused`, `no_match`, etc.).
    pub match_outcome: String,
    /// Geometric fit inlier count when a pose solve ran.
    pub inliers: Option<usize>,
}

/// State estimate, covariance bounds, NEES consistency, and source integrity
/// for one navigator (`Navigate` sensor set or `flight-controller`).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct NavigatorTelemetry {
    /// Navigator / sensor-set identifier.
    pub set: String,
    /// Estimated WGS84 latitude, degrees.
    pub est_lat_deg: f64,
    /// Estimated WGS84 longitude, degrees east.
    pub est_lon_deg: f64,
    /// Estimated orthometric altitude above EGM96 MSL, metres.
    pub est_alt_msl_m: f64,
    /// Estimated NED velocity `[vn, ve, vd]`, m/s.
    pub est_vel_ned_mps: [f64; 3],
    /// True estimation error `estimate - truth` in local ENU `[east, north, up]`, metres.
    pub error_enu_m: [f64; 3],
    /// True 2D horizontal position error, metres.
    pub horizontal_error_m: f64,
    /// True absolute vertical position error, metres.
    pub vertical_error_m: f64,
    /// True 3D velocity error magnitude, m/s.
    pub velocity_error_mps: f64,
    /// Reported 1-sigma east position uncertainty, metres.
    pub sigma_east_m: Option<f64>,
    /// Reported 1-sigma north position uncertainty, metres.
    pub sigma_north_m: Option<f64>,
    /// Reported 1-sigma vertical position uncertainty, metres.
    pub sigma_up_m: Option<f64>,
    /// Reported horizontal radial 1-sigma `sqrt(sigma_e^2 + sigma_n^2)`, metres.
    pub sigma_horiz_m: Option<f64>,
    /// Estimated Position Uncertainty (`95%` circular bound from 2D eigenvalues), metres.
    pub epu_95_m: Option<f64>,
    /// Vertical Position Uncertainty (`95%` Gaussian bound `1.96 * sigma_up`), metres.
    pub vpu_95_m: Option<f64>,
    /// 2-DoF horizontal Normalized Estimation Error Squared (`e_ne^T P_ne^{-1} e_ne`).
    pub nees_horiz_2d: Option<f64>,
    /// 3-DoF position Normalized Estimation Error Squared (`e_ned^T P_ned^{-1} e_ned`).
    pub nees_pos_3d: Option<f64>,
    /// Horizontal calibration ratio `horizontal_error_m / sigma_horiz_m`.
    pub calibration_ratio: Option<f64>,
    /// Active position-source mode (`gnss`, `terrain`, `blended`, `inertial`, `dead_reckoning`, `orbit_hold`, `flight_controller`).
    pub position_source: String,
    /// GNSS cross-check verdict (`normal`, `degraded`, `excluded`, `not_monitored`).
    pub gnss_integrity: String,
    /// Sensor classes declared in `NavigationSolution.integrity.contributing`.
    pub contributing_sources: Vec<String>,
    /// Remaining provenance hold window for the active aiding source, seconds.
    pub source_hold_remaining_s: Option<f64>,
    /// `500 m x 500 m` ENU `MapCell` coordinates `[cell_x, cell_y]` for visual fixes.
    pub map_cell_id: Option<[i64; 2]>,
    /// Whether the latest visual fix in `map_cell_id` entered an unvisited `500 m` cell.
    pub independent_fix: Option<bool>,
}

/// Active mission guidance state and ICAO RNP (`NSE` / `FTE` / `TSE`) triad.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct GuidanceRnpTelemetry {
    /// Navigator whose solution steers Pilotage's mission.
    pub steered_set: String,
    /// Active route waypoint index (`None` in terminal hold or before load).
    pub active_leg_index: Option<usize>,
    /// Active leg geometry (`direct_to_fix`, `track_to_fix`, `holding_pattern`, `none`).
    pub leg_kind: String,
    /// Pilotage lateral guidance mode.
    pub lateral_mode: String,
    /// Pilotage vertical guidance mode.
    pub vertical_mode: String,
    /// ICAO PBN flight phase (`enroute`, `terminal`, `approach`, `missed`, `unloaded`).
    pub flight_phase: String,
    /// Active RNP containment limit, nautical miles.
    pub rnp_limit_nm: f64,
    /// Active RNP containment limit, metres (`1852.0 * rnp_limit_nm`).
    pub rnp_limit_m: f64,
    /// Signed Navigation System Error perpendicular to desired track (`tse - fte`), metres.
    pub nse_cross_track_m: Option<f64>,
    /// Full 2D horizontal Navigation System Error of the steered navigator, metres.
    pub nse_horiz_m: Option<f64>,
    /// Signed Flight Technical Error (estimated cross-track deviation, right positive), metres.
    pub fte_cross_track_m: Option<f64>,
    /// Signed Total System Error (true cross-track deviation, right positive), metres.
    pub tse_cross_track_m: Option<f64>,
    /// Onboard `95%` total cross-track bound `hypot(epu_95_m, fte_cross_track_m)`, metres.
    pub rnp_95_bound_m: Option<f64>,
    /// Whether `|tse_cross_track_m| <= rnp_limit_m` (and `rnp_95_bound_m <= rnp_limit_m` when reported).
    pub within_rnp: Option<bool>,
}

/// Distributional statistics (`P50`, `P95`, `RMSE`, `Max`) for a scalar error series.
#[derive(Debug, Clone, Copy, Default, PartialEq, Serialize, Deserialize)]
pub struct MetricSummary {
    /// Number of finite samples included.
    pub count: usize,
    /// Median (`50th` percentile) magnitude.
    pub p50: f64,
    /// `95th` percentile magnitude.
    pub p95: f64,
    /// Root-mean-square error.
    pub rmse: f64,
    /// Worst-case magnitude observed.
    pub max: f64,
}

/// Mean NEES and two-sided `95%` chi-square consistency band check.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct NeesConsistencySummary {
    /// Degrees of freedom (`2` for horizontal, `3` for 3D position).
    pub dof: usize,
    /// Number of samples evaluated.
    pub samples: usize,
    /// Sample mean NEES (`expected == dof` for a calibrated estimator).
    pub mean: f64,
    /// `95th` percentile single-epoch NEES.
    pub p95: f64,
    /// Lower `2.5%` bound on the sample mean under `chi^2(samples * dof) / samples`.
    pub chi2_lower_95: f64,
    /// Upper `97.5%` bound on the sample mean under `chi^2(samples * dof) / samples`.
    pub chi2_upper_95: f64,
    /// Whether `mean` lies within `[chi2_lower_95, chi2_upper_95]`.
    pub within_95_band: bool,
}

/// Adversarial GNSS spoofing detection, exclusion, and post-spoof recovery summary.
#[derive(Debug, Clone, Copy, Default, PartialEq, Serialize, Deserialize)]
pub struct SpoofingSummary {
    /// Whether any spoofing window became active during the run.
    pub spoof_injected: bool,
    /// First route time at which `spoof_active` became `true`, seconds.
    pub spoof_start_s: Option<f64>,
    /// Last route time at which an active spoofing window ended, seconds.
    pub spoof_end_s: Option<f64>,
    /// Time from `spoof_start_s` until GNSS integrity first left `normal` (`degraded` or `excluded`), seconds.
    pub time_to_detect_s: Option<f64>,
    /// Time from `spoof_start_s` until GNSS integrity reached `excluded`, seconds.
    pub time_to_exclude_s: Option<f64>,
    /// Peak steered horizontal error from `spoof_start_s` up to exclusion (or run end if never excluded), metres.
    pub max_error_before_exclusion_m: Option<f64>,
    /// Time from `spoof_end_s` until GNSS integrity returned to `normal`, seconds.
    pub time_to_recover_s: Option<f64>,
}

/// Per-navigator run summary.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct NavigatorSummary {
    /// Horizontal position error statistics, metres.
    pub horizontal_error_m: MetricSummary,
    /// Vertical position error statistics, metres.
    pub vertical_error_m: MetricSummary,
    /// 2-DoF horizontal NEES summary.
    pub nees_horiz_2d: Option<NeesConsistencySummary>,
    /// 3-DoF position NEES summary.
    pub nees_pos_3d: Option<NeesConsistencySummary>,
    /// Percentage of samples spent in each `position_source` (`0.0..=100.0`).
    pub position_source_pct: BTreeMap<String, f64>,
    /// GNSS spoofing detection & recovery metrics for this navigator.
    pub spoofing: SpoofingSummary,
}

/// Run-level statistical report emitted to `--summary <summary.json>`.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct TelemetrySummary {
    /// Telemetry schema version ([`TELEMETRY_SCHEMA`]).
    pub schema: u32,
    /// Optional Monte Carlo seed tag.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub seed: Option<u64>,
    /// Number of recorded samples.
    pub sample_count: usize,
    /// Simulated duration spanned by the recorded samples, seconds.
    pub duration_s: f64,
    /// Steered navigator name.
    pub steered_set: String,
    /// Active RNP limit at run completion, nautical miles.
    pub rnp_limit_nm: f64,
    /// Active RNP limit at run completion, metres.
    pub rnp_limit_m: f64,
    /// Steered horizontal error distribution, metres.
    pub horizontal_error_m: MetricSummary,
    /// Cross-track Navigation System Error (`|NSE|`) distribution, metres.
    pub nse_cross_track_m: MetricSummary,
    /// Cross-track Flight Technical Error (`|FTE|`) distribution, metres.
    pub fte_cross_track_m: MetricSummary,
    /// Cross-track Total System Error (`|TSE|`) distribution, metres.
    pub tse_cross_track_m: MetricSummary,
    /// Steered 2-DoF horizontal NEES consistency summary.
    pub nees_horiz_2d: Option<NeesConsistencySummary>,
    /// Steered 3-DoF position NEES consistency summary.
    pub nees_pos_3d: Option<NeesConsistencySummary>,
    /// Percentage of run time spent in each steered position source (`0.0..=100.0`).
    pub position_source_pct: BTreeMap<String, f64>,
    /// Fraction of leg-guided samples meeting `within_rnp` (`0.0..=1.0`).
    pub rnp_containment_fraction: f64,
    /// Steered navigator GNSS spoofing detection, exclusion, and recovery metrics.
    pub spoofing: SpoofingSummary,
    /// Per-navigator breakdown across all compared sensor sets.
    pub per_navigator: BTreeMap<String, NavigatorSummary>,
}
