//! 4-way outcome taxonomy separating:
//! 1. **Constraint unsatisfied** ([`AccessRestrictionReason`]),
//! 2. **Data missing** ([`DataCoverageError`]),
//! 3. **Model not applicable** ([`ModelNotApplicableReason`]), and
//! 4. **Search completeness** ([`WindowSearchCompleteness`]).

use serde::{Deserialize, Serialize};
use thiserror::Error;

use crate::{
    DynamicalModelIdentity, NaifId, SpatiotemporalOrigin, TdbEpoch, UnitSemantics,
    ValidEpochInterval, terrain::VerticalDatumKind,
};

/// Maximum number of uniform coarse-grid steps permitted in a single access query
/// before failing closed with [`ModelNotApplicableReason::QueryStepCountExceeded`].
pub const MAX_ACCESS_GRID_STEPS: usize = 200_000;

/// Category 1: Specific geometric/physical constraint that is unsatisfied at a valid sample epoch.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub enum AccessRestrictionReason {
    /// Topocentric elevation is below the station's minimum elevation mask (`elevation_rad < min_elevation_rad`).
    BelowMinimumElevation {
        /// Evaluated topocentric elevation (`rad`).
        elevation_rad: f64,
        /// Required minimum elevation (`rad`).
        min_elevation_rad: f64,
    },
    /// Topocentric elevation at the target endpoint is below its minimum horizon mask.
    BelowTargetEndpointMinimumElevation {
        /// Evaluated target-endpoint local elevation (`rad`).
        elevation_rad: f64,
        /// Required minimum elevation (`rad`).
        min_elevation_rad: f64,
    },
    /// Line of sight falls outside the ground station's antenna/telescope `FOV`.
    OutsideStationFov {
        /// Station off-boresight angle (`rad`).
        off_boresight_rad: f64,
    },
    /// Line of sight falls outside the vehicle sensor's `FOV`.
    OutsideVehicleSensorFov {
        /// Vehicle sensor off-boresight angle (`rad`).
        off_boresight_rad: f64,
    },
    /// Line of sight falls outside the target moving platform sensor's `FOV`.
    OutsideTargetSensorFov {
        /// Target endpoint identifier.
        endpoint_id: u32,
        /// Target sensor off-boresight angle (`rad`).
        off_boresight_rad: f64,
    },
    /// Line of sight is geometrically occulted by a celestial body (`min_altitude_above_surface_m < 0`).
    OccultedByBody {
        /// Occulting body NAIF ID.
        body: NaifId,
        /// Minimum chord clearance above the occulting surface (`m`, negative).
        min_altitude_above_surface_m: f64,
    },
    /// Line of sight is obstructed by local DEM terrain (`min_clearance_m < required_clearance_m`).
    OccultedByTerrain {
        /// Host body NAIF ID.
        body: NaifId,
        /// Minimum observed vertical clearance above DEM terrain along the chord (`m`).
        min_clearance_m: f64,
        /// Required minimum vertical clearance above DEM terrain (`m`).
        required_clearance_m: f64,
        /// Geodetic latitude of the worst terrain obstruction (`rad`).
        obstruction_lat_rad: f64,
        /// Geodetic longitude of the worst terrain obstruction (`rad`).
        obstruction_lon_rad: f64,
    },
    /// Slant range is shorter than `AccessQuerySpec::min_range_m`.
    RangeBelowMinimum {
        /// Evaluated slant range (`m`).
        slant_range_m: f64,
        /// Required minimum range (`m`).
        min_range_m: f64,
    },
    /// Slant range exceeds `AccessQuerySpec::max_range_m`.
    RangeAboveMaximum {
        /// Evaluated slant range (`m`).
        slant_range_m: f64,
        /// Allowed maximum range (`m`).
        max_range_m: f64,
    },
}

/// Category 2: Missing trajectory, attitude, ephemeris, orientation, or DEM terrain data coverage.
#[derive(Clone, Copy, Debug, PartialEq, Error, Serialize, Deserialize)]
pub enum DataCoverageError {
    /// Requested query interval or epoch falls outside the translational trajectory valid interval.
    #[error(
        "requested epoch {requested_epoch:?} is outside translational trajectory coverage [{:?}, {:?}]",
        .valid_interval.start_epoch,
        .valid_interval.end_epoch
    )]
    TrajectoryOutOfCoverage {
        /// Epoch that fell outside coverage.
        requested_epoch: TdbEpoch,
        /// Valid coverage interval of the trajectory source.
        valid_interval: ValidEpochInterval,
    },
    /// Sensor requires vehicle attitude (`require_attitude = true`), but no attitude source was provided or epoch fell outside coverage.
    #[error("requested epoch {requested_epoch:?} is outside attitude trajectory coverage")]
    AttitudeOutOfCoverage {
        /// Epoch that lacked attitude coverage.
        requested_epoch: TdbEpoch,
        /// Valid coverage interval of the attitude source (if provided).
        valid_interval: Option<ValidEpochInterval>,
    },
    /// Celestial ephemeris catalog lacks coverage for `body` at `requested_epoch`.
    #[error("ephemeris coverage missing for body {body} at {requested_epoch:?}")]
    EphemerisOutOfCoverage {
        /// NAIF body whose ephemeris was unavailable.
        body: NaifId,
        /// Requested epoch.
        requested_epoch: TdbEpoch,
    },
    /// Planetary orientation catalog lacks coverage for `body` at `requested_epoch`.
    #[error("orientation coverage missing for body {body} at {requested_epoch:?}")]
    OrientationOutOfCoverage {
        /// NAIF body whose orientation was unavailable.
        body: NaifId,
        /// Requested epoch.
        requested_epoch: TdbEpoch,
    },
    /// Local DEM terrain coverage is missing or has a void cell at `(lat_rad, lon_rad)` (never defaults to `0.0 m`).
    #[error(
        "DEM terrain coverage missing for body {body} at (lat={lat_rad:.6} rad, lon={lon_rad:.6} rad) at {requested_epoch:?}"
    )]
    TerrainDemOutOfCoverage {
        /// NAIF body whose DEM coverage was missing.
        body: NaifId,
        /// Geodetic latitude (`rad`) outside DEM coverage or on a void cell.
        lat_rad: f64,
        /// Geodetic longitude (`rad`) outside DEM coverage or on a void cell.
        lon_rad: f64,
        /// Requested epoch.
        requested_epoch: TdbEpoch,
    },
    /// Required vertical datum (`DemSurfaceAgl` or `OrthometricMsl`) is unavailable at `(lat_rad, lon_rad)`.
    #[error(
        "vertical datum {datum_kind:?} unavailable at (lat={lat_rad:.6} rad, lon={lon_rad:.6} rad)"
    )]
    VerticalDatumUnavailable {
        /// Vertical datum kind that could not be resolved.
        datum_kind: VerticalDatumKind,
        /// Geodetic latitude (`rad`).
        lat_rad: f64,
        /// Geodetic longitude (`rad`).
        lon_rad: f64,
    },
}

/// Category 3: Dynamical model, coordinate unit, or query configuration is not physically applicable.
#[derive(Clone, Debug, PartialEq, Error, Serialize, Deserialize)]
pub enum ModelNotApplicableReason {
    /// Dynamical model is an idealized dimensionless abstraction (`AutonomousCr3bp`) that does not
    /// support physical solar-system planetary ground station access without an ephemeris embedding.
    #[error("dynamical model {model:?} is not applicable to physical planetary station access")]
    UnsupportedDynamicalModel {
        /// Dynamical model identity reported by the trajectory source.
        model: DynamicalModelIdentity,
    },
    /// Trajectory uses non-SI normalized units (such as dimensionless `CR3BP`), which cannot be
    /// evaluated against physical SI planetary ground stations without an explicit dimensional scale.
    #[error("trajectory uses non-SI units ({actual:?}); physical station access requires SiMetric")]
    NonSiTrajectoryUnits {
        /// Unit semantics reported by the trajectory source.
        actual: UnitSemantics,
    },
    /// Trajectory origin cannot be resolved to solar-system barycentric `ICRF`.
    #[error("trajectory origin {origin:?} is not supported for planetary station access")]
    UnsupportedTrajectoryOrigin {
        /// Origin reported by the trajectory source.
        origin: SpatiotemporalOrigin,
    },
    /// Query time step or boundary tolerance is non-positive or non-finite.
    #[error(
        "invalid query step ({sample_step_s} s) or boundary tolerance ({boundary_time_tolerance_s} s)"
    )]
    InvalidQueryParameters {
        /// Supplied coarse sampling step (`s`).
        sample_step_s: f64,
        /// Supplied boundary refinement tolerance (`s`).
        boundary_time_tolerance_s: f64,
    },
    /// Ground station minimum elevation is outside $[-\pi/2, \pi/2]$ or non-finite.
    #[error("invalid station minimum elevation {min_elevation_rad} rad")]
    InvalidStationParameters {
        /// Supplied minimum elevation (`rad`).
        min_elevation_rad: f64,
    },
    /// DEM terrain grid or terrain LOS parameters are malformed.
    #[error("invalid terrain parameters: {reason}")]
    InvalidTerrainParameters {
        /// Explanation of the invalid terrain parameter.
        reason: String,
    },
    /// Query interval and `sample_step_s` require more coarse grid steps than [`MAX_ACCESS_GRID_STEPS`].
    #[error(
        "query interval duration ({interval_duration_s} s) at sample_step_s ({sample_step_s} s) requires {requested_steps} steps, exceeding MAX_ACCESS_GRID_STEPS ({max_steps})"
    )]
    QueryStepCountExceeded {
        /// Required coarse grid steps $\lceil \Delta t / \delta t \rceil$.
        requested_steps: u64,
        /// Maximum allowed coarse grid steps ([`MAX_ACCESS_GRID_STEPS`]).
        max_steps: u64,
        /// Query interval duration (`s`).
        interval_duration_s: f64,
        /// Requested sample step (`s`).
        sample_step_s: f64,
    },
    /// DEM terrain line-of-sight chord within the terrain interaction shell requires more samples
    /// than `TerrainLosConfig::max_ray_samples`, failing closed instead of silently coarsening the step.
    #[error(
        "terrain LOS chord in shell ({chord_length_in_shell_m:.2} m) at step ({sample_step_m:.2} m) requires {required_samples} samples, exceeding max_ray_samples ({max_ray_samples})"
    )]
    TerrainRaySampleBudgetExceeded {
        /// Chord length inside the terrain interaction shell (`m`).
        chord_length_in_shell_m: f64,
        /// Effective required sample spacing (`m`).
        sample_step_m: f64,
        /// Required sample count along the shell chord.
        required_samples: u64,
        /// Configured `TerrainLosConfig::max_ray_samples` ceiling.
        max_ray_samples: u32,
    },
    /// Request coordinate-frame or physical-unit contract does not match the evaluator's SI/ICRF/WGS84 contract.
    #[error(
        "unsupported coordinate frame or unit contract: inertial={inertial_frame}, fixed={body_fixed_frame}, length={length_unit}, angle={angle_unit}"
    )]
    UnsupportedCoordinateFrameOrUnitContract {
        /// Requested inertial frame.
        inertial_frame: String,
        /// Requested body-fixed frame.
        body_fixed_frame: String,
        /// Requested length unit.
        length_unit: String,
        /// Requested angle unit.
        angle_unit: String,
    },
    /// Request time-scale contract does not match the evaluator's `TdbJ2000` master-clock contract.
    #[error("unsupported time-scale contract: {reason}")]
    UnsupportedTimeScaleContract {
        /// Explanation of the time-scale contract mismatch.
        reason: String,
    },
    /// Caller requested a position interpolation accuracy bound that cannot be certified from discrete trajectory nodes alone.
    #[error(
        "requested position interpolation error bound ({requested_error_m} m) cannot be proven for uncertified discrete trajectory nodes"
    )]
    UnprovableInterpolationAccuracyBound {
        /// Caller-requested position interpolation error ceiling (`m`).
        requested_error_m: f64,
    },
}

/// Top-level error returned by access and relay evaluators, strictly distinguishing
/// missing data coverage ([`DataCoverageError`]) from model inapplicability ([`ModelNotApplicableReason`]).
#[derive(Clone, Debug, PartialEq, Error, Serialize, Deserialize)]
pub enum AccessAnalysisError {
    /// Required trajectory, attitude, ephemeris, orientation, or DEM data was missing.
    #[error("data coverage error: {0}")]
    DataCoverage(#[from] DataCoverageError),
    /// Model, coordinate frame, unit, or sampling budget is not applicable.
    #[error("model not applicable: {0}")]
    ModelNotApplicable(#[from] ModelNotApplicableReason),
}

/// Category 4: Explicit declaration of window-search completeness and sub-grid miss bounds.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub enum WindowSearchCompleteness {
    /// Evaluated on a fixed uniform grid (`sample_step_s`) without bracket root refinement.
    SampledGridOnly {
        /// Uniform grid spacing (`s`).
        sample_step_s: f64,
    },
    /// Sign transitions between adjacent samples on the `sample_step_s` grid were refined via bisection
    /// to `boundary_tolerance_s`, and accessible coarse intervals are scanned across
    /// [`Self::INTERIOR_DROPOUT_SUBSTEPS`] interior substeps for brief dropouts. Short access windows
    /// of duration `< may_miss_subgrid_windows_shorter_than_s` that both open and close inside a single
    /// coarse grid interval are not guaranteed to be detected.
    BracketRefinedOnGrid {
        /// Coarse grid step used for bracket detection (`s`).
        sample_step_s: f64,
        /// Bisection convergence tolerance on entry/exit boundaries (`s`).
        boundary_tolerance_s: f64,
        /// Duration bound (`s`) below which a window wholly inside one coarse step may be missed.
        may_miss_subgrid_windows_shorter_than_s: f64,
    },
    /// Completeness guarantees are unknown (e.g. externally imported window list).
    CompletenessUnknown,
}

impl WindowSearchCompleteness {
    /// Number of uniform sub-intervals probed inside each accessible coarse step `[t_k, t_{k+1}]`
    /// during bracket-refined window extraction to detect brief interior occlusions/dropouts.
    pub const INTERIOR_DROPOUT_SUBSTEPS: u32 = 16;

    /// Returns `true` only when bracket root refinement and interior dropout scanning were performed.
    #[must_use]
    pub const fn is_bracket_refined(&self) -> bool {
        matches!(self, Self::BracketRefinedOnGrid { .. })
    }

    /// Upper bound (`s`) on the duration of an interior dropout that could occur strictly between
    /// probed samples without being detected, or `None` if completeness is unknown.
    #[must_use]
    pub fn max_undetected_dropout_s(&self) -> Option<f64> {
        match self {
            Self::SampledGridOnly { sample_step_s } => Some(*sample_step_s),
            Self::BracketRefinedOnGrid { sample_step_s, .. } => {
                Some(*sample_step_s / f64::from(Self::INTERIOR_DROPOUT_SUBSTEPS))
            }
            Self::CompletenessUnknown => None,
        }
    }

    /// Returns `true` only when this completeness level was bracket-refined (`BracketRefinedOnGrid`)
    /// and its maximum undetected interior dropout bound is at most `max_allowed_undetected_dropout_s`.
    ///
    /// Discrete grid sampling (`SampledGridOnly`) never certifies continuous interval coverage.
    #[must_use]
    pub fn certifies_continuous_coverage(&self, max_allowed_undetected_dropout_s: f64) -> bool {
        if !self.is_bracket_refined()
            || !max_allowed_undetected_dropout_s.is_finite()
            || max_allowed_undetected_dropout_s <= 0.0
        {
            return false;
        }
        self.max_undetected_dropout_s()
            .is_some_and(|bound| bound <= max_allowed_undetected_dropout_s + 1.0e-12)
    }
}

/// Continuous access interval `[start_epoch, end_epoch]` during which all access constraints hold.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct AccessWindow {
    /// Window opening (rise / acquisition of signal) epoch (`TDB`).
    pub start_epoch: TdbEpoch,
    /// Window closing (set / loss of signal) epoch (`TDB`).
    pub end_epoch: TdbEpoch,
    /// Window duration (`s`).
    pub duration_s: f64,
    /// Peak topocentric elevation observed during the window (`rad`).
    pub max_elevation_rad: f64,
    /// Minimum slant range observed during the window (`m`).
    pub min_slant_range_m: f64,
}
