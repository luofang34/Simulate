//! Geodesic H3 planetary coverage grid evaluation (`evaluate_coverage_grid`)
//! with N-fold simultaneous constellation visibility and revisit-gap statistics.

mod evaluator;
mod region;

use serde::{Deserialize, Serialize};
use sim_geo::{BodyEllipsoid, Geodetic};
use sim_spacetime::{
    AttitudeTrajectorySource, BodyOrientationCatalog, EphemerisCatalog, NaifId, SensorMount,
    TranslationalTrajectorySource, ValidEpochInterval, body_ellipsoid_serde,
};
use thiserror::Error;

use crate::{
    access::AccessAnalysisError,
    models::{GroundStationModel, OccultationEnvironmentModel, SensorFovModel, VehicleSensorModel},
};

use evaluator::{run_coverage_grid, validate_coverage_inputs};
use region::resolve_region_cells;

/// Maximum H3 resolution permitted for [`CoverageRegionSpec::Global`] (`Resolution::Three`, 41,162 cells).
pub const MAX_GLOBAL_H3_RESOLUTION: u8 = 3;

/// Typed errors returned by geodesic H3 coverage grid validation and evaluation.
#[derive(Clone, Debug, PartialEq, Error, Serialize, Deserialize)]
pub enum CoverageGridError {
    /// H3 resolution byte is outside the valid range `0..=15`.
    #[error("invalid H3 resolution {resolution} (valid range is 0..=15)")]
    InvalidResolution {
        /// Supplied H3 resolution value.
        resolution: u8,
    },
    /// Global coverage grid resolution exceeds [`MAX_GLOBAL_H3_RESOLUTION`].
    #[error("global H3 resolution {resolution} exceeds maximum global resolution {max_resolution}")]
    GlobalResolutionTooHigh {
        /// Supplied H3 resolution.
        resolution: u8,
        /// Maximum supported global H3 resolution.
        max_resolution: u8,
    },
    /// One or more raw `u64` entries in [`CoverageRegionSpec::ExplicitCells`] are invalid H3 cell indexes.
    #[error("invalid H3 cell index {raw_index:#018x}: {reason}")]
    InvalidCellIndex {
        /// Raw `u64` H3 cell identifier.
        raw_index: u64,
        /// Validation failure message from `h3o`.
        reason: String,
    },
    /// Resolved coverage region produced zero H3 cells.
    #[error("coverage region contains zero H3 cells")]
    EmptyRegion,
    /// Resolved cell count exceeds [`CoverageQuerySpec::max_cells`].
    #[error("coverage cell count {cell_count} exceeds max_cells limit ({max_cells})")]
    CellCountExceeded {
        /// Resolved or required cell count.
        cell_count: usize,
        /// Configured `max_cells` ceiling.
        max_cells: usize,
    },
    /// Latitude/longitude bounding box is non-finite or out of valid bounds.
    #[error("invalid LatLonBox bounds: {reason}")]
    InvalidRegionBounds {
        /// Explanation of the invalid region bounds.
        reason: String,
    },
    /// Asset constellation slice is empty.
    #[error("coverage asset constellation is empty")]
    EmptyConstellation,
    /// Required simultaneous visibility fold is zero or exceeds the constellation asset count.
    #[error(
        "invalid N-fold requirement min_fold={required_fold} for constellation of {asset_count} asset(s)"
    )]
    InvalidRequiredFold {
        /// Requested minimum simultaneous visible asset count.
        required_fold: usize,
        /// Number of assets in the constellation.
        asset_count: usize,
    },
    /// Minimum elevation mask is non-finite or outside $[-\pi/2, \pi/2]$.
    #[error("invalid minimum elevation {min_elevation_rad} rad")]
    InvalidMinElevation {
        /// Supplied minimum elevation (`rad`).
        min_elevation_rad: f64,
    },
    /// Query interval or sample step is malformed.
    #[error("invalid coverage query time specification: {reason}")]
    InvalidTimeSpecification {
        /// Explanation of the invalid time parameter.
        reason: String,
    },
    /// Underlying trajectory, attitude, ephemeris, orientation, or grid-step failure.
    #[error("access evaluation error: {0}")]
    Access(#[from] AccessAnalysisError),
}

/// Spatial region specification on a planetary ellipsoid discretized into H3 cells.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum CoverageRegionSpec {
    /// Whole planetary globe at H3 `resolution` (`0..=3`).
    Global {
        /// H3 grid resolution (`0..=3`).
        resolution: u8,
    },
    /// Geographic latitude/longitude box (`rad`), supporting antimeridian crossing when `min_lon_rad > max_lon_rad`.
    LatLonBox {
        /// H3 grid resolution (`0..=15`).
        resolution: u8,
        /// Minimum geodetic latitude (`rad`, `[-pi/2, pi/2]`).
        min_lat_rad: f64,
        /// Maximum geodetic latitude (`rad`, `[min_lat_rad, pi/2]`).
        max_lat_rad: f64,
        /// Western longitude bound (`rad`, `[-pi, pi]`).
        min_lon_rad: f64,
        /// Eastern longitude bound (`rad`, `[-pi, pi]`; `min_lon_rad > max_lon_rad` crosses the antimeridian).
        max_lon_rad: f64,
    },
    /// Caller-supplied list of raw `u64` H3 cell indexes (validated and deduplicated).
    ExplicitCells {
        /// Raw `u64` H3 cell indexes.
        h3_cells: Vec<u64>,
    },
}

/// One constellation asset evaluated against every H3 surface cell in the coverage grid.
#[derive(Clone, Copy)]
pub struct CoverageAssetSpec<'a> {
    /// Translational trajectory source (`SiMetric`, `IcrfInertial`).
    pub trajectory: &'a dyn TranslationalTrajectorySource,
    /// Optional attitude source (required when `sensor.require_attitude == true`).
    pub attitude: Option<&'a dyn AttitudeTrajectorySource>,
    /// Vehicle-mounted sensor or antenna model.
    pub sensor: &'a VehicleSensorModel,
}

/// Query configuration for evaluating a geodesic H3 coverage grid.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct CoverageQuerySpec {
    /// Host celestial body covered by the surface grid (`NaifId::EARTH`, `MOON`, `MARS`, etc.).
    pub body: NaifId,
    /// Reference ellipsoid of `body`.
    #[serde(with = "body_ellipsoid_serde")]
    pub ellipsoid: BodyEllipsoid,
    /// H3 grid region specification.
    pub region: CoverageRegionSpec,
    /// Minimum local topocentric horizon elevation mask (`rad`, $[-\pi/2, \pi/2]$).
    pub min_elevation_rad: f64,
    /// Required N-fold simultaneous visibility threshold (`>= 1`).
    pub min_fold: usize,
    /// Closed `TDB` query interval `[start_epoch, end_epoch]`.
    pub query_interval: ValidEpochInterval,
    /// Uniform grid sampling step (`s`, `> 0`).
    pub sample_step_s: f64,
    /// Celestial bodies tested for line-of-sight occultation.
    pub occultation_env: OccultationEnvironmentModel,
    /// Maximum number of H3 cells permitted before failing closed with [`CoverageGridError::CellCountExceeded`].
    pub max_cells: usize,
}

/// Coverage, fold, and revisit-gap summary for one H3 surface cell.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct CellCoverageSummary {
    /// Canonical `u64` H3 cell index.
    pub h3_index: u64,
    /// Geodetic centroid latitude (`rad`).
    pub lat_rad: f64,
    /// Geodetic centroid longitude (`rad`).
    pub lon_rad: f64,
    /// Count of sample instants achieving `fold >= min_fold`.
    pub covered_samples: usize,
    /// Total number of evaluated time instants.
    pub total_samples: usize,
    /// Fraction of time instants achieving `fold >= min_fold` (`covered_samples / total_samples`).
    pub coverage_fraction: f64,
    /// Minimum simultaneous visible asset fold observed at this cell.
    pub min_fold_observed: usize,
    /// Maximum simultaneous visible asset fold observed at this cell.
    pub max_fold_observed: usize,
    /// Longest contiguous revisit gap duration (`s`, `0.0` when continuously covered).
    pub max_revisit_gap_s: f64,
    /// Mean revisit gap duration across all gaps (`s`, `0.0` when continuously covered).
    pub mean_revisit_gap_s: f64,
    /// Contiguous intervals where observed fold fell below `min_fold`.
    pub gaps: Vec<ValidEpochInterval>,
}

impl CellCoverageSummary {
    /// Builds the equivalent [`GroundStationModel`] at this cell's geodetic centroid (`height_m = 0`).
    #[must_use]
    pub fn as_ground_station(
        &self,
        body: NaifId,
        ellipsoid: BodyEllipsoid,
        min_elevation_rad: f64,
    ) -> GroundStationModel {
        let fov = if min_elevation_rad >= 0.0 {
            SensorFovModel::OmnidirectionalHemisphere
        } else {
            SensorFovModel::ConicalHalfAngle {
                max_half_angle_rad: core::f64::consts::PI,
            }
        };
        GroundStationModel {
            station_id: (self.h3_index & 0xffff_ffff) as u32,
            name: format!("h3-{:016x}", self.h3_index),
            body,
            geodetic: Geodetic {
                lat_rad: self.lat_rad,
                lon_rad: self.lon_rad,
                height_m: 0.0,
            },
            ellipsoid,
            mount: SensorMount::IDENTITY,
            fov,
            min_elevation_rad,
        }
    }
}

/// Complete deterministic H3 coverage grid report sorted strictly by `h3_index`.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct CoverageGridReport {
    /// Per-cell summaries sorted strictly ascending by [`CellCoverageSummary::h3_index`].
    pub cells: Vec<CellCoverageSummary>,
    /// Arithmetic mean of [`CellCoverageSummary::coverage_fraction`] across all cells.
    pub mean_coverage_fraction: f64,
    /// Minimum [`CellCoverageSummary::coverage_fraction`] across all cells.
    pub min_coverage_fraction: f64,
    /// Maximum [`CellCoverageSummary::max_revisit_gap_s`] across all cells (`s`).
    pub global_max_revisit_gap_s: f64,
    /// Fraction of all `(cell, time)` instants achieving `fold >= min_fold`.
    pub n_fold_fraction: f64,
    /// Total number of evaluated H3 cells (`cells.len()`).
    pub total_cells: usize,
}

/// Evaluates N-fold simultaneous visibility and revisit gaps across a geodesic H3 surface grid.
///
/// Cells are ordered strictly by their canonical `u64` H3 index for deterministic cross-target output.
pub fn evaluate_coverage_grid(
    assets: &[CoverageAssetSpec<'_>],
    query: &CoverageQuerySpec,
    ephemeris: &dyn EphemerisCatalog,
    orientation: &dyn BodyOrientationCatalog,
) -> Result<CoverageGridReport, CoverageGridError> {
    validate_coverage_inputs(assets, query)?;
    let resolved_cells = resolve_region_cells(&query.region, query.max_cells)?;
    run_coverage_grid(&resolved_cells, assets, query, ephemeris, orientation)
}
