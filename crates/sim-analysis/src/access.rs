//! Propagator-agnostic station access evaluation (`evaluate_station_access`) and
//! the 4-way outcome taxonomy separating:
//! 1. **Constraint unsatisfied** ([`AccessRestrictionReason`]),
//! 2. **Data missing** ([`DataCoverageError`]),
//! 3. **Model not applicable** ([`ModelNotApplicableReason`]), and
//! 4. **Search completeness** ([`WindowSearchCompleteness`]).

mod evaluator;

use serde::{Deserialize, Serialize};
use sim_spacetime::{
    AttitudeTrajectorySource, BodyOrientationCatalog, DataProvenance, EphemerisCatalog,
    TranslationalTrajectorySource, ValidEpochInterval,
};

pub use sim_analysis_contract::{
    AccessAnalysisError, AccessRestrictionReason, AccessWindow, DataCoverageError,
    MAX_ACCESS_GRID_STEPS, ModelNotApplicableReason, WindowSearchCompleteness,
};

use crate::{
    geometry::LinkGeometrySample,
    models::{
        AccessQuerySpec, GroundStationModel, OccultationEnvironmentModel, VehicleSensorModel,
    },
};

use evaluator::{
    AccessEvalCtx, evaluate_single_epoch, extract_access_windows,
    validate_applicability_and_coverage,
};
pub(crate) use evaluator::{build_query_grid_epochs, validate_query_grid_step_count};

/// One time-series sample in a [`StationAccessReport`].
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct StationAccessSample {
    /// Evaluated link geometry at `geometry.epoch`.
    pub geometry: LinkGeometrySample,
    /// Active restriction reasons (`empty` iff `accessible == true`).
    pub active_restrictions: Vec<AccessRestrictionReason>,
    /// `true` iff every station, vehicle sensor, range, and occultation constraint is satisfied.
    pub accessible: bool,
}

/// Complete station-to-vehicle access report carrying [`DataProvenance`], [`ValidEpochInterval`],
/// explicit [`WindowSearchCompleteness`], time-series samples, and refined [`AccessWindow`]s.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct StationAccessReport {
    /// Station identifier (`GroundStationModel::station_id`).
    pub station_id: u32,
    /// Station name.
    pub station_name: String,
    /// Vehicle sensor identifier (`VehicleSensorModel::sensor_id`).
    pub sensor_id: u32,
    /// Evaluated TDB interval.
    pub valid_interval: ValidEpochInterval,
    /// Explicit search completeness declaration.
    pub completeness: WindowSearchCompleteness,
    /// Input dataset and trajectory provenance records.
    pub provenance: Vec<DataProvenance>,
    /// Uniform grid samples across `valid_interval`.
    pub samples: Vec<StationAccessSample>,
    /// Detected continuous access windows.
    pub windows: Vec<AccessWindow>,
}

/// Evaluates station-to-vehicle link geometry, constraint restrictions, and access windows
/// over any [`TranslationalTrajectorySource`] and optional [`AttitudeTrajectorySource`].
#[allow(clippy::too_many_arguments)]
pub fn evaluate_station_access(
    trajectory: &dyn TranslationalTrajectorySource,
    attitude: Option<&dyn AttitudeTrajectorySource>,
    station: &GroundStationModel,
    sensor: &VehicleSensorModel,
    occultation_env: &OccultationEnvironmentModel,
    query: &AccessQuerySpec,
    ephemeris: &dyn EphemerisCatalog,
    orientation: &dyn BodyOrientationCatalog,
) -> Result<StationAccessReport, AccessAnalysisError> {
    validate_applicability_and_coverage(trajectory, attitude, station, sensor, query)?;

    let ctx = AccessEvalCtx {
        trajectory,
        attitude,
        station,
        sensor,
        occultation_env,
        query,
        ephemeris,
        orientation,
    };

    let epochs = build_query_grid_epochs(query.query_interval, query.sample_step_s)?;
    let mut samples = Vec::with_capacity(epochs.len());
    for &ep in &epochs {
        samples.push(evaluate_single_epoch(&ctx, ep)?);
    }

    let refine = query.boundary_time_tolerance_s < query.sample_step_s;
    let windows = extract_access_windows(&ctx, &samples, refine)?;
    let completeness = if refine {
        WindowSearchCompleteness::BracketRefinedOnGrid {
            sample_step_s: query.sample_step_s,
            boundary_tolerance_s: query.boundary_time_tolerance_s,
            may_miss_subgrid_windows_shorter_than_s: query.sample_step_s,
        }
    } else {
        WindowSearchCompleteness::SampledGridOnly {
            sample_step_s: query.sample_step_s,
        }
    };

    Ok(StationAccessReport {
        station_id: station.station_id,
        station_name: station.name.clone(),
        sensor_id: sensor.sensor_id,
        valid_interval: query.query_interval,
        completeness,
        provenance: vec![trajectory.provenance().clone()],
        samples,
        windows,
    })
}
