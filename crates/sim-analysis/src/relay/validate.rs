//! Validation and provenance helpers for [`AnalysisEndpoint`] and [`TerrainLosConfig`].

use sim_spacetime::{
    AttitudeTrajectorySource, DataProvenance, TICKS_PER_SECOND, TranslationalTrajectorySource,
    ValidEpochInterval,
};

use crate::{
    access::{
        AccessAnalysisError, DataCoverageError, ModelNotApplicableReason,
        validate_query_grid_step_count,
    },
    models::{AccessQuerySpec, VehicleSensorModel},
    relay::AnalysisEndpoint,
    terrain::{TerrainElevationSource, TerrainLosConfig},
};

pub(crate) fn validate_terrain_config(
    terrain_dem: Option<(&dyn TerrainElevationSource, &TerrainLosConfig)>,
) -> Result<(), AccessAnalysisError> {
    if let Some((_, cfg)) = terrain_dem
        && (!cfg.sample_step_m.is_finite()
            || cfg.sample_step_m <= 0.0
            || !cfg.min_clearance_m.is_finite()
            || !cfg.max_terrain_interaction_altitude_m.is_finite()
            || cfg.max_ray_samples == 0)
    {
        return Err(ModelNotApplicableReason::InvalidTerrainParameters {
            reason: "non-finite, non-positive, or zero-budget TerrainLosConfig parameters"
                .to_owned(),
        }
        .into());
    }
    Ok(())
}

pub(super) fn validate_query_and_terrain(
    query: &AccessQuerySpec,
    terrain_dem: Option<(&dyn TerrainElevationSource, &TerrainLosConfig)>,
) -> Result<(), AccessAnalysisError> {
    let min_step_s = 1.0 / (TICKS_PER_SECOND as f64);
    if !query.sample_step_s.is_finite()
        || query.sample_step_s < min_step_s
        || !query.boundary_time_tolerance_s.is_finite()
        || query.boundary_time_tolerance_s <= 0.0
    {
        return Err(ModelNotApplicableReason::InvalidQueryParameters {
            sample_step_s: query.sample_step_s,
            boundary_time_tolerance_s: query.boundary_time_tolerance_s,
        }
        .into());
    }
    validate_query_grid_step_count(query.query_interval, query.sample_step_s)?;
    validate_terrain_config(terrain_dem)
}

pub(crate) fn validate_endpoint_static(
    endpoint: &AnalysisEndpoint<'_>,
) -> Result<(), AccessAnalysisError> {
    match *endpoint {
        AnalysisEndpoint::SurfaceStation { station, .. } => {
            if !station.min_elevation_rad.is_finite()
                || station.min_elevation_rad < -core::f64::consts::FRAC_PI_2
                || station.min_elevation_rad > core::f64::consts::FRAC_PI_2
            {
                return Err(ModelNotApplicableReason::InvalidStationParameters {
                    min_elevation_rad: station.min_elevation_rad,
                }
                .into());
            }
            Ok(())
        }
        AnalysisEndpoint::MovingPlatform {
            trajectory,
            min_local_elevation_rad,
            ..
        } => {
            let model = trajectory.model_identity();
            if !model.supports_physical_ephemeris_analysis() {
                return Err(ModelNotApplicableReason::UnsupportedDynamicalModel { model }.into());
            }
            if let Some(min_el) = min_local_elevation_rad
                && (!min_el.is_finite()
                    || min_el < -core::f64::consts::FRAC_PI_2
                    || min_el > core::f64::consts::FRAC_PI_2)
            {
                return Err(ModelNotApplicableReason::InvalidStationParameters {
                    min_elevation_rad: min_el,
                }
                .into());
            }
            Ok(())
        }
    }
}

pub(crate) fn validate_moving_platform_interval(
    trajectory: &dyn TranslationalTrajectorySource,
    attitude: Option<&dyn AttitudeTrajectorySource>,
    sensor: &VehicleSensorModel,
    interval: ValidEpochInterval,
) -> Result<(), AccessAnalysisError> {
    let model = trajectory.model_identity();
    if !model.supports_physical_ephemeris_analysis() {
        return Err(ModelNotApplicableReason::UnsupportedDynamicalModel { model }.into());
    }
    let traj_cov = trajectory.valid_interval();
    for ep in [interval.start_epoch, interval.end_epoch] {
        if !traj_cov.contains(ep) {
            return Err(DataCoverageError::TrajectoryOutOfCoverage {
                requested_epoch: ep,
                valid_interval: traj_cov,
            }
            .into());
        }
    }
    if sensor.require_attitude {
        let Some(att) = attitude else {
            return Err(DataCoverageError::AttitudeOutOfCoverage {
                requested_epoch: interval.start_epoch,
                valid_interval: None,
            }
            .into());
        };
        let att_cov = att.attitude_valid_interval();
        for ep in [interval.start_epoch, interval.end_epoch] {
            if !att_cov.contains(ep) {
                return Err(DataCoverageError::AttitudeOutOfCoverage {
                    requested_epoch: ep,
                    valid_interval: Some(att_cov),
                }
                .into());
            }
        }
    }
    Ok(())
}

pub(crate) fn validate_endpoint(
    endpoint: &AnalysisEndpoint<'_>,
    query: &AccessQuerySpec,
) -> Result<(), AccessAnalysisError> {
    validate_endpoint_static(endpoint)?;
    if let AnalysisEndpoint::MovingPlatform {
        trajectory,
        attitude,
        sensor,
        ..
    } = *endpoint
    {
        validate_moving_platform_interval(trajectory, attitude, sensor, query.query_interval)?;
    }
    Ok(())
}

pub(crate) fn collect_endpoint_provenance(
    endpoint: &AnalysisEndpoint<'_>,
    out: &mut Vec<DataProvenance>,
) {
    if let AnalysisEndpoint::MovingPlatform { trajectory, .. } = *endpoint {
        let p = trajectory.provenance();
        if !out.iter().any(|existing| existing == p) {
            out.push(p.clone());
        }
    }
}
