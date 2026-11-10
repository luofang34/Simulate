//! Headless evaluator (`evaluate_analysis_request_artifact`) for [`AnalysisRequestArtifact`].

use sim_geo::BodyEllipsoid;
use sim_spacetime::{BodyOrientationCatalog, EphemerisCatalog, NaifId};

use crate::{
    access::{AccessAnalysisError, DataCoverageError, ModelNotApplicableReason},
    contract::{
        ANALYSIS_CONTRACT_SCHEMA_VERSION, AnalysisJobStatus, AnalysisOutcomePayload,
        AnalysisRequestArtifact, AnalysisResultArtifact, AnalysisWorkloadSpec,
        ReproducibilityStamp, SerializableGroundStationSpec, SerializablePlatformSpec,
        sample_platform_trajectory,
    },
    models::VehicleSensorModel,
    relay::{
        AnalysisEndpoint, VerificationScope, evaluate_endpoint_link_access,
        evaluate_geometric_relay_access,
    },
    terrain::{DemElevationGrid, TerrainElevationSource, TerrainLosConfig},
};

/// Evaluates an [`AnalysisRequestArtifact`] headlessly in `Simulate` and returns a sealed [`AnalysisResultArtifact`].
pub fn evaluate_analysis_request_artifact(
    request: &AnalysisRequestArtifact,
    result_id: impl Into<String>,
    ephemeris: &dyn EphemerisCatalog,
    orientation: &dyn BodyOrientationCatalog,
) -> AnalysisResultArtifact {
    let result_id_str = result_id.into();
    let req_digest = request.content_hash();
    let eval_res = execute_workload(request, ephemeris, orientation);
    let mut numerical_accuracy = request.numerical_accuracy.clone();
    let (job_status, outcome) = match eval_res {
        Ok(payload) => {
            numerical_accuracy.proven_position_interpolation_error_m = None;
            let proven_tol =
                if request.query.boundary_time_tolerance_s < request.query.sample_step_s {
                    request.query.boundary_time_tolerance_s
                } else {
                    request.query.sample_step_s
                };
            numerical_accuracy.proven_window_boundary_time_tolerance_s = Some(proven_tol);
            (AnalysisJobStatus::Completed, payload)
        }
        Err(err) => {
            numerical_accuracy.proven_position_interpolation_error_m = None;
            numerical_accuracy.proven_window_boundary_time_tolerance_s = None;
            (
                AnalysisJobStatus::Failed {
                    error_summary: err.to_string(),
                },
                AnalysisOutcomePayload::EvaluationFailed(err),
            )
        }
    };

    let reproducibility = ReproducibilityStamp {
        evaluator_id: "sim-analysis".to_owned(),
        evaluator_version: env!("CARGO_PKG_VERSION").to_owned(),
        ephemeris_catalog_hash: ephemeris.content_hash(),
        orientation_catalog_hash: orientation.content_hash(),
        terrain_dataset_id: request
            .terrain_dem
            .as_ref()
            .map(|dem| dem.dataset_id.clone()),
        terrain_dataset_hash: request
            .terrain_dem
            .as_ref()
            .map(DemElevationGrid::content_hash),
    };

    let mut artifact = AnalysisResultArtifact {
        schema_version: ANALYSIS_CONTRACT_SCHEMA_VERSION,
        result_id: result_id_str,
        request_id: request.request_id.clone(),
        bound_design_revision: request.design_revision.clone(),
        execution_candidate_digest: request.execution_candidate_digest,
        request_digest: req_digest,
        job_status,
        verification_scope: VerificationScope::geometric_visibility_only(),
        time_scale: request.time_scale.clone(),
        coordinate_frames: request.coordinate_frames.clone(),
        numerical_accuracy,
        reproducibility,
        outcome,
        result_digest: req_digest,
    };
    artifact.result_digest = artifact.compute_expected_digest();
    artifact
}

fn validate_request_contracts_and_terrain(
    request: &AnalysisRequestArtifact,
    terrain_cfg: &TerrainLosConfig,
) -> Result<(), AccessAnalysisError> {
    if request.schema_version != ANALYSIS_CONTRACT_SCHEMA_VERSION {
        return Err(ModelNotApplicableReason::UnsupportedSchemaVersion {
            schema_version: request.schema_version,
        }
        .into());
    }
    request.time_scale.validate()?;
    request.coordinate_frames.validate()?;
    request.numerical_accuracy.validate_for_request(
        request.query.sample_step_s,
        request.query.boundary_time_tolerance_s,
    )?;

    if request.terrain_los_config.is_some() && request.terrain_dem.is_none() {
        return Err(DataCoverageError::TerrainDemOutOfCoverage {
            body: NaifId::EARTH,
            lat_rad: 0.0,
            lon_rad: 0.0,
            requested_epoch: request.query.query_interval.start_epoch,
        }
        .into());
    }
    if let Some(acc_step) = request.numerical_accuracy.terrain_chord_sample_step_m
        && request.terrain_los_config.is_some()
        && (acc_step - terrain_cfg.sample_step_m).abs() > 1.0e-9
    {
        return Err(ModelNotApplicableReason::InvalidTerrainParameters {
            reason: "numerical_accuracy.terrain_chord_sample_step_m does not match terrain_los_config.sample_step_m"
                .to_owned(),
        }
        .into());
    }
    if let Some(acc_max_samples) = request.numerical_accuracy.max_terrain_ray_samples_per_chord
        && request.terrain_los_config.is_some()
        && acc_max_samples != terrain_cfg.max_ray_samples
    {
        return Err(ModelNotApplicableReason::InvalidTerrainParameters {
            reason: "numerical_accuracy.max_terrain_ray_samples_per_chord does not match terrain_los_config.max_ray_samples"
                .to_owned(),
        }
        .into());
    }
    Ok(())
}

fn execute_workload(
    request: &AnalysisRequestArtifact,
    ephemeris: &dyn EphemerisCatalog,
    orientation: &dyn BodyOrientationCatalog,
) -> Result<AnalysisOutcomePayload, AccessAnalysisError> {
    let default_terrain_cfg = TerrainLosConfig::default();
    let terrain_cfg = request
        .terrain_los_config
        .as_ref()
        .unwrap_or(&default_terrain_cfg);
    validate_request_contracts_and_terrain(request, terrain_cfg)?;

    let terrain_arg: Option<(&dyn TerrainElevationSource, &TerrainLosConfig)> =
        if let Some(grid) = &request.terrain_dem {
            grid.validate()?;
            Some((grid as &dyn TerrainElevationSource, terrain_cfg))
        } else {
            None
        };

    match &request.workload {
        AnalysisWorkloadSpec::EndpointPairLos {
            source_platform,
            source_sensor,
            target_station,
        } => execute_endpoint_pair_workload(
            request,
            source_platform,
            source_sensor,
            target_station,
            terrain_arg,
            ephemeris,
            orientation,
        ),
        AnalysisWorkloadSpec::AircraftSatelliteGroundRelay {
            aircraft,
            aircraft_to_sat_sensor,
            aircraft_to_ground_sensor,
            satellite,
            sat_to_aircraft_sensor,
            sat_to_ground_sensor,
            ground_station,
        } => execute_relay_workload(
            request,
            aircraft,
            aircraft_to_sat_sensor,
            aircraft_to_ground_sensor.as_ref(),
            satellite,
            sat_to_aircraft_sensor,
            sat_to_ground_sensor,
            ground_station,
            terrain_arg,
            ephemeris,
            orientation,
        ),
    }
}

fn execute_endpoint_pair_workload(
    request: &AnalysisRequestArtifact,
    source_platform: &SerializablePlatformSpec,
    source_sensor: &VehicleSensorModel,
    target_station: &SerializableGroundStationSpec,
    terrain_arg: Option<(&dyn TerrainElevationSource, &TerrainLosConfig)>,
    ephemeris: &dyn EphemerisCatalog,
    orientation: &dyn BodyOrientationCatalog,
) -> Result<AnalysisOutcomePayload, AccessAnalysisError> {
    let src_traj = sample_platform_trajectory(source_platform, orientation)?;
    let src_ep = AnalysisEndpoint::MovingPlatform {
        endpoint_id: source_platform.endpoint_id,
        label: &source_platform.label,
        host_body: source_platform.host_body,
        ellipsoid: BodyEllipsoid::WGS84,
        trajectory: &src_traj,
        attitude: Some(&src_traj),
        sensor: source_sensor,
        min_local_elevation_rad: source_platform.min_local_elevation_rad,
    };
    let dst_ep = AnalysisEndpoint::SurfaceStation {
        station: &target_station.station,
        altitude_spec: target_station.altitude_spec.as_ref(),
    };
    let report = evaluate_endpoint_link_access(
        src_ep,
        dst_ep,
        &request.occultation_env,
        terrain_arg,
        &request.query,
        ephemeris,
        orientation,
    )?;
    Ok(AnalysisOutcomePayload::EndpointPairReport(report))
}

#[allow(clippy::too_many_arguments)]
fn execute_relay_workload(
    request: &AnalysisRequestArtifact,
    aircraft: &SerializablePlatformSpec,
    aircraft_to_sat_sensor: &VehicleSensorModel,
    aircraft_to_ground_sensor: Option<&VehicleSensorModel>,
    satellite: &SerializablePlatformSpec,
    sat_to_aircraft_sensor: &VehicleSensorModel,
    sat_to_ground_sensor: &VehicleSensorModel,
    ground_station: &SerializableGroundStationSpec,
    terrain_arg: Option<(&dyn TerrainElevationSource, &TerrainLosConfig)>,
    ephemeris: &dyn EphemerisCatalog,
    orientation: &dyn BodyOrientationCatalog,
) -> Result<AnalysisOutcomePayload, AccessAnalysisError> {
    let air_traj = sample_platform_trajectory(aircraft, orientation)?;
    let sat_traj = sample_platform_trajectory(satellite, orientation)?;
    let air_sat_ep = AnalysisEndpoint::MovingPlatform {
        endpoint_id: aircraft.endpoint_id,
        label: &aircraft.label,
        host_body: aircraft.host_body,
        ellipsoid: BodyEllipsoid::WGS84,
        trajectory: &air_traj,
        attitude: Some(&air_traj),
        sensor: aircraft_to_sat_sensor,
        min_local_elevation_rad: aircraft.min_local_elevation_rad,
    };
    let sat_air_ep = AnalysisEndpoint::MovingPlatform {
        endpoint_id: satellite.endpoint_id,
        label: &satellite.label,
        host_body: satellite.host_body,
        ellipsoid: BodyEllipsoid::WGS84,
        trajectory: &sat_traj,
        attitude: Some(&sat_traj),
        sensor: sat_to_aircraft_sensor,
        min_local_elevation_rad: satellite.min_local_elevation_rad,
    };
    let sat_gnd_ep = AnalysisEndpoint::MovingPlatform {
        endpoint_id: satellite.endpoint_id,
        label: &satellite.label,
        host_body: satellite.host_body,
        ellipsoid: BodyEllipsoid::WGS84,
        trajectory: &sat_traj,
        attitude: Some(&sat_traj),
        sensor: sat_to_ground_sensor,
        min_local_elevation_rad: satellite.min_local_elevation_rad,
    };
    let gnd_ep = AnalysisEndpoint::SurfaceStation {
        station: &ground_station.station,
        altitude_spec: ground_station.altitude_spec.as_ref(),
    };
    let direct_air_ep =
        aircraft_to_ground_sensor.map(|dir_sensor| AnalysisEndpoint::MovingPlatform {
            endpoint_id: aircraft.endpoint_id,
            label: &aircraft.label,
            host_body: aircraft.host_body,
            ellipsoid: BodyEllipsoid::WGS84,
            trajectory: &air_traj,
            attitude: Some(&air_traj),
            sensor: dir_sensor,
            min_local_elevation_rad: aircraft.min_local_elevation_rad,
        });
    let report = evaluate_geometric_relay_access(
        air_sat_ep,
        sat_air_ep,
        sat_gnd_ep,
        gnd_ep,
        direct_air_ep,
        &request.occultation_env,
        terrain_arg,
        &request.query,
        ephemeris,
        orientation,
    )?;
    Ok(AnalysisOutcomePayload::GeometricRelayReport(report))
}
