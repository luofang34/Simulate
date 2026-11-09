//! Versioned, serializable external analysis request/result contract (`ANALYSIS_CONTRACT_SCHEMA_VERSION = 1`).
//!
//! Re-exports the single-source contract types from [`sim_analysis_contract`] and provides the
//! headless evaluator [`evaluate_analysis_request_artifact`] and platform trajectory materializer
//! [`sample_platform_trajectory`].

mod evaluator;

use sim_spacetime::{
    BodyOrientationCatalog, DynamicalModelIdentity, SampledKinematicTrajectory,
    TrajectoryQueryError,
};

pub use evaluator::evaluate_analysis_request_artifact;
pub use sim_analysis_contract::{
    ANALYSIS_CONTRACT_SCHEMA_VERSION, AnalysisJobStatus, AnalysisOutcomePayload,
    AnalysisRequestArtifact, AnalysisResultArtifact, AnalysisWorkloadSpec, ArtifactBindingError,
    CoordinateFrameContract, DesignRevisionIdentity, NumericalAccuracyContract,
    ReproducibilityStamp, SerializableGroundStationSpec, SerializablePlatformSpec,
    TimeScaleContract, TimeScaleKind, TrajectoryNodesFrame,
};

use crate::access::{AccessAnalysisError, DataCoverageError, ModelNotApplicableReason};

/// Materializes a [`SampledKinematicTrajectory`] in `ICRF` from a [`SerializablePlatformSpec`].
pub fn sample_platform_trajectory(
    platform: &SerializablePlatformSpec,
    orientation: &dyn BodyOrientationCatalog,
) -> Result<SampledKinematicTrajectory, AccessAnalysisError> {
    let map_traj_err = |err: TrajectoryQueryError| match err {
        TrajectoryQueryError::Orientation(_) => {
            AccessAnalysisError::DataCoverage(DataCoverageError::OrientationOutOfCoverage {
                body: platform.host_body,
                requested_epoch: platform
                    .nodes
                    .first()
                    .map_or(sim_spacetime::TdbEpoch::J2000, |n| n.epoch),
            })
        }
        TrajectoryQueryError::EpochOutOfCoverage {
            requested_epoch,
            valid_start,
            valid_end,
        } => AccessAnalysisError::DataCoverage(DataCoverageError::TrajectoryOutOfCoverage {
            requested_epoch,
            valid_interval: sim_spacetime::ValidEpochInterval::new(valid_start, valid_end),
        }),
        TrajectoryQueryError::EmptyTrajectoryProduct => AccessAnalysisError::ModelNotApplicable(
            ModelNotApplicableReason::InvalidQueryParameters {
                sample_step_s: 0.0,
                boundary_time_tolerance_s: 0.0,
            },
        ),
    };

    match platform.nodes_frame {
        TrajectoryNodesFrame::BodyFixedEcef { body } => {
            SampledKinematicTrajectory::from_body_fixed_samples(
                u64::from(platform.endpoint_id),
                body,
                &platform.nodes,
                orientation,
            )
            .map_err(map_traj_err)
        }
        TrajectoryNodesFrame::BodyCenteredIcrf { body } => {
            SampledKinematicTrajectory::from_icrf_nodes(
                u64::from(platform.endpoint_id),
                body,
                DynamicalModelIdentity::PlanetaryKinematicOrFlight { body },
                platform.nodes.clone(),
            )
            .map_err(map_traj_err)
        }
    }
}
