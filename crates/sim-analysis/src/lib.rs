//! Propagator-agnostic spatiotemporal geometric relations, constraint evaluation,
//! station access window analysis, local DEM terrain occultation, multi-hop geometric relay,
//! and versioned external request/result contracts.
//!
//! Operates over [`sim_spacetime::TranslationalTrajectorySource`] and
//! [`sim_spacetime::AttitudeTrajectorySource`] so both orbital trajectories
//! (`sim-orbital`) and atmospheric/surface vehicle trajectories (`sim-core`)
//! share an identical analysis contract.

mod access;
mod contract;
mod geometry;
mod models;
mod relay;
mod terrain;

pub use access::{
    AccessAnalysisError, AccessRestrictionReason, AccessWindow, DataCoverageError,
    MAX_ACCESS_GRID_STEPS, ModelNotApplicableReason, StationAccessReport, StationAccessSample,
    WindowSearchCompleteness, evaluate_station_access,
};
pub use contract::{
    ANALYSIS_CONTRACT_SCHEMA_VERSION, AnalysisJobStatus, AnalysisOutcomePayload,
    AnalysisRequestArtifact, AnalysisResultArtifact, AnalysisWorkloadSpec, ArtifactBindingError,
    CoordinateFrameContract, DesignRevisionIdentity, NumericalAccuracyContract,
    ReproducibilityStamp, SerializableGroundStationSpec, SerializablePlatformSpec,
    TimeScaleContract, TimeScaleKind, TrajectoryNodesFrame, evaluate_analysis_request_artifact,
    sample_platform_trajectory,
};
pub use geometry::{
    LinkGeometryEvalError, LinkGeometrySample, MediumIntegratedSample, PropagationMediumQuery,
    check_line_of_sight_occultation, check_signal_path_occultation, compute_link_geometry_sample,
};
pub use models::{
    AccessQuerySpec, GroundStationModel, OccultationEnvironmentModel, OccultingBodySpec,
    OccultingShape, SensorFovModel, SignalTimeModel, VehicleSensorModel,
};
pub use relay::{
    AnalysisEndpoint, EndpointLinkReport, EndpointLinkSample, GeometricRelayReport,
    RelayAccessSample, RelayHopKind, RelayRestrictionEntry, SharedTrajectoryAccessSuite,
    VerificationScope, evaluate_endpoint_link_access, evaluate_geometric_relay_access,
    evaluate_shared_trajectory_access_suite,
};
pub use terrain::{
    AltitudeSpec, DEFAULT_MAX_TERRAIN_RAY_SAMPLES, DemElevationGrid, TerrainElevationSource,
    TerrainLosConfig, VerticalDatumKind, VerticalDatumSpec, check_terrain_line_of_sight_fixed,
    resolve_altitude_ellipsoidal_height_m,
};
