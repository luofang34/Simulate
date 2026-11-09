//! Shared versioned analysis request/result contract (`ANALYSIS_CONTRACT_SCHEMA_VERSION = 1`),
//! terrain/sensor models, and 4-way outcome taxonomy between `Simulate` (`sim-analysis`)
//! and `Sokoly` (`sokoly-planning`), built on top of [`sim_spacetime`].

mod artifact;
mod models;
mod relay;
mod taxonomy;
mod terrain;

pub use artifact::{
    ANALYSIS_CONTRACT_SCHEMA_VERSION, AnalysisJobStatus, AnalysisOutcomePayload,
    AnalysisRequestArtifact, AnalysisResultArtifact, AnalysisWorkloadSpec, ArtifactBindingError,
    CoordinateFrameContract, DesignRevisionIdentity, NumericalAccuracyContract,
    ReproducibilityStamp, SerializableGroundStationSpec, SerializablePlatformSpec,
    TimeScaleContract, TimeScaleKind, TrajectoryNodesFrame,
};
pub use glam::{DMat3, DQuat, DVec3};
pub use models::{
    AccessQuerySpec, GroundStationModel, OccultationEnvironmentModel, OccultingBodySpec,
    OccultingShape, SensorFovModel, VehicleSensorModel,
};
pub use relay::{
    EndpointLinkReport, EndpointLinkSample, GeometricRelayReport, RelayAccessSample, RelayHopKind,
    RelayRestrictionEntry, VerificationScope,
};
pub use sim_geo::{BodyEllipsoid, Geodetic};
pub use sim_spacetime::{
    AttitudeSample, ContentHash, CrossDomainAxesSpec, DataProvenance, DomainHasher,
    DynamicalModelIdentity, FrameAxesSpec, FrameCode, FramedState, HashParseError,
    KinematicTrajectoryNode, MountedKinematics, NaifId, SensorMount, SimInstant,
    SpatiotemporalOrigin, StateFrameSpec, TICKS_PER_SECOND, TdbEpoch, TickSpan, TransformDirection,
    UnitSemantics, ValidEpochInterval, VehicleInternalSample, VesselId, acos_clamped, asin_clamped,
    atan2, body_ellipsoid_serde, clamp_vec_norm, cos, exp, geodetic_serde, ln, mul_add_unfused,
    powf, quat_from_wire_xyzw, quat_geodesic_angle_rad, quat_normalize_portable,
    quat_omega_derivative, quat_rotate_vec_portable, serde_dquat, serde_dvec3, sim_instant_serde,
    sin, sin_cos, sqrt,
};
pub use taxonomy::{
    AccessAnalysisError, AccessRestrictionReason, AccessWindow, DataCoverageError,
    MAX_ACCESS_GRID_STEPS, ModelNotApplicableReason, WindowSearchCompleteness,
};
pub use terrain::{
    AltitudeSpec, DEFAULT_MAX_TERRAIN_RAY_SAMPLES, DemElevationGrid, TerrainLosConfig,
    VerticalDatumKind, VerticalDatumSpec,
};
