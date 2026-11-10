//! Spacetime foundation for `Simulate`: master simulation clock (`SimInstant`, `TickSpan`),
//! astronomical time scales (`TdbEpoch`, `TimeScaleCatalog`, `SimClockAdapter`, `SimEpoch`),
//! relativistic proper time of moving clocks (`ClockPotential`, `ProperTimeClock`),
//! coordinate frame identities and transforms (`FramedState`, `SynodicFrame`, `SensorMount`),
//! propagator-agnostic trajectory query contracts (`TranslationalTrajectorySource`,
//! `AttitudeTrajectorySource`, `VehicleInternalStateSource`), dataset provenance
//! (`DataProvenance`, `ValidEpochInterval`), and planetary collision terrain traits (`CollisionTerrain`).

mod catalogs;
mod clock;
mod frames;
mod hash;
mod ids;
mod kpl;
mod mount;
mod provenance;
mod relativity;
mod scalar_math;
mod solar;
mod synodic;
mod terrain;
mod time;
mod trajectory;

pub use catalogs::{
    BodyOrientationCatalog, BodyRotationState, CollinearLagrangePoint, DerivativeOrder,
    EphemerisCatalog, EphemerisError, KinematicStateIcrf, MarsCenterApproximation,
    OrientationError, OrientationPolicy, SurfaceNavGrade, SynodicError, SynodicFrame,
    coverage_for_grade, default_iau_frame_code, solve_collinear_lagrange_gamma,
};
pub use clock::{SimInstant, TICKS_PER_SECOND, TickSpan, sim_instant_serde};
pub use frames::{
    AttitudeSample, CrossDomainAxesSpec, DynamicalModelIdentity, FrameAxesSpec, FramedState,
    FramedStateTransformExt, SpacetimeTransformError, SpatiotemporalOrigin, StateFrameSpec,
    TransformDirection, UnitSemantics, VehicleInternalSample, geodetic_serde,
    surface_station_state_icrf,
};
pub use hash::{ContentHash, DomainHasher, HashParseError};
pub use ids::{FrameCode, NaifId, VesselId};
pub use kpl::{KplAtom, KplError, KplPool};
#[cfg(feature = "world-mbd-bridges")]
pub use mount::SensorMountPlacementExt;
pub use mount::{MountedKinematics, SensorMount};
pub use provenance::{DataProvenance, ValidEpochInterval};
#[cfg(feature = "world-mbd-bridges")]
pub use provenance::{DataProvenanceWorldExt, ValidEpochIntervalWorldExt};
pub use relativity::{
    BarycentricClockModel, ClockPotential, L_B, L_G, MassiveBody, ProperTimeClock,
    SPEED_OF_LIGHT_M_S, orbit_periodic_clock_term_s, tdb_minus_tt_position_term_s,
};
pub use scalar_math::{
    acos_clamped, asin_clamped, atan2, body_ellipsoid_serde, clamp_vec_norm, cos, exp, ln,
    mul_add_unfused, powf, quat_from_wire_xyzw, quat_geodesic_angle_rad, quat_normalize_portable,
    quat_omega_derivative, quat_rotate_vec_portable, serde_dquat, serde_dvec3, sin, sin_cos, sqrt,
};
pub use solar::{
    ASTRONOMICAL_UNIT_M, AtmosphericTransmission, OccultingSphere, SOLAR_IRRADIANCE_1AU_W_M2,
    SOLAR_RADIUS_M, SolarFluxModel, SolarFluxSample,
};
pub use terrain::{
    CollisionGroundSample, CollisionTerrain, DiscontinuousStepPatch, ElevationSlab,
    PatchedEllipsoidTerrain, wrap_lon_diff,
};
pub use time::{
    BUILTIN_LSK_BYTES, BUILTIN_LSK_SHA256, CoverageSet, EarthEopEntry, EarthEopTable,
    FutureUtcPolicy, LeapSecondEntry, LeapSecondTable, OutOfEopPolicy, SimClockAdapter, SimEpoch,
    TdbEpoch, TdbSpanNs, TimeError, TimeScaleCatalog, Ut1Sample,
};
pub use trajectory::{
    AttitudeTrajectorySource, KinematicTrajectoryNode, SampledKinematicTrajectory,
    TrajectoryQueryError, TranslationalTrajectorySource, VehicleInternalStateSource,
};
