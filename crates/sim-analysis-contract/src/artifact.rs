//! Versioned, serializable external analysis request/result contract (`ANALYSIS_CONTRACT_SCHEMA_VERSION = 1`).
//!
//! Maintained on the `Simulate` side (`sim-analysis-contract`) and shared directly by `sim-analysis`
//! and `sokoly-planning`. Uses serializable identities, trajectory node tables, DEM grids, vertical
//! datums, explicit unit/frame/accuracy contracts, and cryptographic `ReproducibilityStamp`s.

mod result;

use serde::{Deserialize, Serialize};

pub use result::{AnalysisOutcomePayload, AnalysisResultArtifact, ArtifactBindingError};

use crate::{
    ContentHash, DomainHasher, KinematicTrajectoryNode, NaifId, TICKS_PER_SECOND,
    models::{
        AccessQuerySpec, GroundStationModel, OccultationEnvironmentModel, VehicleSensorModel,
    },
    taxonomy::ModelNotApplicableReason,
    terrain::{AltitudeSpec, DemElevationGrid, TerrainLosConfig},
};

/// Canonical schema version for [`AnalysisRequestArtifact`] and [`AnalysisResultArtifact`].
pub const ANALYSIS_CONTRACT_SCHEMA_VERSION: u32 = 1;

/// Identifies the immutable planning design revision that produced an [`AnalysisRequestArtifact`].
#[derive(Clone, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct DesignRevisionIdentity {
    /// Logical mission design identifier in the planning workspace.
    pub design_id: String,
    /// Monotonically increasing revision number (`1, 2, ...`).
    pub revision_number: u64,
    /// Hex-encoded SHA-256 digest of the canonical mission design state at `revision_number`.
    pub design_digest_sha256: String,
}

impl DesignRevisionIdentity {
    /// Writes this revision identity into a [`DomainHasher`].
    pub fn write_to_hasher(&self, h: &mut DomainHasher) {
        h.write_bytes(self.design_id.as_bytes());
        h.write_u64(self.revision_number);
        h.write_bytes(self.design_digest_sha256.as_bytes());
    }
}

/// Lifecycle state of an analysis evaluation job.
#[derive(Clone, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum AnalysisJobStatus {
    /// Queued and awaiting headless evaluation.
    Pending,
    /// Currently evaluating in `Simulate`.
    Running,
    /// Explicitly cancelled before completion; late results for this job must be rejected.
    Cancelled,
    /// Evaluation completed and produced a valid report.
    Completed,
    /// Evaluation failed closed due to missing data coverage or inapplicable model parameters.
    Failed {
        /// Human-readable summary of the structured [`AccessAnalysisError`](crate::AccessAnalysisError).
        error_summary: String,
    },
    /// Legacy/external status indicating supersession by `newer_revision_number`.
    SupersededByNewerRevision {
        /// Revision number that superseded this job's target revision.
        newer_revision_number: u64,
    },
}

impl AnalysisJobStatus {
    /// Returns `true` if this status is a terminal job state (`Cancelled`, `Completed`, or `Failed`).
    #[must_use]
    pub const fn is_terminal(&self) -> bool {
        matches!(
            self,
            Self::Cancelled | Self::Completed | Self::Failed { .. }
        )
    }

    /// Returns `true` if transitioning from `self` to `next` is a valid lifecycle state transition.
    ///
    /// Terminal states (`Cancelled`, `Completed`, `Failed`) cannot transition to any other state.
    #[must_use]
    pub fn can_transition_to(&self, next: &Self) -> bool {
        if self == next {
            return true;
        }
        match self {
            Self::Pending | Self::Running | Self::SupersededByNewerRevision { .. } => {
                matches!(
                    next,
                    Self::Running
                        | Self::Cancelled
                        | Self::Completed
                        | Self::Failed { .. }
                        | Self::SupersededByNewerRevision { .. }
                )
            }
            Self::Cancelled | Self::Completed | Self::Failed { .. } => false,
        }
    }

    /// Writes this job status into a [`DomainHasher`].
    pub fn write_to_hasher(&self, h: &mut DomainHasher) {
        match self {
            Self::Pending => h.write_u8(1),
            Self::Running => h.write_u8(2),
            Self::Cancelled => h.write_u8(3),
            Self::Completed => h.write_u8(4),
            Self::Failed { error_summary } => {
                h.write_u8(5);
                h.write_bytes(error_summary.as_bytes());
            }
            Self::SupersededByNewerRevision {
                newer_revision_number,
            } => {
                h.write_u8(6);
                h.write_u64(*newer_revision_number);
            }
        }
    }
}

/// Time scale identifier for external analysis artifacts.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum TimeScaleKind {
    /// Barycentric Dynamical Time (`TDB`) seconds and subsecond ticks since `J2000.0`.
    TdbJ2000,
}

/// Explicit time-scale contract governing all epochs in an analysis request and result.
#[derive(Clone, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct TimeScaleContract {
    /// Astronomical time scale (`TdbJ2000`).
    pub time_scale: TimeScaleKind,
    /// Subsecond ticks per second (`TICKS_PER_SECOND`).
    pub subsecond_ticks_per_second: u64,
    /// Optional leap-second table identifier used when converting operator UTC timestamps into `TDB`.
    pub utc_leap_second_table_id: Option<String>,
}

impl Default for TimeScaleContract {
    fn default() -> Self {
        Self {
            time_scale: TimeScaleKind::TdbJ2000,
            subsecond_ticks_per_second: TICKS_PER_SECOND,
            utc_leap_second_table_id: Some("IERS-Bulletin-C-2026".to_owned()),
        }
    }
}

impl TimeScaleContract {
    /// Validates that this time-scale contract matches `Simulate`'s `TdbJ2000` master clock.
    pub fn validate(&self) -> Result<(), ModelNotApplicableReason> {
        if self.time_scale != TimeScaleKind::TdbJ2000
            || self.subsecond_ticks_per_second != TICKS_PER_SECOND
            || self
                .utc_leap_second_table_id
                .as_ref()
                .is_some_and(String::is_empty)
        {
            return Err(ModelNotApplicableReason::UnsupportedTimeScaleContract {
                reason: format!(
                    "expected TdbJ2000 with subsecond_ticks_per_second={TICKS_PER_SECOND}, got {:?}",
                    self
                ),
            });
        }
        Ok(())
    }

    /// Writes this time-scale contract into a [`DomainHasher`].
    pub fn write_to_hasher(&self, h: &mut DomainHasher) {
        let Self {
            time_scale: TimeScaleKind::TdbJ2000,
            subsecond_ticks_per_second,
            utc_leap_second_table_id,
        } = self;
        h.write_u8(1);
        h.write_u64(*subsecond_ticks_per_second);
        match utc_leap_second_table_id {
            None => h.write_u8(0),
            Some(id) => {
                h.write_u8(1);
                h.write_bytes(id.as_bytes());
            }
        }
    }
}

/// Explicit coordinate-frame and physical-unit contract for external analysis artifacts.
#[derive(Clone, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct CoordinateFrameContract {
    /// Inertial evaluation frame (`"SSB_ICRF_J2000"`).
    pub inertial_frame: String,
    /// Planetary body-fixed reference frame (`"EARTH_IAU_FIXED_WGS84"`).
    pub body_fixed_frame: String,
    /// SI length unit (`"m"`).
    pub length_unit: String,
    /// SI plane angle unit (`"rad"`).
    pub angle_unit: String,
}

impl Default for CoordinateFrameContract {
    fn default() -> Self {
        Self {
            inertial_frame: "SSB_ICRF_J2000".to_owned(),
            body_fixed_frame: "EARTH_IAU_FIXED_WGS84".to_owned(),
            length_unit: "m".to_owned(),
            angle_unit: "rad".to_owned(),
        }
    }
}

impl CoordinateFrameContract {
    /// Validates that frames and physical units match the evaluator's SI/ICRF/WGS84 contract.
    pub fn validate(&self) -> Result<(), ModelNotApplicableReason> {
        if self.inertial_frame != "SSB_ICRF_J2000"
            || self.body_fixed_frame != "EARTH_IAU_FIXED_WGS84"
            || self.length_unit != "m"
            || self.angle_unit != "rad"
        {
            return Err(
                ModelNotApplicableReason::UnsupportedCoordinateFrameOrUnitContract {
                    inertial_frame: self.inertial_frame.clone(),
                    body_fixed_frame: self.body_fixed_frame.clone(),
                    length_unit: self.length_unit.clone(),
                    angle_unit: self.angle_unit.clone(),
                },
            );
        }
        Ok(())
    }

    /// Writes this coordinate-frame contract into a [`DomainHasher`].
    pub fn write_to_hasher(&self, h: &mut DomainHasher) {
        h.write_bytes(self.inertial_frame.as_bytes());
        h.write_bytes(self.body_fixed_frame.as_bytes());
        h.write_bytes(self.length_unit.as_bytes());
        h.write_bytes(self.angle_unit.as_bytes());
    }
}

/// Explicit numerical interpolation settings, caller-requested accuracy bounds, and evaluator-proven bounds.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct NumericalAccuracyContract {
    /// Trajectory interpolation method (`"CubicHermiteIcrf"`).
    pub trajectory_interpolation_method: String,
    /// Optional caller-requested upper bound on position interpolation error between sample nodes (`m`).
    #[serde(default)]
    pub requested_position_interpolation_error_m: Option<f64>,
    /// Optional caller-requested upper bound on window boundary time tolerance (`s`).
    #[serde(default)]
    pub requested_boundary_time_tolerance_s: Option<f64>,
    /// Configured bisection tolerance on window opening/closing boundaries (`s`).
    pub window_boundary_time_tolerance_s: f64,
    /// Configured maximum chord sample spacing used for DEM terrain occultation (`m`, when DEM is active).
    pub terrain_chord_sample_step_m: Option<f64>,
    /// Configured maximum DEM ray samples per chord inside the terrain interaction shell.
    #[serde(default)]
    pub max_terrain_ray_samples_per_chord: Option<u32>,
    /// Certified position interpolation error bound (`m`) proven by the evaluator (`None` for discrete node tables).
    #[serde(default)]
    pub proven_position_interpolation_error_m: Option<f64>,
    /// Certified bracketed window boundary time error bound (`s`) proven by the evaluator.
    #[serde(default)]
    pub proven_window_boundary_time_tolerance_s: Option<f64>,
}

impl NumericalAccuracyContract {
    /// Validates request-side numerical settings and requested bounds against `query_boundary_tolerance_s`.
    pub fn validate_for_request(
        &self,
        query_sample_step_s: f64,
        query_boundary_tolerance_s: f64,
    ) -> Result<(), ModelNotApplicableReason> {
        if self.trajectory_interpolation_method != "CubicHermiteIcrf"
            || !self.window_boundary_time_tolerance_s.is_finite()
            || self.window_boundary_time_tolerance_s <= 0.0
            || (self.window_boundary_time_tolerance_s - query_boundary_tolerance_s).abs() > 1.0e-12
        {
            return Err(ModelNotApplicableReason::InvalidQueryParameters {
                sample_step_s: query_sample_step_s,
                boundary_time_tolerance_s: self.window_boundary_time_tolerance_s,
            });
        }
        if let Some(req_pos_err) = self.requested_position_interpolation_error_m {
            return Err(
                ModelNotApplicableReason::UnprovableInterpolationAccuracyBound {
                    requested_error_m: req_pos_err,
                },
            );
        }
        if let Some(req_tol_s) = self.requested_boundary_time_tolerance_s
            && (!req_tol_s.is_finite()
                || req_tol_s <= 0.0
                || query_boundary_tolerance_s > req_tol_s)
        {
            return Err(ModelNotApplicableReason::InvalidQueryParameters {
                sample_step_s: query_sample_step_s,
                boundary_time_tolerance_s: query_boundary_tolerance_s,
            });
        }
        Ok(())
    }

    /// Checks that a result's numerical accuracy contract preserves the request's algorithm settings and requested bounds.
    #[must_use]
    pub fn matches_request(&self, req: &Self) -> bool {
        self.trajectory_interpolation_method == req.trajectory_interpolation_method
            && self.requested_position_interpolation_error_m
                == req.requested_position_interpolation_error_m
            && self.requested_boundary_time_tolerance_s == req.requested_boundary_time_tolerance_s
            && (self.window_boundary_time_tolerance_s - req.window_boundary_time_tolerance_s).abs()
                <= 1.0e-12
            && self.terrain_chord_sample_step_m == req.terrain_chord_sample_step_m
            && self.max_terrain_ray_samples_per_chord == req.max_terrain_ray_samples_per_chord
    }

    /// Writes all fields of this numerical accuracy contract into a [`DomainHasher`].
    pub fn write_to_hasher(&self, h: &mut DomainHasher) {
        h.write_bytes(self.trajectory_interpolation_method.as_bytes());
        h.write_opt_f64_canonical(self.requested_position_interpolation_error_m);
        h.write_opt_f64_canonical(self.requested_boundary_time_tolerance_s);
        h.write_f64_canonical(self.window_boundary_time_tolerance_s);
        h.write_opt_f64_canonical(self.terrain_chord_sample_step_m);
        h.write_opt_u32(self.max_terrain_ray_samples_per_chord);
        h.write_opt_f64_canonical(self.proven_position_interpolation_error_m);
        h.write_opt_f64_canonical(self.proven_window_boundary_time_tolerance_s);
    }
}

/// Cryptographic provenance stamp recording the exact evaluator version and catalog hashes used to compute an [`AnalysisResultArtifact`].
#[derive(Clone, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct ReproducibilityStamp {
    /// Evaluator crate identifier (`"sim-analysis"`).
    pub evaluator_id: String,
    /// Evaluator crate version (`env!("CARGO_PKG_VERSION")`).
    pub evaluator_version: String,
    /// Canonical SHA-256 content hash of the planetary ephemeris catalog used during evaluation.
    pub ephemeris_catalog_hash: ContentHash,
    /// Canonical SHA-256 content hash of the planetary orientation catalog used during evaluation.
    pub orientation_catalog_hash: ContentHash,
    /// Dataset identifier of the DEM terrain grid used during evaluation (if any).
    pub terrain_dataset_id: Option<String>,
    /// Canonical SHA-256 content hash of the DEM terrain grid used during evaluation (if any).
    pub terrain_dataset_hash: Option<ContentHash>,
}

impl ReproducibilityStamp {
    /// Writes this reproducibility stamp into a [`DomainHasher`].
    pub fn write_to_hasher(&self, h: &mut DomainHasher) {
        h.write_bytes(self.evaluator_id.as_bytes());
        h.write_bytes(self.evaluator_version.as_bytes());
        h.write_hash(self.ephemeris_catalog_hash);
        h.write_hash(self.orientation_catalog_hash);
        match &self.terrain_dataset_id {
            None => h.write_u8(0),
            Some(id) => {
                h.write_u8(1);
                h.write_bytes(id.as_bytes());
            }
        }
        match self.terrain_dataset_hash {
            None => h.write_u8(0),
            Some(digest) => {
                h.write_u8(1);
                h.write_hash(digest);
            }
        }
    }
}

/// Reference frame in which a [`SerializablePlatformSpec`]'s discrete nodes are expressed.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum TrajectoryNodesFrame {
    /// Nodes are expressed in `body`-fixed rotating `ECEF` (`m`, `m/s`) and transformed to `ICRF`.
    BodyFixedEcef {
        /// Central rotating body (`NaifId::EARTH`, etc.).
        body: NaifId,
    },
    /// Nodes are expressed in `body`-centered `ICRF` (`m`, `m/s`).
    BodyCenteredIcrf {
        /// Central body (`NaifId::EARTH`, `MOON`, `SSB`, etc.).
        body: NaifId,
    },
}

/// Serializable moving platform specification (aircraft or satellite) for external analysis requests.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct SerializablePlatformSpec {
    /// Unique numeric endpoint ID.
    pub endpoint_id: u32,
    /// Human-readable platform label.
    pub label: String,
    /// Host body for local horizon (`ENU`) elevation calculations.
    pub host_body: NaifId,
    /// Coordinate frame of `nodes`.
    pub nodes_frame: TrajectoryNodesFrame,
    /// Chronological 6-DoF kinematic sample nodes.
    pub nodes: Vec<KinematicTrajectoryNode>,
    /// Optional minimum local topocentric elevation mask (`rad`) at this platform.
    pub min_local_elevation_rad: Option<f64>,
}

/// Serializable ground station specification pairing [`GroundStationModel`] with an optional [`AltitudeSpec`].
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct SerializableGroundStationSpec {
    /// Ground station model.
    pub station: GroundStationModel,
    /// Optional explicit vertical datum altitude specification (`WGS84`, `AGL`, or `MSL`).
    pub altitude_spec: Option<AltitudeSpec>,
}

/// Analysis workload payload inside an [`AnalysisRequestArtifact`].
#[allow(clippy::large_enum_variant)]
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum AnalysisWorkloadSpec {
    /// Two-endpoint geometric line-of-sight and sensor `FOV` analysis between a moving platform and a ground station.
    EndpointPairLos {
        /// Moving platform trajectory specification.
        source_platform: SerializablePlatformSpec,
        /// Sensor mounted on `source_platform`.
        source_sensor: VehicleSensorModel,
        /// Target surface station specification.
        target_station: SerializableGroundStationSpec,
    },
    /// Aircraft – Satellite – Ground Station simultaneous geometric relay and direct-link analysis.
    AircraftSatelliteGroundRelay {
        /// Aircraft trajectory specification.
        aircraft: SerializablePlatformSpec,
        /// Aircraft dorsal/satcom antenna used on Hop 1 (`Aircraft <-> Satellite`).
        aircraft_to_sat_sensor: VehicleSensorModel,
        /// Optional aircraft belly/direct antenna used on the direct link (`Aircraft <-> Ground Station`).
        aircraft_to_ground_sensor: Option<VehicleSensorModel>,
        /// Relay satellite trajectory specification.
        satellite: SerializablePlatformSpec,
        /// Satellite crosslink/spot-beam antenna facing the aircraft on Hop 1.
        sat_to_aircraft_sensor: VehicleSensorModel,
        /// Satellite feeder-link antenna facing the ground station on Hop 2.
        sat_to_ground_sensor: VehicleSensorModel,
        /// Ground station specification.
        ground_station: SerializableGroundStationSpec,
    },
}

/// Versioned external analysis request artifact emitted by `Sokoly` and evaluated by `Simulate`.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct AnalysisRequestArtifact {
    /// Contract schema version (`ANALYSIS_CONTRACT_SCHEMA_VERSION = 1`).
    pub schema_version: u32,
    /// Unique request identifier (`request_id`), distinct from `design_revision`.
    pub request_id: String,
    /// Immutable design revision identity to which this request is bound.
    pub design_revision: DesignRevisionIdentity,
    /// Optional digest of a compiled execution candidate (e.g. a Pilotage survey trajectory candidate)
    /// when this request verifies an execution candidate prior to proposal promotion.
    #[serde(default)]
    pub execution_candidate_digest: Option<ContentHash>,
    /// Time-scale contract.
    pub time_scale: TimeScaleContract,
    /// Coordinate-frame and unit contract.
    pub coordinate_frames: CoordinateFrameContract,
    /// Numerical accuracy and tolerance contract.
    pub numerical_accuracy: NumericalAccuracyContract,
    /// Query interval, coarse step, boundary bisection tolerance, and range bounds.
    pub query: AccessQuerySpec,
    /// Planetary body occultation environment.
    pub occultation_env: OccultationEnvironmentModel,
    /// Optional local DEM terrain elevation grid (`None` cells fail closed).
    pub terrain_dem: Option<DemElevationGrid>,
    /// Optional DEM terrain line-of-sight sampling configuration.
    pub terrain_los_config: Option<TerrainLosConfig>,
    /// Requested analysis workload.
    pub workload: AnalysisWorkloadSpec,
}

impl AnalysisRequestArtifact {
    /// Computes a deterministic SHA-256 [`ContentHash`] over the entire request artifact.
    #[must_use]
    pub fn content_hash(&self) -> ContentHash {
        let mut h = DomainHasher::new(b"sim-analysis:request-artifact:v2");
        h.write_u32(self.schema_version);
        h.write_bytes(self.request_id.as_bytes());
        self.design_revision.write_to_hasher(&mut h);
        match self.execution_candidate_digest {
            None => h.write_u8(0),
            Some(d) => {
                h.write_u8(1);
                h.write_hash(d);
            }
        }
        self.time_scale.write_to_hasher(&mut h);
        self.coordinate_frames.write_to_hasher(&mut h);
        self.numerical_accuracy.write_to_hasher(&mut h);
        if let Ok(json_bytes) = serde_json::to_vec(self) {
            h.write_bytes(&json_bytes);
        }
        h.finish()
    }
}
