//! Propagator-agnostic state query traits (`TranslationalTrajectorySource`,
//! `AttitudeTrajectorySource`, `VehicleInternalStateSource`) and non-orbital
//! sampled kinematic trajectory (`SampledKinematicTrajectory`).

use glam::{DQuat, DVec3};
use serde::{Deserialize, Serialize};

use crate::{
    catalogs::{BodyOrientationCatalog, OrientationError},
    frames::{
        AttitudeSample, DynamicalModelIdentity, FrameAxesSpec, FramedState, StateFrameSpec,
        UnitSemantics, VehicleInternalSample,
    },
    hash::{ContentHash, DomainHasher},
    ids::NaifId,
    provenance::{DataProvenance, ValidEpochInterval},
    scalar_math::{quat_normalize_portable, serde_dquat, serde_dvec3},
    time::TdbEpoch,
};

/// One time-tagged 6-DoF kinematic sample from a moving platform in `ICRF` or body-fixed `ECEF` coordinates.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct KinematicTrajectoryNode {
    /// Sample epoch (`TDB`).
    pub epoch: TdbEpoch,
    /// Position vector (`m`) in `axes` relative to `origin_body`.
    #[serde(with = "serde_dvec3")]
    pub pos_m: DVec3,
    /// Velocity vector (`m/s`) in `axes` relative to `origin_body`.
    #[serde(with = "serde_dvec3")]
    pub vel_m_s: DVec3,
    /// Unit quaternion rotating vehicle body coordinates (`+X` Fwd, `+Y` Right, `+Z` Down) into `axes`.
    #[serde(with = "serde_dquat")]
    pub q_body_to_axes: DQuat,
    /// Body angular velocity relative to `axes`, expressed in body coordinates (`rad/s`).
    #[serde(with = "serde_dvec3")]
    pub omega_body_rad_s: DVec3,
}

/// Errors returned when constructing or querying a trajectory source.
#[derive(Clone, Debug, PartialEq, thiserror::Error)]
pub enum TrajectoryQueryError {
    /// Requested `epoch` lies outside the trajectory source's [`ValidEpochInterval`].
    #[error(
        "requested epoch {requested_epoch:?} is outside trajectory coverage [{valid_start:?}, {valid_end:?}]"
    )]
    EpochOutOfCoverage {
        /// Queried `TDB` epoch.
        requested_epoch: TdbEpoch,
        /// Start of valid trajectory coverage.
        valid_start: TdbEpoch,
        /// End of valid trajectory coverage.
        valid_end: TdbEpoch,
    },
    /// Attempted to build a trajectory source from an empty or non-monotonic sample sequence.
    #[error("trajectory source contains no valid segments or samples")]
    EmptyTrajectoryProduct,
    /// Underlying body-orientation lookup failed while converting a body-fixed trajectory sample to `ICRF`.
    #[error("orientation error during trajectory query: {0}")]
    Orientation(#[from] OrientationError),
}

/// Propagator-agnostic query interface for 3-DoF translational position and velocity.
pub trait TranslationalTrajectorySource: Send + Sync {
    /// Unique identifier of this trajectory product.
    fn trajectory_id(&self) -> u64;

    /// Dynamical model identity under which this trajectory was generated.
    fn model_identity(&self) -> DynamicalModelIdentity;

    /// Dataset provenance and validity interval of this trajectory.
    fn provenance(&self) -> &DataProvenance;

    /// Closed `TDB` epoch interval `[start_epoch, end_epoch]` over which queries are valid.
    fn valid_interval(&self) -> ValidEpochInterval {
        self.provenance().valid_interval
    }

    /// Evaluates the translational [`FramedState`] (`SiMetric`, `IcrfInertial` relative to `spec.origin`) at `epoch`.
    fn sample_translational_icrf(
        &self,
        epoch: TdbEpoch,
    ) -> Result<FramedState, TrajectoryQueryError>;
}

/// Propagator-agnostic query interface for vehicle attitude and body angular velocity.
pub trait AttitudeTrajectorySource: Send + Sync {
    /// Closed `TDB` epoch interval `[start_epoch, end_epoch]` over which attitude queries are valid.
    fn attitude_valid_interval(&self) -> ValidEpochInterval;

    /// Evaluates the vehicle's [`AttitudeSample`] (`q_body_to_icrf`, `omega_body_rad_s`) at `epoch`.
    fn sample_attitude_icrf(&self, epoch: TdbEpoch)
    -> Result<AttitudeSample, TrajectoryQueryError>;
}

/// Propagator-agnostic query interface for vehicle internal state (propellant mass, dry mass, wheel momentum).
pub trait VehicleInternalStateSource: Send + Sync {
    /// Closed `TDB` epoch interval `[start_epoch, end_epoch]` over which internal state queries are valid.
    fn internal_valid_interval(&self) -> ValidEpochInterval;

    /// Evaluates the vehicle's [`VehicleInternalSample`] at `epoch`.
    fn sample_internal_state(
        &self,
        epoch: TdbEpoch,
    ) -> Result<VehicleInternalSample, TrajectoryQueryError>;
}

/// Propagator-agnostic cubic-Hermite interpolated trajectory constructed from discrete 6-DoF samples
/// in `IcrfInertial` (or pre-converted from planetary `ECEF` via [`Self::from_body_fixed_samples`]).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct SampledKinematicTrajectory {
    /// Unique trajectory identifier.
    pub trajectory_id: u64,
    /// Central coordinate origin body (`NaifId::EARTH`, `MOON`, `MARS`, etc.).
    pub origin_body: NaifId,
    /// Dynamical model identity.
    pub model_identity: DynamicalModelIdentity,
    /// Dataset provenance and validity window.
    pub provenance: DataProvenance,
    /// Strictly chronological `IcrfInertial` 6-DoF nodes.
    pub nodes_icrf: Vec<KinematicTrajectoryNode>,
}

impl SampledKinematicTrajectory {
    /// Builds a [`SampledKinematicTrajectory`] from chronological `IcrfInertial` nodes.
    pub fn from_icrf_nodes(
        trajectory_id: u64,
        origin_body: NaifId,
        model_identity: DynamicalModelIdentity,
        nodes_icrf: Vec<KinematicTrajectoryNode>,
    ) -> Result<Self, TrajectoryQueryError> {
        let (Some(first), Some(last)) = (nodes_icrf.first(), nodes_icrf.last()) else {
            return Err(TrajectoryQueryError::EmptyTrajectoryProduct);
        };
        let mut prev_t = first.epoch;
        for node in nodes_icrf.iter().skip(1) {
            if node.epoch <= prev_t {
                return Err(TrajectoryQueryError::EmptyTrajectoryProduct);
            }
            prev_t = node.epoch;
        }
        let valid_interval = ValidEpochInterval::new(first.epoch, last.epoch);
        let mut h = DomainHasher::new(b"sim-spacetime:sampled-kinematic-trajectory:v1");
        h.write_u64(trajectory_id);
        h.write_i32(origin_body.0);
        model_identity.write_to_hasher(&mut h);
        h.write_u64(nodes_icrf.len() as u64);
        for n in &nodes_icrf {
            h.write_i64(n.epoch.secs_j2000());
            h.write_u64(n.epoch.subsec_ticks());
            h.write_vec3(n.pos_m);
            h.write_vec3(n.vel_m_s);
            h.write_quat(n.q_body_to_axes, false);
            h.write_vec3(n.omega_body_rad_s);
        }
        let provenance = DataProvenance::new(
            "sim-spacetime/sampled-kinematic",
            "v1",
            valid_interval,
            h.finish(),
        );
        Ok(Self {
            trajectory_id,
            origin_body,
            model_identity,
            provenance,
            nodes_icrf,
        })
    }

    /// Builds a [`SampledKinematicTrajectory`] from body-fixed (`ECEF`, e.g. `sim_core::VehicleState`)
    /// nodes by transforming each node into `origin_body`-centered `ICRF` using `orientation`.
    pub fn from_body_fixed_samples(
        trajectory_id: u64,
        origin_body: NaifId,
        nodes_fixed: &[KinematicTrajectoryNode],
        orientation: &dyn BodyOrientationCatalog,
    ) -> Result<Self, TrajectoryQueryError> {
        let mut nodes_icrf = Vec::with_capacity(nodes_fixed.len());
        for node in nodes_fixed {
            let rot = orientation.body_rotation(origin_body, node.epoch)?;
            let (pos_icrf, vel_icrf) = rot.fixed_to_icrf_pv(node.pos_m, node.vel_m_s);
            let q_fixed_to_icrf = DQuat::from_mat3(&rot.r_icrf_to_fixed.transpose());
            let q_body_to_icrf = quat_normalize_portable(q_fixed_to_icrf * node.q_body_to_axes);
            let omega_planet_body = node.q_body_to_axes.conjugate() * rot.omega_body_rad_s;
            nodes_icrf.push(KinematicTrajectoryNode {
                epoch: node.epoch,
                pos_m: pos_icrf,
                vel_m_s: vel_icrf,
                q_body_to_axes: q_body_to_icrf,
                omega_body_rad_s: node.omega_body_rad_s + omega_planet_body,
            });
        }
        Self::from_icrf_nodes(
            trajectory_id,
            origin_body,
            DynamicalModelIdentity::PlanetaryKinematicOrFlight { body: origin_body },
            nodes_icrf,
        )
    }

    /// Canonical SHA-256 content hash of this sampled trajectory.
    #[must_use]
    pub const fn content_hash(&self) -> ContentHash {
        self.provenance.content_hash
    }

    fn interpolate_node(
        &self,
        epoch: TdbEpoch,
    ) -> Result<KinematicTrajectoryNode, TrajectoryQueryError> {
        let valid = self.provenance.valid_interval;
        if !valid.contains(epoch) {
            return Err(TrajectoryQueryError::EpochOutOfCoverage {
                requested_epoch: epoch,
                valid_start: valid.start_epoch,
                valid_end: valid.end_epoch,
            });
        }
        if let Some(first) = self.nodes_icrf.first()
            && epoch == first.epoch
        {
            return Ok(*first);
        }
        if let Some(last) = self.nodes_icrf.last()
            && epoch == last.epoch
        {
            return Ok(*last);
        }
        let idx = self.nodes_icrf.partition_point(|n| n.epoch < epoch);
        let (Some(n0), Some(n1)) = (
            self.nodes_icrf.get(idx.saturating_sub(1)),
            self.nodes_icrf.get(idx),
        ) else {
            return Err(TrajectoryQueryError::EmptyTrajectoryProduct);
        };
        let dt = n1.epoch.signed_offset_seconds_from(n0.epoch);
        if dt <= 1.0e-15 {
            return Ok(*n0);
        }
        let s = (epoch.signed_offset_seconds_from(n0.epoch) / dt).clamp(0.0, 1.0);
        let s2 = s * s;
        let s3 = s2 * s;
        let h00 = 2.0 * s3 - 3.0 * s2 + 1.0;
        let h10 = s3 - 2.0 * s2 + s;
        let h01 = -2.0 * s3 + 3.0 * s2;
        let h11 = s3 - s2;
        let pos_m =
            h00 * n0.pos_m + (h10 * dt) * n0.vel_m_s + h01 * n1.pos_m + (h11 * dt) * n1.vel_m_s;
        let dh00 = (6.0 * s2 - 6.0 * s) / dt;
        let dh10 = 3.0 * s2 - 4.0 * s + 1.0;
        let dh01 = (-6.0 * s2 + 6.0 * s) / dt;
        let dh11 = 3.0 * s2 - 2.0 * s;
        let vel_m_s = dh00 * n0.pos_m + dh10 * n0.vel_m_s + dh01 * n1.pos_m + dh11 * n1.vel_m_s;
        let q1_aligned = if n0.q_body_to_axes.dot(n1.q_body_to_axes) < 0.0 {
            -n1.q_body_to_axes
        } else {
            n1.q_body_to_axes
        };
        let q_interp = quat_normalize_portable(n0.q_body_to_axes * (1.0 - s) + q1_aligned * s);
        let omega = n0.omega_body_rad_s * (1.0 - s) + n1.omega_body_rad_s * s;
        Ok(KinematicTrajectoryNode {
            epoch,
            pos_m,
            vel_m_s,
            q_body_to_axes: q_interp,
            omega_body_rad_s: omega,
        })
    }
}

impl TranslationalTrajectorySource for SampledKinematicTrajectory {
    fn trajectory_id(&self) -> u64 {
        self.trajectory_id
    }

    fn model_identity(&self) -> DynamicalModelIdentity {
        self.model_identity
    }

    fn provenance(&self) -> &DataProvenance {
        &self.provenance
    }

    fn sample_translational_icrf(
        &self,
        epoch: TdbEpoch,
    ) -> Result<FramedState, TrajectoryQueryError> {
        let node = self.interpolate_node(epoch)?;
        Ok(FramedState {
            spec: StateFrameSpec {
                epoch,
                origin: self.origin_body,
                axes: FrameAxesSpec::IcrfInertial,
                units: UnitSemantics::SiMetric,
            },
            pos: node.pos_m,
            vel: node.vel_m_s,
        })
    }
}

impl AttitudeTrajectorySource for SampledKinematicTrajectory {
    fn attitude_valid_interval(&self) -> ValidEpochInterval {
        self.provenance.valid_interval
    }

    fn sample_attitude_icrf(
        &self,
        epoch: TdbEpoch,
    ) -> Result<AttitudeSample, TrajectoryQueryError> {
        let node = self.interpolate_node(epoch)?;
        Ok(AttitudeSample {
            epoch,
            q_body_to_icrf: node.q_body_to_axes,
            omega_body_rad_s: node.omega_body_rad_s,
        })
    }
}
