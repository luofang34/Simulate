//! Orbital trajectory product (`OrbitalTrajectoryProduct`) implementing `sim-spacetime`'s
//! propagator-agnostic trajectory query traits (`TranslationalTrajectorySource`,
//! `AttitudeTrajectorySource`, `VehicleInternalStateSource`), plus `PlanningInputFingerprint`.

use serde::{Deserialize, Serialize};
use sim_spacetime::{
    AttitudeSample, AttitudeTrajectorySource, BodyOrientationCatalog, ContentHash, DataProvenance,
    DataProvenanceWorldExt, DomainHasher, DynamicalModelIdentity, EphemerisCatalog, FrameAxesSpec,
    FramedState, NaifId, StateFrameSpec, TdbEpoch, TrajectoryQueryError,
    TranslationalTrajectorySource, UnitSemantics, ValidEpochInterval, VehicleInternalSample,
    VehicleInternalStateSource,
};
use world_core::DatasetId;

use crate::{
    checkpoint::VesselConfig, dense_segment::PlantTrajectorySegment, gravity::ForceModel,
    plant::ContinuousPlantState,
};

/// Complete cryptographic fingerprint of the dynamical model, force model, ephemeris,
/// body orientation, integrator tolerances, and vessel configuration under which a plan
/// or trajectory product was synthesized.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct PlanningInputFingerprint {
    /// Dynamical model identity (`SolarSystemEphemerisNBody` vs `AutonomousCr3bp`).
    pub model_identity: DynamicalModelIdentity,
    /// Canonical SHA-256 hash of the active [`ForceModel`].
    pub force_model_hash: ContentHash,
    /// Canonical SHA-256 hash of the [`EphemerisCatalog`].
    pub ephemeris_hash: ContentHash,
    /// Canonical SHA-256 hash of the [`BodyOrientationCatalog`].
    pub orientation_hash: ContentHash,
    /// Canonical SHA-256 hash of the [`crate::Dop853Tolerances`].
    pub tolerances_hash: ContentHash,
    /// Canonical SHA-256 hash of the [`VesselConfig`].
    pub vessel_config_hash: ContentHash,
    /// Combined domain-separated SHA-256 digest of all planning inputs.
    pub combined_hash: ContentHash,
}

impl PlanningInputFingerprint {
    /// Computes the canonical [`PlanningInputFingerprint`] for the given model and environment catalogs.
    #[must_use]
    pub fn compute(
        model_identity: DynamicalModelIdentity,
        force_model: &ForceModel,
        ephemeris: &dyn EphemerisCatalog,
        orientation: &dyn BodyOrientationCatalog,
        vessel_config: &VesselConfig,
    ) -> Self {
        let force_model_hash = force_model.content_hash();
        let ephemeris_hash = ephemeris.content_hash();
        let orientation_hash = orientation.content_hash();
        let tolerances_hash = vessel_config.tolerances.content_hash();
        let vessel_config_hash = vessel_config.content_hash();

        let mut h = DomainHasher::new(b"sim-orbital:planning-input-fingerprint:v1");
        model_identity.write_to_hasher(&mut h);
        h.write_hash(force_model_hash);
        h.write_hash(ephemeris_hash);
        h.write_hash(orientation_hash);
        h.write_hash(tolerances_hash);
        h.write_hash(vessel_config_hash);
        Self {
            model_identity,
            force_model_hash,
            ephemeris_hash,
            orientation_hash,
            tolerances_hash,
            vessel_config_hash,
            combined_hash: h.finish(),
        }
    }
}

/// Independent, immutable time-queried orbital trajectory data product backed by degree-7
/// `DOP853` `CONTD8` dense segments.
///
/// Implements [`TranslationalTrajectorySource`], [`AttitudeTrajectorySource`], and
/// [`VehicleInternalStateSource`] from `sim-spacetime` so downstream analysis consumers
/// (`sim-analysis`) can query translational states, attitudes, or internal propellant/wheel
/// states without depending on `PlantTrajectorySegment` or 17-state ODE internals.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct OrbitalTrajectoryProduct {
    /// Unique identifier for this trajectory product.
    pub product_id: u64,
    /// Central coordinate origin body (`NaifId`).
    pub origin_body: NaifId,
    /// Coordinate axes of the stored segments (`IcrfInertial`).
    pub axes: FrameAxesSpec,
    /// Exact epoch interval `[start_epoch, end_epoch]` covered by this trajectory.
    pub valid_interval: ValidEpochInterval,
    /// Dataset provenance and validity interval for `sim-spacetime` consumers.
    pub provenance: DataProvenance,
    /// Polynomial interpolation degree (`7` for `DOP853` `CONTD8` dense output).
    pub interpolation_degree: u8,
    /// Complete model and environment fingerprint under which this trajectory was generated.
    pub input_fingerprint: PlanningInputFingerprint,
    /// Dry structural mass (`kg`) of the vessel.
    pub dry_mass_kg: f64,
    /// Initial 6-DoF plant state at `valid_interval.start_epoch`.
    pub initial_plant: ContinuousPlantState,
    /// Terminal 6-DoF plant state at `valid_interval.end_epoch`.
    pub terminal_plant: ContinuousPlantState,
    /// Contiguous degree-7 dense segments covering `valid_interval`.
    pub segments: Vec<PlantTrajectorySegment>,
}

impl OrbitalTrajectoryProduct {
    /// Constructs an [`OrbitalTrajectoryProduct`] from contiguous [`PlantTrajectorySegment`]s.
    #[must_use]
    pub fn from_segments(
        product_id: u64,
        input_fingerprint: PlanningInputFingerprint,
        dry_mass_kg: f64,
        initial_plant: ContinuousPlantState,
        terminal_plant: ContinuousPlantState,
        default_epoch: TdbEpoch,
        segments: Vec<PlantTrajectorySegment>,
    ) -> Self {
        let start_epoch = segments.first().map_or(default_epoch, |s| s.start_epoch);
        let end_epoch = segments
            .last()
            .map_or(default_epoch, PlantTrajectorySegment::end_epoch);
        let valid_interval = ValidEpochInterval::new(start_epoch, end_epoch);

        let mut h = DomainHasher::new(b"sim-orbital:orbital-trajectory-product:v1");
        h.write_u64(product_id);
        h.write_hash(input_fingerprint.combined_hash);
        h.write_f64_canonical(dry_mass_kg);
        h.write_u64(segments.len() as u64);
        for seg in &segments {
            h.write_hash(seg.content_hash());
        }
        let provenance = DataProvenance::from_dataset_id(
            DatasetId {
                name: "sim-orbital/contd8-trajectory",
                edition: "v1",
            },
            valid_interval,
            h.finish(),
        );

        Self {
            product_id,
            origin_body: initial_plant.primary_body,
            axes: FrameAxesSpec::IcrfInertial,
            valid_interval,
            provenance,
            interpolation_degree: 7,
            input_fingerprint,
            dry_mass_kg,
            initial_plant,
            terminal_plant,
            segments,
        }
    }

    /// Evaluates the full 6-DoF [`ContinuousPlantState`] at `epoch`, failing closed with
    /// [`TrajectoryQueryError::EpochOutOfCoverage`] outside `self.valid_interval`.
    pub fn sample_full_plant(
        &self,
        epoch: TdbEpoch,
    ) -> Result<ContinuousPlantState, TrajectoryQueryError> {
        if !self.valid_interval.contains(epoch) {
            return Err(TrajectoryQueryError::EpochOutOfCoverage {
                requested_epoch: epoch,
                valid_start: self.valid_interval.start_epoch,
                valid_end: self.valid_interval.end_epoch,
            });
        }
        if epoch == self.valid_interval.start_epoch && self.segments.is_empty() {
            return Ok(self.initial_plant);
        }
        if epoch == self.valid_interval.end_epoch {
            return Ok(self.terminal_plant);
        }
        let idx = self.segments.partition_point(|s| s.end_epoch() < epoch);
        if let Some(seg) = self.segments.get(idx)
            && epoch >= seg.start_epoch
            && epoch <= seg.end_epoch()
        {
            if epoch == seg.end_epoch() {
                return Ok(seg.end_state);
            }
            return Ok(seg.sample_plant_and_accel(epoch).0);
        }
        Err(TrajectoryQueryError::EpochOutOfCoverage {
            requested_epoch: epoch,
            valid_start: self.valid_interval.start_epoch,
            valid_end: self.valid_interval.end_epoch,
        })
    }
}

impl TranslationalTrajectorySource for OrbitalTrajectoryProduct {
    fn trajectory_id(&self) -> u64 {
        self.product_id
    }

    fn model_identity(&self) -> DynamicalModelIdentity {
        self.input_fingerprint.model_identity
    }

    fn provenance(&self) -> &DataProvenance {
        &self.provenance
    }

    fn sample_translational_icrf(
        &self,
        epoch: TdbEpoch,
    ) -> Result<FramedState, TrajectoryQueryError> {
        let plant = self.sample_full_plant(epoch)?;
        Ok(FramedState {
            spec: StateFrameSpec {
                epoch,
                origin: plant.primary_body,
                axes: FrameAxesSpec::IcrfInertial,
                units: UnitSemantics::SiMetric,
            },
            pos: plant.pos_rel_icrf_m,
            vel: plant.vel_rel_icrf_m_s,
        })
    }
}

impl AttitudeTrajectorySource for OrbitalTrajectoryProduct {
    fn attitude_valid_interval(&self) -> ValidEpochInterval {
        self.valid_interval
    }

    fn sample_attitude_icrf(
        &self,
        epoch: TdbEpoch,
    ) -> Result<AttitudeSample, TrajectoryQueryError> {
        let plant = self.sample_full_plant(epoch)?;
        Ok(AttitudeSample {
            epoch,
            q_body_to_icrf: plant.q_body_to_icrf,
            omega_body_rad_s: plant.omega_body_rad_s,
        })
    }
}

impl VehicleInternalStateSource for OrbitalTrajectoryProduct {
    fn internal_valid_interval(&self) -> ValidEpochInterval {
        self.valid_interval
    }

    fn sample_internal_state(
        &self,
        epoch: TdbEpoch,
    ) -> Result<VehicleInternalSample, TrajectoryQueryError> {
        let plant = self.sample_full_plant(epoch)?;
        Ok(VehicleInternalSample {
            epoch,
            propellant_mass_kg: plant.propellant_mass_kg,
            dry_mass_kg: self.dry_mass_kg,
            wheel_momentum_body_nms: plant.h_rw_body_nms,
        })
    }
}
