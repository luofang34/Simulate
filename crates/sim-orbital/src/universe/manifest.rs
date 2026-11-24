//! Complete, Eq/Hash-safe replay and save-file identity (`SimulationManifestDigest`).
//!
//! Ensures save/load and deterministic replay verify every dynamical, orientation,
//! time-scale, terrain, force, and numerical policy input rather than a raw SPK hash alone.

use serde::{Deserialize, Serialize};

use crate::{
    hash::{ContentHash, DomainHasher},
    time::TdbEpoch,
};

/// Which subsystem digest differed when comparing two [`SimulationManifestDigest`] headers.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, thiserror::Error)]
pub enum ManifestMismatchField {
    /// Base `TDB` epoch at `SimInstant::START` differed.
    #[error("base_tdb_epoch")]
    BaseTdbEpoch,
    /// Ephemeris segment catalog or precedence differed.
    #[error("ephemeris_hash")]
    Ephemeris,
    /// Gravitational constants or spherical harmonics differed.
    #[error("gravity_constants_hash")]
    GravityConstants,
    /// Planetary orientation / EOP catalog or fallback policy differed.
    #[error("orientation_eop_hash")]
    OrientationEop,
    /// Leap-second table or future-UTC / out-of-EOP policy differed.
    #[error("time_scale_policy_hash")]
    TimeScalePolicy,
    /// Authoritative pinned collision terrain catalog differed.
    #[error("collision_terrain_hash")]
    CollisionTerrain,
    /// Active attracting-body set, dynamical mode, or atmosphere configuration differed.
    #[error("force_model_hash")]
    ForceModel,
    /// Integrator tolerances, step bounds, or warp manifold settings differed.
    #[error("numerical_policy_hash")]
    NumericalPolicy,
}

/// Complete hashable identity of an orbital universe's physical and numerical configuration.
///
/// Contains **zero** floating-point fields so `Eq`, `Ord`, and `Hash` hold unconditionally.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub struct SimulationManifestDigest {
    /// Combined root SHA-256 digest over all fields below.
    pub root_hash: ContentHash,
    /// `TDB` epoch anchored to `SimInstant::START`.
    pub base_tdb_epoch: TdbEpoch,
    /// SHA-256 of the ephemeris segment collection, load order, and `MarsCenterApproximation`.
    pub ephemeris_hash: ContentHash,
    /// SHA-256 of the gravitational constants and spherical-harmonic catalog.
    pub gravity_constants_hash: ContentHash,
    /// SHA-256 of the stacked orientation / BPC / EOP catalog and per-body policies.
    pub orientation_eop_hash: ContentHash,
    /// SHA-256 of the `TimeScaleCatalog` (`LSK`, `EOP`, `FutureUtcPolicy`, `OutOfEopPolicy`).
    pub time_scale_policy_hash: ContentHash,
    /// SHA-256 of the pinned authoritative collision-terrain catalog.
    pub collision_terrain_hash: ContentHash,
    /// SHA-256 of the `ForceModel` configuration.
    pub force_model_hash: ContentHash,
    /// SHA-256 of the numerical propagator and event-localization policy.
    pub numerical_policy_hash: ContentHash,
}

impl SimulationManifestDigest {
    /// Computes a canonical `SimulationManifestDigest` from all component hashes.
    #[must_use]
    #[allow(clippy::too_many_arguments)]
    pub fn compute(
        base_tdb_epoch: TdbEpoch,
        ephemeris_hash: ContentHash,
        gravity_constants_hash: ContentHash,
        orientation_eop_hash: ContentHash,
        time_scale_policy_hash: ContentHash,
        collision_terrain_hash: ContentHash,
        force_model_hash: ContentHash,
        numerical_policy_hash: ContentHash,
    ) -> Self {
        let mut h = DomainHasher::new(b"sim-orbital:manifest-digest:v1");
        h.write_i64(base_tdb_epoch.j2000_seconds());
        h.write_u64(base_tdb_epoch.subsec_ticks());
        h.write_hash(ephemeris_hash);
        h.write_hash(gravity_constants_hash);
        h.write_hash(orientation_eop_hash);
        h.write_hash(time_scale_policy_hash);
        h.write_hash(collision_terrain_hash);
        h.write_hash(force_model_hash);
        h.write_hash(numerical_policy_hash);
        Self {
            root_hash: h.finish(),
            base_tdb_epoch,
            ephemeris_hash,
            gravity_constants_hash,
            orientation_eop_hash,
            time_scale_policy_hash,
            collision_terrain_hash,
            force_model_hash,
            numerical_policy_hash,
        }
    }

    /// Verifies exact equality against `expected`, returning the first mismatched component if any.
    pub fn verify_matches(&self, expected: &Self) -> Result<(), ManifestMismatchField> {
        if self.base_tdb_epoch != expected.base_tdb_epoch {
            return Err(ManifestMismatchField::BaseTdbEpoch);
        }
        if self.ephemeris_hash != expected.ephemeris_hash {
            return Err(ManifestMismatchField::Ephemeris);
        }
        if self.gravity_constants_hash != expected.gravity_constants_hash {
            return Err(ManifestMismatchField::GravityConstants);
        }
        if self.orientation_eop_hash != expected.orientation_eop_hash {
            return Err(ManifestMismatchField::OrientationEop);
        }
        if self.time_scale_policy_hash != expected.time_scale_policy_hash {
            return Err(ManifestMismatchField::TimeScalePolicy);
        }
        if self.collision_terrain_hash != expected.collision_terrain_hash {
            return Err(ManifestMismatchField::CollisionTerrain);
        }
        if self.force_model_hash != expected.force_model_hash {
            return Err(ManifestMismatchField::ForceModel);
        }
        if self.numerical_policy_hash != expected.numerical_policy_hash {
            return Err(ManifestMismatchField::NumericalPolicy);
        }
        Ok(())
    }
}
