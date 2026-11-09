//! Validity intervals (`ValidEpochInterval`) and dataset provenance (`DataProvenance`)
//! bridging `sim-spacetime` and `world-core`.

use serde::{Deserialize, Serialize};
#[cfg(feature = "world-mbd-bridges")]
use world_core::{DatasetId, ValidityWindow};

#[cfg(feature = "world-mbd-bridges")]
use crate::time::{TimeError, TimeScaleCatalog};
use crate::{
    hash::{ContentHash, DomainHasher},
    time::TdbEpoch,
};

/// Closed time interval `[start_epoch, end_epoch]` over which a state, trajectory, or ephemeris product is valid.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct ValidEpochInterval {
    /// Inclusive start epoch (`TDB`).
    pub start_epoch: TdbEpoch,
    /// Inclusive end epoch (`TDB`).
    pub end_epoch: TdbEpoch,
}

impl ValidEpochInterval {
    /// Creates a validated `[start_epoch, end_epoch]` interval (`end_epoch >= start_epoch`).
    #[must_use]
    pub fn new(start_epoch: TdbEpoch, end_epoch: TdbEpoch) -> Self {
        if end_epoch >= start_epoch {
            Self {
                start_epoch,
                end_epoch,
            }
        } else {
            Self {
                start_epoch,
                end_epoch: start_epoch,
            }
        }
    }

    /// Returns `true` iff `epoch` lies inside `[self.start_epoch, self.end_epoch]`.
    #[must_use]
    pub fn contains(&self, epoch: TdbEpoch) -> bool {
        epoch >= self.start_epoch && epoch <= self.end_epoch
    }

    /// Total duration of the interval in seconds (`s`).
    #[must_use]
    pub fn duration_s(&self) -> f64 {
        self.end_epoch.signed_offset_seconds_from(self.start_epoch)
    }
}

/// Unified dataset provenance and validity descriptor.
#[derive(Clone, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct DataProvenance {
    /// Dataset name (matches `world_core::DatasetId::name`).
    pub dataset_name: String,
    /// Dataset edition (matches `world_core::DatasetId::edition`).
    pub dataset_edition: String,
    /// Exact `TDB` validity interval of the dataset or trajectory product.
    pub valid_interval: ValidEpochInterval,
    /// Canonical SHA-256 content digest.
    pub content_hash: ContentHash,
}

impl DataProvenance {
    /// Creates a new [`DataProvenance`] descriptor from dataset name, edition, validity interval, and content hash.
    #[must_use]
    pub fn new(
        dataset_name: impl Into<String>,
        dataset_edition: impl Into<String>,
        valid_interval: ValidEpochInterval,
        content_hash: ContentHash,
    ) -> Self {
        Self {
            dataset_name: dataset_name.into(),
            dataset_edition: dataset_edition.into(),
            valid_interval,
            content_hash,
        }
    }

    /// Writes this provenance record into a [`DomainHasher`].
    pub fn write_to_hasher(&self, h: &mut DomainHasher) {
        h.write_bytes(self.dataset_name.as_bytes());
        h.write_bytes(self.dataset_edition.as_bytes());
        h.write_i64(self.valid_interval.start_epoch.j2000_seconds());
        h.write_u64(self.valid_interval.start_epoch.subsec_ticks());
        h.write_i64(self.valid_interval.end_epoch.j2000_seconds());
        h.write_u64(self.valid_interval.end_epoch.subsec_ticks());
        h.write_hash(self.content_hash);
    }
}

/// Extension trait bridging [`ValidEpochInterval`] with `world_core::ValidityWindow`.
#[cfg(feature = "world-mbd-bridges")]
pub trait ValidEpochIntervalWorldExt {
    /// Converts this `TDB` interval into a `world_core::ValidityWindow` (`DecimalYear`) using `time_scales`.
    fn to_world_validity_window(
        &self,
        time_scales: &TimeScaleCatalog,
    ) -> Result<ValidityWindow, TimeError>;
}

#[cfg(feature = "world-mbd-bridges")]
impl ValidEpochIntervalWorldExt for ValidEpochInterval {
    fn to_world_validity_window(
        &self,
        time_scales: &TimeScaleCatalog,
    ) -> Result<ValidityWindow, TimeError> {
        let from = time_scales.tdb_to_decimal_year(self.start_epoch)?;
        let until = time_scales.tdb_to_decimal_year(self.end_epoch)?;
        Ok(ValidityWindow { from, until })
    }
}

/// Extension trait bridging [`DataProvenance`] with `world_core::DatasetId`.
#[cfg(feature = "world-mbd-bridges")]
pub trait DataProvenanceWorldExt {
    /// Creates a new [`DataProvenance`] descriptor from a `world_core::DatasetId`.
    fn from_dataset_id(
        dataset_id: DatasetId,
        valid_interval: ValidEpochInterval,
        content_hash: ContentHash,
    ) -> Self;
}

#[cfg(feature = "world-mbd-bridges")]
impl DataProvenanceWorldExt for DataProvenance {
    fn from_dataset_id(
        dataset_id: DatasetId,
        valid_interval: ValidEpochInterval,
        content_hash: ContentHash,
    ) -> Self {
        Self::new(
            dataset_id.name,
            dataset_id.edition,
            valid_interval,
            content_hash,
        )
    }
}
