//! Catalog-tier `SGP4`/`SDP4` TLE & OMM propagator behind [`TranslationalTrajectorySource`].
//!
//! Explicitly tagged with [`DynamicalModelIdentity::Sgp4Catalog`] so catalog predictions
//! feed downstream visibility and RF link analysis while staying strictly outside
//! authoritative N-body truth digests (`SimulationManifestDigest`, `PlanningInputFingerprint`).

mod nutation;
mod teme;

use std::sync::Arc;

use glam::DVec3;
use serde::{Deserialize, Serialize};
use sim_spacetime::{
    ContentHash, DataProvenance, DomainHasher, DynamicalModelIdentity, FrameAxesSpec, FramedState,
    NaifId, StateFrameSpec, TdbEpoch, TimeError, TimeScaleCatalog, TrajectoryQueryError,
    TranslationalTrajectorySource, UnitSemantics, ValidEpochInterval, serde_dvec3,
};

pub use teme::teme_to_icrf_rotation;

/// SGP4 orbital-theory compatibility convention.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum Sgp4Mode {
    /// Modern `WGS-84` geopotential and `IAU` GMST model (`sgp4::Constants::from_elements`).
    #[default]
    ModernWgs84Iau,
    /// Legacy AFSPC `WGS-72` and Vallado AIAA 2006-6753 compatibility mode.
    AfspcCompatibilityWgs72,
}

impl Sgp4Mode {
    const fn tag_byte(self) -> u8 {
        match self {
            Self::ModernWgs84Iau => 0,
            Self::AfspcCompatibilityWgs72 => 1,
        }
    }
}

/// Propagated state vector in the `TEME` frame using standard TLE units (`km`, `km/s`).
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct Sgp4TemeState {
    /// Geocentric position vector in `TEME` (`km`).
    #[serde(with = "serde_dvec3")]
    pub position_km: DVec3,
    /// Geocentric velocity vector in `TEME` (`km/s`).
    #[serde(with = "serde_dvec3")]
    pub velocity_km_s: DVec3,
}

/// Errors returned while constructing or evaluating an [`Sgp4CatalogTrajectory`].
#[derive(Clone, Debug, PartialEq, thiserror::Error)]
pub enum Sgp4Error {
    /// Failed to parse two-line element lines.
    #[error("invalid TLE input: {0}")]
    InvalidTle(String),
    /// Failed to parse CCSDS/Celestrak JSON OMM record.
    #[error("invalid OMM JSON input: {0}")]
    InvalidOmm(String),
    /// Orbital elements failed Kozai-Brouwer conversion or eccentricity validation.
    #[error("invalid SGP4 elements: {0}")]
    InvalidElements(String),
    /// SGP4 propagation diverged (eccentricity out of `[0, 1)` or negative semi-latus rectum).
    #[error("SGP4 propagation failed at t={minutes_since_epoch:.3} min: {reason}")]
    PropagationDiverged {
        /// Minutes since the TLE/OMM epoch.
        minutes_since_epoch: f64,
        /// Underlying propagator failure description.
        reason: String,
    },
    /// Time-scale conversion failed while mapping `TDB` to `UTC`.
    #[error("time-scale conversion failed: {0}")]
    TimeScale(#[from] TimeError),
    /// TLE/OMM UTC timestamp lies outside the representable nanosecond Unix timestamp range.
    #[error("element UTC epoch cannot be converted to Unix timestamp")]
    InvalidElementTimestamp,
}

/// Analytical `SGP4`/`SDP4` Earth-orbit trajectory implementing [`TranslationalTrajectorySource`].
#[derive(Clone, Debug)]
pub struct Sgp4CatalogTrajectory {
    trajectory_id: u64,
    mode: Sgp4Mode,
    elements: sgp4::Elements,
    constants: sgp4::Constants,
    elem_unix_s: f64,
    elements_hash: ContentHash,
    provenance: DataProvenance,
    time_scales: Arc<TimeScaleCatalog>,
}

impl Sgp4CatalogTrajectory {
    /// Parses a Two-Line Element set (`line1`, `line2`) using [`Sgp4Mode::ModernWgs84Iau`].
    pub fn from_tle(
        trajectory_id: u64,
        object_name: Option<&str>,
        line1: &[u8],
        line2: &[u8],
        valid_interval: ValidEpochInterval,
        time_scales: impl Into<Arc<TimeScaleCatalog>>,
    ) -> Result<Self, Sgp4Error> {
        Self::from_tle_with_mode(
            trajectory_id,
            object_name,
            line1,
            line2,
            Sgp4Mode::ModernWgs84Iau,
            valid_interval,
            time_scales,
        )
    }

    /// Parses a Two-Line Element set with an explicit [`Sgp4Mode`].
    pub fn from_tle_with_mode(
        trajectory_id: u64,
        object_name: Option<&str>,
        line1: &[u8],
        line2: &[u8],
        mode: Sgp4Mode,
        valid_interval: ValidEpochInterval,
        time_scales: impl Into<Arc<TimeScaleCatalog>>,
    ) -> Result<Self, Sgp4Error> {
        let elements = sgp4::Elements::from_tle(object_name.map(str::to_owned), line1, line2)
            .map_err(|err| Sgp4Error::InvalidTle(err.to_string()))?;
        Self::from_elements_with_mode(trajectory_id, elements, mode, valid_interval, time_scales)
    }

    /// Parses a single CCSDS / CelesTrak JSON Orbit Mean-Elements Message (`OMM`).
    pub fn from_omm_json(
        trajectory_id: u64,
        omm_json: &str,
        valid_interval: ValidEpochInterval,
        time_scales: impl Into<Arc<TimeScaleCatalog>>,
    ) -> Result<Self, Sgp4Error> {
        Self::from_omm_json_with_mode(
            trajectory_id,
            omm_json,
            Sgp4Mode::ModernWgs84Iau,
            valid_interval,
            time_scales,
        )
    }

    /// Parses a JSON `OMM` object with an explicit [`Sgp4Mode`].
    pub fn from_omm_json_with_mode(
        trajectory_id: u64,
        omm_json: &str,
        mode: Sgp4Mode,
        valid_interval: ValidEpochInterval,
        time_scales: impl Into<Arc<TimeScaleCatalog>>,
    ) -> Result<Self, Sgp4Error> {
        let elements: sgp4::Elements =
            serde_json::from_str(omm_json).map_err(|err| Sgp4Error::InvalidOmm(err.to_string()))?;
        Self::from_elements_with_mode(trajectory_id, elements, mode, valid_interval, time_scales)
    }

    /// Constructs a trajectory from pre-parsed [`sgp4::Elements`] using [`Sgp4Mode::ModernWgs84Iau`].
    pub fn from_elements(
        trajectory_id: u64,
        elements: sgp4::Elements,
        valid_interval: ValidEpochInterval,
        time_scales: impl Into<Arc<TimeScaleCatalog>>,
    ) -> Result<Self, Sgp4Error> {
        Self::from_elements_with_mode(
            trajectory_id,
            elements,
            Sgp4Mode::ModernWgs84Iau,
            valid_interval,
            time_scales,
        )
    }

    /// Constructs a trajectory from pre-parsed [`sgp4::Elements`] and explicit [`Sgp4Mode`].
    pub fn from_elements_with_mode(
        trajectory_id: u64,
        elements: sgp4::Elements,
        mode: Sgp4Mode,
        valid_interval: ValidEpochInterval,
        time_scales: impl Into<Arc<TimeScaleCatalog>>,
    ) -> Result<Self, Sgp4Error> {
        let constants = match mode {
            Sgp4Mode::ModernWgs84Iau => sgp4::Constants::from_elements(&elements),
            Sgp4Mode::AfspcCompatibilityWgs72 => {
                sgp4::Constants::from_elements_afspc_compatibility_mode(&elements)
            }
        }
        .map_err(|err| Sgp4Error::InvalidElements(err.to_string()))?;
        let dt_utc = elements.datetime.and_utc();
        let elem_unix_s =
            (dt_utc.timestamp() as f64) + f64::from(dt_utc.timestamp_subsec_nanos()) * 1.0e-9;
        if !elem_unix_s.is_finite() {
            return Err(Sgp4Error::InvalidElementTimestamp);
        }
        let elements_hash = hash_elements(mode, &elements, elem_unix_s);
        let provenance = DataProvenance::new(
            "sgp4-catalog",
            format!("norad-{}", elements.norad_id),
            valid_interval,
            elements_hash,
        );
        Ok(Self {
            trajectory_id,
            mode,
            elements,
            constants,
            elem_unix_s,
            elements_hash,
            provenance,
            time_scales: time_scales.into(),
        })
    }

    /// Returns the parsed orbital elements.
    #[must_use]
    pub const fn elements(&self) -> &sgp4::Elements {
        &self.elements
    }

    /// Returns the active [`Sgp4Mode`].
    #[must_use]
    pub const fn mode(&self) -> Sgp4Mode {
        self.mode
    }

    /// Returns the canonical SHA-256 hash of these catalog elements and propagation mode.
    #[must_use]
    pub const fn elements_hash(&self) -> ContentHash {
        self.elements_hash
    }

    /// Propagates the SGP4/SDP4 state in `TEME` (`km`, `km/s`) at `minutes_since_epoch`.
    pub fn propagate_teme_minutes(
        &self,
        minutes_since_epoch: f64,
    ) -> Result<Sgp4TemeState, Sgp4Error> {
        let m = sgp4::MinutesSinceEpoch(minutes_since_epoch);
        let pred = match self.mode {
            Sgp4Mode::ModernWgs84Iau => self.constants.propagate(m),
            Sgp4Mode::AfspcCompatibilityWgs72 => {
                self.constants.propagate_afspc_compatibility_mode(m)
            }
        }
        .map_err(|err| Sgp4Error::PropagationDiverged {
            minutes_since_epoch,
            reason: err.to_string(),
        })?;
        Ok(Sgp4TemeState {
            position_km: DVec3::from_array(pred.position),
            velocity_km_s: DVec3::from_array(pred.velocity),
        })
    }
}

impl TranslationalTrajectorySource for Sgp4CatalogTrajectory {
    fn trajectory_id(&self) -> u64 {
        self.trajectory_id
    }

    fn model_identity(&self) -> DynamicalModelIdentity {
        DynamicalModelIdentity::Sgp4Catalog {
            elements_hash: self.elements_hash,
        }
    }

    fn provenance(&self) -> &DataProvenance {
        &self.provenance
    }

    fn sample_translational_icrf(
        &self,
        epoch: TdbEpoch,
    ) -> Result<FramedState, TrajectoryQueryError> {
        if !self.provenance.valid_interval.contains(epoch) {
            return Err(TrajectoryQueryError::EpochOutOfCoverage {
                requested_epoch: epoch,
                valid_start: self.provenance.valid_interval.start_epoch,
                valid_end: self.provenance.valid_interval.end_epoch,
            });
        }
        let utc = self
            .time_scales
            .tdb_to_utc(epoch)
            .map_err(|err| TrajectoryQueryError::PropagationFailed(err.to_string()))?;
        let minutes_since_epoch = (utc.unix_seconds() - self.elem_unix_s) / 60.0;
        let teme = self
            .propagate_teme_minutes(minutes_since_epoch)
            .map_err(|err| TrajectoryQueryError::PropagationFailed(err.to_string()))?;
        let (pos, vel) = teme::teme_km_to_icrf_si(epoch, teme.position_km, teme.velocity_km_s);
        Ok(FramedState {
            spec: StateFrameSpec {
                epoch,
                origin: NaifId::EARTH,
                axes: FrameAxesSpec::IcrfInertial,
                units: UnitSemantics::SiMetric,
            },
            pos,
            vel,
        })
    }
}

fn hash_elements(mode: Sgp4Mode, elements: &sgp4::Elements, elem_unix_s: f64) -> ContentHash {
    let mut h = DomainHasher::new(b"sim-orbital:sgp4-catalog:v1");
    h.write_u8(mode.tag_byte());
    h.write_u64(elements.norad_id);
    h.write_f64_canonical(elem_unix_s);
    h.write_f64_canonical(elements.mean_motion_dot);
    h.write_f64_canonical(elements.mean_motion_ddot);
    h.write_f64_canonical(elements.drag_term);
    h.write_u64(elements.element_set_number);
    h.write_f64_canonical(elements.inclination);
    h.write_f64_canonical(elements.right_ascension);
    h.write_f64_canonical(elements.eccentricity);
    h.write_f64_canonical(elements.argument_of_perigee);
    h.write_f64_canonical(elements.mean_anomaly);
    h.write_f64_canonical(elements.mean_motion);
    h.write_u64(elements.revolution_number);
    h.write_u8(elements.ephemeris_type);
    h.finish()
}
