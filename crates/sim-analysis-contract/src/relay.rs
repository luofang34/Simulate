//! Serializable report structures for two-endpoint geometric link access (`EndpointLinkReport`)
//! and Aircraft – Satellite – Ground Station simultaneous geometric relay (`GeometricRelayReport`).

use glam::DVec3;
use serde::{Deserialize, Serialize};

use crate::{
    DataProvenance, TdbEpoch, ValidEpochInterval, serde_dvec3,
    taxonomy::{AccessRestrictionReason, AccessWindow, WindowSearchCompleteness},
};

/// Explicit declaration of which physical phenomena were verified by an analysis report.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct VerificationScope {
    /// `true` when geometric line-of-sight, planetary ellipsoid/terrain occultation, and sensor mount `FOV` were evaluated.
    pub geometric_los_and_fov_verified: bool,
    /// `false` for geometric analysis: RF link budget (`EIRP`, `G/T`, $E_b/N_0$, rain/ionospheric fade) is NOT verified.
    pub rf_link_budget_verified: bool,
    /// `false` for simultaneous geometric relay: onboard store-and-forward buffering is NOT modeled.
    pub store_and_forward_modeled: bool,
}

impl VerificationScope {
    /// Scope for pure geometric line-of-sight, terrain occultation, and sensor mount `FOV` analysis.
    #[must_use]
    pub const fn geometric_visibility_only() -> Self {
        Self {
            geometric_los_and_fov_verified: true,
            rf_link_budget_verified: false,
            store_and_forward_modeled: false,
        }
    }
}

/// Instantaneous geometric evaluation and restriction breakdown between two endpoints at `epoch`.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct EndpointLinkSample {
    /// Sample epoch (`TDB`).
    pub epoch: TdbEpoch,
    /// Source endpoint antenna/sensor phase-center position in `SSB ICRF` (`m`).
    #[serde(with = "serde_dvec3")]
    pub source_pos_icrf_m: DVec3,
    /// Target endpoint antenna/sensor phase-center position in `SSB ICRF` (`m`).
    #[serde(with = "serde_dvec3")]
    pub target_pos_icrf_m: DVec3,
    /// Euclidean slant range (`m`).
    pub slant_range_m: f64,
    /// Line-of-sight range rate (`m/s`).
    pub range_rate_m_s: f64,
    /// Local horizon elevation angle from source to target (`rad`, $[-\pi/2, \pi/2]$).
    pub elevation_from_source_rad: f64,
    /// Local horizon elevation angle from target to source (`rad`, $[-\pi/2, \pi/2]$).
    pub elevation_from_target_rad: f64,
    /// Off-boresight angle in source sensor mount (`rad`, `Some` when source mount orientation is defined).
    pub source_off_boresight_rad: Option<f64>,
    /// Off-boresight angle in target sensor mount (`rad`, `Some` when target mount orientation is defined).
    pub target_off_boresight_rad: Option<f64>,
    /// Active geometric/terrain/FOV restrictions (`empty` iff `accessible == true`).
    pub active_restrictions: Vec<AccessRestrictionReason>,
    /// `true` iff all horizon, mount `FOV`, range, planetary body, and DEM terrain constraints hold.
    pub accessible: bool,
}

/// Complete two-endpoint geometric link access report.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct EndpointLinkReport {
    /// Source endpoint numeric ID.
    pub source_endpoint_id: u32,
    /// Source endpoint label.
    pub source_label: String,
    /// Target endpoint numeric ID.
    pub target_endpoint_id: u32,
    /// Target endpoint label.
    pub target_label: String,
    /// Evaluated `TDB` interval.
    pub valid_interval: ValidEpochInterval,
    /// Search completeness declaration.
    pub completeness: WindowSearchCompleteness,
    /// Explicit verification scope (`rf_link_budget_verified = false`).
    pub verification_scope: VerificationScope,
    /// Input trajectory and dataset provenance.
    pub provenance: Vec<DataProvenance>,
    /// Uniform grid samples across `valid_interval`.
    pub samples: Vec<EndpointLinkSample>,
    /// Continuous access windows.
    pub windows: Vec<AccessWindow>,
}

/// Identifies which hop in an Aircraft – Satellite – Ground Station relay chain produced a restriction.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum RelayHopKind {
    /// Hop 1: Aircraft $\leftrightarrow$ Relay Satellite.
    AircraftToSatellite,
    /// Hop 2: Relay Satellite $\leftrightarrow$ Ground Station.
    SatelliteToGround,
    /// Direct link: Aircraft $\leftrightarrow$ Ground Station.
    DirectAircraftToGround,
}

/// Restriction reason tagged with the specific [`RelayHopKind`] where it occurred.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct RelayRestrictionEntry {
    /// Hop that failed the constraint.
    pub hop: RelayHopKind,
    /// Underlying geometric, FOV, or terrain restriction reason.
    pub reason: AccessRestrictionReason,
}

/// Instantaneous sample of an Aircraft – Satellite – Ground Station geometric relay evaluation.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct RelayAccessSample {
    /// Sample epoch (`TDB`).
    pub epoch: TdbEpoch,
    /// Hop 1 (`Aircraft <-> Relay Satellite`) evaluation.
    pub hop1_aircraft_to_satellite: EndpointLinkSample,
    /// Hop 2 (`Relay Satellite <-> Ground Station`) evaluation.
    pub hop2_satellite_to_ground: EndpointLinkSample,
    /// Optional direct link (`Aircraft <-> Ground Station`) evaluation.
    pub direct_aircraft_to_ground: Option<EndpointLinkSample>,
    /// Combined restrictions blocking the simultaneous two-hop relay (`Hop 1` $\cup$ `Hop 2`).
    pub relay_restrictions: Vec<RelayRestrictionEntry>,
    /// `true` iff both `hop1_aircraft_to_satellite.accessible` and `hop2_satellite_to_ground.accessible` hold at `epoch`.
    pub relay_accessible: bool,
}

/// Complete Aircraft – Satellite – Ground Station simultaneous geometric relay report.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct GeometricRelayReport {
    /// Aircraft endpoint ID.
    pub aircraft_endpoint_id: u32,
    /// Relay satellite endpoint ID.
    pub satellite_endpoint_id: u32,
    /// Ground station ID.
    pub ground_station_id: u32,
    /// Evaluated `TDB` interval.
    pub valid_interval: ValidEpochInterval,
    /// Search completeness declaration.
    pub completeness: WindowSearchCompleteness,
    /// Explicit verification scope (`rf_link_budget_verified = false`, `store_and_forward_modeled = false`).
    pub verification_scope: VerificationScope,
    /// Provenance of aircraft and satellite trajectories.
    pub provenance: Vec<DataProvenance>,
    /// Uniform grid samples across `valid_interval`.
    pub samples: Vec<RelayAccessSample>,
    /// Simultaneous two-hop geometric relay windows (`Hop 1` $\cap$ `Hop 2`).
    pub relay_windows: Vec<AccessWindow>,
    /// Individual Hop 1 (`Aircraft <-> Satellite`) access windows.
    pub hop1_windows: Vec<AccessWindow>,
    /// Individual Hop 2 (`Satellite <-> Ground Station`) access windows.
    pub hop2_windows: Vec<AccessWindow>,
    /// Direct (`Aircraft <-> Ground Station`) access windows (when direct link evaluation is enabled).
    pub direct_windows: Vec<AccessWindow>,
}
