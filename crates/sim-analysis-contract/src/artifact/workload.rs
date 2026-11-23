//! Platform, station, and workload descriptions carried by an analysis request.

use serde::{Deserialize, Serialize};

use crate::{
    KinematicTrajectoryNode, NaifId,
    models::{GroundStationModel, VehicleSensorModel},
    rf::{LinkDirection, RfLinkSpec},
    terrain::AltitudeSpec,
};

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
    /// Link budget between a moving platform and a ground station: geometric access plus the
    /// radio budget on every accessible grid epoch.
    EndpointPairLinkBudget {
        /// Moving platform trajectory specification.
        source_platform: SerializablePlatformSpec,
        /// Antenna mount and field of view on `source_platform`.
        source_sensor: VehicleSensorModel,
        /// Ground station and its antenna mount.
        target_station: SerializableGroundStationSpec,
        /// Which end transmits.
        direction: LinkDirection,
        /// Radio link description.
        link: RfLinkSpec,
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
