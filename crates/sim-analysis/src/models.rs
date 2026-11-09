//! Separated domain models for spatiotemporal geometric and access analysis:
//! [`GroundStationModel`], [`VehicleSensorModel`], [`SensorFovModel`],
//! [`OccultationEnvironmentModel`], and [`AccessQuerySpec`].

pub use sim_analysis_contract::{
    AccessQuerySpec, GroundStationModel, OccultationEnvironmentModel, OccultingBodySpec,
    OccultingShape, SensorFovModel, VehicleSensorModel,
};
