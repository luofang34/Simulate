//! The vehicle's own control chain around a simulated airframe.
//!
//! Pilotage commands, the Aviate flight controller flies, and Simulate
//! supplies only the airframe, its sensors and the world. [`HostedBoard`]
//! is an Aviate kernel on an [`Airframe`], fed by the vehicle's sensor
//! suite; [`HostedVehicle`] puts it behind Pilotage's `VehicleAdapter`,
//! exchanging the MAVLink frames Pilotage's codec encodes and decodes, and
//! [`MissionHost`] flies a route on it with Pilotage's mission engine.
//! [`FlightChain`] is the two together. The C172 flies this way;
//! [`AviateX500Vehicle`] is the X500 on Aviate's own sensor synthesis.

mod airframe;
mod aviate_vehicle;
mod c172_airframe;
mod c172_vehicle;
mod chain;
mod error;
mod host_chain;
mod hosted_board;
mod mission_host;
mod x500_plant;

pub use airframe::{Airframe, SharedAir};
pub use aviate_vehicle::{
    AviateVehicleConfig, AviateX500Vehicle, FLIGHT_RATE_HZ, HostedVehicle, VehicleIdentity,
};
pub use c172_airframe::{C172Airframe, C172Start};
pub use c172_vehicle::{
    AccelCalibration, C172_PLANNING_BANK_DEG, C172_RATE_HZ, C172Board, C172Setup, C172Vehicle,
    c172_board, c172_pilotage_airframe, c172_sensor_suite, c172_sensor_suite_with, c172_vehicle,
};
pub use chain::{ChainSetup, ChainVehicle, FlightChain, MISSION_FRAME_HZ};
pub use error::ChainError;
pub use host_chain::{ChainRoute, HostChain, LinkCounters};
pub use hosted_board::{DeviceOutages, HostedBoard};
pub use mission_host::{FrameReport, MissionHost, MissionSetup, OwnshipFeed, RouteFix};
pub use pilotage_host_core::{HostEvent, HostStatus, MISSION_CLOCK};
pub use pilotage_mission::{MissionEvent, MissionState};
pub use pilotage_protocol::{VehicleClass, VehicleId};
pub use sim_aviate::DirectNoiseTier;
pub use x500_plant::{PlantStimulus, X500Plant};
