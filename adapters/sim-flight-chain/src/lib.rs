//! The vehicle's own control chain around a simulated airframe.
//!
//! Pilotage commands, the Aviate flight controller flies, and Simulate
//! supplies only the airframe, its sensors and the world. [`HostedBoard`]
//! is an Aviate kernel on an [`Airframe`], fed by the vehicle's sensor
//! suite. [`HostChain`] puts it under Pilotage's embeddable host core,
//! which flies a route on it through the MAVLink bytes the board and the
//! core exchange. The C172 flies this way.

mod airframe;
mod c172_airframe;
mod c172_vehicle;
mod error;
mod host_chain;
mod hosted_board;
/// Temporary profiling probe.
pub mod profile;

pub use airframe::{Airframe, SharedAir};
pub use c172_airframe::{C172Airframe, C172Start};
pub use c172_vehicle::{
    AccelCalibration, C172_PLANNING_BANK_DEG, C172_RATE_HZ, C172Board, C172Setup, c172_board,
    c172_pilotage_airframe, c172_sensor_suite, c172_sensor_suite_with,
};
pub use error::ChainError;
pub use host_chain::{ChainRoute, HostChain, LinkCounters, OwnshipFeed, RouteFix};
pub use hosted_board::{DeviceOutages, HostedBoard};
pub use pilotage_host_core::{HostEvent, HostStatus, MISSION_CLOCK};
pub use pilotage_mission::{MissionEvent, MissionState};
pub use pilotage_protocol::{VehicleClass, VehicleId};
