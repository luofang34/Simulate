//! The vehicle's own control chain around a simulated X500 quadrotor.
//!
//! Pilotage commands, the Aviate flight controller flies, and Simulate
//! supplies only the airframe and the world: [`AviateX500Vehicle`] puts the
//! Aviate kernel and the [`X500Plant`] behind Pilotage's `VehicleAdapter`,
//! exchanging the MAVLink frames Pilotage's codec encodes and decodes.

mod aviate_vehicle;
mod error;
mod x500_plant;

pub use aviate_vehicle::{AviateVehicleConfig, AviateX500Vehicle, FLIGHT_RATE_HZ};
pub use error::ChainError;
pub use x500_plant::{PlantStimulus, X500Plant};
