//! What the displays read of a vehicle on the real control chain: the flight
//! controller's account for the instruments, Pilotage's navigation display for the
//! guidance, and the steered navigator's fix for the moving maps.

use sim_core::VehicleState;

use super::{FreeFlightPlant, SteeredFix};

impl FreeFlightPlant {
    /// The flight controller's account of its last cycle and the estimate the displays
    /// hold, when flying the real chain.
    #[must_use]
    pub fn chain_avionics(&self) -> Option<(sim_flight_chain::FcAvionics, VehicleState)> {
        match self {
            Self::C172Mission(rig) => Some((rig.avionics(), rig.held_estimate())),
            _ => None,
        }
    }

    /// Where the navigator the real chain's mission steers on places the vehicle.
    #[must_use]
    pub fn chain_steered(&self) -> Option<SteeredFix> {
        match self {
            Self::C172Mission(rig) => rig.steered(),
            _ => None,
        }
    }

    /// Pilotage's navigation display, when flying the real chain.
    #[must_use]
    pub fn chain_navigation(&self) -> Option<sim_flight_chain::NavigationDisplay> {
        match self {
            Self::C172Mission(rig) => rig.navigation(),
            _ => None,
        }
    }
}
