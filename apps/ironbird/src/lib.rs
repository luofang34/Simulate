//! Headless closed-loop software- and hardware-in-the-loop (`XIL`) ironbird
//! test rig for Simulate.
//!
//! Couples high-fidelity [`sim_core::JsbsimFdmBlock`] fixed-wing aircraft and
//! [`sim_core::RigidBody6DofBlock`] multirotors across all three XIL tiers:
//! - **Tier 0 (`direct-aviate`)**: zero-overhead, syscall-free, allocation-free
//!   in-process stepping of `aviate-xil-direct` (`>1,000x` real-time).
//! - **Tier 1 (`shm-aviate`)**: in-process [`sim_aviate::SharedStateV2`] seqlock
//!   exchange matching Gazebo's XIL contract.
//! - **Tier 2 (`sim-xil`)**: controller-blind MAVLink v2 `HIL_*` (TCP/UDP) and
//!   Betaflight SITL UDP endpoints for PX4, ArduPilot, Betaflight, or UART HITL.
//!
//! Every rig steps its plant through a [`sim_core::SimSession`] with a 30 Hz
//! optical sensor channel, so runs report the same exposures at any pacing.

#[cfg(not(target_arch = "wasm32"))]
pub mod cli;
mod error;
mod plant;
mod rigs;

pub use error::IronbirdError;
pub use plant::{
    FixedWingVehicle, JsbsimPlant, JsbsimPlantSnapshot, X500PlantSnapshot, X500RigidPlant,
};
pub use rigs::{
    OPTICAL_CHANNEL, OPTICAL_PERIOD, RigSummary, run_tier0_fixed_wing_cruise,
    run_tier0_x500_hover_climb, run_tier1_shm_x500,
};
#[cfg(not(target_arch = "wasm32"))]
pub use rigs::{run_tier2_betaflight_x500, run_tier2_mavlink_fixed_wing};
