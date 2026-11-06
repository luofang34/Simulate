//! Bridge between a Simulate vehicle and an Aviate flight controller across two
//! in-process tiers:
//! - **Tier 0 (`direct`)**: zero-overhead, allocation-free, syscall-free
//!   stepping of `aviate-xil-direct`'s [`DirectXilBoard`] directly from
//!   [`sim_core::VehicleState`] and body specific force (`>1,000x` real-time).
//! - **Tier 1 (`endpoint` / `wire`)**: shared-memory seqlock protocol over
//!   [`SharedStateV2`], matching Gazebo's XIL contract. The peer runs on the
//!   calling thread; [`AviateExecutionBoundary`] and [`Tier1StepMetrics`]
//!   keep that explicit for harnesses that also drive out-of-process peers.

mod direct;
mod endpoint;
mod error;
mod frame;
mod rotors;
mod wire;

pub use aviate_xil_contract::SharedStateV2;
pub use aviate_xil_direct::{
    DirectActuatorCmd, DirectBoardConfig, DirectNoiseTier, DirectSensorRates, DirectTelemetryRates,
    DirectTruthSample, DirectXilBoard,
};
pub use direct::{
    AviateDirectConfig, AviateDirectRunner, DirectCgStimulus, DirectFixedWingKernel,
    DirectX500Kernel, NedWorld, PassthroughFixedWingMixer, build_direct_fixed_wing_kernel,
    build_direct_x500_kernel, direct_fixed_wing_controls,
};
pub use endpoint::{
    AviateExecutionBoundary, AviateSharedMemoryEndpoint, AviateStimulus, AviateXilBridge,
    FlightControllerCommand, Tier1StepMetrics,
};
pub use error::{AviateBridgeError, DirectInitStage};
pub use frame::{EnuWorld, ModelLanes};
pub use rotors::{Rotor, RotorLayout, RotorSpin};
pub use wire::{MotorCommand, new_shared_state, validate_shared_state};
