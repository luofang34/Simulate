//! Simulation core: an integer master clock, the block contract every
//! simulated component implements (flight dynamics, guidance computer, IMU,
//! radar, engines), multi-rate scheduling, and the vehicle state they share,
//! with rigid-body and JSBSim dynamics, crew and hardware-in-the-loop bridges,
//! source-stamped instrument readings, and a mounted camera sensor with its
//! visual-navigation evaluation harness.
//!
//! Time is an integer count of [`TICKS_PER_SECOND`] ticks so blocks with
//! unrelated rates (a 120 Hz flight model, an AGC memory cycle of
//! 12/1,024,000 s) interleave exactly and replay bit-identically.

mod block;
mod camera_sensor;
mod controls;
mod crew_port;
mod hitl_bridge;
mod instrument;
mod jsbsim_fdm;
mod kinematic;
mod rigid_body;
mod schedule;
mod state;
mod time;
mod visual_harness;

pub use block::{Block, BlockError};
pub use camera_sensor::{
    CAMERA_DEFAULT_PERIOD, CameraFrameCapture, CameraMount, CameraPoseEcef, CameraSensorBlock,
    CameraSensorConfig, CameraSensorInput, CameraSensorSnapshot, EnuAnchor, NavigateCameraPose,
};
pub use controls::ControlInputs;
pub use crew_port::{
    CrewAction, CrewObservation, CrewPortError, CrewStationPort, StampedCrewAction,
    StampedCrewObservation,
};
pub use hitl_bridge::{
    BridgeError, BridgeSnapshot, BridgeStats, DeliveredResponse, EndpointError, ExternalEndpoint,
    ExternalNodeBridge, HitlSyncMode, StampedResponse,
};
pub use instrument::{SourceValidity, StampedReading};
pub use jsbsim_fdm::{
    EngineStartMode, EngineStatus, JSBSIM_DEFAULT_PERIOD, JsbsimFdmBlock, JsbsimFdmConfig,
    JsbsimFdmSnapshot, MagnetoSwitch, SystemControls,
};
pub use kinematic::{KinematicInput, KinematicModel};
pub use rigid_body::{
    PropellantBurn, RigidBody6DofBlock, RigidBodyConfig, RigidBodyInput, RigidBodySnapshot,
    ThrusterWrench,
};
pub use schedule::{RateAdapter, RealTimeClock};
pub use sim_geo::SensorIntrinsics;
pub use state::VehicleState;
pub use time::{SimInstant, TICKS_PER_SECOND, TickSpan};
pub use visual_harness::{
    FusionEligibility, MapCell, NeesSummary, TwoTierConfig, TwoTierVisualHarness, VisualEstimate,
    VisualFix, VisualFrame, VisualHarnessError,
};
