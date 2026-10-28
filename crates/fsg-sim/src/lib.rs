//! Simulation core: an integer master clock, the block contract every
//! simulated component implements (flight dynamics, guidance computer, IMU,
//! radar, engines), multi-rate scheduling, and the vehicle state they share.
//!
//! Time is an integer count of [`TICKS_PER_SECOND`] ticks so blocks with
//! unrelated rates (a 120 Hz flight model, an AGC memory cycle of
//! 12/1,024,000 s) interleave exactly and replay bit-identically.

mod block;
mod controls;
mod kinematic;
mod schedule;
mod state;
mod time;

pub use block::{Block, BlockError};
pub use controls::ControlInputs;
pub use kinematic::{KinematicInput, KinematicModel};
pub use schedule::{RateAdapter, RealTimeClock};
pub use state::VehicleState;
pub use time::{SimInstant, TICKS_PER_SECOND, TickSpan};
