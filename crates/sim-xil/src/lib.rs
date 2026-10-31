//! Controller-blind external software- and hardware-in-the-loop (XIL) wire
//! protocols and [`sim_core::ExternalEndpoint`] transports for Simulate.
//!
//! All byte codecs and kinematic conversions are sans-IO and `wasm32`-clean:
//! - [`mavlink_hil`]: MAVLink v2 (`0xFD`) `HEARTBEAT`, `HIL_SENSOR`, `HIL_GPS`,
//!   `HIL_STATE_QUATERNION`, and streaming `HIL_ACTUATOR_CONTROLS` decoding,
//!   paired with native TCP and UDP endpoints for PX4, ArduPilot, external
//!   Aviate SITL, or UART-bridged physical flight controllers.
//! - [`betaflight`]: Betaflight SITL `144`-byte `FdmPacket` and motor servo
//!   packets, paired with sans-IO MultiWii Serial Protocol (`MSP` v1 and v2)
//!   framing and native UDP endpoints.

pub mod betaflight;
mod error;
mod kinematics;
pub mod mavlink_hil;

pub use error::XilError;
pub use kinematics::{NedKinematics, SEA_LEVEL_DENSITY_KG_M3, STANDARD_GRAVITY_M_S2, XilStimulus};
