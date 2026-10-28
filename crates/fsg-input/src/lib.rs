//! Maps raw controller events to [`fsg_sim::ControlInputs`], independent of
//! how the platform reads devices (gilrs natively, the Gamepad API in the
//! browser).
//!
//! Gamepads with the W3C "standard" layout bind by semantic axis. Yokes,
//! throttle quadrants, and other joysticks report a non-standard layout whose
//! axis indices vary by device, so they bind by raw index, either through a
//! profile matched on USB vendor/product ID or through the generic fallback.

mod event;
mod mapper;
mod profile;
mod shaping;

pub use event::{AxisSource, DeviceId, DeviceInfo, GamepadAxis, Layout, RawInput};
pub use mapper::InputMapper;
pub use profile::{AxisBinding, Control, DeviceProfile, ProfileLibrary};
pub use shaping::{AxisShape, InputRange, OutputRange};
