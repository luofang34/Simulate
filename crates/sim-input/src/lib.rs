//! Maps raw controller and keyboard input to [`sim_core::ControlInputs`],
//! independent of how the platform reads devices (gilrs natively, the
//! Gamepad API in the browser).
//!
//! Standard-layout gamepads and the keyboard follow a switchable
//! [`ControlPreset`] chosen per [`VehicleControlClass`]. Yokes, throttle
//! quadrants, and other joysticks report a non-standard layout whose axis
//! indices vary by device, so they bind by raw index, either through a
//! profile matched on USB vendor/product ID or through the
//! [`ControlPresetId::FlightStickHotas`] preset.
//!
//! Every source keeps its own state and a control follows whichever source
//! last moved it, so a gamepad resting in its deadzone never overrides held
//! keys. Bindings can be captured interactively, tuned per control, and
//! saved as a [`ControlConfig`].

mod binding;
mod config;
mod event;
mod keyboard;
mod keys;
mod mapper;
mod preset;
mod profile;
mod shaping;

pub use binding::{BindingSource, ButtonBinding, InputTarget, MappedBinding};
pub use config::{ConfigError, ControlConfig, ControlTuningEntry, PinnedPreset};
pub use event::{AxisSource, DeviceId, DeviceInfo, GamepadAxis, GamepadButton, Layout, RawInput};
pub use keyboard::{KeyboardFlightAxes, KeyboardRates};
pub use keys::{CapturedKey, FlightKeyAction, KeyBinding, KeyId, KeyResponse};
pub use mapper::{
    AxisReading, BindingCapture, CapturedBinding, ControlMonitorSnapshot, ControlOwner,
    ControlTuning, DeviceMonitor, InputMapper, TargetReading,
};
pub use preset::{ControlPreset, ControlPresetId, ThrottleBehavior, VehicleControlClass};
pub use profile::{AxisBinding, Control, DeviceProfile, ProfileLibrary};
pub use shaping::{AxisShape, InputRange, OutputRange};
