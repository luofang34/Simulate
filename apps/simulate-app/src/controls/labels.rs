//! Names, indices, and labels of controls and device inputs.

use sim_core::ControlInputs;
use sim_input::{AxisSource, Control, GamepadAxis};

use super::PadButton;

/// Index of `control` in [`super::CONTROLS`].
#[must_use]
pub const fn control_index(control: Control) -> usize {
    match control {
        Control::Roll => 0,
        Control::Pitch => 1,
        Control::Yaw => 2,
        Control::Throttle => 3,
    }
}

/// Display name of `control`.
#[must_use]
pub const fn control_title(control: Control) -> &'static str {
    match control {
        Control::Roll => "Roll",
        Control::Pitch => "Pitch",
        Control::Yaw => "Yaw",
        Control::Throttle => "Throttle",
    }
}

/// `control`'s value in `controls`.
#[must_use]
pub const fn control_value(controls: &ControlInputs, control: Control) -> f64 {
    match control {
        Control::Roll => controls.roll,
        Control::Pitch => controls.pitch,
        Control::Yaw => controls.yaw,
        Control::Throttle => controls.throttle,
    }
}

pub(super) fn set_control(controls: &mut ControlInputs, control: Control, value: f64) {
    match control {
        Control::Roll => controls.roll = value,
        Control::Pitch => controls.pitch = value,
        Control::Yaw => controls.yaw = value,
        Control::Throttle => controls.throttle = value,
    }
}

/// Short label of a device axis (`LS X`, `RT`, `AXIS 5`).
#[must_use]
pub fn source_label(source: AxisSource) -> String {
    match source {
        AxisSource::Gamepad(axis) => match axis {
            GamepadAxis::LeftStickX => "LS X",
            GamepadAxis::LeftStickY => "LS Y",
            GamepadAxis::RightStickX => "RS X",
            GamepadAxis::RightStickY => "RS Y",
            GamepadAxis::LeftTrigger => "LT",
            GamepadAxis::RightTrigger => "RT",
        }
        .to_owned(),
        AxisSource::Raw(index) => format!("AXIS {index}"),
    }
}

pub(super) const fn button_label(button: PadButton) -> &'static str {
    match button {
        PadButton::LeftBumper => "LB",
        PadButton::RightBumper => "RB",
    }
}
