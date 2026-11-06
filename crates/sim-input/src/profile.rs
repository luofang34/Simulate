use serde::{Deserialize, Serialize};

use crate::{AxisShape, AxisSource, DeviceInfo, GamepadAxis, InputRange, Layout};

/// A pilot control an axis can drive.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub enum Control {
    /// Roll (ailerons).
    Roll,
    /// Pitch (elevator); positive is nose up.
    Pitch,
    /// Yaw (rudder).
    Yaw,
    /// Throttle.
    Throttle,
}

/// One axis bound to one control.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct AxisBinding {
    /// The device axis.
    pub source: AxisSource,
    /// The control it drives.
    pub control: Control,
    /// How its value is shaped.
    pub shape: AxisShape,
}

/// Bindings for one kind of device.
#[derive(Debug, Clone, PartialEq)]
pub struct DeviceProfile {
    /// Profile name, shown in a controls UI.
    pub name: String,
    /// USB vendor and product ID this profile is for; `None` for fallbacks.
    pub usb_id: Option<(u16, u16)>,
    /// Axis bindings.
    pub bindings: Vec<AxisBinding>,
}

/// Every known profile plus the two fallbacks.
#[derive(Debug, Clone, PartialEq)]
pub struct ProfileLibrary {
    specific: Vec<DeviceProfile>,
    standard_gamepad: DeviceProfile,
    generic_joystick: DeviceProfile,
}

impl Default for ProfileLibrary {
    fn default() -> Self {
        Self::with_profiles(Vec::new())
    }
}

impl ProfileLibrary {
    /// The fallbacks plus `specific` profiles matched by USB ID first.
    pub fn with_profiles(specific: Vec<DeviceProfile>) -> Self {
        Self {
            specific,
            standard_gamepad: standard_gamepad(),
            generic_joystick: generic_joystick(),
        }
    }

    /// The profile for `device`: an exact USB ID match, else the layout's
    /// fallback.
    pub fn select(&self, device: &DeviceInfo) -> &DeviceProfile {
        let usb_id = device.vendor_id.zip(device.product_id);
        if let Some(found) = self
            .specific
            .iter()
            .find(|p| p.usb_id.is_some() && p.usb_id == usb_id)
        {
            return found;
        }
        match device.layout {
            Layout::Standard => &self.standard_gamepad,
            Layout::NonStandard => &self.generic_joystick,
        }
    }
}

fn bind(source: AxisSource, control: Control, shape: AxisShape) -> AxisBinding {
    AxisBinding {
        source,
        control,
        shape,
    }
}

/// Mode-2 style: left stick flies, right stick X is rudder, right trigger is
/// throttle. Stick-back is nose up, and the standard layout reports stick-up
/// as positive, so pitch is inverted.
fn standard_gamepad() -> DeviceProfile {
    let trigger = AxisShape {
        input: InputRange::Unipolar,
        ..AxisShape::LEVER
    };
    DeviceProfile {
        name: "Standard gamepad".to_owned(),
        usb_id: None,
        bindings: vec![
            bind(
                AxisSource::Gamepad(GamepadAxis::LeftStickX),
                Control::Roll,
                AxisShape::STICK,
            ),
            bind(
                AxisSource::Gamepad(GamepadAxis::LeftStickY),
                Control::Pitch,
                AxisShape::STICK.inverted(),
            ),
            bind(
                AxisSource::Gamepad(GamepadAxis::RightStickX),
                Control::Yaw,
                AxisShape::STICK,
            ),
            bind(
                AxisSource::Gamepad(GamepadAxis::RightTrigger),
                Control::Throttle,
                trigger,
            ),
        ],
    }
}

/// The common HID joystick and yoke order: axis 0 roll, axis 1 pitch (pushing
/// forward reads positive, so it is inverted), axis 2 a throttle lever whose
/// forward end reads −1. Devices that differ need a specific profile or a
/// calibration UI; rudder is unbound because its index varies most.
fn generic_joystick() -> DeviceProfile {
    DeviceProfile {
        name: "Generic joystick or yoke".to_owned(),
        usb_id: None,
        bindings: vec![
            bind(AxisSource::Raw(0), Control::Roll, AxisShape::STICK),
            bind(
                AxisSource::Raw(1),
                Control::Pitch,
                AxisShape::STICK.inverted(),
            ),
            bind(
                AxisSource::Raw(2),
                Control::Throttle,
                AxisShape::LEVER.inverted(),
            ),
        ],
    }
}
