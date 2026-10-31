/// A connected controller, numbered by the platform backend.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct DeviceId(pub u32);

/// How a device reports its axes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Layout {
    /// The W3C Gamepad "standard" mapping: axes have fixed meanings.
    Standard,
    /// Anything else: axis indices are device-specific.
    NonStandard,
}

/// What the platform knows about a device when it connects.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DeviceInfo {
    /// Backend-assigned identifier.
    pub id: DeviceId,
    /// Human-readable name reported by the device.
    pub name: String,
    /// USB vendor ID, when the backend exposes it.
    pub vendor_id: Option<u16>,
    /// USB product ID, when the backend exposes it.
    pub product_id: Option<u16>,
    /// Axis layout.
    pub layout: Layout,
}

/// Axes of the standard gamepad layout.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum GamepadAxis {
    /// Left stick, right positive.
    LeftStickX,
    /// Left stick, up positive.
    LeftStickY,
    /// Right stick, right positive.
    RightStickX,
    /// Right stick, up positive.
    RightStickY,
    /// Left trigger, 0 released to 1 pressed.
    LeftTrigger,
    /// Right trigger, 0 released to 1 pressed.
    RightTrigger,
}

/// Where an axis value comes from.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum AxisSource {
    /// A standard-layout axis.
    Gamepad(GamepadAxis),
    /// A device-specific axis index.
    Raw(u32),
}

/// One event from a platform input backend.
#[derive(Debug, Clone, PartialEq)]
pub enum RawInput {
    /// A device appeared.
    Connected(DeviceInfo),
    /// A device went away; its last values are released.
    Disconnected(DeviceId),
    /// An axis moved. Sticks report −1..1, triggers 0..1.
    Axis {
        /// Reporting device.
        device: DeviceId,
        /// Which axis.
        source: AxisSource,
        /// New position.
        value: f32,
    },
}
