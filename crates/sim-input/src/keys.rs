use serde::{Deserialize, Serialize};

/// A portable physical key name, following the W3C `KeyboardEvent.code`
/// values (`"KeyW"`, `"ArrowUp"`, `"Space"`), which winit's `KeyCode` debug
/// names also use, so both the native and web front ends can produce it.
#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
#[serde(transparent)]
pub struct KeyId(pub String);

impl KeyId {
    /// A key from its `KeyboardEvent.code` name.
    pub fn new(code: impl Into<String>) -> Self {
        Self(code.into())
    }

    /// The `KeyboardEvent.code` name.
    pub fn as_str(&self) -> &str {
        &self.0
    }
}

/// A flight action a key or gamepad button holds while pressed.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub enum FlightKeyAction {
    /// Roll stick left.
    RollLeft,
    /// Roll stick right.
    RollRight,
    /// Pitch stick back, nose up.
    PitchUp,
    /// Pitch stick forward, nose down.
    PitchDown,
    /// Left rudder or left yaw.
    YawLeft,
    /// Right rudder or right yaw.
    YawRight,
    /// Throttle up or climb.
    ThrottleUp,
    /// Throttle down or descend.
    ThrottleDown,
    /// Wheel brakes, reported through
    /// [`InputMapper::wheel_brake_held`](crate::InputMapper::wheel_brake_held).
    WheelBrake,
    /// Return the throttle lever to its trim and centre the keyboard stick.
    ResetTrim,
}

impl FlightKeyAction {
    /// Every action, in display order.
    pub const ALL: [Self; 10] = [
        Self::RollLeft,
        Self::RollRight,
        Self::PitchUp,
        Self::PitchDown,
        Self::YawLeft,
        Self::YawRight,
        Self::ThrottleUp,
        Self::ThrottleDown,
        Self::WheelBrake,
        Self::ResetTrim,
    ];
}

/// One key bound to one action.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct KeyBinding {
    /// The key.
    pub key: KeyId,
    /// The action it holds.
    pub action: FlightKeyAction,
}

impl KeyBinding {
    /// Binds the key named `code` to `action`.
    pub fn new(code: &str, action: FlightKeyAction) -> Self {
        Self {
            key: KeyId::new(code),
            action,
        }
    }
}

/// What [`InputMapper::handle_key`](crate::InputMapper::handle_key) did with a key.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum KeyResponse {
    /// The key is not a flight key; the caller may use it.
    Ignored,
    /// The key pressed or released this action.
    Action(FlightKeyAction),
    /// A key capture was pending and bound this key.
    Captured(CapturedKey),
}

/// A key bound by [`InputMapper::start_key_capture`](crate::InputMapper::start_key_capture).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct CapturedKey {
    /// The key pressed.
    pub key: KeyId,
    /// The action it now holds.
    pub action: FlightKeyAction,
}
