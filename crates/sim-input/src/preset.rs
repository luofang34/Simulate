use serde::{Deserialize, Serialize};

use crate::{ButtonBinding, FlightKeyAction, KeyBinding, KeyId, MappedBinding};

mod builtin;

#[cfg(test)]
mod tests;

/// The kind of vehicle being flown, which picks the default preset.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub enum VehicleControlClass {
    /// Aeroplanes: stick, rudder, and a throttle lever that stays where set.
    FixedWing,
    /// Multirotors: a self-centring climb-rate throttle stick.
    Multirotor,
    /// Spacecraft: flown with the fixed-wing stick layout.
    Spacecraft,
}

impl VehicleControlClass {
    /// The preset this class uses for standard-layout gamepads and the
    /// keyboard unless the user pinned another.
    pub fn default_preset(self) -> ControlPresetId {
        match self {
            Self::FixedWing | Self::Spacecraft => ControlPresetId::FixedWingGamepad,
            Self::Multirotor => ControlPresetId::QuadCamDroneMode2,
        }
    }
}

/// A built-in control layout.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub enum ControlPresetId {
    /// Mode 2 camera drone: left stick climb rate and yaw, right stick roll and pitch.
    QuadCamDroneMode2,
    /// Mode 2 FPV acro: the camera-drone sticks with an uncentred throttle lever.
    QuadAcroMode2,
    /// Fixed-wing on a gamepad: left stick flies, triggers are rudder,
    /// bumpers move the throttle, right stick looks around.
    FixedWingGamepad,
    /// Fixed-wing Mode 2 twin sticks: left stick throttle and rudder, right stick flies.
    FixedWingMode2,
    /// HID joystick, yoke, or HOTAS: roll, pitch, twist rudder, throttle lever.
    /// Also the preset for every non-standard-layout device.
    FlightStickHotas,
}

impl ControlPresetId {
    /// Every preset, in display order.
    pub const ALL: [Self; 5] = [
        Self::QuadCamDroneMode2,
        Self::QuadAcroMode2,
        Self::FixedWingGamepad,
        Self::FixedWingMode2,
        Self::FlightStickHotas,
    ];

    /// The preset as shipped.
    pub fn builtin(self) -> ControlPreset {
        builtin::preset(self)
    }
}

/// How throttle inputs become the throttle lever position.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub enum ThrottleBehavior {
    /// A physical lever maps straight into 0..1; throttle keys slide the
    /// lever at the keyboard throttle rate and it stays where left.
    DirectLever,
    /// A self-centring stick commands climb rate: rest is 0.5 (hover), full
    /// up 1.0, full down 0.0, and releasing returns to 0.5.
    CenteredClimbRate,
    /// Throttle inputs move a latched lever at `rate_per_s` full travels per
    /// second; releasing them holds the lever where it is.
    LatchingIncremental {
        /// Lever travel per second at full deflection.
        rate_per_s: f64,
    },
}

/// A complete control layout: device axes, gamepad buttons, keys, and
/// throttle behaviour.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ControlPreset {
    /// Which preset this is.
    pub id: ControlPresetId,
    /// Name shown in a controls UI.
    pub name: String,
    /// Axis bindings for devices using this preset.
    pub bindings: Vec<MappedBinding>,
    /// Gamepad buttons that hold flight actions.
    pub buttons: Vec<ButtonBinding>,
    /// Keyboard bindings.
    pub keys: Vec<KeyBinding>,
    /// How throttle inputs move the throttle.
    pub throttle: ThrottleBehavior,
}

impl ControlPreset {
    /// The action `key` holds, if any.
    pub fn key_action(&self, key: &KeyId) -> Option<FlightKeyAction> {
        self.keys.iter().find(|b| b.key == *key).map(|b| b.action)
    }

    /// The keys bound to `action`.
    pub fn keys_for(&self, action: FlightKeyAction) -> impl Iterator<Item = &KeyId> {
        self.keys
            .iter()
            .filter(move |b| b.action == action)
            .map(|b| &b.key)
    }
}
