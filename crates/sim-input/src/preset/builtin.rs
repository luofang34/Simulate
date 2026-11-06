use super::{ControlPreset, ControlPresetId, ThrottleBehavior};
use crate::FlightKeyAction as A;
use crate::{
    AxisShape, AxisSource, ButtonBinding, Control, GamepadAxis, GamepadButton, InputTarget,
    KeyBinding, MappedBinding,
};

const FIXED_WING_GAMEPAD_RATE_PER_S: f64 = 0.45;
const FIXED_WING_MODE2_RATE_PER_S: f64 = 0.55;
const ACRO_RATE_PER_S: f64 = 0.8;

pub(super) fn preset(id: ControlPresetId) -> ControlPreset {
    match id {
        ControlPresetId::QuadCamDroneMode2 => ControlPreset {
            id,
            name: "Quad camera drone (Mode 2)".to_owned(),
            bindings: quad_mode2(),
            buttons: Vec::new(),
            keys: keys(),
            throttle: ThrottleBehavior::CenteredClimbRate,
        },
        ControlPresetId::QuadAcroMode2 => ControlPreset {
            id,
            name: "Quad FPV acro (Mode 2)".to_owned(),
            bindings: quad_mode2(),
            buttons: Vec::new(),
            keys: keys(),
            throttle: ThrottleBehavior::LatchingIncremental {
                rate_per_s: ACRO_RATE_PER_S,
            },
        },
        ControlPresetId::FixedWingGamepad => ControlPreset {
            id,
            name: "Fixed-wing gamepad".to_owned(),
            bindings: fixed_wing_gamepad(),
            buttons: shoulder_throttle_buttons(),
            keys: keys(),
            throttle: ThrottleBehavior::LatchingIncremental {
                rate_per_s: FIXED_WING_GAMEPAD_RATE_PER_S,
            },
        },
        ControlPresetId::FixedWingMode2 => ControlPreset {
            id,
            name: "Fixed-wing Mode 2 twin sticks".to_owned(),
            bindings: fixed_wing_mode2(),
            buttons: shoulder_throttle_buttons(),
            keys: keys(),
            throttle: ThrottleBehavior::LatchingIncremental {
                rate_per_s: FIXED_WING_MODE2_RATE_PER_S,
            },
        },
        ControlPresetId::FlightStickHotas => ControlPreset {
            id,
            name: "Flight stick / yoke / HOTAS".to_owned(),
            bindings: hotas(),
            buttons: Vec::new(),
            keys: keys(),
            throttle: ThrottleBehavior::DirectLever,
        },
    }
}

fn pad(axis: GamepadAxis) -> AxisSource {
    AxisSource::Gamepad(axis)
}

/// The standard layout reports stick-up as positive and pitch wants
/// stick-back positive, so every pitch stick is inverted.
fn pitch_stick(source: AxisSource) -> MappedBinding {
    MappedBinding::axis(source, Control::Pitch, AxisShape::STICK.inverted())
}

/// Throttle sticks are centred (climb-rate or lever-rate commands), so the
/// stick deadzone keeps rest from drifting the throttle.
fn quad_mode2() -> Vec<MappedBinding> {
    vec![
        MappedBinding::axis(
            pad(GamepadAxis::LeftStickY),
            Control::Throttle,
            AxisShape::STICK,
        ),
        MappedBinding::axis(pad(GamepadAxis::LeftStickX), Control::Yaw, AxisShape::STICK),
        MappedBinding::axis(
            pad(GamepadAxis::RightStickX),
            Control::Roll,
            AxisShape::STICK,
        ),
        pitch_stick(pad(GamepadAxis::RightStickY)),
    ]
}

fn fixed_wing_gamepad() -> Vec<MappedBinding> {
    vec![
        MappedBinding::axis(
            pad(GamepadAxis::LeftStickX),
            Control::Roll,
            AxisShape::STICK,
        ),
        pitch_stick(pad(GamepadAxis::LeftStickY)),
        MappedBinding::differential(
            pad(GamepadAxis::RightTrigger),
            pad(GamepadAxis::LeftTrigger),
            Control::Yaw,
            AxisShape::STICK,
        ),
        MappedBinding::axis(
            pad(GamepadAxis::RightStickX),
            InputTarget::CameraPan,
            AxisShape::STICK,
        ),
        MappedBinding::axis(
            pad(GamepadAxis::RightStickY),
            InputTarget::CameraTilt,
            AxisShape::STICK,
        ),
    ]
}

fn fixed_wing_mode2() -> Vec<MappedBinding> {
    vec![
        MappedBinding::axis(
            pad(GamepadAxis::LeftStickY),
            Control::Throttle,
            AxisShape::STICK,
        ),
        MappedBinding::axis(pad(GamepadAxis::LeftStickX), Control::Yaw, AxisShape::STICK),
        MappedBinding::axis(
            pad(GamepadAxis::RightStickX),
            Control::Roll,
            AxisShape::STICK,
        ),
        pitch_stick(pad(GamepadAxis::RightStickY)),
    ]
}

/// The common HID order: axis 0 roll, axis 1 pitch (forward reads positive),
/// axis 2 a throttle lever whose forward end reads −1, axis 3 twist rudder.
fn hotas() -> Vec<MappedBinding> {
    let lever = AxisShape::LEVER.inverted();
    vec![
        MappedBinding::axis(AxisSource::Raw(0), Control::Roll, AxisShape::STICK),
        pitch_stick(AxisSource::Raw(1)),
        MappedBinding::axis(AxisSource::Raw(3), Control::Yaw, AxisShape::STICK),
        MappedBinding::axis(AxisSource::Raw(2), Control::Throttle, lever),
    ]
}

fn shoulder_throttle_buttons() -> Vec<ButtonBinding> {
    vec![
        ButtonBinding {
            button: GamepadButton::RightShoulder,
            action: A::ThrottleUp,
        },
        ButtonBinding {
            button: GamepadButton::LeftShoulder,
            action: A::ThrottleDown,
        },
    ]
}

/// Arrow keys fly (Down is stick back), A/D yaw, W/S throttle, B brakes,
/// Home resets trim.
fn keys() -> Vec<KeyBinding> {
    vec![
        KeyBinding::new("ArrowLeft", A::RollLeft),
        KeyBinding::new("ArrowRight", A::RollRight),
        KeyBinding::new("ArrowDown", A::PitchUp),
        KeyBinding::new("ArrowUp", A::PitchDown),
        KeyBinding::new("KeyA", A::YawLeft),
        KeyBinding::new("KeyD", A::YawRight),
        KeyBinding::new("KeyW", A::ThrottleUp),
        KeyBinding::new("KeyS", A::ThrottleDown),
        KeyBinding::new("KeyB", A::WheelBrake),
        KeyBinding::new("Home", A::ResetTrim),
    ]
}
