//! Built-in control presets: which device axis and button drives which pilot control.

use sim_input::{AxisShape, AxisSource, Control, GamepadAxis, InputRange, OutputRange};

use crate::controls::{AxisBind, BindKind, ButtonBind, PadButton};

/// Throttle travel per second at full deflection of a rate (latching) throttle input.
pub const THROTTLE_RATE_PER_S: f64 = 0.5;

/// Vehicle class a preset is tuned for; picks the automatic default preset.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum VehicleClass {
    /// Aeroplanes flown with yoke, rudder, and throttle lever.
    FixedWing,
    /// Multirotor drones flown in climb-rate / attitude modes.
    Multirotor,
    /// Spacecraft flown by scripted guidance.
    Spacecraft,
}

/// A complete stick, trigger, and button layout.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum ControlPreset {
    /// Camera-drone Mode 2: left stick climb (centred `0.5`) and yaw, right stick roll/pitch.
    QuadCamDrone,
    /// FPV acro Mode 2: left stick direct throttle (bottom `0`) and yaw, right stick roll/pitch.
    QuadFpvAcro,
    /// Fixed-wing gamepad: left stick yoke, triggers rudder, bumpers latching throttle.
    FixedWingGamepad,
    /// Fixed-wing Mode 2: left stick and bumpers latching throttle, left stick rudder, right stick yoke.
    FixedWingMode2,
    /// Flight stick or yoke with HOTAS: X/Y roll/pitch, Rz yaw, slider throttle.
    StickHotas,
}

impl ControlPreset {
    /// Every preset in selector order.
    pub const ALL: [Self; 5] = [
        Self::QuadCamDrone,
        Self::QuadFpvAcro,
        Self::FixedWingGamepad,
        Self::FixedWingMode2,
        Self::StickHotas,
    ];

    /// Automatic preset for `class`.
    #[must_use]
    pub const fn default_for(class: VehicleClass) -> Self {
        match class {
            VehicleClass::Multirotor => Self::QuadCamDrone,
            VehicleClass::FixedWing | VehicleClass::Spacecraft => Self::FixedWingGamepad,
        }
    }

    /// Display name.
    #[must_use]
    pub const fn title(self) -> &'static str {
        match self {
            Self::QuadCamDrone => "Quad Cam-Drone Mode 2",
            Self::QuadFpvAcro => "Quad FPV Acro Mode 2",
            Self::FixedWingGamepad => "Fixed-Wing Gamepad",
            Self::FixedWingMode2 => "Fixed-Wing Mode 2 Dual-Stick",
            Self::StickHotas => "Flight Stick / Yoke - HOTAS",
        }
    }

    /// One-line summary of the layout.
    #[must_use]
    pub const fn summary(self) -> &'static str {
        match self {
            Self::QuadCamDrone => "LS climb/yaw  RS roll/pitch",
            Self::QuadFpvAcro => "LS throttle/yaw  RS roll/pitch",
            Self::FixedWingGamepad => "LS yoke  LT/RT rudder  LB/RB throttle",
            Self::FixedWingMode2 => "LS/LB/RB throttle  LS rudder  RS yoke",
            Self::StickHotas => "X/Y roll/pitch  Rz yaw  slider throttle",
        }
    }

    /// The neighbouring preset, cycling.
    #[must_use]
    pub fn step(self, forward: bool) -> Self {
        let count = Self::ALL.len();
        let index = Self::ALL.iter().position(|&p| p == self).unwrap_or(0);
        let next = if forward {
            index.wrapping_add(1) % count
        } else {
            index.wrapping_add(count.saturating_sub(1)) % count
        };
        Self::ALL.get(next).copied().unwrap_or(self)
    }

    /// Whether the gamepad layout leaves the right stick for camera look.
    #[must_use]
    pub const fn frees_right_stick(self) -> bool {
        matches!(self, Self::FixedWingGamepad)
    }

    /// Axis bindings: the preset's gamepad layout plus the HID joystick/yoke layout, so a
    /// yoke keeps working whichever gamepad preset is chosen.
    #[must_use]
    pub fn axis_binds(self) -> Vec<AxisBind> {
        let mut binds = match self {
            Self::QuadCamDrone => mode2_sticks(climb_lever()),
            Self::QuadFpvAcro => mode2_sticks(direct_lever()),
            Self::FixedWingGamepad => fixed_wing_gamepad(),
            Self::FixedWingMode2 => {
                let mut b = right_stick_yoke();
                b.push(abs(pad(GamepadAxis::LeftStickX), Control::Yaw, stick()));
                b.push(AxisBind {
                    source: pad(GamepadAxis::LeftStickY),
                    control: Control::Throttle,
                    kind: BindKind::Rate,
                    shape: stick(),
                });
                b
            }
            Self::StickHotas => Vec::new(),
        };
        binds.extend(hid_joystick(self == Self::StickHotas));
        binds
    }

    /// Button bindings (latching throttle on the bumpers across all fixed-wing presets).
    #[must_use]
    pub fn button_binds(self) -> Vec<ButtonBind> {
        match self {
            Self::FixedWingGamepad | Self::FixedWingMode2 => vec![
                ButtonBind {
                    button: PadButton::RightBumper,
                    control: Control::Throttle,
                    rate: 1.0,
                },
                ButtonBind {
                    button: PadButton::LeftBumper,
                    control: Control::Throttle,
                    rate: -1.0,
                },
            ],
            _ => Vec::new(),
        }
    }
}

const fn pad(axis: GamepadAxis) -> AxisSource {
    AxisSource::Gamepad(axis)
}

const fn stick() -> AxisShape {
    AxisShape::STICK
}

const fn abs(source: AxisSource, control: Control, shape: AxisShape) -> AxisBind {
    AxisBind {
        source,
        control,
        kind: BindKind::Absolute,
        shape,
    }
}

/// A self-centring stick as a lever: released reads `0.5` (hover / hold altitude); the
/// standard layout reports stick-up as positive, which is more throttle.
const fn climb_lever() -> AxisShape {
    AxisShape {
        invert: false,
        deadzone: 0.0,
        expo: 0.0,
        input: InputRange::Bipolar,
        output: OutputRange::Lever,
    }
}

/// A non-centring throttle stick: bottom `0`, top `1`.
const fn direct_lever() -> AxisShape {
    climb_lever()
}

fn right_stick_yoke() -> Vec<AxisBind> {
    vec![
        abs(pad(GamepadAxis::RightStickX), Control::Roll, stick()),
        abs(
            pad(GamepadAxis::RightStickY),
            Control::Pitch,
            stick().inverted(),
        ),
    ]
}

fn mode2_sticks(throttle: AxisShape) -> Vec<AxisBind> {
    let mut binds = right_stick_yoke();
    binds.push(abs(pad(GamepadAxis::LeftStickX), Control::Yaw, stick()));
    binds.push(abs(
        pad(GamepadAxis::LeftStickY),
        Control::Throttle,
        throttle,
    ));
    binds
}

fn fixed_wing_gamepad() -> Vec<AxisBind> {
    let trigger = AxisShape {
        input: InputRange::Unipolar,
        ..AxisShape::LEVER
    };
    vec![
        abs(pad(GamepadAxis::LeftStickX), Control::Roll, stick()),
        abs(
            pad(GamepadAxis::LeftStickY),
            Control::Pitch,
            stick().inverted(),
        ),
        AxisBind {
            source: pad(GamepadAxis::RightTrigger),
            control: Control::Yaw,
            kind: BindKind::Additive(1.0),
            shape: trigger,
        },
        AxisBind {
            source: pad(GamepadAxis::LeftTrigger),
            control: Control::Yaw,
            kind: BindKind::Additive(-1.0),
            shape: trigger,
        },
    ]
}

/// HID order X, Y, Z, Rx, Ry, Rz: X roll, Y pitch (forward reads positive), Z a throttle
/// lever whose forward end reads −1, and with `hotas` Rz twist yaw.
fn hid_joystick(hotas: bool) -> Vec<AxisBind> {
    let mut binds = vec![
        abs(AxisSource::Raw(0), Control::Roll, stick()),
        abs(AxisSource::Raw(1), Control::Pitch, stick().inverted()),
        abs(
            AxisSource::Raw(2),
            Control::Throttle,
            AxisShape::LEVER.inverted(),
        ),
    ];
    if hotas {
        binds.push(abs(AxisSource::Raw(5), Control::Yaw, stick()));
    }
    binds
}
