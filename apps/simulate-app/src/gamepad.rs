use gilrs::{Axis, Button, EventType, GamepadId, Gilrs, MappingSource};
use sim_input::{AxisSource, GamepadAxis, InputMapper, Layout};
use tracing::{info, warn};

use crate::controls::{ControlScheme, PadButton};
use crate::esc_menu::MenuInput;

/// Offset for gilrs `Unknown` axes so their raw codes cannot collide with the
/// six indices assigned to gilrs's named axes.
const UNKNOWN_AXIS_BASE: u32 = 16;
/// macOS IOKit code (`page << 16 | usage`) of HID Simulation `Brake`, the physical left
/// trigger of Bluetooth Xbox-layout pads.
const HID_SIM_BRAKE: u32 = 0x0002_00C5;
/// macOS IOKit code of HID Simulation `Accelerator`, the physical right trigger.
const HID_SIM_ACCELERATOR: u32 = 0x0002_00C4;
/// Microsoft USB/Bluetooth vendor ID (`0x045E`).
const VENDOR_MICROSOFT: u16 = 0x045E;
/// macOS IOKit code (`page << 16 | usage`) of HID Button 6 (`0x0009_0006`).
const HID_BUTTON_6: u32 = 0x0009_0006;
/// macOS IOKit code of HID Button 7 (`0x0009_0007`), physical `LB` on Bluetooth Xbox pads.
const HID_BUTTON_7: u32 = 0x0009_0007;
/// macOS IOKit code of HID Button 8 (`0x0009_0008`), physical `RB` on Bluetooth Xbox pads.
const HID_BUTTON_8: u32 = 0x0009_0008;
/// macOS IOKit code of HID Button 12 (`0x0009_000C`), physical `Menu` on Bluetooth Xbox pads.
const HID_BUTTON_12: u32 = 0x0009_000C;
/// macOS IOKit GenericDesktop `Start` (`0x0001_003D`) and `System Main Menu` (`0x0001_0085`),
/// plus Consumer `Menu` (`0x000C_0040`) and `AC Home` (`0x000C_0223`).
const HID_GD_START: u32 = 0x0001_003D;
const HID_GD_MAIN_MENU: u32 = 0x0001_0085;
const HID_CONSUMER_MENU: u32 = 0x000C_0040;
const HID_CONSUMER_AC_HOME: u32 = 0x000C_0223;

/// Reads gamepads, joysticks, and yokes through gilrs (OS HID natively, the
/// Gamepad API in the browser) and feeds them to the input mapper.
pub struct Gamepads {
    gilrs: Option<Gilrs>,
    /// Whether the pad reports its analog triggers as `*Trigger2` buttons, in which case
    /// its `LeftZ` / `RightZ` axes are not triggers.
    button_triggers: bool,
}

impl Gamepads {
    pub fn new() -> Self {
        match Gilrs::new() {
            Ok(gilrs) => Self {
                gilrs: Some(gilrs),
                button_triggers: false,
            },
            Err(err) => {
                warn!(%err, "controller input unavailable; keyboard only");
                Self {
                    gilrs: None,
                    button_triggers: false,
                }
            }
        }
    }

    /// Drains pending controller events through `scheme`: its control writes reach
    /// `mapper` unless the hub is open (they still drive the hub's live telemetry), and
    /// discrete hub button presses are returned in order.
    pub fn poll(
        &mut self,
        scheme: &mut ControlScheme,
        mapper: &mut InputMapper,
        menu_open: bool,
    ) -> Vec<MenuInput> {
        let mut menu = Vec::new();
        let Some(gilrs) = self.gilrs.as_mut() else {
            return menu;
        };
        while let Some(event) = gilrs.next_event() {
            let (vendor_id, product_id, standard) = {
                let pad = gilrs.gamepad(event.id);
                (
                    pad.vendor_id(),
                    pad.product_id(),
                    !matches!(pad.mapping_source(), MappingSource::None),
                )
            };
            let macos = cfg!(target_os = "macos");
            match event.event {
                EventType::Connected => {
                    let name = connected_name(gilrs, event.id);
                    scheme.set_controller(Some(name));
                }
                EventType::Disconnected => {
                    info!("controller disconnected");
                    scheme.set_controller(None);
                    self.button_triggers = false;
                }
                EventType::ButtonPressed(raw_btn, code) => {
                    let button = resolve_gamepad_button(
                        raw_btn,
                        code.into_u32(),
                        vendor_id,
                        product_id,
                        standard,
                        macos,
                    );
                    if let Some(input) = menu_input(button, menu_open) {
                        menu.push(input);
                    } else if button == Button::RightThumb && !menu_open {
                        scheme.request_look_reset();
                    }
                }
                _ => {}
            }
            if scheme.controller().is_none() && !matches!(event.event, EventType::Disconnected) {
                let name = connected_name(gilrs, event.id);
                scheme.set_controller(Some(name));
            }
            if let Some((button, pressed)) =
                bumper_event(event.event, vendor_id, product_id, standard, macos)
                && !menu_open
            {
                scheme.handle_button(button, pressed);
            }
            if let Some((source, value)) =
                translate(gilrs, event.id, event.event, &mut self.button_triggers)
            {
                for (control, value) in scheme.handle_axis(source, value) {
                    if !menu_open {
                        mapper.set(control, value);
                    }
                }
            }
        }
        menu
    }
}

/// The hub input a gamepad button press means: `Start` toggles the hub, the D-pad moves,
/// `South` confirms, `East` backs out, and while the hub is open the bumpers switch tabs.
#[must_use]
pub const fn menu_input(button: Button, menu_open: bool) -> Option<MenuInput> {
    match button {
        Button::LeftTrigger if menu_open => Some(MenuInput::PrevTab),
        Button::RightTrigger if menu_open => Some(MenuInput::NextTab),
        Button::Start => Some(MenuInput::Toggle),
        Button::DPadUp => Some(MenuInput::Up),
        Button::DPadDown => Some(MenuInput::Down),
        Button::DPadLeft => Some(MenuInput::Left),
        Button::DPadRight => Some(MenuInput::Right),
        Button::South => Some(MenuInput::Confirm),
        Button::East => Some(MenuInput::Back),
        _ => None,
    }
}

fn layout(gilrs: &Gilrs, id: GamepadId) -> Layout {
    match gilrs.gamepad(id).mapping_source() {
        MappingSource::None => Layout::NonStandard,
        MappingSource::SdlMappings | MappingSource::Driver => Layout::Standard,
    }
}

fn connected_name(gilrs: &Gilrs, id: GamepadId) -> String {
    let pad = gilrs.gamepad(id);
    let name = pad.name().to_owned();
    info!(name = %name, vendor = ?pad.vendor_id(), product = ?pad.product_id(), layout = ?layout(gilrs, id), "controller connected");
    name
}

/// Whether `(vendor_id, product_id)` is a Microsoft Bluetooth / BLE Xbox controller on macOS
/// whose HID Gamepad descriptor numbers buttons by the USB-IF Gamepad spec (`0x0009_0001..0x0F`,
/// omitting `0x03`, `0x06`, `0x09`, `0x0A`) rather than the 11-button USB Xbox 360 layout.
const fn is_macos_hid_xbox(vendor_id: Option<u16>, product_id: Option<u16>) -> bool {
    matches!(vendor_id, Some(VENDOR_MICROSOFT))
        && matches!(
            product_id,
            Some(0x02E0 | 0x02FD | 0x0B05 | 0x0B12 | 0x0B13 | 0x0B20 | 0x0B22)
        )
}

/// Normalizes a gilrs `Button` and raw OS code into the physical semantic `Button`.
///
/// On macOS IOKit, Bluetooth / BLE Xbox controllers (`0x045E:0x0B22` etc.) declare HID Button
/// usages `1, 2, 4, 5, 7, 8, 11, 12, 13, 14, 15` (omitting `3, 6, 9, 10`). Because `gilrs-core`
/// compacts its button array while SDL's macOS mapping indexes compacted slots mismatched to BLE
/// revisions (for example `0x0B22 v0x0513` maps `LB` `0x07` -> `Start`, `RB` `0x08` -> `LeftThumb`,
/// `View` `0x0B` -> `RightTrigger`, and `Menu` `0x0C` -> `paddle1` / `Button::Unknown`), reading
/// the IOKit HID usage directly restores every physical button (`A`, `B`, `X`, `Y`, `LB`, `RB`,
/// `View`, `Menu`, `Guide`, `LS`, `RS`) without affecting USB Xbox controllers (`0x028E`, `0x02EA`,
/// etc.). GenericDesktop / Consumer menu usages and unmapped `0x0009_0006` are also normalized.
#[must_use]
pub const fn resolve_gamepad_button(
    button: Button,
    raw_code: u32,
    vendor_id: Option<u16>,
    product_id: Option<u16>,
    standard: bool,
    is_macos: bool,
) -> Button {
    if !is_macos {
        return button;
    }
    if is_macos_hid_xbox(vendor_id, product_id) {
        return match raw_code {
            0x0009_0001 => Button::South,
            0x0009_0002 => Button::East,
            0x0009_0004 => Button::West,
            0x0009_0005 => Button::North,
            HID_BUTTON_7 => Button::LeftTrigger,
            HID_BUTTON_8 => Button::RightTrigger,
            0x0009_000B => Button::Select,
            HID_BUTTON_12 => Button::Start,
            0x0009_000D => Button::Mode,
            0x0009_000E => Button::LeftThumb,
            0x0009_000F => Button::RightThumb,
            0x0009_0003 | HID_BUTTON_6 | 0x0009_0009 | 0x0009_000A => Button::Unknown,
            _ => resolve_generic_macos_button(button, raw_code, standard),
        };
    }
    if matches!(vendor_id, Some(VENDOR_MICROSOFT)) {
        match (button, raw_code) {
            (Button::North | Button::Unknown, HID_BUTTON_7) => return Button::LeftTrigger,
            (Button::Unknown, HID_BUTTON_8) => return Button::RightTrigger,
            (Button::Unknown, HID_BUTTON_12) => return Button::Start,
            _ => {}
        }
    }
    resolve_generic_macos_button(button, raw_code, standard)
}

const fn resolve_generic_macos_button(button: Button, raw_code: u32, standard: bool) -> Button {
    if matches!(button, Button::Unknown) {
        match raw_code {
            HID_GD_START | HID_GD_MAIN_MENU | HID_CONSUMER_MENU | HID_CONSUMER_AC_HOME => {
                return Button::Start;
            }
            HID_BUTTON_6 if !standard => return Button::RightTrigger,
            _ => {}
        }
    }
    button
}

/// Maps a normalized semantic [`Button`] to a shoulder bumper [`PadButton`].
#[must_use]
pub const fn pad_button(button: Button) -> Option<PadButton> {
    match button {
        Button::LeftTrigger => Some(PadButton::LeftBumper),
        Button::RightTrigger => Some(PadButton::RightBumper),
        _ => None,
    }
}

fn bumper_event(
    event: EventType,
    vendor_id: Option<u16>,
    product_id: Option<u16>,
    standard: bool,
    is_macos: bool,
) -> Option<(PadButton, bool)> {
    match event {
        EventType::ButtonPressed(button, code) => Some((
            pad_button(resolve_gamepad_button(
                button,
                code.into_u32(),
                vendor_id,
                product_id,
                standard,
                is_macos,
            ))?,
            true,
        )),
        EventType::ButtonReleased(button, code) => Some((
            pad_button(resolve_gamepad_button(
                button,
                code.into_u32(),
                vendor_id,
                product_id,
                standard,
                is_macos,
            ))?,
            false,
        )),
        _ => None,
    }
}

fn translate(
    gilrs: &Gilrs,
    id: GamepadId,
    event: EventType,
    button_triggers: &mut bool,
) -> Option<(AxisSource, f32)> {
    let standard = layout(gilrs, id) == Layout::Standard;
    let macos = cfg!(target_os = "macos");
    match event {
        EventType::AxisChanged(axis, value, code) => {
            let raw = code.into_u32();
            if standard
                && !*button_triggers
                && let Some(trigger) = resolve_z_trigger_axis(axis, raw, macos)
            {
                let pull = ((value + 1.0) * 0.5).clamp(0.0, 1.0);
                return Some((AxisSource::Gamepad(trigger), pull));
            }
            Some((axis_source(axis, raw, standard)?, value))
        }
        EventType::ButtonChanged(button, value, code) => {
            let trigger = resolve_trigger_axis(button, code.into_u32(), macos)?;
            *button_triggers = true;
            Some((AxisSource::Gamepad(trigger), value))
        }
        _ => None,
    }
}

/// The physical analog trigger behind a gilrs `ButtonChanged`.
///
/// gilrs on macOS orders a pad's axes by HID usage, so Bluetooth Xbox-layout pads, which
/// declare the left trigger as Simulation `Brake` (`0xC5`) and the right as `Accelerator`
/// (`0xC4`), reach their SDL mapping swapped and report `LeftTrigger2` for the right
/// trigger. The IOKit usage names the physical trigger, so it wins over gilrs's name there.
#[must_use]
pub const fn resolve_trigger_axis(
    button: Button,
    raw_code: u32,
    is_macos: bool,
) -> Option<GamepadAxis> {
    if let Some(trigger) = hid_simulation_trigger(raw_code, is_macos) {
        return Some(trigger);
    }
    match button {
        Button::LeftTrigger2 => Some(GamepadAxis::LeftTrigger),
        Button::RightTrigger2 => Some(GamepadAxis::RightTrigger),
        _ => None,
    }
}

/// The physical analog trigger behind a standard-layout `LeftZ` / `RightZ` axis (pads whose
/// mapping names triggers as Z axes report them in `-1..=1`, released at `-1`).
#[must_use]
pub const fn resolve_z_trigger_axis(
    axis: Axis,
    raw_code: u32,
    is_macos: bool,
) -> Option<GamepadAxis> {
    if !matches!(axis, Axis::LeftZ | Axis::RightZ) {
        return None;
    }
    if let Some(trigger) = hid_simulation_trigger(raw_code, is_macos) {
        return Some(trigger);
    }
    match axis {
        Axis::LeftZ => Some(GamepadAxis::LeftTrigger),
        _ => Some(GamepadAxis::RightTrigger),
    }
}

const fn hid_simulation_trigger(raw_code: u32, is_macos: bool) -> Option<GamepadAxis> {
    match raw_code {
        HID_SIM_BRAKE if is_macos => Some(GamepadAxis::LeftTrigger),
        HID_SIM_ACCELERATOR if is_macos => Some(GamepadAxis::RightTrigger),
        _ => None,
    }
}

/// Named gilrs axes become semantic axes on standard-layout pads. On other
/// devices gilrs's names carry no meaning, so they are numbered in HID order
/// (X, Y, Z, Rx, Ry, Rz), the order the generic joystick profile assumes.
fn axis_source(axis: Axis, code: u32, standard: bool) -> Option<AxisSource> {
    if standard {
        let semantic = match axis {
            Axis::LeftStickX => GamepadAxis::LeftStickX,
            Axis::LeftStickY => GamepadAxis::LeftStickY,
            Axis::RightStickX => GamepadAxis::RightStickX,
            Axis::RightStickY => GamepadAxis::RightStickY,
            _ => return None,
        };
        return Some(AxisSource::Gamepad(semantic));
    }
    let index = match axis {
        Axis::LeftStickX => 0,
        Axis::LeftStickY => 1,
        Axis::LeftZ => 2,
        Axis::RightStickX => 3,
        Axis::RightStickY => 4,
        Axis::RightZ => 5,
        Axis::DPadX | Axis::DPadY => return None,
        Axis::Unknown => UNKNOWN_AXIS_BASE.saturating_add(code),
    };
    Some(AxisSource::Raw(index))
}

#[cfg(test)]
mod tests;
