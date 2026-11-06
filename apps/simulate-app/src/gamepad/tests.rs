use gilrs::{Axis, Button};
use sim_input::GamepadAxis;

use super::{
    HID_BUTTON_6, HID_BUTTON_7, HID_BUTTON_8, HID_BUTTON_12, HID_CONSUMER_MENU, HID_GD_MAIN_MENU,
    HID_SIM_ACCELERATOR, HID_SIM_BRAKE, VENDOR_MICROSOFT, menu_input, pad_button,
    resolve_gamepad_button, resolve_trigger_axis, resolve_z_trigger_axis,
};
use crate::controls::PadButton;
use crate::esc_menu::MenuInput;

#[test]
fn macos_hid_simulation_usages_name_the_physical_trigger() {
    assert_eq!(HID_SIM_BRAKE, 0x0002_00C5);
    assert_eq!(HID_SIM_ACCELERATOR, 0x0002_00C4);
    assert_eq!(
        resolve_trigger_axis(Button::RightTrigger2, HID_SIM_BRAKE, true),
        Some(GamepadAxis::LeftTrigger)
    );
    assert_eq!(
        resolve_trigger_axis(Button::LeftTrigger2, HID_SIM_ACCELERATOR, true),
        Some(GamepadAxis::RightTrigger)
    );
    assert_eq!(
        resolve_trigger_axis(Button::RightTrigger2, HID_SIM_BRAKE, false),
        Some(GamepadAxis::RightTrigger)
    );
}

#[test]
fn trigger_names_are_the_fallback() {
    let generic_z = 0x0001_0032;
    assert_eq!(
        resolve_trigger_axis(Button::LeftTrigger2, generic_z, true),
        Some(GamepadAxis::LeftTrigger)
    );
    assert_eq!(
        resolve_trigger_axis(Button::RightTrigger2, 5, false),
        Some(GamepadAxis::RightTrigger)
    );
    assert_eq!(resolve_trigger_axis(Button::South, generic_z, true), None);
    assert_eq!(
        resolve_trigger_axis(Button::LeftTrigger, generic_z, true),
        None,
        "bumpers are not analog triggers"
    );
}

#[test]
fn z_axes_resolve_to_triggers_by_usage_then_name() {
    assert_eq!(
        resolve_z_trigger_axis(Axis::RightZ, HID_SIM_BRAKE, true),
        Some(GamepadAxis::LeftTrigger)
    );
    assert_eq!(
        resolve_z_trigger_axis(Axis::LeftZ, HID_SIM_ACCELERATOR, true),
        Some(GamepadAxis::RightTrigger)
    );
    assert_eq!(
        resolve_z_trigger_axis(Axis::LeftZ, 2, false),
        Some(GamepadAxis::LeftTrigger)
    );
    assert_eq!(
        resolve_z_trigger_axis(Axis::RightZ, 5, false),
        Some(GamepadAxis::RightTrigger)
    );
    assert_eq!(
        resolve_z_trigger_axis(Axis::LeftStickX, HID_SIM_BRAKE, true),
        None
    );
}

#[test]
fn macos_ble_xbox_0b22_normalizes_menu_bumpers_and_sticks() {
    let ms = Some(VENDOR_MICROSOFT);
    let ble_series = Some(0x0B22);

    // Physical Menu (0x0009_000C) arrives as Button::Unknown on 0x0B22 v0x0513 -> Start -> Toggle!
    let menu_btn =
        resolve_gamepad_button(Button::Unknown, HID_BUTTON_12, ms, ble_series, true, true);
    assert_eq!(menu_btn, Button::Start);
    assert_eq!(menu_input(menu_btn, false), Some(MenuInput::Toggle));
    assert_eq!(menu_input(menu_btn, true), Some(MenuInput::Toggle));

    // Physical LB (0x0009_0007) arrives as Button::Start on 0x0B22 -> LeftTrigger (bumper), NOT Toggle!
    let lb = resolve_gamepad_button(Button::Start, HID_BUTTON_7, ms, ble_series, true, true);
    assert_eq!(lb, Button::LeftTrigger);
    assert_eq!(pad_button(lb), Some(PadButton::LeftBumper));
    assert_eq!(menu_input(lb, false), None);
    assert_eq!(menu_input(lb, true), Some(MenuInput::PrevTab));

    // Physical RB (0x0009_0008) arrives as Button::LeftThumb on 0x0B22 -> RightTrigger (bumper).
    let rb = resolve_gamepad_button(Button::LeftThumb, HID_BUTTON_8, ms, ble_series, true, true);
    assert_eq!(rb, Button::RightTrigger);
    assert_eq!(pad_button(rb), Some(PadButton::RightBumper));
    assert_eq!(menu_input(rb, false), None);
    assert_eq!(menu_input(rb, true), Some(MenuInput::NextTab));

    // Physical View (0x0B), X (0x04), Y (0x05), LS (0x0E), RS (0x0F).
    assert_eq!(
        resolve_gamepad_button(
            Button::RightTrigger,
            0x0009_000B,
            ms,
            ble_series,
            true,
            true
        ),
        Button::Select
    );
    assert_eq!(
        resolve_gamepad_button(Button::North, 0x0009_0004, ms, ble_series, true, true),
        Button::West
    );
    assert_eq!(
        resolve_gamepad_button(Button::Select, 0x0009_0005, ms, ble_series, true, true),
        Button::North
    );
    assert_eq!(
        resolve_gamepad_button(Button::Unknown, 0x0009_000E, ms, ble_series, true, true),
        Button::LeftThumb
    );
    assert_eq!(
        resolve_gamepad_button(Button::Unknown, 0x0009_000F, ms, ble_series, true, true),
        Button::RightThumb
    );
}

#[test]
fn usb_xbox_and_generic_macos_menu_usages_remain_intact() {
    let ms = Some(VENDOR_MICROSOFT);
    let usb_360 = Some(0x028E);

    // USB Xbox pads use 0x07/0x08 for stick clicks and 0x05/0x06 for bumpers; keep them untouched.
    assert_eq!(
        resolve_gamepad_button(Button::LeftThumb, HID_BUTTON_7, ms, usb_360, true, true),
        Button::LeftThumb
    );
    assert_eq!(
        pad_button(resolve_gamepad_button(
            Button::LeftThumb,
            HID_BUTTON_7,
            ms,
            usb_360,
            true,
            true,
        )),
        None
    );
    assert_eq!(
        pad_button(resolve_gamepad_button(
            Button::LeftTrigger,
            0x0009_0005,
            ms,
            usb_360,
            true,
            true,
        )),
        Some(PadButton::LeftBumper)
    );

    // Generic macOS pads reporting System Main Menu / Consumer Menu toggle the hub.
    let sys_menu =
        resolve_gamepad_button(Button::Unknown, HID_GD_MAIN_MENU, None, None, true, true);
    assert_eq!(menu_input(sys_menu, false), Some(MenuInput::Toggle));
    let consumer_menu =
        resolve_gamepad_button(Button::Unknown, HID_CONSUMER_MENU, None, None, true, true);
    assert_eq!(menu_input(consumer_menu, true), Some(MenuInput::Toggle));

    // Unmapped generic macOS HID pads rescue Button 6 (0x0009_0006 -> RightBumper).
    assert_eq!(
        pad_button(resolve_gamepad_button(
            Button::Unknown,
            HID_BUTTON_6,
            None,
            None,
            false,
            true,
        )),
        Some(PadButton::RightBumper)
    );
}
