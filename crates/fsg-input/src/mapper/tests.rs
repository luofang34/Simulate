use super::*;
use crate::{AxisShape, AxisSource, DeviceInfo, DeviceProfile, GamepadAxis, Layout};

fn device(id: u32, layout: Layout, usb: Option<(u16, u16)>) -> DeviceInfo {
    DeviceInfo {
        id: DeviceId(id),
        name: format!("device {id}"),
        vendor_id: usb.map(|u| u.0),
        product_id: usb.map(|u| u.1),
        layout,
    }
}

fn axis(id: u32, source: AxisSource, value: f32) -> RawInput {
    RawInput::Axis {
        device: DeviceId(id),
        source,
        value,
    }
}

#[test]
fn standard_gamepad_flies_with_stick_back_as_nose_up() {
    let mut mapper = InputMapper::new(ProfileLibrary::default());
    mapper.handle(&RawInput::Connected(device(1, Layout::Standard, None)));
    mapper.handle(&axis(1, AxisSource::Gamepad(GamepadAxis::LeftStickY), -1.0));
    mapper.handle(&axis(
        1,
        AxisSource::Gamepad(GamepadAxis::RightTrigger),
        1.0,
    ));
    let controls = mapper.controls();
    assert!((controls.pitch - 1.0).abs() < 1e-12);
    assert!((controls.throttle - 1.0).abs() < 1e-12);
}

#[test]
fn yoke_throttle_lever_holds_mid_travel() {
    let mut mapper = InputMapper::new(ProfileLibrary::default());
    mapper.handle(&RawInput::Connected(device(2, Layout::NonStandard, None)));
    mapper.handle(&axis(2, AxisSource::Raw(2), 0.0));
    assert!((mapper.controls().throttle - 0.5).abs() < 1e-12);
    mapper.handle(&axis(2, AxisSource::Raw(2), -1.0));
    assert!((mapper.controls().throttle - 1.0).abs() < 1e-12);
}

#[test]
fn usb_id_profile_beats_the_fallback() {
    let rudder_pedals = DeviceProfile {
        name: "test pedals".to_owned(),
        usb_id: Some((0x1234, 0x5678)),
        bindings: vec![AxisBinding {
            source: AxisSource::Raw(0),
            control: Control::Yaw,
            shape: AxisShape::STICK,
        }],
    };
    let mut mapper = InputMapper::new(ProfileLibrary::with_profiles(vec![rudder_pedals]));
    mapper.handle(&RawInput::Connected(device(
        3,
        Layout::NonStandard,
        Some((0x1234, 0x5678)),
    )));
    mapper.handle(&axis(3, AxisSource::Raw(0), 1.0));
    let controls = mapper.controls();
    assert!((controls.yaw - 1.0).abs() < 1e-12);
    assert_eq!(controls.roll, 0.0);
}

#[test]
fn last_moved_device_wins_and_unknown_devices_are_ignored() {
    let mut mapper = InputMapper::new(ProfileLibrary::default());
    mapper.handle(&RawInput::Connected(device(1, Layout::Standard, None)));
    mapper.handle(&RawInput::Connected(device(2, Layout::NonStandard, None)));
    mapper.handle(&axis(1, AxisSource::Gamepad(GamepadAxis::LeftStickX), 1.0));
    mapper.handle(&axis(2, AxisSource::Raw(0), -1.0));
    assert!((mapper.controls().roll + 1.0).abs() < 1e-12);
    mapper.handle(&axis(9, AxisSource::Raw(0), 1.0));
    assert!((mapper.controls().roll + 1.0).abs() < 1e-12);
    mapper.handle(&RawInput::Disconnected(DeviceId(2)));
    assert!(!mapper.is_bound(DeviceId(2)));
}
