use super::*;
use crate::{AxisBinding, AxisShape, AxisSource, DeviceInfo, DeviceProfile, GamepadAxis, Layout};

mod arbitration;
mod capture;
mod differential;
mod presets;

pub(super) const DT: f64 = 0.01;

pub(super) fn pad(axis: GamepadAxis) -> AxisSource {
    AxisSource::Gamepad(axis)
}

pub(super) fn approx(actual: f64, expected: f64) {
    assert!(
        (actual - expected).abs() < 1e-9,
        "expected {expected}, got {actual}"
    );
}

/// Runs `seconds` of ticks at `DT`.
pub(super) fn run(mapper: &mut InputMapper, seconds: f64) {
    for _ in 0..(seconds / DT).round() as usize {
        mapper.tick(DT);
    }
}

/// A mapper with one standard gamepad (id 1) connected under `preset`.
pub(super) fn gamepad_mapper(preset: crate::ControlPresetId) -> InputMapper {
    let mut mapper = InputMapper::new(ProfileLibrary::default());
    mapper.set_preset(preset);
    mapper.handle(&RawInput::Connected(device(1, Layout::Standard, None)));
    mapper
}

pub(super) fn device(id: u32, layout: Layout, usb: Option<(u16, u16)>) -> DeviceInfo {
    DeviceInfo {
        id: DeviceId(id),
        name: format!("device {id}"),
        vendor_id: usb.map(|u| u.0),
        product_id: usb.map(|u| u.1),
        layout,
    }
}

pub(super) fn axis(id: u32, source: AxisSource, value: f32) -> RawInput {
    RawInput::Axis {
        device: DeviceId(id),
        source,
        value,
    }
}

#[test]
fn standard_gamepad_flies_with_stick_back_as_nose_up_and_triggers_as_rudder() {
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
    assert!((controls.yaw - 1.0).abs() < 1e-12);
    assert_eq!(
        controls.throttle, 0.0,
        "triggers drive rudder, not throttle"
    );
}

#[test]
fn direct_set_throttle_reads_back_without_tick() {
    let mut mapper = InputMapper::new(ProfileLibrary::default());
    mapper.set(Control::Throttle, 0.65);
    approx(mapper.controls().throttle, 0.65);
    let next = mapper.controls().throttle + 0.5 * 0.1;
    mapper.set(Control::Throttle, next);
    approx(mapper.controls().throttle, 0.7);
    mapper.set(Control::Throttle, 3.0);
    approx(mapper.controls().throttle, 1.0);
    mapper.set(Control::Roll, -1.0);
    approx(mapper.controls().roll, -1.0);
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
