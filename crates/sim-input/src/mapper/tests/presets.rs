use super::*;
use crate::{
    ControlPresetId, FlightKeyAction, GamepadButton, ThrottleBehavior, VehicleControlClass,
};

#[test]
fn quad_cam_drone_hovers_at_centre_and_maps_mode2_sticks() {
    let mut mapper = InputMapper::new(ProfileLibrary::default());
    mapper.set_vehicle_class(VehicleControlClass::Multirotor);
    assert_eq!(mapper.preset(), ControlPresetId::QuadCamDroneMode2);
    mapper.handle(&RawInput::Connected(device(1, Layout::Standard, None)));
    approx(mapper.controls().throttle, 0.5);
    mapper.handle(&axis(1, pad(GamepadAxis::LeftStickY), 0.0));
    approx(mapper.controls().throttle, 0.5);
    mapper.handle(&axis(1, pad(GamepadAxis::LeftStickY), 1.0));
    approx(mapper.controls().throttle, 1.0);
    mapper.handle(&axis(1, pad(GamepadAxis::LeftStickY), -1.0));
    approx(mapper.controls().throttle, 0.0);
    mapper.handle(&axis(1, pad(GamepadAxis::LeftStickY), 0.0));
    approx(mapper.controls().throttle, 0.5);

    mapper.handle(&axis(1, pad(GamepadAxis::LeftStickX), 1.0));
    mapper.handle(&axis(1, pad(GamepadAxis::RightStickX), -1.0));
    mapper.handle(&axis(1, pad(GamepadAxis::RightStickY), -1.0));
    let controls = mapper.controls();
    approx(controls.yaw, 1.0);
    approx(controls.roll, -1.0);
    approx(controls.pitch, 1.0);
    mapper.handle(&axis(1, pad(GamepadAxis::RightStickY), 1.0));
    approx(mapper.controls().pitch, -1.0);
}

#[test]
fn quad_keyboard_throttle_climbs_descends_and_returns_to_hover() {
    let mut mapper = InputMapper::new(ProfileLibrary::default());
    mapper.set_vehicle_class(VehicleControlClass::Multirotor);
    approx(mapper.controls().throttle, 0.5);
    mapper.set_key_action(FlightKeyAction::ThrottleUp, true);
    run(&mut mapper, 1.0);
    approx(mapper.controls().throttle, 1.0);
    mapper.set_key_action(FlightKeyAction::ThrottleUp, false);
    run(&mut mapper, 1.0);
    approx(mapper.controls().throttle, 0.5);
    mapper.set_key_action(FlightKeyAction::ThrottleDown, true);
    mapper.tick(0.1);
    assert!(mapper.controls().throttle < 0.5);
}

#[test]
fn fixed_wing_gamepad_holds_trim_and_flies_rudder_on_triggers() {
    let mut mapper = gamepad_mapper(ControlPresetId::FixedWingGamepad);
    mapper.set_trim_throttle(0.65);
    run(&mut mapper, 2.0);
    approx(mapper.controls().throttle, 0.65);
    mapper.handle(&axis(1, pad(GamepadAxis::LeftStickY), 0.01));
    mapper.handle(&axis(1, pad(GamepadAxis::LeftStickX), -0.01));
    run(&mut mapper, 2.0);
    approx(mapper.controls().throttle, 0.65);

    mapper.handle(&axis(1, pad(GamepadAxis::RightTrigger), 1.0));
    approx(mapper.controls().yaw, 1.0);
    mapper.handle(&axis(1, pad(GamepadAxis::LeftTrigger), 1.0));
    approx(mapper.controls().yaw, 0.0);
    mapper.handle(&axis(1, pad(GamepadAxis::RightTrigger), 0.0));
    approx(mapper.controls().yaw, -1.0);
    approx(mapper.controls().throttle, 0.65);
}

#[test]
fn fixed_wing_keyboard_throttle_latches() {
    let mut mapper = InputMapper::new(ProfileLibrary::default());
    mapper.set_trim_throttle(0.0);
    mapper.set_key_action(FlightKeyAction::ThrottleUp, true);
    run(&mut mapper, 1.0);
    approx(mapper.controls().throttle, 0.45);
    mapper.set_key_action(FlightKeyAction::ThrottleUp, false);
    run(&mut mapper, 1.0);
    approx(mapper.controls().throttle, 0.45);
    mapper.set_trim_throttle(0.65);
    mapper.set_key_action(FlightKeyAction::ThrottleDown, true);
    run(&mut mapper, 1.0);
    approx(mapper.controls().throttle, 0.2);
    mapper.set_key_action(FlightKeyAction::ResetTrim, true);
    approx(mapper.controls().throttle, 0.65);
}

#[test]
fn fixed_wing_mode2_left_stick_nudges_throttle_and_holds_on_release() {
    let mut mapper = gamepad_mapper(ControlPresetId::FixedWingMode2);
    mapper.set_trim_throttle(0.65);
    run(&mut mapper, 1.0);
    approx(mapper.controls().throttle, 0.65);
    mapper.handle(&axis(1, pad(GamepadAxis::LeftStickY), 1.0));
    run(&mut mapper, 0.5);
    approx(mapper.controls().throttle, 0.65 + 0.55 * 0.5);
    mapper.handle(&axis(1, pad(GamepadAxis::LeftStickY), 0.0));
    run(&mut mapper, 1.0);
    approx(mapper.controls().throttle, 0.65 + 0.55 * 0.5);
    mapper.handle(&axis(1, pad(GamepadAxis::LeftStickY), -1.0));
    run(&mut mapper, 0.5);
    approx(mapper.controls().throttle, 0.65);
    mapper.handle(&axis(1, pad(GamepadAxis::LeftStickY), 0.0));

    mapper.handle(&axis(1, pad(GamepadAxis::LeftStickX), 1.0));
    mapper.handle(&axis(1, pad(GamepadAxis::RightStickX), 1.0));
    mapper.handle(&axis(1, pad(GamepadAxis::RightStickY), -1.0));
    let controls = mapper.controls();
    approx(controls.yaw, 1.0);
    approx(controls.roll, 1.0);
    approx(controls.pitch, 1.0);

    // Shoulder bumpers also step throttle smoothly while LeftStickY rests at centre.
    assert_eq!(
        mapper.handle_button(DeviceId(1), GamepadButton::RightShoulder, true),
        Some(FlightKeyAction::ThrottleUp)
    );
    run(&mut mapper, 0.5);
    approx(mapper.controls().throttle, 0.65 + 0.55 * 0.5);
    mapper.handle_button(DeviceId(1), GamepadButton::RightShoulder, false);
    assert_eq!(
        mapper.handle_button(DeviceId(1), GamepadButton::LeftShoulder, true),
        Some(FlightKeyAction::ThrottleDown)
    );
    run(&mut mapper, 0.5);
    approx(mapper.controls().throttle, 0.65);
    mapper.handle_button(DeviceId(1), GamepadButton::LeftShoulder, false);
}

#[test]
fn hotas_twist_is_rudder_and_lever_is_direct() {
    let mut mapper = InputMapper::new(ProfileLibrary::default());
    mapper.set_trim_throttle(0.65);
    mapper.handle(&RawInput::Connected(device(2, Layout::NonStandard, None)));
    mapper.handle(&axis(2, AxisSource::Raw(3), 1.0));
    approx(mapper.controls().yaw, 1.0);
    mapper.handle(&axis(2, AxisSource::Raw(2), -0.5));
    approx(mapper.controls().throttle, 0.75);
    mapper.set_key_action(FlightKeyAction::ThrottleDown, true);
    run(&mut mapper, 1.0);
    approx(mapper.controls().throttle, 0.3);
}

#[test]
fn vehicle_class_picks_defaults_unless_pinned() {
    let mut mapper = InputMapper::new(ProfileLibrary::default());
    assert_eq!(mapper.preset(), ControlPresetId::FixedWingGamepad);
    mapper.pin_preset(
        VehicleControlClass::Multirotor,
        Some(ControlPresetId::QuadAcroMode2),
    );
    assert_eq!(mapper.preset(), ControlPresetId::FixedWingGamepad);
    mapper.set_vehicle_class(VehicleControlClass::Multirotor);
    assert_eq!(mapper.preset(), ControlPresetId::QuadAcroMode2);
    assert!(matches!(
        mapper.throttle_behavior(),
        ThrottleBehavior::LatchingIncremental { .. }
    ));
    mapper.set_preset(ControlPresetId::FixedWingMode2);
    mapper.set_vehicle_class(VehicleControlClass::FixedWing);
    assert_eq!(mapper.preset(), ControlPresetId::FixedWingGamepad);
    mapper.pin_preset(VehicleControlClass::Multirotor, None);
    mapper.set_vehicle_class(VehicleControlClass::Multirotor);
    assert_eq!(mapper.preset(), ControlPresetId::QuadCamDroneMode2);
}

#[test]
fn monitor_snapshot_reports_owners_devices_and_readings() {
    let mut mapper = gamepad_mapper(ControlPresetId::FixedWingGamepad);
    mapper.handle(&axis(1, pad(GamepadAxis::LeftStickX), 1.0));
    mapper.set_key_action(FlightKeyAction::ThrottleUp, true);
    let snapshot = mapper.monitor_snapshot();
    assert_eq!(snapshot.preset, ControlPresetId::FixedWingGamepad);
    assert_eq!(snapshot.vehicle_class, VehicleControlClass::FixedWing);
    assert_eq!(
        snapshot.active_owner,
        [
            ControlOwner::Device(DeviceId(1)),
            ControlOwner::Keyboard,
            ControlOwner::Keyboard,
            ControlOwner::Keyboard,
        ]
    );
    assert_eq!(snapshot.held_actions, vec![FlightKeyAction::ThrottleUp]);
    let [pad_monitor] = snapshot.devices.as_slice() else {
        panic!("one device expected");
    };
    assert_eq!(pad_monitor.preset, Some(ControlPresetId::FixedWingGamepad));
    assert_eq!(pad_monitor.axes.len(), 1);
    assert_eq!(pad_monitor.outputs.len(), 1);
    approx(pad_monitor.outputs[0].shaped, 1.0);
}
