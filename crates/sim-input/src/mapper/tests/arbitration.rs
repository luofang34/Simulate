use super::*;
use crate::{ControlPresetId, FlightKeyAction, GamepadButton, KeyId, KeyResponse};

#[test]
fn sub_deadzone_noise_never_clobbers_held_keyboard_axes() {
    let mut mapper = gamepad_mapper(ControlPresetId::FixedWingGamepad);
    mapper.set(Control::Roll, -1.0);
    mapper.set_key_action(FlightKeyAction::PitchUp, true);
    mapper.set_key_action(FlightKeyAction::YawRight, true);
    run(&mut mapper, 1.0);
    for step in 0..200 {
        let noise: f32 = if step % 2 == 0 { 0.02 } else { -0.02 };
        for source in [
            pad(GamepadAxis::LeftStickX),
            pad(GamepadAxis::LeftStickY),
            pad(GamepadAxis::RightTrigger),
            pad(GamepadAxis::LeftTrigger),
        ] {
            let value = if matches!(
                source,
                AxisSource::Gamepad(GamepadAxis::LeftTrigger | GamepadAxis::RightTrigger)
            ) {
                noise.abs()
            } else {
                noise
            };
            mapper.handle(&axis(1, source, value));
        }
        mapper.tick(DT);
    }
    let controls = mapper.controls();
    approx(controls.roll, -1.0);
    approx(controls.pitch, 1.0);
    approx(controls.yaw, 1.0);
    assert_eq!(mapper.owner(Control::Roll), ControlOwner::Keyboard);
}

#[test]
fn stick_claims_past_deadzone_and_keyboard_retakes_after_centring() {
    let mut mapper = gamepad_mapper(ControlPresetId::FixedWingGamepad);
    mapper.set(Control::Roll, -1.0);
    mapper.handle(&axis(1, pad(GamepadAxis::LeftStickX), 1.0));
    approx(mapper.controls().roll, 1.0);
    assert_eq!(
        mapper.owner(Control::Roll),
        ControlOwner::Device(DeviceId(1))
    );
    mapper.handle(&axis(1, pad(GamepadAxis::LeftStickX), 0.01));
    approx(mapper.controls().roll, 0.0);
    mapper.handle(&axis(1, pad(GamepadAxis::LeftStickX), -0.02));
    mapper.set(Control::Roll, -1.0);
    approx(mapper.controls().roll, -1.0);
    mapper.set(Control::Roll, 0.0);
    mapper.set_key_action(FlightKeyAction::RollRight, true);
    assert_eq!(mapper.owner(Control::Roll), ControlOwner::Keyboard);
    mapper.tick(0.1);
    assert!(mapper.controls().roll > 0.0);
}

#[test]
fn held_keys_slew_and_self_centre() {
    let mut mapper = InputMapper::new(ProfileLibrary::default());
    mapper.set_key_action(FlightKeyAction::RollLeft, true);
    mapper.tick(0.1);
    let partial = mapper.controls().roll;
    assert!(
        partial < 0.0 && partial > -1.0,
        "ramps, not jumps: {partial}"
    );
    run(&mut mapper, 1.0);
    approx(mapper.controls().roll, -1.0);
    mapper.set_key_action(FlightKeyAction::RollLeft, false);
    run(&mut mapper, 1.0);
    approx(mapper.controls().roll, 0.0);
}

#[test]
fn keymap_routes_physical_keys_and_reports_unknown_keys() {
    let mut mapper = InputMapper::new(ProfileLibrary::default());
    assert_eq!(
        mapper.handle_key(&KeyId::new("ArrowDown"), true),
        KeyResponse::Action(FlightKeyAction::PitchUp)
    );
    assert_eq!(
        mapper.handle_key(&KeyId::new("KeyB"), true),
        KeyResponse::Action(FlightKeyAction::WheelBrake)
    );
    assert!(mapper.wheel_brake_held());
    assert_eq!(
        mapper.handle_key(&KeyId::new("KeyQ"), true),
        KeyResponse::Ignored
    );
    run(&mut mapper, 1.0);
    approx(mapper.controls().pitch, 1.0);
    mapper.release_keys();
    approx(mapper.controls().pitch, 0.0);
    assert!(!mapper.wheel_brake_held());
}

#[test]
fn disconnect_hands_controls_back_to_the_keyboard() {
    let mut mapper = gamepad_mapper(ControlPresetId::FixedWingGamepad);
    mapper.handle(&axis(1, pad(GamepadAxis::LeftStickX), 1.0));
    mapper.handle(&axis(1, pad(GamepadAxis::RightStickX), 1.0));
    approx(mapper.camera_look()[0], 1.0);
    mapper.handle(&RawInput::Disconnected(DeviceId(1)));
    assert_eq!(mapper.owner(Control::Roll), ControlOwner::Keyboard);
    approx(mapper.controls().roll, 0.0);
    approx(mapper.camera_look()[0], 0.0);
}

#[test]
fn right_stick_looks_around_without_touching_flight_controls() {
    let mut mapper = gamepad_mapper(ControlPresetId::FixedWingGamepad);
    mapper.set(Control::Roll, 0.5);
    mapper.handle(&axis(1, pad(GamepadAxis::RightStickX), -1.0));
    mapper.handle(&axis(1, pad(GamepadAxis::RightStickY), 1.0));
    assert_eq!(mapper.camera_look(), [-1.0, 1.0]);
    approx(mapper.controls().roll, 0.5);
    approx(mapper.controls().yaw, 0.0);
}

#[test]
fn bumpers_nudge_the_latched_throttle() {
    let mut mapper = gamepad_mapper(ControlPresetId::FixedWingGamepad);
    mapper.set_trim_throttle(0.5);
    let held = mapper.handle_button(DeviceId(1), GamepadButton::RightShoulder, true);
    assert_eq!(held, Some(FlightKeyAction::ThrottleUp));
    run(&mut mapper, 0.5);
    approx(mapper.controls().throttle, 0.5 + 0.45 * 0.5);
    mapper.handle_button(DeviceId(1), GamepadButton::RightShoulder, false);
    run(&mut mapper, 1.0);
    approx(mapper.controls().throttle, 0.5 + 0.45 * 0.5);
    assert_eq!(
        mapper.handle_button(DeviceId(1), GamepadButton::South, true),
        None
    );
}

#[test]
fn per_control_tuning_reshapes_bindings() {
    let mut mapper = gamepad_mapper(ControlPresetId::FixedWingGamepad);
    mapper.set_control_inverted(Control::Roll, true);
    mapper.set_control_expo(Control::Roll, 0.0);
    mapper.set_control_deadzone(Control::Roll, 0.5);
    mapper.handle(&axis(1, pad(GamepadAxis::LeftStickX), 0.4));
    approx(mapper.controls().roll, 0.0);
    mapper.handle(&axis(1, pad(GamepadAxis::LeftStickX), 0.75));
    approx(mapper.controls().roll, -0.5);
    mapper.set_control_deadzone(Control::Roll, f64::NAN);
    assert_eq!(mapper.control_tuning(Control::Roll).deadzone, Some(0.5));
}

#[test]
fn invalid_tick_steps_are_ignored() {
    let mut mapper = InputMapper::new(ProfileLibrary::default());
    mapper.set_key_action(FlightKeyAction::RollRight, true);
    for dt in [f64::NAN, f64::INFINITY, -1.0, 0.0] {
        mapper.tick(dt);
    }
    approx(mapper.controls().roll, 0.0);
}
