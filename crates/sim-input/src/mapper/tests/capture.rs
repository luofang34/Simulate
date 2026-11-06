use super::*;
use crate::{
    BindingCapture, BindingSource, ControlConfig, ControlPresetId, FlightKeyAction, InputTarget,
    KeyId, KeyResponse,
};

#[test]
fn axis_capture_binds_the_first_large_excursion_and_round_trips() {
    let mut mapper = gamepad_mapper(ControlPresetId::FixedWingGamepad);
    mapper.handle(&axis(1, pad(GamepadAxis::RightStickX), 0.6));
    mapper.start_axis_capture(Control::Yaw);
    assert_eq!(
        mapper.capture_state(),
        BindingCapture::Axis(InputTarget::Flight(Control::Yaw))
    );
    // Already deflected before capture began, and small moves elsewhere,
    // do not bind.
    assert_eq!(
        mapper.handle_event(&axis(1, pad(GamepadAxis::RightStickX), 0.9)),
        None
    );
    assert_eq!(
        mapper.handle_event(&axis(1, pad(GamepadAxis::LeftStickX), 0.3)),
        None
    );
    let captured = mapper
        .handle_event(&axis(1, pad(GamepadAxis::RightStickY), -0.8))
        .expect("capture completes");
    assert_eq!(captured.preset, Some(ControlPresetId::FixedWingGamepad));
    assert!(captured.inverted);
    assert_eq!(mapper.capture_state(), BindingCapture::Idle);

    let bindings = &mapper
        .preset_table(ControlPresetId::FixedWingGamepad)
        .bindings;
    let yaw: Vec<_> = bindings
        .iter()
        .filter(|b| b.target == InputTarget::Flight(Control::Yaw))
        .collect();
    assert_eq!(yaw.len(), 1, "the trigger pair was replaced");
    assert_eq!(
        yaw[0].source,
        BindingSource::Single(pad(GamepadAxis::RightStickY))
    );
    assert!(
        !bindings.iter().any(|b| b.target == InputTarget::CameraTilt),
        "the stick no longer drives the camera"
    );

    mapper.handle(&axis(1, pad(GamepadAxis::RightStickY), -1.0));
    approx(mapper.controls().yaw, 1.0);

    let json = mapper.config().to_json_pretty().expect("serialises");
    let loaded = ControlConfig::from_json_str(&json).expect("parses");
    assert_eq!(loaded, mapper.config());
    let mut fresh = gamepad_mapper(ControlPresetId::FixedWingGamepad);
    fresh.apply_config(&loaded);
    fresh.handle(&axis(1, pad(GamepadAxis::RightStickY), -1.0));
    approx(fresh.controls().yaw, 1.0);
}

#[test]
fn key_capture_rebinds_an_action_and_unbinds_the_old_key() {
    let mut mapper = InputMapper::new(ProfileLibrary::default());
    mapper.start_key_capture(FlightKeyAction::ThrottleUp);
    assert_eq!(
        mapper.handle_key(&KeyId::new("KeyW"), false),
        KeyResponse::Action(FlightKeyAction::ThrottleUp),
        "a release does not complete capture"
    );
    let response = mapper.handle_key(&KeyId::new("PageUp"), true);
    assert!(
        matches!(response, KeyResponse::Captured(ref c) if c.action == FlightKeyAction::ThrottleUp)
    );
    assert_eq!(
        mapper.keys_for(FlightKeyAction::ThrottleUp),
        vec![KeyId::new("PageUp")]
    );
    assert_eq!(
        mapper.handle_key(&KeyId::new("KeyW"), true),
        KeyResponse::Ignored
    );
}

#[test]
fn cancelled_capture_leaves_bindings_alone() {
    let mut mapper = gamepad_mapper(ControlPresetId::FixedWingGamepad);
    let before = mapper
        .preset_table(ControlPresetId::FixedWingGamepad)
        .clone();
    mapper.start_axis_capture(Control::Roll);
    mapper.cancel_capture();
    mapper.handle(&axis(1, pad(GamepadAxis::RightStickX), 1.0));
    assert_eq!(
        mapper.preset_table(ControlPresetId::FixedWingGamepad),
        &before
    );
    mapper.reset_preset(ControlPresetId::FixedWingGamepad);
    assert_eq!(
        mapper.preset_table(ControlPresetId::FixedWingGamepad),
        &ControlPresetId::FixedWingGamepad.builtin()
    );
}
