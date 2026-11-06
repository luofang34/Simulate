use sim_core::ControlInputs;
use sim_input::{AxisSource, Control, GamepadAxis};
use winit::keyboard::KeyCode;

use super::*;

const LSX: AxisSource = AxisSource::Gamepad(GamepadAxis::LeftStickX);
const LSY: AxisSource = AxisSource::Gamepad(GamepadAxis::LeftStickY);
const RSX: AxisSource = AxisSource::Gamepad(GamepadAxis::RightStickX);
const LT: AxisSource = AxisSource::Gamepad(GamepadAxis::LeftTrigger);
const RT: AxisSource = AxisSource::Gamepad(GamepadAxis::RightTrigger);

fn write_for(writes: &[(Control, f64)], control: Control) -> Option<f64> {
    writes.iter().find(|(c, _)| *c == control).map(|(_, v)| *v)
}

#[test]
fn class_defaults_and_user_override() {
    let mut scheme = ControlScheme::new(VehicleClass::Multirotor);
    assert_eq!(scheme.preset(), ControlPreset::QuadCamDrone);
    assert!(scheme.is_automatic());
    scheme.set_vehicle_class(VehicleClass::FixedWing);
    assert_eq!(scheme.preset(), ControlPreset::FixedWingGamepad);
    let chosen = scheme.step_preset(true);
    assert_eq!(chosen, ControlPreset::FixedWingMode2);
    assert!(!scheme.is_automatic());
    scheme.set_vehicle_class(VehicleClass::Multirotor);
    assert_eq!(
        scheme.preset(),
        ControlPreset::FixedWingMode2,
        "override sticks"
    );
    scheme.reset_automatic();
    assert_eq!(scheme.preset(), ControlPreset::QuadCamDrone);
    for preset in ControlPreset::ALL {
        assert_eq!(preset.step(true).step(false), preset);
    }
}

#[test]
fn sub_deadzone_noise_never_overwrites_the_keyboard() {
    let mut scheme = ControlScheme::new(VehicleClass::FixedWing);
    assert!(scheme.handle_axis(LSX, 0.03).is_empty(), "idle noise");
    assert!(scheme.handle_axis(LSX, -0.05).is_empty(), "idle noise");
    let deflected = scheme.handle_axis(LSX, 0.9);
    assert!(write_for(&deflected, Control::Roll).is_some_and(|v| v > 0.5));
    let centred = scheme.handle_axis(LSX, 0.02);
    assert_eq!(
        write_for(&centred, Control::Roll),
        Some(0.0),
        "one return-to-centre"
    );
    assert!(scheme.handle_axis(LSX, 0.01).is_empty());
    assert_eq!(scheme.source(), InputSource::Gamepad);
}

#[test]
fn quad_climb_stick_centres_at_hover_throttle() {
    let mut scheme = ControlScheme::new(VehicleClass::Multirotor);
    let up = scheme.handle_axis(LSY, 1.0);
    assert_eq!(write_for(&up, Control::Throttle), Some(1.0));
    let rest = scheme.handle_axis(LSY, 0.0);
    assert_eq!(write_for(&rest, Control::Throttle), Some(0.5));
    let yaw = scheme.handle_axis(LSX, -1.0);
    assert_eq!(write_for(&yaw, Control::Yaw), Some(-1.0));
    let roll = scheme.handle_axis(RSX, 1.0);
    assert_eq!(write_for(&roll, Control::Roll), Some(1.0));
}

#[test]
fn triggers_share_the_rudder_and_bumpers_latch_throttle() {
    let mut scheme = ControlScheme::new(VehicleClass::FixedWing);
    let right = scheme.handle_axis(RT, 1.0);
    assert_eq!(write_for(&right, Control::Yaw), Some(1.0));
    let both = scheme.handle_axis(LT, 1.0);
    assert_eq!(write_for(&both, Control::Yaw), Some(0.0));
    let left = scheme.handle_axis(RT, 0.0);
    assert_eq!(write_for(&left, Control::Yaw), Some(-1.0));
    assert_eq!(scheme.integrate_throttle(1.0, 0.5), None);
    scheme.handle_button(PadButton::RightBumper, true);
    let up = scheme.integrate_throttle(0.5, 0.5).expect("rate");
    assert!((up - 0.75).abs() < 1e-12);
    scheme.handle_button(PadButton::RightBumper, false);
    scheme.handle_button(PadButton::LeftBumper, true);
    assert_eq!(scheme.integrate_throttle(10.0, 0.5), Some(0.0));
    scheme.handle_button(PadButton::LeftBumper, false);

    let gamepad_throttle = scheme
        .axis_rows()
        .into_iter()
        .find(|r| r.control == Control::Throttle)
        .expect("throttle row");
    assert!(
        gamepad_throttle.sources.contains("RB") && gamepad_throttle.sources.contains("LB"),
        "{}",
        gamepad_throttle.sources
    );

    // FixedWingMode2 also integrates throttle on LB/RB while LS Y rests near centre.
    assert_eq!(scheme.step_preset(true), ControlPreset::FixedWingMode2);
    assert!(scheme.handle_axis(LSY, 0.02).is_empty(), "resting stick");
    let mode2_throttle = scheme
        .axis_rows()
        .into_iter()
        .find(|r| r.control == Control::Throttle)
        .expect("mode2 throttle row");
    assert!(
        mode2_throttle.sources.contains("LS Y")
            && mode2_throttle.sources.contains("RB")
            && mode2_throttle.sources.contains("LB"),
        "{}",
        mode2_throttle.sources
    );
    scheme.handle_button(PadButton::RightBumper, true);
    let mode2_up = scheme.integrate_throttle(0.5, 0.5).expect("mode2 RB rate");
    assert!((mode2_up - 0.75).abs() < 1e-12);
    scheme.handle_button(PadButton::RightBumper, false);
    scheme.handle_button(PadButton::LeftBumper, true);
    let mode2_down = scheme
        .integrate_throttle(0.5, mode2_up)
        .expect("mode2 LB rate");
    assert!((mode2_down - 0.5).abs() < 1e-12);
}

#[test]
fn click_to_rebind_an_axis_and_a_key() {
    let mut scheme = ControlScheme::new(VehicleClass::FixedWing);
    scheme.begin_capture(CaptureTarget::Axis(Control::Yaw));
    assert!(scheme.handle_axis(AxisSource::Raw(7), 0.3).is_empty());
    assert_eq!(scheme.capture(), Some(CaptureTarget::Axis(Control::Yaw)));
    assert!(scheme.handle_axis(AxisSource::Raw(7), -0.9).is_empty());
    assert_eq!(scheme.capture(), None);
    let twist = scheme.handle_axis(AxisSource::Raw(7), 0.8);
    assert!(write_for(&twist, Control::Yaw).is_some_and(|v| v > 0.5));
    let yaw_row = scheme
        .axis_rows()
        .into_iter()
        .find(|r| r.control == Control::Yaw)
        .expect("yaw row");
    assert!(yaw_row.sources.contains("AXIS 7"), "{}", yaw_row.sources);

    scheme.begin_capture(CaptureTarget::Key(KeyAction::ThrottleUp));
    assert!(scheme.capture_key(KeyCode::KeyI));
    assert_eq!(
        scheme.keys().action_for(KeyCode::KeyI),
        Some(KeyAction::ThrottleUp)
    );
    assert!(!scheme.capture_key(KeyCode::KeyJ), "no capture pending");
    scheme.begin_capture(CaptureTarget::Key(KeyAction::Brakes));
    scheme.begin_capture(CaptureTarget::Key(KeyAction::Brakes));
    assert_eq!(
        scheme.capture(),
        None,
        "clicking the same rebind again cancels"
    );
}

#[test]
fn key_rebind_swaps_a_taken_key() {
    let mut keys = KeyBindings::default();
    keys.bind(KeyAction::YawLeft, KeyCode::KeyD);
    assert_eq!(keys.action_for(KeyCode::KeyD), Some(KeyAction::YawLeft));
    assert_eq!(keys.key_for(KeyAction::YawRight), KeyCode::KeyA);
    assert_eq!(key_label(KeyCode::ArrowLeft), "LEFT");
    assert_eq!(key_label(KeyCode::KeyQ), "Q");
    assert_eq!(key_label(KeyCode::BracketLeft), "LBRACKET");
    assert_eq!(key_label(KeyCode::Backquote), "GRAVE");
    assert_eq!(key_label(KeyCode::Equal), "EQUALS");
}

#[test]
fn invert_deadzone_and_expo_tune_the_bound_axes() {
    let mut scheme = ControlScheme::new(VehicleClass::FixedWing);
    let roll = |s: &ControlScheme| {
        s.axis_rows()
            .into_iter()
            .find(|r| r.control == Control::Roll)
            .expect("roll")
    };
    let before = roll(&scheme);
    scheme.toggle_invert(Control::Roll);
    scheme.step_deadzone(Control::Roll, true);
    scheme.step_expo(Control::Roll, false);
    let after = roll(&scheme);
    assert_ne!(after.invert, before.invert);
    assert!((after.deadzone - before.deadzone - DEADZONE_STEP).abs() < 1e-12);
    assert!((before.expo - after.expo - EXPO_STEP).abs() < 1e-12);
    let inverted = scheme.handle_axis(LSX, 1.0);
    assert!(write_for(&inverted, Control::Roll).is_some_and(|v| v < -0.5));
}

#[test]
fn flight_keys_preview_on_the_live_telemetry() {
    let mut scheme = ControlScheme::new(VehicleClass::FixedWing);
    scheme.sync_live(ControlInputs {
        throttle: 0.5,
        ..ControlInputs::default()
    });
    scheme.preview_key(KeyAction::RollRight, true);
    scheme.preview_key(KeyAction::YawLeft, true);
    assert_eq!((scheme.live().roll, scheme.live().yaw), (1.0, -1.0));
    scheme.preview_key(KeyAction::RollRight, false);
    assert_eq!(scheme.live().roll, 0.0);
    scheme.preview_key(KeyAction::ThrottleUp, true);
    scheme.preview_key(KeyAction::ThrottleUp, true);
    assert!((scheme.live().throttle - 0.6).abs() < 1e-12);
    assert_eq!(scheme.source(), InputSource::Keyboard);
    scheme.clear_preview();
    scheme.preview_key(KeyAction::PitchUp, false);
    assert_eq!(scheme.live().yaw, 0.0, "cleared keys no longer deflect");
}

const RSY: AxisSource = AxisSource::Gamepad(GamepadAxis::RightStickY);

#[test]
fn right_stick_looks_around_only_where_the_preset_frees_it() {
    let mut scheme = ControlScheme::new(VehicleClass::FixedWing);
    assert_eq!(scheme.preset(), ControlPreset::FixedWingGamepad);
    assert!(scheme.handle_axis(RSX, 0.02).is_empty());
    assert_eq!(scheme.camera_look_axes(), [0.0, 0.0], "deadzone noise");
    let before = scheme.live();
    assert!(scheme.handle_axis(RSX, 0.8).is_empty());
    assert!(scheme.handle_axis(RSY, -0.6).is_empty());
    let [pan, tilt] = scheme.camera_look_axes();
    assert!(pan > 0.5 && tilt < -0.3, "{pan} {tilt}");
    assert_eq!(scheme.live(), before, "flight controls untouched");
    let roll = scheme.handle_axis(LSX, 0.5);
    assert!(write_for(&roll, Control::Roll).is_some_and(|v| v > 0.0));
    assert_eq!(scheme.camera_look_axes(), [pan, tilt]);

    for preset in [ControlPreset::QuadCamDrone, ControlPreset::FixedWingMode2] {
        while scheme.preset() != preset {
            scheme.step_preset(true);
        }
        assert_eq!(scheme.camera_look_axes(), [0.0, 0.0], "{preset:?}");
        let writes = scheme.handle_axis(RSX, 0.8);
        assert!(write_for(&writes, Control::Roll).is_some_and(|v| v > 0.5));
        let writes = scheme.handle_axis(RSY, 0.8);
        assert!(write_for(&writes, Control::Pitch).is_some());
        assert_eq!(scheme.camera_look_axes(), [0.0, 0.0], "{preset:?}");
    }
}

#[test]
fn inverting_the_trigger_rudder_swaps_sides_and_rests_at_zero() {
    let yaw_inverted = |s: &ControlScheme| {
        s.axis_rows()
            .into_iter()
            .find(|r| r.control == Control::Yaw)
            .is_some_and(|r| r.invert)
    };
    let mut scheme = ControlScheme::new(VehicleClass::FixedWing);
    assert_eq!(scheme.preset(), ControlPreset::FixedWingGamepad);
    assert!(!yaw_inverted(&scheme));
    assert_eq!(
        write_for(&scheme.handle_axis(LT, 1.0), Control::Yaw),
        Some(-1.0)
    );
    assert_eq!(
        write_for(&scheme.handle_axis(LT, 0.0), Control::Yaw),
        Some(0.0)
    );
    assert_eq!(
        write_for(&scheme.handle_axis(RT, 1.0), Control::Yaw),
        Some(1.0)
    );
    assert_eq!(
        write_for(&scheme.handle_axis(RT, 0.0), Control::Yaw),
        Some(0.0)
    );

    scheme.toggle_invert(Control::Yaw);
    assert!(yaw_inverted(&scheme), "the row reports the swapped sign");
    // A single released trigger reports rest, never a locked full deflection.
    let rest = scheme.handle_axis(LT, 0.0);
    assert!(
        write_for(&rest, Control::Yaw).is_none_or(|v| v == 0.0),
        "{rest:?}"
    );
    assert_eq!(
        write_for(&scheme.handle_axis(LT, 1.0), Control::Yaw),
        Some(1.0)
    );
    assert_eq!(
        write_for(&scheme.handle_axis(LT, 0.0), Control::Yaw),
        Some(0.0)
    );
    assert_eq!(
        write_for(&scheme.handle_axis(RT, 1.0), Control::Yaw),
        Some(-1.0)
    );
    assert_eq!(
        write_for(&scheme.handle_axis(RT, 0.0), Control::Yaw),
        Some(0.0)
    );

    scheme.toggle_invert(Control::Yaw);
    assert!(!yaw_inverted(&scheme));
    assert_eq!(
        write_for(&scheme.handle_axis(RT, 1.0), Control::Yaw),
        Some(1.0)
    );
}

#[test]
fn control_strings_draw_in_the_panel_glyph_pack() {
    use indicate_instrument_glyphs::PANEL_GLYPHS;
    let scheme = ControlScheme::new(VehicleClass::FixedWing);
    let texts = ControlPreset::ALL
        .iter()
        .flat_map(|p| [p.title().to_owned(), p.summary().to_owned()])
        .chain(scheme.axis_rows().into_iter().map(|r| r.sources))
        .chain(
            [
                KeyCode::BracketLeft,
                KeyCode::BracketRight,
                KeyCode::Comma,
                KeyCode::Period,
                KeyCode::Semicolon,
                KeyCode::Quote,
                KeyCode::Equal,
                KeyCode::Minus,
                KeyCode::Slash,
                KeyCode::Backslash,
                KeyCode::Backquote,
                KeyCode::NumpadAdd,
            ]
            .into_iter()
            .map(key_label),
        );
    for text in texts {
        let lost: Vec<char> = text
            .chars()
            .filter(|&c| c != ' ' && PANEL_GLYPHS.glyph(c.to_ascii_uppercase()).is_err())
            .collect();
        assert!(lost.is_empty(), "{text:?} loses {lost:?}");
    }
}
