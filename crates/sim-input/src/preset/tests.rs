use super::*;
use crate::{AxisSource, Control, GamepadAxis, InputTarget};

#[test]
fn every_builtin_has_a_complete_keymap_and_unique_targets() {
    for id in ControlPresetId::ALL {
        let preset = id.builtin();
        assert_eq!(preset.id, id);
        for action in FlightKeyAction::ALL {
            assert!(
                preset.keys_for(action).next().is_some(),
                "{id:?} {action:?}"
            );
        }
        let mut targets: Vec<_> = preset.bindings.iter().map(|b| b.target).collect();
        targets.sort();
        targets.dedup();
        assert_eq!(targets.len(), preset.bindings.len(), "{id:?}");
    }
}

#[test]
fn class_defaults_match_the_spec() {
    assert_eq!(
        VehicleControlClass::Multirotor.default_preset(),
        ControlPresetId::QuadCamDroneMode2
    );
    assert_eq!(
        VehicleControlClass::FixedWing.default_preset(),
        ControlPresetId::FixedWingGamepad
    );
    let quad = ControlPresetId::QuadCamDroneMode2.builtin();
    assert_eq!(quad.throttle, ThrottleBehavior::CenteredClimbRate);
    assert!(
        quad.bindings
            .iter()
            .any(|b| b.target == InputTarget::Flight(Control::Throttle)
                && b.source.uses(AxisSource::Gamepad(GamepadAxis::LeftStickY)))
    );
}
