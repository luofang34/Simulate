use super::*;
use crate::{AxisShape, BindingSource, ControlPresetId, InputRange, InputTarget};

fn inverted_rudder_gamepad() -> InputMapper {
    let mut mapper = gamepad_mapper(ControlPresetId::FixedWingGamepad);
    mapper.set_control_inverted(Control::Yaw, true);
    mapper
}

#[test]
fn inverted_trigger_rudder_rests_at_centre_with_one_trigger_unreported() {
    let mut mapper = inverted_rudder_gamepad();
    approx(mapper.controls().yaw, 0.0);
    // Only one trigger has spoken; the other is still unreported.
    mapper.handle(&axis(1, pad(GamepadAxis::RightTrigger), 0.0));
    approx(mapper.controls().yaw, 0.0);
    mapper.handle(&axis(1, pad(GamepadAxis::LeftTrigger), 0.0));
    approx(mapper.controls().yaw, 0.0);
}

#[test]
fn inverted_trigger_rudder_flips_each_trigger_cleanly() {
    let mut mapper = inverted_rudder_gamepad();
    mapper.handle(&axis(1, pad(GamepadAxis::RightTrigger), 1.0));
    approx(mapper.controls().yaw, -1.0);
    mapper.handle(&axis(1, pad(GamepadAxis::RightTrigger), 0.0));
    approx(mapper.controls().yaw, 0.0);
    mapper.handle(&axis(1, pad(GamepadAxis::LeftTrigger), 1.0));
    approx(mapper.controls().yaw, 1.0);
}

/// A saved preset may carry a trigger-style shape on a differential binding;
/// the pair still spans −1..1, so released triggers must read as centre in
/// either sense rather than as full rudder.
#[test]
fn a_unipolar_shape_on_a_trigger_pair_still_rests_at_centre() {
    let mut mapper = InputMapper::new(ProfileLibrary::default());
    let mut config = mapper.config();
    let unipolar = AxisShape {
        input: InputRange::Unipolar,
        ..AxisShape::STICK
    };
    for preset in &mut config.presets {
        for binding in &mut preset.bindings {
            if matches!(binding.source, BindingSource::Differential { .. })
                && binding.target == InputTarget::Flight(Control::Yaw)
            {
                binding.shape = unipolar;
            }
        }
    }
    mapper.apply_config(&config);
    mapper.set_preset(ControlPresetId::FixedWingGamepad);
    mapper.handle(&RawInput::Connected(device(1, Layout::Standard, None)));

    for inverted in [false, true] {
        mapper.set_control_inverted(Control::Yaw, inverted);
        let sense = if inverted { -1.0 } else { 1.0 };
        mapper.handle(&axis(1, pad(GamepadAxis::LeftTrigger), 0.0));
        mapper.handle(&axis(1, pad(GamepadAxis::RightTrigger), 0.0));
        approx(mapper.controls().yaw, 0.0);
        mapper.handle(&axis(1, pad(GamepadAxis::RightTrigger), 1.0));
        approx(mapper.controls().yaw, sense);
        mapper.handle(&axis(1, pad(GamepadAxis::RightTrigger), 0.0));
        mapper.handle(&axis(1, pad(GamepadAxis::LeftTrigger), 1.0));
        approx(mapper.controls().yaw, -sense);
    }
}
