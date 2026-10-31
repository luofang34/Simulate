use super::*;

#[test]
fn stick_deadzone_reads_zero_and_full_travel_reads_one() {
    let shape = AxisShape::STICK;
    assert_eq!(shape.apply(0.04), 0.0);
    assert_eq!(shape.apply(-0.04), 0.0);
    assert!((shape.apply(1.0) - 1.0).abs() < 1e-12);
    assert!((shape.apply(-1.0) + 1.0).abs() < 1e-12);
    assert!(
        shape.apply(0.5) > 0.0 && shape.apply(0.5) < 0.5,
        "expo softens the centre"
    );
}

#[test]
fn lever_has_no_centre_deadzone() {
    let lever = AxisShape::LEVER;
    assert!((lever.apply(0.0) - 0.5).abs() < 1e-12);
    assert!((lever.apply(0.02) - 0.51).abs() < 1e-6);
    assert_eq!(lever.apply(-1.0), 0.0);
    assert_eq!(lever.apply(1.0), 1.0);
}

#[test]
fn inversion_and_trigger_range() {
    assert!((AxisShape::LEVER.inverted().apply(-1.0) - 1.0).abs() < 1e-12);
    let trigger = AxisShape {
        input: InputRange::Unipolar,
        ..AxisShape::LEVER
    };
    assert_eq!(trigger.apply(0.0), 0.0);
    assert_eq!(trigger.apply(1.0), 1.0);
    assert_eq!(AxisShape::STICK.apply(f32::NAN), 0.0);
}
