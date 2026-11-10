use super::{InstrumentNeedles, NEEDLE_HANDS_PER_SLOT, NEEDLE_SLOTS};

#[test]
fn every_channel_holds_its_angle_and_visibility() {
    let mut needles = InstrumentNeedles::default();
    let last = InstrumentNeedles::channel(NEEDLE_SLOTS - 1, NEEDLE_HANDS_PER_SLOT - 1).unwrap();
    needles.set(NEEDLE_SLOTS - 1, NEEDLE_HANDS_PER_SLOT - 1, 1.25);
    needles.set_visible(NEEDLE_SLOTS - 1, NEEDLE_HANDS_PER_SLOT - 1, false);
    assert!((needles.angle_rad(last) - 1.25).abs() < 1e-6);
    assert!(!needles.visible(last));
    assert!(needles.visible(0));
    needles.set_visible(NEEDLE_SLOTS - 1, NEEDLE_HANDS_PER_SLOT - 1, true);
    assert!(needles.visible(last));
    assert_eq!(InstrumentNeedles::channel(NEEDLE_SLOTS, 0), None);
    needles.set(0, 0, f64::NAN);
    assert_eq!(needles.angle_rad(0), 0.0);
}

#[test]
fn drum_wheels_and_flags_are_kept_per_slot() {
    use super::DRUM_WHEELS_PER_SLOT;
    let mut needles = InstrumentNeedles::default();
    let last = NEEDLE_SLOTS - 1;
    needles.set_wheel(last, DRUM_WHEELS_PER_SLOT - 1, 2.5);
    let channel = InstrumentNeedles::wheel_channel(last, DRUM_WHEELS_PER_SLOT - 1).unwrap();
    assert!((needles.wheel(channel) - 2.5).abs() < 1e-6);
    assert!(needles.drum_live(last) && !needles.flagged(last));
    needles.set_slot_state(last, false, true);
    assert!(!needles.drum_live(last) && needles.flagged(last));
    assert!(needles.drum_live(0) && !needles.flagged(0));
    assert_eq!(InstrumentNeedles::wheel_channel(NEEDLE_SLOTS, 0), None);
}
