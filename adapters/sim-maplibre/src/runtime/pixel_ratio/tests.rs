use super::pixel_ratio_update;

#[test]
fn first_finite_ratio_is_applied() {
    assert_eq!(pixel_ratio_update(None, 2.0), Some(2.0));
}

#[test]
fn non_finite_ratios_are_ignored() {
    assert_eq!(pixel_ratio_update(None, f64::NAN), None);
    assert_eq!(pixel_ratio_update(Some(1.0), f64::INFINITY), None);
}

#[test]
fn ratios_are_clamped() {
    assert_eq!(pixel_ratio_update(None, 0.1), Some(0.5));
    assert_eq!(pixel_ratio_update(None, 9.0), Some(4.0));
}

#[test]
fn unchanged_ratios_are_not_reapplied() {
    assert_eq!(pixel_ratio_update(Some(2.0), 2.0005), None);
    assert_eq!(pixel_ratio_update(Some(4.0), 6.0), None);
    assert_eq!(pixel_ratio_update(Some(2.0), 2.01), Some(2.01));
}
