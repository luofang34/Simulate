use super::*;

#[test]
fn full_scale_matches_bit_depth() {
    assert_eq!(full_scale(12), 4095);
    assert_eq!(full_scale(14), 16383);
    assert_eq!(full_scale(16), u16::MAX);
    assert_eq!(full_scale(0), 0);
}

#[test]
fn digitise_rounds_and_clips() {
    assert_eq!(digitise(10.4, 4095), 10);
    assert_eq!(digitise(10.6, 4095), 11);
    assert_eq!(digitise(-3.0, 4095), 0);
    assert_eq!(digitise(1e9, 4095), 4095);
    assert_eq!(digitise(f64::NAN, 4095), 0);
}

#[test]
fn count_is_row_major_and_bounded() {
    let frame = SensorCounts {
        width: 3,
        height: 2,
        significant_bits: 12,
        counts: vec![0, 1, 2, 3, 4, 5],
    };
    assert_eq!(frame.count(2, 1), Some(5));
    assert_eq!(frame.count(0, 1), Some(3));
    assert_eq!(frame.count(3, 0), None);
    assert_eq!(frame.full_scale(), 4095);
}
