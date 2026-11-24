use super::*;

#[test]
fn lower_triangle_indexes_both_ways() {
    let lower: [f64; 21] = std::array::from_fn(|i| i as f64);
    let cov = Covariance6::from_lower(lower);
    assert_eq!(cov.get(0, 0), Some(0.0));
    assert_eq!(cov.get(1, 0), Some(1.0));
    assert_eq!(cov.get(0, 1), Some(1.0));
    assert_eq!(cov.get(5, 5), Some(20.0));
    assert_eq!(cov.get(3, 2), Some(8.0));
    assert_eq!(cov.get(6, 0), None);
    let matrix = cov.to_matrix();
    assert_eq!(matrix[2][4], matrix[4][2]);
}

#[test]
fn units_follow_the_block_structure() {
    assert_eq!(KEYWORDS[9], "CX_DOT_X_DOT");
    assert_eq!(unit_of(5), "km**2");
    assert_eq!(unit_of(8), "km**2/s");
    assert_eq!(unit_of(9), "km**2/s**2");
    assert_eq!(unit_of(12), "km**2/s");
    assert_eq!(unit_of(13), "km**2/s**2");
    assert_eq!(unit_of(17), "km**2/s");
    assert_eq!(unit_of(20), "km**2/s**2");
}
