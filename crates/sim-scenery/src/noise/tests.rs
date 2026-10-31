use super::*;

#[test]
fn hashes_are_stable_and_depend_on_every_input() {
    assert_eq!(lattice_hash(1, 2, 3), lattice_hash(1, 2, 3));
    assert_ne!(lattice_hash(1, 2, 3), lattice_hash(2, 2, 3));
    assert_ne!(lattice_hash(1, 2, 3), lattice_hash(1, 3, 3));
    assert_ne!(lattice_hash(1, 2, 3), lattice_hash(1, 2, 4));
    assert_ne!(lattice_hash(1, 2, 3), lattice_hash(1, 3, 2));
}

#[test]
fn value_noise_stays_in_range_and_is_continuous_across_cells() {
    for step in 0..2000 {
        let x = f64::from(step) * 0.0137 - 7.0;
        let value = value_noise(9, x, 0.25 * x);
        assert!((0.0..1.0).contains(&value), "{value}");
    }
    let below = value_noise(9, 2.999_999, 1.5);
    let above = value_noise(9, 3.000_001, 1.5);
    assert!((below - above).abs() < 1e-4);
}
