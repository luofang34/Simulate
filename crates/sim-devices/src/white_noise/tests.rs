use super::*;

#[test]
fn the_same_seed_gives_the_same_stream() {
    let mut a = WhiteNoise::from_seed(42);
    let mut b = WhiteNoise::from_seed(42);
    for _ in 0..100 {
        assert_eq!(a.next_gaussian().to_bits(), b.next_gaussian().to_bits());
    }
    assert_ne!(
        WhiteNoise::from_seed(1).next_gaussian(),
        WhiteNoise::from_seed(2).next_gaussian()
    );
}

#[test]
fn a_copy_resumes_the_stream_exactly() {
    let mut noise = WhiteNoise::from_seed(7);
    noise.next_gaussian();
    let mut resumed = noise;
    assert_eq!(noise.next_gaussian(), resumed.next_gaussian());
}

#[test]
fn draws_have_zero_mean_and_unit_variance() {
    let mut noise = WhiteNoise::from_seed(2_026);
    let samples = 200_000;
    let (mut sum, mut squares, mut fourths) = (0.0, 0.0, 0.0);
    for _ in 0..samples {
        let value = noise.next_gaussian();
        assert!(value.is_finite());
        sum += value;
        squares += value * value;
        fourths += value.powi(4);
    }
    let count = f64::from(samples);
    assert!((sum / count).abs() < 0.01);
    assert!((squares / count - 1.0).abs() < 0.01);
    assert!((fourths / count - 3.0).abs() < 0.1, "not Gaussian");
}

#[test]
fn axes_scale_independently() {
    let mut noise = WhiteNoise::from_seed(5);
    let draw = noise.next_vec3(DVec3::new(0.0, 2.0, 0.0));
    assert_eq!(draw.x, 0.0);
    assert_eq!(draw.z, 0.0);
    assert_ne!(draw.y, 0.0);
}
