use super::*;

fn moments(samples: &[f64]) -> (f64, f64) {
    let n = samples.len() as f64;
    let mean = samples.iter().sum::<f64>() / n;
    let variance = samples.iter().map(|x| (x - mean).powi(2)).sum::<f64>() / (n - 1.0);
    (mean, variance)
}

#[test]
fn same_key_repeats_and_other_keys_differ() {
    let draw = |seed, sequence, stream| {
        let mut noise = NoiseStream::new(seed, sequence, stream);
        (0..8).map(|_| noise.gaussian()).collect::<Vec<_>>()
    };
    let reference = draw(7, 3, NoiseStreamId::Detector);
    assert_eq!(reference, draw(7, 3, NoiseStreamId::Detector));
    assert_ne!(reference, draw(8, 3, NoiseStreamId::Detector));
    assert_ne!(reference, draw(7, 4, NoiseStreamId::Detector));
    assert_ne!(reference, draw(7, 3, NoiseStreamId::Temporal));
}

#[test]
fn gaussian_is_standard_normal() {
    let mut noise = NoiseStream::new(1, 0, NoiseStreamId::Detector);
    let samples: Vec<f64> = (0..200_000).map(|_| noise.gaussian()).collect();
    let (mean, variance) = moments(&samples);
    assert!(mean.abs() < 0.01, "mean {mean}");
    assert!((variance - 1.0).abs() < 0.01, "variance {variance}");
}

#[test]
fn poisson_mean_and_variance_match_on_both_branches() {
    for mean in [0.7, 4.0, 29.0, 30.0, 400.0] {
        let mut noise = NoiseStream::new(2, 0, NoiseStreamId::Detector);
        let samples: Vec<f64> = (0..100_000).map(|_| noise.poisson(mean)).collect();
        let (got_mean, got_variance) = moments(&samples);
        assert!(
            (got_mean - mean).abs() < 0.02 * mean.max(1.0),
            "mean {mean}: {got_mean}"
        );
        assert!(
            (got_variance - mean).abs() < 0.04 * mean.max(1.0),
            "mean {mean}: variance {got_variance}"
        );
        assert!(samples.iter().all(|&k| k >= 0.0 && k.fract() == 0.0));
    }
}

#[test]
fn poisson_of_nothing_is_zero() {
    let mut noise = NoiseStream::new(3, 0, NoiseStreamId::Detector);
    assert_eq!(noise.poisson(0.0), 0.0);
    assert_eq!(noise.poisson(-1.0), 0.0);
    assert_eq!(noise.poisson(f64::NAN), 0.0);
}
