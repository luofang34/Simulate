use super::{
    DistributionSpec, EnsembleSummary, MonteCarloConfig, MonteCarloError, MonteCarloRunManifest,
    ParameterDispersionSpec, TrialRng, TrialStreamKey, cholesky_lower, sample_trial_parameters,
};
use crate::hash::ContentHash;

#[test]
fn trial_rng_unit_interval_and_box_muller_spare_cache_behave_cleanly() {
    let mut rng = TrialRng::from_key(TrialStreamKey::new(99, 7, 3));
    for _ in 0..512 {
        let u = rng.next_f64_unit();
        assert!((0.0..1.0).contains(&u), "unit draw out of [0, 1): {u}");
    }

    let mut pair_rng = TrialRng::from_key(TrialStreamKey::new(123, 0, 1));
    let z0 = pair_rng.standard_normal();
    let mut after_first = pair_rng.clone();
    let u_after_first = after_first.next_u64();
    let z1 = pair_rng.standard_normal();
    // Second Box-Muller sample must come from the cached spare without consuming another ChaCha word.
    assert_eq!(pair_rng.next_u64(), u_after_first);
    assert!(z0.is_finite() && z1.is_finite());
    assert_ne!(z0.to_bits(), z1.to_bits());
}

#[test]
fn distribution_parameter_and_bound_validation_rejects_invalid_inputs() {
    let mut rng = TrialRng::from_key(TrialStreamKey::new(1, 0, 0));

    assert!(matches!(
        rng.uniform(5.0, 5.0),
        Err(MonteCarloError::InvalidBounds { .. })
    ));
    assert!(matches!(
        rng.uniform(2.0, -1.0),
        Err(MonteCarloError::InvalidBounds { .. })
    ));
    assert!(matches!(
        rng.uniform(f64::NAN, 1.0),
        Err(MonteCarloError::InvalidBounds { .. })
    ));
    assert!(matches!(
        rng.discrete_uniform(10, 9),
        Err(MonteCarloError::InvalidDiscreteBounds { .. })
    ));
    assert_eq!(rng.discrete_uniform(-7, -7), Ok(-7));
    let full_range_draw = rng.discrete_uniform(i64::MIN, i64::MAX);
    assert!(full_range_draw.is_ok());

    assert!(matches!(
        rng.normal(0.0, 0.0),
        Err(MonteCarloError::InvalidParameter {
            parameter: "std_dev",
            ..
        })
    ));
    assert!(matches!(
        rng.normal(f64::INFINITY, 1.0),
        Err(MonteCarloError::InvalidParameter {
            parameter: "mean",
            ..
        })
    ));
    assert!(matches!(
        rng.log_normal(0.0, -0.5),
        Err(MonteCarloError::InvalidParameter {
            parameter: "sigma_ln",
            ..
        })
    ));
    assert!(matches!(
        rng.truncated_normal(0.0, 1.0, 3.0, 3.0),
        Err(MonteCarloError::InvalidBounds { .. })
    ));
    assert!(matches!(
        DistributionSpec::Constant { value: f64::NAN }.validate(),
        Err(MonteCarloError::InvalidParameter {
            parameter: "value",
            ..
        })
    ));
}

#[test]
fn truncated_normal_rejection_limit_triggers_on_unreachable_tail() {
    let mut rng = TrialRng::from_key(TrialStreamKey::new(42, 0, 0));
    let err = rng.truncated_normal(0.0, 1.0, 20.0, 21.0).unwrap_err();
    assert!(matches!(
        err,
        MonteCarloError::TruncatedNormalRejectionLimitExceeded {
            max_attempts: 10_000,
            ..
        }
    ));
}

#[test]
fn cholesky_lower_rejects_bad_dimensions_asymmetry_and_non_spd() {
    assert!(matches!(
        cholesky_lower(&[], 0),
        Err(MonteCarloError::InvalidDimension {
            expected: 1,
            actual: 0
        })
    ));
    assert!(matches!(
        cholesky_lower(&[1.0, 0.0, 1.0], 2),
        Err(MonteCarloError::NonSquareMatrix { len: 3, dim: 2 })
    ));
    assert!(matches!(
        cholesky_lower(&[1.0, 0.5, 0.1, 1.0], 2),
        Err(MonteCarloError::NotSymmetricPositiveDefinite { row: 1, col: 0 })
    ));
    assert!(matches!(
        cholesky_lower(&[1.0, 1.0, 1.0, 1.0], 2),
        Err(MonteCarloError::NotSymmetricPositiveDefinite { row: 1, col: 1 })
    ));
    assert!(matches!(
        cholesky_lower(&[1.0, 2.0, 2.0, 1.0], 2),
        Err(MonteCarloError::NotSymmetricPositiveDefinite { row: 1, col: 1 })
    ));
    assert!(matches!(
        cholesky_lower(&[1.0, f64::NAN, f64::NAN, 1.0], 2),
        Err(MonteCarloError::NotSymmetricPositiveDefinite { row: 1, col: 0 })
    ));

    let mut rng = TrialRng::from_key(TrialStreamKey::new(7, 1, 2));
    assert!(matches!(
        rng.correlated_normal(&[], &[]),
        Err(MonteCarloError::InvalidDimension { .. })
    ));
    assert!(matches!(
        rng.correlated_normal(&[0.0, 0.0], &[1.0, 0.0, 1.0]),
        Err(MonteCarloError::NonSquareMatrix { len: 3, dim: 2 })
    ));
}

#[test]
fn cholesky_lower_reconstructs_tiny_and_nominal_spd_matrices() {
    let cov = [4.0, 2.0, -2.0, 2.0, 10.0, 5.0, -2.0, 5.0, 14.0];
    let lower = cholesky_lower(&cov, 3).unwrap();
    for i in 0..3 {
        for j in 0..3 {
            if j > i {
                assert_eq!(lower[i * 3 + j], 0.0);
            }
            let dot: f64 = (0..3).map(|k| lower[i * 3 + k] * lower[j * 3 + k]).sum();
            assert!((dot - cov[i * 3 + j]).abs() < 1e-12);
        }
    }
    let tiny = [1.0e-20, 0.0, 0.0, 1.0e-20];
    let tiny_l = cholesky_lower(&tiny, 2).unwrap();
    assert!((tiny_l[0] - 1.0e-10).abs() < 1e-22);
    assert!((tiny_l[3] - 1.0e-10).abs() < 1e-22);
}

#[test]
fn config_validation_rejects_empty_and_duplicate_parameter_names() {
    let empty_name = MonteCarloConfig {
        campaign_name: "demo".to_owned(),
        run_seed: 1,
        trial_count: 10,
        dispersions: vec![ParameterDispersionSpec {
            name: "   ".to_owned(),
            stream_tag: 0,
            distribution: DistributionSpec::Constant { value: 1.0 },
        }],
    };
    assert_eq!(
        empty_name.validate(),
        Err(MonteCarloError::EmptyParameterName)
    );

    let dup_name = MonteCarloConfig {
        campaign_name: "demo".to_owned(),
        run_seed: 1,
        trial_count: 10,
        dispersions: vec![
            ParameterDispersionSpec {
                name: "cd".to_owned(),
                stream_tag: 1,
                distribution: DistributionSpec::Constant { value: 2.2 },
            },
            ParameterDispersionSpec {
                name: "cd".to_owned(),
                stream_tag: 2,
                distribution: DistributionSpec::Normal {
                    mean: 2.2,
                    std_dev: 0.1,
                },
            },
        ],
    };
    assert_eq!(
        dup_name.validate(),
        Err(MonteCarloError::DuplicateParameterName {
            name: "cd".to_owned()
        })
    );
    assert!(sample_trial_parameters(&dup_name, 0).is_err());
}

#[test]
fn manifest_integrity_detects_tampering_and_canonicalizes_negative_zero() {
    let make_cfg = |val: f64| MonteCarloConfig {
        campaign_name: "canon".to_owned(),
        run_seed: 55,
        trial_count: 100,
        dispersions: vec![ParameterDispersionSpec {
            name: "bias".to_owned(),
            stream_tag: 0,
            distribution: DistributionSpec::Normal {
                mean: val,
                std_dev: 0.5,
            },
        }],
    };
    let m_neg =
        MonteCarloRunManifest::build(make_cfg(-0.0), Some(ContentHash([7_u8; 32]))).unwrap();
    let m_pos = MonteCarloRunManifest::build(make_cfg(0.0), Some(ContentHash([7_u8; 32]))).unwrap();
    assert_eq!(m_neg.config_hash, m_pos.config_hash);
    assert_eq!(m_neg.digest(), m_pos.digest());
    assert!(m_neg.digest_eq(&m_pos));
    m_neg.verify_integrity().unwrap();

    let mut tampered = m_pos.clone();
    tampered.config.trial_count = 101;
    assert!(matches!(
        tampered.verify_integrity(),
        Err(MonteCarloError::ManifestIntegrityMismatch { .. })
    ));

    let mut bad_ver = m_pos;
    bad_ver.schema_version = 99;
    assert_eq!(
        bad_ver.verify_integrity(),
        Err(MonteCarloError::UnsupportedSchemaVersion { version: 99 })
    );
}

#[test]
fn welford_summary_and_empirical_quantiles_match_exact_hand_calculations() {
    let empty = EnsembleSummary::new();
    assert_eq!(empty.count(), 0);
    assert_eq!(empty.mean(), None);
    assert_eq!(empty.variance(), None);
    assert_eq!(empty.sample_variance(), None);
    assert_eq!(empty.std_dev(), None);
    assert_eq!(empty.sample_std_dev(), None);
    assert_eq!(empty.min(), None);
    assert_eq!(empty.max(), None);
    assert_eq!(EnsembleSummary::empirical_quantile(&[], 0.5), None);

    let single = EnsembleSummary::from_samples(&[42.0]);
    assert_eq!(single.count(), 1);
    assert_eq!(single.mean(), Some(42.0));
    assert_eq!(single.variance(), Some(0.0));
    assert_eq!(single.sample_variance(), None);
    assert_eq!(single.std_dev(), Some(0.0));
    assert_eq!(single.min(), Some(42.0));
    assert_eq!(single.max(), Some(42.0));
    assert_eq!(
        EnsembleSummary::empirical_quantile(&[42.0], 0.5),
        Some(42.0)
    );

    let summary = EnsembleSummary::from_samples(&[2.0, 4.0, 4.0, 4.0, 5.0, 5.0, 7.0, 9.0]);
    assert_eq!(summary.count(), 8);
    assert!((summary.mean().unwrap() - 5.0).abs() < 1e-12);
    assert!((summary.variance().unwrap() - 4.0).abs() < 1e-12);
    assert!((summary.std_dev().unwrap() - 2.0).abs() < 1e-12);
    assert!((summary.sample_variance().unwrap() - (32.0 / 7.0)).abs() < 1e-12);
    assert_eq!(summary.min(), Some(2.0));
    assert_eq!(summary.max(), Some(9.0));

    let sorted = [10.0, 20.0, 30.0, 40.0];
    assert_eq!(
        EnsembleSummary::empirical_quantile(&sorted, 0.0),
        Some(10.0)
    );
    assert_eq!(
        EnsembleSummary::empirical_quantile(&sorted, 0.5),
        Some(25.0)
    );
    assert_eq!(
        EnsembleSummary::empirical_quantile(&sorted, 1.0),
        Some(40.0)
    );
    assert_eq!(EnsembleSummary::empirical_quantile(&sorted, -0.01), None);
    assert_eq!(EnsembleSummary::empirical_quantile(&sorted, 1.01), None);
    assert_eq!(EnsembleSummary::empirical_quantile(&sorted, f64::NAN), None);
}
