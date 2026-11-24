//! Integration tests for `sim-spacetime` reproducible Monte Carlo streams,
//! cross-platform pinned golden vectors, empirical distribution moments,
//! Cholesky-correlated Gaussians, and tamper-evident run manifests.

use std::collections::{BTreeSet, HashSet};

use sim_spacetime::{
    ContentHash, DistributionSpec, EnsembleSummary, MONTE_CARLO_MANIFEST_SCHEMA_VERSION,
    MonteCarloConfig, MonteCarloError, MonteCarloRunManifest, ParameterDispersionSpec, TrialRng,
    TrialStreamKey, cholesky_lower, sample_trial_parameters,
};

fn sample_config() -> MonteCarloConfig {
    MonteCarloConfig {
        campaign_name: "lunar-descent-dispersion".to_owned(),
        run_seed: 0x2026_1124_0001,
        trial_count: 100,
        dispersions: vec![
            ParameterDispersionSpec {
                name: "dry_mass_kg".to_owned(),
                stream_tag: 1,
                distribution: DistributionSpec::Constant { value: 4_850.0 },
            },
            ParameterDispersionSpec {
                name: "isp_scale".to_owned(),
                stream_tag: 2,
                distribution: DistributionSpec::Uniform {
                    min: 0.985,
                    max: 1.015,
                },
            },
            ParameterDispersionSpec {
                name: "entry_az_bias_rad".to_owned(),
                stream_tag: 0,
                distribution: DistributionSpec::Normal {
                    mean: 0.0,
                    std_dev: 1.5e-3,
                },
            },
            ParameterDispersionSpec {
                name: "entry_el_bias_rad".to_owned(),
                stream_tag: 0,
                distribution: DistributionSpec::Normal {
                    mean: 0.0,
                    std_dev: 1.5e-3,
                },
            },
            ParameterDispersionSpec {
                name: "cd_multiplier".to_owned(),
                stream_tag: 3,
                distribution: DistributionSpec::LogNormal {
                    mu_ln: -0.02,
                    sigma_ln: 0.08,
                },
            },
            ParameterDispersionSpec {
                name: "gimbal_misalignment_rad".to_owned(),
                stream_tag: 4,
                distribution: DistributionSpec::TruncatedNormal {
                    mean: 0.0,
                    std_dev: 2.0e-3,
                    min: -5.0e-3,
                    max: 5.0e-3,
                },
            },
        ],
    }
}

#[test]
fn pinned_golden_vectors_match_exact_bits() {
    let key = TrialStreamKey::new(0x5A17_2026_0001, 42, 7);

    let mut rng_u64 = TrialRng::from_key(key);
    let u64_draws = [
        rng_u64.next_u64(),
        rng_u64.next_u64(),
        rng_u64.next_u64(),
        rng_u64.next_u64(),
    ];

    let mut rng_unit = TrialRng::from_key(key);
    let unit_bits = [
        rng_unit.next_f64_unit().to_bits(),
        rng_unit.next_f64_unit().to_bits(),
        rng_unit.next_f64_unit().to_bits(),
        rng_unit.next_f64_unit().to_bits(),
    ];

    let mut rng_norm = TrialRng::from_key(key);
    let normal_bits = [
        rng_norm.standard_normal().to_bits(),
        rng_norm.standard_normal().to_bits(),
        rng_norm.standard_normal().to_bits(),
        rng_norm.standard_normal().to_bits(),
    ];

    let mut rng_logn = TrialRng::from_key(key);
    let log_normal_bits = [
        rng_logn.log_normal(-0.5, 0.25).unwrap().to_bits(),
        rng_logn.log_normal(-0.5, 0.25).unwrap().to_bits(),
    ];

    assert_eq!(
        u64_draws,
        [
            0x9920_8f74_d6a4_b2e5,
            0x4e92_1582_ae2e_f004,
            0x16a7_a826_d1b1_7b61,
            0xecc1_9327_b809_be1b,
        ]
    );
    assert_eq!(
        unit_bits,
        [
            0x3fe3_2411_ee9a_d496,
            0x3fd3_a485_60ab_8bbc,
            0x3fb6_a7a8_26d1_b178,
            0x3fed_9832_64f7_0137,
        ]
    );
    assert_eq!(
        normal_bits,
        [
            0xbfd6_b654_55e7_70b1,
            0x3fee_63b8_298d_af80,
            0x3fff_6096_1e51_6fe7,
            0xbff0_07b3_bf95_fe1e,
        ]
    );
    assert_eq!(
        log_normal_bits,
        [0x3fe1_c2dd_f8f0_0417, 0x3fe8_9c2a_f2a3_ce69]
    );
}

#[test]
fn trial_order_subset_and_parameter_stream_independence() {
    let cfg = sample_config();
    let sweep_forward: Vec<_> = (0..100)
        .map(|i| sample_trial_parameters(&cfg, i).unwrap())
        .collect();
    let solo_42 = sample_trial_parameters(&cfg, 42).unwrap();
    for (k, v_solo) in &solo_42 {
        assert_eq!(v_solo.to_bits(), sweep_forward[42][k].to_bits());
    }

    for i in (0..100_u64).rev() {
        let rev_map = sample_trial_parameters(&cfg, i).unwrap();
        for (k, v_rev) in &rev_map {
            assert_eq!(v_rev.to_bits(), sweep_forward[i as usize][k].to_bits());
        }
    }

    // Reordering and adding a new parameter must not perturb any existing parameter's draws.
    let mut extended = cfg.clone();
    extended.dispersions.reverse();
    extended.dispersions.push(ParameterDispersionSpec {
        name: "solar_flux_scale".to_owned(),
        stream_tag: 99,
        distribution: DistributionSpec::Uniform {
            min: 0.95,
            max: 1.05,
        },
    });
    let extended_42 = sample_trial_parameters(&extended, 42).unwrap();
    for (k, v_orig) in &solo_42 {
        assert_eq!(extended_42[k].to_bits(), v_orig.to_bits());
    }

    // Parameters sharing `stream_tag = 0` must still be independent because `name` is mixed in.
    let same_values = (0..100)
        .filter(|&i| {
            sweep_forward[i]["entry_az_bias_rad"].to_bits()
                == sweep_forward[i]["entry_el_bias_rad"].to_bits()
        })
        .count();
    assert_eq!(same_values, 0);
}

#[test]
fn uniform_and_discrete_uniform_empirical_moments_and_bounds() {
    let mut rng = TrialRng::from_key(TrialStreamKey::new(11, 0, 1));
    let mut u_sum = EnsembleSummary::new();
    let mut u_samples = Vec::with_capacity(20_000);
    for _ in 0..20_000 {
        let x = rng.uniform(-3.0, 5.0).unwrap();
        assert!((-3.0..5.0).contains(&x));
        u_sum.push(x);
        u_samples.push(x);
    }
    assert!((u_sum.mean().unwrap() - 1.0).abs() < 0.05);
    assert!((u_sum.sample_variance().unwrap() - (64.0 / 12.0)).abs() < 0.15);
    u_samples.sort_by(f64::total_cmp);
    let median = EnsembleSummary::empirical_quantile(&u_samples, 0.5).unwrap();
    assert!((median - 1.0).abs() < 0.1);

    let mut d_rng = TrialRng::from_key(TrialStreamKey::new(11, 1, 2));
    let mut d_sum = EnsembleSummary::new();
    let mut counts = [0_u32; 7];
    for _ in 0..20_000 {
        let k = d_rng.discrete_uniform(-2, 4).unwrap();
        assert!((-2..=4).contains(&k));
        counts[(k + 2) as usize] += 1;
        d_sum.push(k as f64);
    }
    assert!(counts.iter().all(|&c| c > 2_500 && c < 3_200));
    assert!((d_sum.mean().unwrap() - 1.0).abs() < 0.05);
    assert!((d_sum.sample_variance().unwrap() - 4.0).abs() < 0.15);
}

#[test]
fn normal_log_normal_and_truncated_normal_empirical_moments() {
    let mut n_rng = TrialRng::from_key(TrialStreamKey::new(22, 0, 1));
    let mut n_sum = EnsembleSummary::new();
    let mut n_samples = Vec::with_capacity(20_000);
    for _ in 0..20_000 {
        let x = n_rng.normal(12.5, 2.5).unwrap();
        n_sum.push(x);
        n_samples.push(x);
    }
    assert!((n_sum.mean().unwrap() - 12.5).abs() < 0.08);
    assert!((n_sum.sample_variance().unwrap() - 6.25).abs() < 0.25);
    n_samples.sort_by(f64::total_cmp);
    let p95 = EnsembleSummary::empirical_quantile(&n_samples, 0.95).unwrap();
    assert!((p95 - (12.5 + 1.644_853_6 * 2.5)).abs() < 0.15);

    let mut ln_rng = TrialRng::from_key(TrialStreamKey::new(22, 1, 2));
    let mut ln_sum = EnsembleSummary::new();
    let (mu_ln, sigma_ln) = (-0.2_f64, 0.4_f64);
    for _ in 0..20_000 {
        let x = ln_rng.log_normal(mu_ln, sigma_ln).unwrap();
        assert!(x > 0.0);
        ln_sum.push(x);
    }
    let expected_mean = (mu_ln + 0.5 * sigma_ln * sigma_ln).exp();
    let expected_var =
        ((sigma_ln * sigma_ln).exp() - 1.0) * (2.0 * mu_ln + sigma_ln * sigma_ln).exp();
    assert!((ln_sum.mean().unwrap() - expected_mean).abs() < 0.015);
    assert!((ln_sum.sample_variance().unwrap() - expected_var).abs() < 0.015);

    let mut tn_rng = TrialRng::from_key(TrialStreamKey::new(22, 2, 3));
    let mut tn_sum = EnsembleSummary::new();
    for _ in 0..20_000 {
        let x = tn_rng.truncated_normal(0.0, 1.0, -1.5, 1.5).unwrap();
        assert!((-1.5..=1.5).contains(&x));
        tn_sum.push(x);
    }
    assert!(tn_sum.mean().unwrap().abs() < 0.03);
    let tn_var = tn_sum.sample_variance().unwrap();
    assert!(
        tn_var > 0.55 && tn_var < 0.72,
        "unexpected truncated var {tn_var}"
    );
}

#[test]
fn correlated_gaussian_3d_reproduces_target_mean_and_covariance() {
    let mean = [10.0, -5.0, 2.0];
    let cov = [
        4.0, 1.8, -1.2, //
        1.8, 9.0, 2.4, //
        -1.2, 2.4, 16.0,
    ];
    let lower = cholesky_lower(&cov, 3).unwrap();
    let mut rng = TrialRng::from_key(TrialStreamKey::new(77, 3, 5));

    let n = 25_000_usize;
    let mut summaries = [EnsembleSummary::new(); 3];
    let mut samples = Vec::with_capacity(n);
    for _ in 0..n {
        let v = rng.correlated_normal(&mean, &lower).unwrap();
        for axis in 0..3 {
            summaries[axis].push(v[axis]);
        }
        samples.push(v);
    }
    let emp_means = [
        summaries[0].mean().unwrap(),
        summaries[1].mean().unwrap(),
        summaries[2].mean().unwrap(),
    ];
    for axis in 0..3 {
        assert!((emp_means[axis] - mean[axis]).abs() < 0.1);
    }
    for i in 0..3 {
        for j in 0..3 {
            let cov_ij: f64 = samples
                .iter()
                .map(|v| (v[i] - emp_means[i]) * (v[j] - emp_means[j]))
                .sum::<f64>()
                / ((n - 1) as f64);
            assert!(
                (cov_ij - cov[i * 3 + j]).abs() < 0.35,
                "cov[{i},{j}] empirical {cov_ij} vs target {}",
                cov[i * 3 + j]
            );
        }
    }
}

#[test]
fn manifest_hash_sensitivity_and_json_roundtrip() {
    let base_cfg = sample_config();
    let model_a = ContentHash::of_bytes(b"apollo-lm-model-v1");
    let model_b = ContentHash::of_bytes(b"apollo-lm-model-v2");
    let m_base = MonteCarloRunManifest::build(base_cfg.clone(), Some(model_a)).unwrap();
    assert_eq!(m_base.schema_version, MONTE_CARLO_MANIFEST_SCHEMA_VERSION);
    m_base.verify_integrity().unwrap();

    let json = serde_json::to_string_pretty(&m_base).unwrap();
    let decoded: MonteCarloRunManifest = serde_json::from_str(&json).unwrap();
    assert_eq!(decoded, m_base);
    assert_eq!(decoded.digest(), m_base.digest());
    decoded.verify_integrity().unwrap();
    assert_eq!(
        sample_trial_parameters(&decoded.config, 17).unwrap(),
        sample_trial_parameters(&m_base.config, 17).unwrap()
    );

    let mut variants = Vec::new();
    let mut c1 = base_cfg.clone();
    c1.campaign_name = "other-campaign".to_owned();
    variants.push(MonteCarloRunManifest::build(c1, Some(model_a)).unwrap());

    let mut c2 = base_cfg.clone();
    c2.run_seed += 1;
    variants.push(MonteCarloRunManifest::build(c2, Some(model_a)).unwrap());

    let mut c3 = base_cfg.clone();
    c3.trial_count += 1;
    variants.push(MonteCarloRunManifest::build(c3, Some(model_a)).unwrap());

    let mut c4 = base_cfg.clone();
    c4.dispersions[1].distribution = DistributionSpec::Uniform {
        min: 0.985,
        max: 1.016,
    };
    variants.push(MonteCarloRunManifest::build(c4, Some(model_a)).unwrap());

    let mut c5 = base_cfg.clone();
    c5.dispersions[2].stream_tag = 77;
    variants.push(MonteCarloRunManifest::build(c5, Some(model_a)).unwrap());

    variants.push(MonteCarloRunManifest::build(base_cfg.clone(), Some(model_b)).unwrap());
    variants.push(MonteCarloRunManifest::build(base_cfg, None).unwrap());

    let mut seen_hashes = HashSet::new();
    let mut seen_digests = BTreeSet::new();
    seen_hashes.insert(m_base.content_hash());
    seen_digests.insert(m_base.digest());
    for variant in &variants {
        variant.verify_integrity().unwrap();
        assert_ne!(variant.content_hash(), m_base.content_hash());
        assert!(!variant.digest_eq(&m_base));
        assert!(seen_hashes.insert(variant.content_hash()));
        assert!(seen_digests.insert(variant.digest()));
    }

    let mut broken = m_base;
    broken.config.run_seed += 99;
    assert!(matches!(
        broken.verify_integrity(),
        Err(MonteCarloError::ManifestIntegrityMismatch { .. })
    ));
}
