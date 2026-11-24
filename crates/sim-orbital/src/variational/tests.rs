use glam::DVec3;

use crate::variational::{
    ProcessNoiseQ, StateCovariance6,
    bplane::{eval_gk15_panel, integrate_bplane_disk_probability, normal_interval_prob},
    icrf_to_ric_rotation,
};

#[test]
fn gk15_integrates_polynomials_up_to_degree_23_exactly() {
    for deg in 0_i32..=23 {
        let (val, err) = eval_gk15_panel(-1.0, 1.0, &|x| x.powi(deg));
        let exact = if deg % 2 == 0 {
            2.0 / f64::from(deg + 1)
        } else {
            0.0
        };
        assert!((val - exact).abs() < 1.0e-14, "deg {deg}: {val} vs {exact}");
        if deg <= 13 {
            assert!(err < 1.0e-14, "deg {deg} g7 error: {err}");
        }
    }
}

#[test]
fn normal_interval_prob_matches_small_delta_and_deep_tails() {
    // Symmetric straddling zero: erf(1)
    let p_center = normal_interval_prob(-1.0, 1.0);
    assert!((p_center - libm::erf(1.0)).abs() < 1.0e-15);

    // Tiny width far in tail: compare against exact float-interval midpoint integral.
    let ap: f64 = 10.0;
    let bp: f64 = ap + 1.0e-10;
    let delta = bp - ap;
    let mid = 0.5 * (ap + bp);
    let p_narrow = normal_interval_prob(ap, bp);
    let expected = (delta / std::f64::consts::PI.sqrt()) * libm::exp(-mid * mid);
    assert!(
        (p_narrow - expected).abs() / expected < 1.0e-14,
        "{p_narrow:e} vs {expected:e}"
    );
}

#[test]
fn circular_disk_quadrature_matches_closed_form() {
    for &(r_c, sigma) in &[(1.0, 100.0), (5.0, 10.0), (15.0, 10.0), (80.0, 10.0)] {
        let num = integrate_bplane_disk_probability(0.0, 0.0, sigma, sigma, r_c);
        let exact = -libm::expm1(-(r_c * r_c) / (2.0 * sigma * sigma));
        assert!(
            (num - exact).abs() < 1.0e-13,
            "r_c={r_c}, sigma={sigma}: {num} vs {exact}"
        );
    }
}

#[test]
fn psd_check_handles_rank_deficient_and_extreme_pos_vel_scale() {
    assert!(StateCovariance6::ZERO.is_positive_semidefinite());
    let valid_scaled =
        StateCovariance6::from_diagonal([1.0e8, 1.0e8, 1.0e8, 1.0e-6, 1.0e-6, 1.0e-6]);
    assert!(valid_scaled.is_positive_semidefinite());

    let bad_vel = StateCovariance6::from_diagonal([1.0e8, 1.0e8, 1.0e8, 1.0e-6, 1.0e-6, -1.0e-6]);
    assert!(!bad_vel.is_positive_semidefinite());

    let mut bad_corr = [[0.0; 6]; 6];
    bad_corr[0][0] = 4.0;
    bad_corr[1][1] = 4.0;
    bad_corr[0][1] = 4.5;
    bad_corr[1][0] = 4.5;
    assert!(!StateCovariance6::from_matrix(bad_corr).is_positive_semidefinite());

    let mut asymmetric = [[0.0; 6]; 6];
    asymmetric[0][0] = 1.0;
    asymmetric[1][1] = 1.0;
    asymmetric[0][1] = 0.2;
    assert!(!StateCovariance6::from_matrix(asymmetric).is_positive_semidefinite());
}

#[test]
fn cwna_process_noise_is_psd_and_ric_is_right_handed() {
    let q = ProcessNoiseQ::from_accel_psd(1.0e-8, 60.0);
    assert!(q.is_positive_semidefinite());
    let r = DVec3::new(7_000_000.0, 0.0, 0.0);
    let v = DVec3::new(0.0, 7_500.0, 0.0);
    let rot = icrf_to_ric_rotation(r, v).expect("ric");
    assert!((rot.determinant() - 1.0).abs() < 1.0e-15);
    assert!((rot * DVec3::X - DVec3::X).length() < 1.0e-15);
    assert!((rot * DVec3::Y - DVec3::Y).length() < 1.0e-15);
    assert!((rot * DVec3::Z - DVec3::Z).length() < 1.0e-15);
}
