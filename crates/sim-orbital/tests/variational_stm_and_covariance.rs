//! Integration verification for 42-state variational `STM` propagation (`DOP853`),
//! `6x6` covariance / `RIC` transforms, and short-term B-plane encounter collision probability.

use std::sync::Arc;

use glam::{DMat3, DVec3};
use sim_bodies::{BodyCatalog, BodyDescriptor, HarmonicGravity};
use sim_orbital::{
    BuiltinSolarSystem2026_2028, Dop853Tolerances, ForceModel, NaifId, ProcessNoiseQ,
    StateCovariance6, TdbEpoch, VariationalError, compute_bplane_collision_probability,
    icrf_to_ric_rotation, propagate_state_and_stm,
};
use sim_spacetime::TickSpan;

#[cfg(test)]
fn epoch() -> TdbEpoch {
    TdbEpoch::new(845_000_000, 0)
}

#[cfg(test)]
fn mat6_det(m: &[[f64; 6]; 6]) -> f64 {
    let mut a = *m;
    let mut det = 1.0_f64;
    for i in 0..6 {
        let mut piv_r = i;
        let mut max_v = a[i][i].abs();
        for (r, row) in a.iter().enumerate().skip(i + 1) {
            if row[i].abs() > max_v {
                max_v = row[i].abs();
                piv_r = r;
            }
        }
        if piv_r != i {
            a.swap(i, piv_r);
            det = -det;
        }
        let piv = a[i][i];
        if piv.abs() == 0.0 {
            return 0.0;
        }
        det *= piv;
        for r in (i + 1)..6 {
            let f = a[r][i] / piv;
            for c in i..6 {
                a[r][c] -= f * a[i][c];
            }
        }
    }
    det
}

#[cfg(test)]
fn mat6_mul(a: &[[f64; 6]; 6], b: &[[f64; 6]; 6]) -> [[f64; 6]; 6] {
    let mut out = [[0.0_f64; 6]; 6];
    for i in 0..6 {
        for j in 0..6 {
            out[i][j] = (0..6).map(|k| a[i][k] * b[k][j]).sum();
        }
    }
    out
}

#[cfg(test)]
fn symplectic_defect_max(phi: &[[f64; 6]; 6]) -> f64 {
    let mut max_err = 0.0_f64;
    for i in 0..6 {
        for j in 0..6 {
            let val: f64 = (0..3)
                .map(|k| phi[k][i] * phi[k + 3][j] - phi[k + 3][i] * phi[k][j])
                .sum();
            let j_ij = if i + 3 == j {
                1.0
            } else if j + 3 == i {
                -1.0
            } else {
                0.0
            };
            max_err = max_err.max((val - j_ij).abs());
        }
    }
    max_err
}

#[test]
fn stm_matches_finite_differences_and_preserves_symplectic_invariants() {
    let egm96 = HarmonicGravity::egm96_degree_36().expect("EGM96");
    let earth = BodyDescriptor::earth().expect("earth").with_gravity(egm96);
    let bodies = BodyCatalog::standard_solar_system()
        .expect("bodies")
        .with(earth);
    let solar =
        BuiltinSolarSystem2026_2028::load_verified_with_bodies(Arc::new(bodies)).expect("solar");
    let fm = ForceModel::standard_solar_system(solar.constants.clone()).expect("fm");
    let t0 = epoch();
    let tf = t0.add_span(TickSpan::from_seconds_and_subsec_ticks(2_400, 0));
    let r0 = DVec3::new(6_850_000.0, 420_000.0, 1_200_000.0);
    let v0 = DVec3::new(-980.0, 6_450.0, 4_100.0);
    let tol = Dop853Tolerances {
        rtol: 1.0e-12,
        atol_pos_m: 1.0e-8,
        atol_vel_m_s: 1.0e-11,
        ..Dop853Tolerances::default()
    };

    let res = propagate_state_and_stm(
        NaifId::EARTH,
        r0,
        v0,
        t0,
        tf,
        &tol,
        &fm,
        &solar.ephemeris,
        &solar.orientation,
    )
    .expect("stm");

    assert!((mat6_det(&res.stm) - 1.0).abs() < 1.0e-9);
    assert!(symplectic_defect_max(&res.stm) < 1.0e-8);

    let eps = [0.5, 0.5, 0.5, 1.0e-3, 1.0e-3, 1.0e-3];
    for (col, &step) in eps.iter().enumerate() {
        let mut d = [0.0_f64; 6];
        d[col] = step;
        let dr = DVec3::new(d[0], d[1], d[2]);
        let dv = DVec3::new(d[3], d[4], d[5]);
        let plus = propagate_state_and_stm(
            NaifId::EARTH,
            r0 + dr,
            v0 + dv,
            t0,
            tf,
            &tol,
            &fm,
            &solar.ephemeris,
            &solar.orientation,
        )
        .expect("+");
        let minus = propagate_state_and_stm(
            NaifId::EARTH,
            r0 - dr,
            v0 - dv,
            t0,
            tf,
            &tol,
            &fm,
            &solar.ephemeris,
            &solar.orientation,
        )
        .expect("-");
        let fd_r = (plus.pos_rel_icrf_m - minus.pos_rel_icrf_m) / (2.0 * step);
        let fd_v = (plus.vel_rel_icrf_m_s - minus.vel_rel_icrf_m_s) / (2.0 * step);
        let stm_r = DVec3::new(res.stm[0][col], res.stm[1][col], res.stm[2][col]);
        let stm_v = DVec3::new(res.stm[3][col], res.stm[4][col], res.stm[5][col]);
        assert!((stm_r - fd_r).length() / fd_r.length().max(1.0) < 1.0e-7);
        assert!((stm_v - fd_v).length() / fd_v.length().max(1.0e-6) < 1.0e-7);
    }
}

#[test]
fn stm_semigroup_cislunar_and_epoch_boundary_contracts() {
    let solar = BuiltinSolarSystem2026_2028::load_verified().expect("solar");
    let fm = ForceModel::standard_solar_system(solar.constants.clone()).expect("fm");
    let t0 = epoch();
    let t1 = t0.add_span(TickSpan::from_seconds_and_subsec_ticks(21_600, 0));
    let t2 = t0.add_span(TickSpan::from_seconds_and_subsec_ticks(43_200, 0));
    let r0 = DVec3::new(95_000_000.0, 68_000_000.0, 22_000_000.0);
    let v0 = DVec3::new(-920.0, 1_350.0, 240.0);
    let tol = Dop853Tolerances::default();

    let s01 = propagate_state_and_stm(
        NaifId::EARTH,
        r0,
        v0,
        t0,
        t1,
        &tol,
        &fm,
        &solar.ephemeris,
        &solar.orientation,
    )
    .expect("0->1");
    let s12 = propagate_state_and_stm(
        NaifId::EARTH,
        s01.pos_rel_icrf_m,
        s01.vel_rel_icrf_m_s,
        t1,
        t2,
        &tol,
        &fm,
        &solar.ephemeris,
        &solar.orientation,
    )
    .expect("1->2");
    let s02 = propagate_state_and_stm(
        NaifId::EARTH,
        r0,
        v0,
        t0,
        t2,
        &tol,
        &fm,
        &solar.ephemeris,
        &solar.orientation,
    )
    .expect("0->2");

    let composed = mat6_mul(&s12.stm, &s01.stm);
    for i in 0..6 {
        for j in 0..6 {
            let scale = s02.stm[i][j].abs().max(1.0);
            assert!((composed[i][j] - s02.stm[i][j]).abs() / scale < 1.0e-9);
        }
    }
    assert!((mat6_det(&s02.stm) - 1.0).abs() < 1.0e-9);
    assert!(symplectic_defect_max(&s02.stm) < 1.0e-8);
}

#[test]
fn stm_zero_duration_and_backward_epoch_contracts() {
    let solar = BuiltinSolarSystem2026_2028::load_verified().expect("solar");
    let fm = ForceModel::standard_solar_system(solar.constants.clone()).expect("fm");
    let t0 = epoch();
    let t1 = t0.add_span(TickSpan::from_seconds_and_subsec_ticks(21_600, 0));
    let r0 = DVec3::new(95_000_000.0, 68_000_000.0, 22_000_000.0);
    let v0 = DVec3::new(-920.0, 1_350.0, 240.0);
    let tol = Dop853Tolerances::default();

    let zero = propagate_state_and_stm(
        NaifId::EARTH,
        r0,
        v0,
        t0,
        t0,
        &tol,
        &fm,
        &solar.ephemeris,
        &solar.orientation,
    )
    .expect("zero");
    assert_eq!(zero.pos_rel_icrf_m, r0);
    assert_eq!(zero.stm[0][0], 1.0);
    assert_eq!(zero.stm[0][3], 0.0);
    let err = propagate_state_and_stm(
        NaifId::EARTH,
        r0,
        v0,
        t1,
        t0,
        &tol,
        &fm,
        &solar.ephemeris,
        &solar.orientation,
    )
    .expect_err("backward");
    assert!(matches!(err, VariationalError::InvalidEpochInterval { .. }));
}

#[test]
fn covariance_propagation_serde_and_ric_frame_roundtrip() {
    let solar = BuiltinSolarSystem2026_2028::load_verified().expect("solar");
    let fm = ForceModel::standard_solar_system(solar.constants.clone()).expect("fm");
    let t0 = epoch();
    let t1 = t0.add_span(TickSpan::from_seconds_and_subsec_ticks(900, 0));
    let r0 = DVec3::new(7_000_000.0, 0.0, 0.0);
    let v0 = DVec3::new(0.0, 5_500.0, 5_200.0);

    let res = propagate_state_and_stm(
        NaifId::EARTH,
        r0,
        v0,
        t0,
        t1,
        &Dop853Tolerances::default(),
        &fm,
        &solar.ephemeris,
        &solar.orientation,
    )
    .expect("stm");

    let p0_ric = StateCovariance6::from_diagonal_sigmas([25.0, 120.0, 40.0, 0.05, 0.15, 0.08]);
    let p0_icrf = StateCovariance6::from_ric_frame(&p0_ric, r0, v0).expect("from_ric");
    assert!(p0_icrf.is_positive_semidefinite());
    let p0_ric_rt = p0_icrf.in_ric_frame(r0, v0).expect("in_ric");
    for i in 0..6 {
        for j in 0..6 {
            let scale = (p0_ric.as_matrix()[i][i] * p0_ric.as_matrix()[j][j])
                .sqrt()
                .max(1.0e-12);
            let diff = (p0_ric_rt.as_matrix()[i][j] - p0_ric.as_matrix()[i][j]).abs();
            assert!(
                diff / scale < 1.0e-13,
                "[{i}][{j}]: diff={diff:e}, scale={scale:e}"
            );
        }
    }

    let q = ProcessNoiseQ::from_accel_psd(1.0e-9, 900.0);
    let pf = p0_icrf.propagate(&res.stm, Some(&q));
    assert!(pf.is_positive_semidefinite());
    for i in 0..6 {
        assert!(pf.diagonal_sigmas()[i] > 0.0);
        for j in 0..6 {
            assert_eq!(pf.as_matrix()[i][j], pf.as_matrix()[j][i]);
        }
    }
    let json = serde_json::to_string(&pf).expect("ser");
    let back: StateCovariance6 = serde_json::from_str(&json).expect("de");
    assert_eq!(pf, back);

    let rot = icrf_to_ric_rotation(res.pos_rel_icrf_m, res.vel_rel_icrf_m_s).expect("ric");
    assert!(((rot * rot.transpose()) - DMat3::IDENTITY).abs_diff_eq(DMat3::ZERO, 1.0e-14));
    assert!((rot.determinant() - 1.0).abs() < 1.0e-14);
}

#[cfg(test)]
fn polar_gl8_bplane_reference(xm: f64, ym: f64, sx: f64, sy: f64, r_c: f64) -> f64 {
    let gl8 = [
        (-9.602_898_564_975_363e-1, 1.012_285_362_903_762_6e-1),
        (-7.966_664_774_136_267e-1, 2.223_810_344_533_744_8e-1),
        (-5.255_324_099_163_29e-1, 3.137_066_458_778_872_7e-1),
        (-1.834_346_424_956_498e-1, 3.626_837_833_783_62e-1),
        (1.834_346_424_956_498e-1, 3.626_837_833_783_62e-1),
        (5.255_324_099_163_29e-1, 3.137_066_458_778_872_7e-1),
        (7.966_664_774_136_267e-1, 2.223_810_344_533_744_8e-1),
        (9.602_898_564_975_363e-1, 1.012_285_362_903_762_6e-1),
    ];
    let n_r = 6;
    let n_t = 12;
    let dr = r_c / f64::from(n_r);
    let dth = std::f64::consts::TAU / f64::from(n_t);
    let mut sum_2d = 0.0_f64;
    for ir in 0..n_r {
        let r_mid = (f64::from(ir) + 0.5) * dr;
        for &(nr, wr) in &gl8 {
            let r = r_mid + 0.5 * dr * nr;
            for it in 0..n_t {
                let th_mid = (f64::from(it) + 0.5) * dth;
                for &(nt, wt) in &gl8 {
                    let (st, ct) = (th_mid + 0.5 * dth * nt).sin_cos();
                    let zx = (r * ct - xm) / sx;
                    let zy = (r * st - ym) / sy;
                    sum_2d +=
                        (0.5 * dr * wr) * (0.5 * dth * wt) * r * (-0.5 * (zx * zx + zy * zy)).exp();
                }
            }
        }
    }
    sum_2d / (2.0 * std::f64::consts::PI * sx * sy)
}

#[test]
fn bplane_collision_probability_matches_analytical_and_2d_quadrature() {
    let p_vel = DVec3::new(0.0, 7_500.0, 0.0);
    let s_vel = DVec3::new(7_500.0, 0.0, 0.0);
    for &(r_c, sigma) in &[(5.0, 50.0), (20.0, 20.0), (120.0, 40.0)] {
        let cov = StateCovariance6::from_diagonal_sigmas([sigma / 2.0_f64.sqrt(); 6]);
        let out = compute_bplane_collision_probability(
            DVec3::ZERO,
            p_vel,
            &cov,
            DVec3::ZERO,
            s_vel,
            &cov,
            r_c,
        )
        .expect("zero miss");
        let exact = 1.0 - (-r_c * r_c / (2.0 * sigma * sigma)).exp();
        assert!((out.probability - exact).abs() < 1.0e-12);
        assert!(out.miss_distance_m < 1.0e-14);
    }

    let cov1 = StateCovariance6::from_diagonal_sigmas([180.0, 40.0, 25.0, 0.1, 0.1, 0.1])
        .in_ric_frame(DVec3::new(7_000_000.0, 1_000_000.0, 500_000.0), p_vel)
        .expect("ric1");
    let cov2 = StateCovariance6::from_diagonal_sigmas([60.0, 220.0, 15.0, 0.1, 0.1, 0.1])
        .in_ric_frame(DVec3::new(6_900_000.0, -1_200_000.0, 300_000.0), s_vel)
        .expect("ric2");
    let p_pos = DVec3::new(7_000_000.0, 0.0, 0.0);
    let s_pos = p_pos + DVec3::new(35.0, 35.0, -18.0);
    let r_c = 25.0_f64;
    let res = compute_bplane_collision_probability(p_pos, p_vel, &cov1, s_pos, s_vel, &cov2, r_c)
        .expect("aniso");
    let ref_2d = polar_gl8_bplane_reference(
        res.miss_b_plane_m[0],
        res.miss_b_plane_m[1],
        res.b_plane_sigmas_m[0],
        res.b_plane_sigmas_m[1],
        r_c,
    );
    assert!((res.probability - ref_2d).abs() / ref_2d < 1.0e-12);

    let shifted = compute_bplane_collision_probability(
        p_pos,
        p_vel,
        &cov1,
        s_pos + 2.5 * (s_vel - p_vel),
        s_vel,
        &cov2,
        r_c,
    )
    .expect("shifted");
    assert!((shifted.probability - res.probability).abs() < 1.0e-12);
    assert!((shifted.miss_distance_m - res.miss_distance_m).abs() < 1.0e-12);
}

#[test]
fn bplane_tail_error_contracts_and_monte_carlo() {
    let p_vel = DVec3::new(0.0, 7_500.0, 0.0);
    let s_vel = DVec3::new(7_500.0, 0.0, 0.0);
    let cov = StateCovariance6::from_diagonal_sigmas([80.0, 30.0, 20.0, 0.1, 0.1, 0.1]);
    let s_pos = DVec3::new(25.0, 30.0, -10.0);
    let r_c = 40.0_f64;
    let res =
        compute_bplane_collision_probability(DVec3::ZERO, p_vel, &cov, s_pos, s_vel, &cov, r_c)
            .expect("mc");

    // Independent Box-Muller Monte Carlo verification (N = 200_000).
    let [sx, sy] = res.b_plane_sigmas_m;
    let [xm, ym] = res.miss_b_plane_m;
    let mut state: u64 = 0x517cc1b727220a95;
    let mut next_u01 = || -> f64 {
        state = state.wrapping_mul(6364136223846793005).wrapping_add(1);
        ((state >> 11) as f64 + 0.5) / ((1_u64 << 53) as f64)
    };
    let n_mc = 200_000_u32;
    let mut hits = 0_u32;
    for _ in 0..n_mc {
        let u1 = next_u01().max(1.0e-15);
        let u2 = next_u01();
        let rad = (-2.0 * u1.ln()).sqrt();
        let (st, ct) = (std::f64::consts::TAU * u2).sin_cos();
        let x = xm + sx * rad * ct;
        let y = ym + sy * rad * st;
        if x * x + y * y <= r_c * r_c {
            hits += 1;
        }
    }
    let mc_p = f64::from(hits) / f64::from(n_mc);
    let mc_sigma = (res.probability * (1.0 - res.probability) / f64::from(n_mc)).sqrt();
    assert!(
        (res.probability - mc_p).abs() < 4.5 * mc_sigma,
        "B-plane {} vs MC {mc_p} +/- {mc_sigma}",
        res.probability
    );

    let far = compute_bplane_collision_probability(
        DVec3::ZERO,
        p_vel,
        &StateCovariance6::from_diagonal_sigmas([10.0; 6]),
        DVec3::new(0.0, 0.0, 500.0),
        s_vel,
        &StateCovariance6::from_diagonal_sigmas([10.0; 6]),
        5.0,
    )
    .expect("far");
    assert!(far.probability >= 0.0 && far.probability < 1.0e-50);
}

#[test]
fn bplane_error_contracts() {
    let p_vel = DVec3::new(0.0, 7_500.0, 0.0);
    let s_vel = DVec3::new(7_500.0, 0.0, 0.0);
    let cov = StateCovariance6::from_diagonal_sigmas([80.0, 30.0, 20.0, 0.1, 0.1, 0.1]);
    let s_pos = DVec3::new(25.0, 30.0, -10.0);
    assert!(matches!(
        compute_bplane_collision_probability(DVec3::ZERO, p_vel, &cov, s_pos, p_vel, &cov, 10.0),
        Err(VariationalError::DegenerateEncounter)
    ));
    assert!(matches!(
        compute_bplane_collision_probability(DVec3::ZERO, p_vel, &cov, s_pos, s_vel, &cov, 0.0),
        Err(VariationalError::InvalidCombinedRadius { .. })
    ));
    assert!(matches!(
        compute_bplane_collision_probability(
            DVec3::ZERO,
            p_vel,
            &StateCovariance6::ZERO,
            s_pos,
            s_vel,
            &StateCovariance6::ZERO,
            10.0,
        ),
        Err(VariationalError::SingularBPlaneCovariance)
    ));
    let mut bad = *cov.as_matrix();
    bad[0][0] = -1.0;
    assert!(matches!(
        compute_bplane_collision_probability(
            DVec3::ZERO,
            p_vel,
            &StateCovariance6::from_matrix(bad),
            s_pos,
            s_vel,
            &cov,
            10.0,
        ),
        Err(VariationalError::NonPositiveSemidefiniteCovariance)
    ));
}
