//! Multi-body propagation and spherical harmonics verification: Battin $f(q)$ conditioning,
//! nonsingular polar spherical harmonics, `DOP853` 8th-order convergence, `CONTD8` + Bernstein
//! dense output, and 50-period `CR3BP` Jacobi integral conservation.

use glam::DVec3;
use sim_orbital::{
    Dop853GenericConfig, HarmonicTerm, NaifId, dop853_fixed_steps, dop853_integrate_generic,
    evaluate_bernstein_8, evaluate_spherical_harmonics_fixed, third_body_tidal_accel_icrf,
};
use sim_spacetime::{cos, ln, sin, sqrt};

#[cfg(test)]
fn solve_kepler_elliptic(mean_anom: f64, ecc: f64) -> f64 {
    let mut e_anom = mean_anom;
    for _ in 0..16 {
        let f = e_anom - ecc * sin(e_anom) - mean_anom;
        let fp = 1.0 - ecc * cos(e_anom);
        e_anom -= f / fp;
    }
    e_anom
}

#[test]
fn battin_third_body_conditioning_in_leo_and_lunar_flyby_verified() {
    let mu_sun = 1.327_124_400_412_794_2e20_f64;
    let s_sun = DVec3::new(1.495_978_707e11, 2.5e10, -1.1e10);
    let r_leo = DVec3::new(6_778_137.0, -1_200_000.0, 3_400_000.0);

    let battin = third_body_tidal_accel_icrf(r_leo, s_sun, mu_sun, NaifId::SUN).expect("battin");

    let s2 = s_sun.length_squared();
    let s_norm = sqrt(s2);
    let u = r_leo.dot(s_sun) / s2;
    let eps2 = r_leo.length_squared() / s2;
    let c1 = 3.0 * u;
    let c2 = 7.5 * u * u - 1.5 * eps2;
    let c3 = 17.5 * u * u * u - 7.5 * u * eps2;
    let c4 = 39.375 * u.powi(4) - 26.25 * u * u * eps2 + 1.875 * eps2 * eps2;
    let delta_scale = c1 + c2 + c3 + c4;
    let full_scale = 1.0 + delta_scale;
    let exact_ref = -(mu_sun / (s2 * s_norm)) * (full_scale * r_leo - delta_scale * s_sun);

    let rel_err = (battin - exact_ref).length() / exact_ref.length();
    assert!(
        rel_err < 5.0e-15,
        "Battin f(q) LEO relative error {rel_err:e} exceeded 5e-15"
    );

    let s_moon = DVec3::new(384_400_000.0, 0.0, 0.0);
    let r_perilune = s_moon + DVec3::new(0.0, 1_837_400.0, 0.0);
    let mu_moon = 4.902_800_066e12;
    let flyby = third_body_tidal_accel_icrf(r_perilune, s_moon, mu_moon, NaifId::MOON)
        .expect("close approach");
    let d = r_perilune - s_moon;
    let direct = -mu_moon * (d / d.length().powi(3) + s_moon / s_moon.length().powi(3));
    assert!((flyby - direct).length() / direct.length() < 1.0e-15);
}

#[test]
fn nonsingular_spherical_harmonics_gradient_and_poles_verified() {
    let mu = 3.986_004_418e14_f64;
    let r_eq = 6_378_137.0_f64;
    let terms = vec![
        HarmonicTerm::from_unnormalized_j2(1.082_626_68e-3),
        HarmonicTerm {
            degree: 2,
            order: 1,
            c_bar: -2.5e-6,
            s_bar: 1.8e-6,
        },
        HarmonicTerm {
            degree: 2,
            order: 2,
            c_bar: 2.43e-6,
            s_bar: -1.40e-6,
        },
        HarmonicTerm {
            degree: 3,
            order: 0,
            c_bar: 9.57e-7,
            s_bar: 0.0,
        },
        HarmonicTerm {
            degree: 3,
            order: 1,
            c_bar: 2.03e-6,
            s_bar: 2.48e-7,
        },
        HarmonicTerm {
            degree: 4,
            order: 4,
            c_bar: -1.88e-7,
            s_bar: 3.09e-7,
        },
    ];
    let test_points = [
        DVec3::new(0.0, 0.0, 7_000_000.0),
        DVec3::new(0.0, 0.0, -7_000_000.0),
        DVec3::new(7_100_000.0, 0.0, 0.0),
        DVec3::new(4_200_000.0, -3_800_000.0, 4_500_000.0),
    ];

    let h = 500.0_f64;
    let axes = [DVec3::X, DVec3::Y, DVec3::Z];
    for p in test_points {
        let eval = evaluate_spherical_harmonics_fixed(p, mu, r_eq, &terms);
        assert!(eval.accel_fixed_m_s2.is_finite());
        if p.x == 0.0 && p.y == 0.0 {
            assert!(eval.accel_fixed_m_s2.truncate().length() > 1.0e-7);
        }
        let mut grad_neg = [0.0_f64; 3];
        for (k, &ax) in axes.iter().enumerate() {
            let um2 = evaluate_spherical_harmonics_fixed(p - 2.0 * h * ax, mu, r_eq, &terms)
                .potential_j_kg;
            let um1 =
                evaluate_spherical_harmonics_fixed(p - h * ax, mu, r_eq, &terms).potential_j_kg;
            let up1 =
                evaluate_spherical_harmonics_fixed(p + h * ax, mu, r_eq, &terms).potential_j_kg;
            let up2 = evaluate_spherical_harmonics_fixed(p + 2.0 * h * ax, mu, r_eq, &terms)
                .potential_j_kg;
            grad_neg[k] = -(um2 - 8.0 * um1 + 8.0 * up1 - up2) / (12.0 * h);
        }
        let fd_acc = DVec3::from_array(grad_neg);
        let rel = (eval.accel_fixed_m_s2 - fd_acc).length() / eval.accel_fixed_m_s2.length();
        assert!(
            rel < 1.0e-11,
            "harmonic gradient mismatch at {p:?}: rel={rel:e}"
        );
    }
}

#[test]
fn dop853_eighth_order_convergence_dense_output_and_bernstein_verified() {
    let (mu, a, ecc) = (1.0_f64, 1.0_f64, 0.5_f64);
    let r_peri = a * (1.0 - ecc);
    let v_peri = sqrt(mu * (1.0 + ecc) / (a * (1.0 - ecc)));
    let arc_tau = 1.8_f64;
    let y0 = [r_peri, 0.0, 0.0, 0.0, v_peri, 0.0];

    let rhs = |_t: f64, y: &[f64; 6]| -> Result<[f64; 6], ()> {
        let r2 = y[0] * y[0] + y[1] * y[1] + y[2] * y[2];
        let inv_r3 = 1.0 / (r2 * sqrt(r2));
        Ok([
            y[3],
            y[4],
            y[5],
            -mu * y[0] * inv_r3,
            -mu * y[1] * inv_r3,
            -mu * y[2] * inv_r3,
        ])
    };

    let e_end = solve_kepler_elliptic(arc_tau, ecc);
    let r_end = DVec3::new(
        a * (cos(e_end) - ecc),
        a * sqrt(1.0 - ecc * ecc) * sin(e_end),
        0.0,
    );
    let (y_8, _) = dop853_fixed_steps(y0, arc_tau, 8, rhs).expect("8");
    let (y_16, _) = dop853_fixed_steps(y0, arc_tau, 16, rhs).expect("16");
    let (y_32, _) = dop853_fixed_steps(y0, arc_tau, 32, rhs).expect("32");
    let (y_64, segs_64) = dop853_fixed_steps(y0, arc_tau, 64, rhs).expect("64");
    let err_8 = (DVec3::new(y_8[0], y_8[1], y_8[2]) - r_end).length();
    let err_16 = (DVec3::new(y_16[0], y_16[1], y_16[2]) - r_end).length();
    let err_32 = (DVec3::new(y_32[0], y_32[1], y_32[2]) - r_end).length();
    let err_64 = (DVec3::new(y_64[0], y_64[1], y_64[2]) - r_end).length();
    for (label, ord) in [
        ("8->16", ln(err_8 / err_16) / ln(2.0)),
        ("16->32", ln(err_16 / err_32) / ln(2.0)),
        ("32->64", ln(err_32 / err_64) / ln(2.0)),
    ] {
        assert!((7.7..8.5).contains(&ord), "{label} order={ord:.3}");
    }

    let seg = &segs_64[7];
    let x_sample = 0.37_f64;
    let tau_sample = seg.start_tau_s + x_sample * seg.h_s;
    let (y_d, dy_d, _) = seg.evaluate_tau_with_derivs(tau_sample);
    let e_anom = solve_kepler_elliptic(tau_sample, ecc);
    let r_exact = DVec3::new(
        a * (cos(e_anom) - ecc),
        a * sqrt(1.0 - ecc * ecc) * sin(e_anom),
        0.0,
    );
    let r_mag = 1.0 - ecc * cos(e_anom);
    let v_exact = DVec3::new(
        -sin(e_anom) / r_mag,
        sqrt(1.0 - ecc * ecc) * cos(e_anom) / r_mag,
        0.0,
    );
    assert!((DVec3::new(y_d[0], y_d[1], y_d[2]) - r_exact).length() < 1.0e-8);
    assert!((DVec3::new(dy_d[0], dy_d[1], dy_d[2]) - v_exact).length() < 1.0e-7);

    for (comp, &yd_c) in y_d.iter().enumerate() {
        let b_ctrl = seg.component_bernstein_8(comp);
        assert!((evaluate_bernstein_8(&b_ctrl, x_sample) - yd_c).abs() < 1.0e-14);
    }
}

#[test]
fn cr3bp_jacobi_integral_conserved_over_fifty_synodic_periods() {
    let mu = 0.012_150_585_6_f64;
    let one_m_mu = 1.0 - mu;
    let jacobi = |y: &[f64; 6]| -> f64 {
        let (x, yy, z, vx, vy, vz) = (y[0], y[1], y[2], y[3], y[4], y[5]);
        let r1 = sqrt((x + mu) * (x + mu) + yy * yy + z * z);
        let r2 = sqrt((x - one_m_mu) * (x - one_m_mu) + yy * yy + z * z);
        2.0 * (0.5 * (x * x + yy * yy) + one_m_mu / r1 + mu / r2) - (vx * vx + vy * vy + vz * vz)
    };

    // Bounded 3D Earth-Moon L4 tadpole/horseshoe orbit exercising all 3 spatial axes,
    // Coriolis coupling, and both primaries across 50 synodic periods (~4.5 years).
    let y0 = [
        0.5 - mu + 0.008,
        0.5 * sqrt(3.0) + 0.005,
        0.04,
        -0.006,
        0.004,
        0.02,
    ];
    let cj0 = jacobi(&y0);
    let cfg = Dop853GenericConfig {
        rtol: 2.0e-14,
        atol: 2.0e-15,
        h_min_s: 1.0e-10,
        h_max_s: 0.25,
        h_init_s: 0.05,
        max_steps: 500_000,
    };
    let (yf, _, stats) = dop853_integrate_generic(
        y0,
        50.0 * core::f64::consts::TAU,
        &cfg,
        &[],
        |_t, y| -> Result<[f64; 6], ()> {
            let (x, yy, z, vx, vy, vz) = (y[0], y[1], y[2], y[3], y[4], y[5]);
            let r1_sq = (x + mu) * (x + mu) + yy * yy + z * z;
            let r2_sq = (x - one_m_mu) * (x - one_m_mu) + yy * yy + z * z;
            let inv1 = 1.0 / (r1_sq * sqrt(r1_sq));
            let inv2 = 1.0 / (r2_sq * sqrt(r2_sq));
            Ok([
                vx,
                vy,
                vz,
                2.0 * vy + x - one_m_mu * (x + mu) * inv1 - mu * (x - one_m_mu) * inv2,
                -2.0 * vx + yy * (1.0 - one_m_mu * inv1 - mu * inv2),
                -z * (one_m_mu * inv1 + mu * inv2),
            ])
        },
    )
    .expect("cr3bp 50 periods");

    let dcj = (jacobi(&yf) - cj0).abs();
    assert!(dcj < 1.0e-12, "dcj={dcj:e}, yf={yf:?}, stats={stats:?}");
}
