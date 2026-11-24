//! Automatic-differentiation (`num-dual`) and `EGM96` degree-36 finite-difference
//! verification of `gravity_gradient_icrf` across central, $J_2$, harmonic, and tidal terms.

use std::sync::Arc;

use glam::{DMat3, DVec3};
use num_dual::{DualNum, DualSVec64, HyperDualSVec64};
use sim_bodies::{BodyCatalog, BodyDescriptor, HarmonicGravity};
use sim_geo::BodyEllipsoid;
use sim_orbital::{
    BuiltinSolarSystem2026_2028, DerivativeOrder, DynamicalMode, EphemerisCatalog, ForceModel,
    GravityConstantsCatalog, NaifId, TdbEpoch, gravity_gradient_icrf,
};
use sim_spacetime::BodyOrientationCatalog;

#[cfg(test)]
fn epoch() -> TdbEpoch {
    TdbEpoch::new(845_000_000, 0)
}

#[cfg(test)]
fn mat3_frobenius(m: DMat3) -> f64 {
    m.to_cols_array().iter().map(|v| v * v).sum::<f64>().sqrt()
}

#[cfg(test)]
fn mat3_rows(m: DMat3) -> [[f64; 3]; 3] {
    [
        [m.x_axis.x, m.y_axis.x, m.z_axis.x],
        [m.x_axis.y, m.y_axis.y, m.z_axis.y],
        [m.x_axis.z, m.y_axis.z, m.z_axis.z],
    ]
}

#[cfg(test)]
fn generic_j2_potential<D: DualNum<Primitive = f64> + Copy>(
    r_icrf: [D; 3],
    r_i2f: [[f64; 3]; 3],
    mu: f64,
    r_eq: f64,
    j2: f64,
) -> D {
    let rf = [
        r_icrf[0] * D::from(r_i2f[0][0])
            + r_icrf[1] * D::from(r_i2f[0][1])
            + r_icrf[2] * D::from(r_i2f[0][2]),
        r_icrf[0] * D::from(r_i2f[1][0])
            + r_icrf[1] * D::from(r_i2f[1][1])
            + r_icrf[2] * D::from(r_i2f[1][2]),
        r_icrf[0] * D::from(r_i2f[2][0])
            + r_icrf[1] * D::from(r_i2f[2][1])
            + r_icrf[2] * D::from(r_i2f[2][2]),
    ];
    let r2 = rf[0] * rf[0] + rf[1] * rf[1] + rf[2] * rf[2];
    let r = r2.sqrt();
    let z_over_r = rf[2] / r;
    let p2 = (z_over_r * z_over_r * D::from(3.0) - D::from(1.0)) * D::from(0.5);
    (D::from(mu) / r) * (D::from(1.0) - (D::from(r_eq * r_eq) / r2) * p2 * D::from(j2))
}

#[cfg(test)]
fn generic_multi_body_accel<D: DualNum<Primitive = f64> + Copy>(
    r_icrf: [D; 3],
    mu_primary: f64,
    third_bodies: &[([f64; 3], f64)],
    r_i2f: [[f64; 3]; 3],
    r_eq: f64,
    j2: f64,
) -> [D; 3] {
    let r2 = r_icrf[0] * r_icrf[0] + r_icrf[1] * r_icrf[1] + r_icrf[2] * r_icrf[2];
    let r3 = r2 * r2.sqrt();
    let mu_p = D::from(mu_primary);
    let mut acc = [
        -r_icrf[0] * mu_p / r3,
        -r_icrf[1] * mu_p / r3,
        -r_icrf[2] * mu_p / r3,
    ];
    for &(s_b, mu_b) in third_bodies {
        let d = [
            r_icrf[0] - D::from(s_b[0]),
            r_icrf[1] - D::from(s_b[1]),
            r_icrf[2] - D::from(s_b[2]),
        ];
        let d2 = d[0] * d[0] + d[1] * d[1] + d[2] * d[2];
        let d3 = d2 * d2.sqrt();
        let s3 = {
            let s2 = s_b[0] * s_b[0] + s_b[1] * s_b[1] + s_b[2] * s_b[2];
            s2 * s2.sqrt()
        };
        for i in 0..3 {
            acc[i] -= (d[i] / d3 + D::from(s_b[i] / s3)) * D::from(mu_b);
        }
    }
    let rf = [
        r_icrf[0] * D::from(r_i2f[0][0])
            + r_icrf[1] * D::from(r_i2f[0][1])
            + r_icrf[2] * D::from(r_i2f[0][2]),
        r_icrf[0] * D::from(r_i2f[1][0])
            + r_icrf[1] * D::from(r_i2f[1][1])
            + r_icrf[2] * D::from(r_i2f[1][2]),
        r_icrf[0] * D::from(r_i2f[2][0])
            + r_icrf[1] * D::from(r_i2f[2][1])
            + r_icrf[2] * D::from(r_i2f[2][2]),
    ];
    let rf2 = rf[0] * rf[0] + rf[1] * rf[1] + rf[2] * rf[2];
    let rf3 = rf2 * rf2.sqrt();
    let k = D::from(1.5 * j2 * r_eq * r_eq) / rf2;
    let s = (rf[2] * rf[2]) / rf2;
    let fac_xy = -mu_p / rf3 * k * (D::from(1.0) - s * D::from(5.0));
    let fac_z = -mu_p / rf3 * k * (D::from(3.0) - s * D::from(5.0));
    let af = [rf[0] * fac_xy, rf[1] * fac_xy, rf[2] * fac_z];
    for i in 0..3 {
        acc[i] += af[0] * D::from(r_i2f[0][i])
            + af[1] * D::from(r_i2f[1][i])
            + af[2] * D::from(r_i2f[2][i]);
    }
    acc
}

#[cfg(test)]
fn earth_wgs84_j2_force_model(solar: &BuiltinSolarSystem2026_2028) -> ForceModel {
    let wgs84 = BodyEllipsoid::WGS84;
    let j2_field = HarmonicGravity::reference(&wgs84).expect("j2");
    let mut specs: Vec<_> = solar.constants.bodies().values().cloned().collect();
    for sp in &mut specs {
        if sp.naif_id == NaifId::EARTH {
            sp.mu_m3_s2 = wgs84.mu_m3_s2;
            sp.equatorial_radius_m = wgs84.semi_major_m;
            sp.harmonics = Some(j2_field.clone());
        }
    }
    let constants = GravityConstantsCatalog::from_specs(specs)
        .with_body_catalog(solar.constants.body_catalog().clone());
    ForceModel::new(
        constants,
        vec![NaifId::SUN, NaifId::MOON, NaifId::EARTH],
        vec![NaifId::EARTH],
        DynamicalMode::PrescribedEphemeris,
        2,
    )
    .expect("fm")
}

#[cfg(test)]
fn dual_jacobian_mat3(a_dual: [DualSVec64<3>; 3]) -> DMat3 {
    let mut cols = [[0.0_f64; 3]; 3];
    for (row, comp) in a_dual.iter().enumerate() {
        let eps = comp.eps.0.as_ref().expect("eps");
        for (col, col_arr) in cols.iter_mut().enumerate() {
            col_arr[row] = eps[(col, 0)];
        }
    }
    DMat3::from_cols(
        DVec3::from_array(cols[0]),
        DVec3::from_array(cols[1]),
        DVec3::from_array(cols[2]),
    )
}

#[test]
fn num_dual_jacobian_matches_multi_body_and_j2_gravity_gradient() {
    let solar = BuiltinSolarSystem2026_2028::load_verified().expect("solar");
    let t0 = epoch();
    let wgs84 = BodyEllipsoid::WGS84;
    let fm = earth_wgs84_j2_force_model(&solar);
    let rot = solar
        .orientation
        .body_rotation(NaifId::EARTH, t0)
        .expect("rot");
    let r_i2f = mat3_rows(rot.r_icrf_to_fixed);
    let s_sun = solar
        .ephemeris
        .geometric_state_icrf(NaifId::SUN, NaifId::EARTH, t0, DerivativeOrder::Position)
        .expect("sun")
        .pos_m;
    let s_moon = solar
        .ephemeris
        .geometric_state_icrf(NaifId::MOON, NaifId::EARTH, t0, DerivativeOrder::Position)
        .expect("moon")
        .pos_m;
    let third_bodies = [
        (
            s_sun.to_array(),
            fm.constants().require_mu(NaifId::SUN).expect("mu_s"),
        ),
        (
            s_moon.to_array(),
            fm.constants().require_mu(NaifId::MOON).expect("mu_m"),
        ),
    ];
    for pos in [
        DVec3::new(6_778_000.0, 1_250_000.0, -2_300_000.0),
        DVec3::new(26_000_000.0, -18_000_000.0, 31_000_000.0),
        s_moon + DVec3::new(2_500_000.0, -1_100_000.0, 800_000.0),
    ] {
        let g_ana = gravity_gradient_icrf(
            &fm,
            NaifId::EARTH,
            pos,
            t0,
            &solar.ephemeris,
            &solar.orientation,
        )
        .expect("grad");
        let r_dual = [
            DualSVec64::<3>::from_re(pos.x).derivative(0),
            DualSVec64::<3>::from_re(pos.y).derivative(1),
            DualSVec64::<3>::from_re(pos.z).derivative(2),
        ];
        let g_dual = dual_jacobian_mat3(generic_multi_body_accel(
            r_dual,
            wgs84.mu_m3_s2,
            &third_bodies,
            r_i2f,
            wgs84.semi_major_m,
            wgs84.j2,
        ));
        assert!(mat3_frobenius(g_ana - g_dual) / mat3_frobenius(g_dual) < 1.0e-12);
    }
}

#[test]
fn hyperdual_hessian_matches_j2_gravity_gradient() {
    let solar = BuiltinSolarSystem2026_2028::load_verified().expect("solar");
    let t0 = epoch();
    let wgs84 = BodyEllipsoid::WGS84;
    let fm = earth_wgs84_j2_force_model(&solar);
    let rot = solar
        .orientation
        .body_rotation(NaifId::EARTH, t0)
        .expect("rot");
    let r_i2f = mat3_rows(rot.r_icrf_to_fixed);
    let earth_only = ForceModel::new(
        fm.constants().clone(),
        vec![NaifId::EARTH],
        vec![NaifId::EARTH],
        DynamicalMode::SelfConsistentNBody,
        2,
    )
    .expect("earth_only");

    for pos in [
        DVec3::new(6_778_000.0, 1_250_000.0, -2_300_000.0),
        DVec3::new(26_000_000.0, -18_000_000.0, 31_000_000.0),
    ] {
        let r_hd = [
            HyperDualSVec64::<3, 3>::from_re(pos.x)
                .derivative1(0)
                .derivative2(0),
            HyperDualSVec64::<3, 3>::from_re(pos.y)
                .derivative1(1)
                .derivative2(1),
            HyperDualSVec64::<3, 3>::from_re(pos.z)
                .derivative1(2)
                .derivative2(2),
        ];
        let u_hd = generic_j2_potential(r_hd, r_i2f, wgs84.mu_m3_s2, wgs84.semi_major_m, wgs84.j2);
        let hess = u_hd.eps1eps2.0.as_ref().expect("hess");
        let g_hd = DMat3::from_cols(
            DVec3::new(hess[(0, 0)], hess[(1, 0)], hess[(2, 0)]),
            DVec3::new(hess[(0, 1)], hess[(1, 1)], hess[(2, 1)]),
            DVec3::new(hess[(0, 2)], hess[(1, 2)], hess[(2, 2)]),
        );
        let g_earth = gravity_gradient_icrf(
            &earth_only,
            NaifId::EARTH,
            pos,
            t0,
            &solar.ephemeris,
            &solar.orientation,
        )
        .expect("earth_grad");
        assert!(mat3_frobenius(g_earth - g_hd) / mat3_frobenius(g_hd) < 1.0e-12);
    }
}

#[test]
fn egm96_degree_36_gradient_matches_fourth_order_finite_diff_and_ssb_invariance() {
    let egm96 = HarmonicGravity::egm96_degree_36().expect("EGM96");
    let earth = BodyDescriptor::earth().expect("earth").with_gravity(egm96);
    let bodies = BodyCatalog::standard_solar_system()
        .expect("bodies")
        .with(earth);
    let solar =
        BuiltinSolarSystem2026_2028::load_verified_with_bodies(Arc::new(bodies)).expect("solar");
    let fm = ForceModel::standard_solar_system(solar.constants.clone()).expect("fm");
    let t0 = epoch();

    for pos in [
        DVec3::new(6_730_000.0, 1_100_000.0, 1_850_000.0),
        DVec3::new(-4_200_000.0, 5_100_000.0, -5_600_000.0),
    ] {
        let g_ana = gravity_gradient_icrf(
            &fm,
            NaifId::EARTH,
            pos,
            t0,
            &solar.ephemeris,
            &solar.orientation,
        )
        .expect("grad");
        let h = 1_000.0_f64;
        let eval_a = |p: DVec3| {
            fm.relative_gravitational_accel_icrf(
                NaifId::EARTH,
                p,
                t0,
                &solar.ephemeris,
                &solar.orientation,
            )
            .expect("accel")
        };
        let mut cols = [DVec3::ZERO; 3];
        for (j, &e) in [DVec3::X, DVec3::Y, DVec3::Z].iter().enumerate() {
            cols[j] = (-eval_a(pos + 2.0 * h * e) + 8.0 * eval_a(pos + h * e)
                - 8.0 * eval_a(pos - h * e)
                + eval_a(pos - 2.0 * h * e))
                / (12.0 * h);
        }
        let g_fd = DMat3::from_cols(cols[0], cols[1], cols[2]);
        let rel_err = mat3_frobenius(g_ana - g_fd) / mat3_frobenius(g_ana);
        assert!(rel_err < 1.0e-11, "EGM96-36 rel_err={rel_err:e}");
        let trace = g_ana.x_axis.x + g_ana.y_axis.y + g_ana.z_axis.z;
        assert!(trace.abs() / mat3_frobenius(g_ana) < 1.0e-13);
    }

    let pos_e = DVec3::new(7_100_000.0, -2_100_000.0, 3_400_000.0);
    let e_ssb = solar
        .ephemeris
        .geometric_state_icrf(NaifId::EARTH, NaifId::SSB, t0, DerivativeOrder::Position)
        .expect("e_ssb");
    let g_e = gravity_gradient_icrf(
        &fm,
        NaifId::EARTH,
        pos_e,
        t0,
        &solar.ephemeris,
        &solar.orientation,
    )
    .expect("g_e");
    let g_b = gravity_gradient_icrf(
        &fm,
        NaifId::SSB,
        e_ssb.pos_m + pos_e,
        t0,
        &solar.ephemeris,
        &solar.orientation,
    )
    .expect("g_b");
    assert!(mat3_frobenius(g_e - g_b) / mat3_frobenius(g_e) < 1.0e-13);
}
