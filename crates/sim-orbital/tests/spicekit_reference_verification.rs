//! Independent reference verification of `sim-orbital` and `navigate-astrodynamics`
//! against the pure-Rust NAIF SPICE kernel pool `spicekit` (`crates.io`).

use std::{error::Error, path::Path};

use glam::{DMat3, DQuat, DVec3};
use sim_orbital::{
    BodyOrientationCatalog, BuiltinSolarSystem2026_2028, CoastControlLaw, ContinuousPlantState,
    DerivativeOrder, Dop853Tolerances, DynamicalMode, EphemerisCatalog, ForceModel,
    MassPropertiesModel, NaifId, PropellantMode, ReactionWheelMode, SolarSystemKernelPack,
    TdbEpoch, apollo_1969_spk_bytes, propagate_plant_interval,
};
use spicekit::{KernelPool, NaifFrame};

type TestResult<T = ()> = Result<T, Box<dyn Error>>;

fn furnish_all_kernels() -> TestResult<KernelPool> {
    let manifest_dir = Path::new(env!("CARGO_MANIFEST_DIR"));
    let mut pool = KernelPool::new();
    pool.furnish(manifest_dir.join("data/solar_de440s_mar099s_2026_2028.bsp"))?;
    pool.furnish(manifest_dir.join("data/solar_de440s_apollo11_1969.bsp"))?;
    pool.furnish(manifest_dir.join("data/earth_itrf93_2026_2028.bpc"))?;
    pool.furnish(manifest_dir.join("data/moon_pa_de440_2026_2028.bpc"))?;
    Ok(pool)
}

fn verify_2026_epoch(
    solar: &BuiltinSolarSystem2026_2028,
    pool: &mut KernelPool,
    t_s: i64,
    bodies: &[NaifId],
) -> TestResult {
    let ep = TdbEpoch::new(t_s, 0);
    for &body in bodies {
        for &center in &[NaifId::SSB, NaifId::EARTH, NaifId::SUN] {
            let sim_state = solar.ephemeris.geometric_state_icrf(
                body,
                center,
                ep,
                DerivativeOrder::Velocity,
            )?;
            let (nav_p, nav_v) = solar
                .ephemeris
                .as_navigate_spk()
                .relative_state_tdb_j2000(body.0, center.0, t_s as f64)?;
            let [rx, ry, rz, vx, vy, vz] = pool.spk_state(body.0, center.0, t_s as f64)?;

            let spice_pos_m = DVec3::new(rx, ry, rz) * 1_000.0;
            let spice_vel_m_s = DVec3::new(vx, vy, vz) * 1_000.0;
            let nav_pos_m = DVec3::new(nav_p.x, nav_p.y, nav_p.z);
            let nav_vel_m_s = DVec3::new(nav_v.x, nav_v.y, nav_v.z);

            assert!((sim_state.pos_m - spice_pos_m).length() < 5e-4);
            assert!((sim_state.vel_m_s - spice_vel_m_s).length() < 1e-7);
            assert!((nav_pos_m - sim_state.pos_m).length() < 1e-9);
            assert!((nav_vel_m_s - sim_state.vel_m_s).length() < 1e-12);
        }
    }

    let earth_rot = solar.orientation.body_rotation(NaifId::EARTH, ep)?;
    let spice_earth_px = pool.pck_pxform(3000, NaifFrame::J2000, t_s as f64)?;
    let spice_earth_mat = DMat3::from_cols_array_2d(&spice_earth_px).transpose();
    let earth_rot_diff = (earth_rot.r_icrf_to_fixed - spice_earth_mat)
        .to_cols_array()
        .into_iter()
        .map(f64::abs)
        .fold(0.0_f64, f64::max);
    assert!(earth_rot_diff < 1e-12, "Earth BPC diff: {earth_rot_diff}");
    Ok(())
}

#[test]
fn spk_catalog_and_navigate_spk_match_spicekit_across_2026_and_1969_kernels() {
    let apollo_bytes = apollo_1969_spk_bytes().expect("apollo 1969 DE440s slice");
    let spk_slices = [apollo_bytes];
    let solar = BuiltinSolarSystem2026_2028::load_with_kernel_pack(&SolarSystemKernelPack {
        spk_files: &spk_slices,
        earth_bpc_files: &[],
        moon_bpc_files: &[],
        allow_iau_outside_bpc: true,
    })
    .expect("loaded 2026+1969 solar system");
    let mut pool = furnish_all_kernels().expect("kernels");

    let bodies_2026 = [
        NaifId::SUN,
        NaifId::MERCURY_BARYCENTER,
        NaifId::VENUS_BARYCENTER,
        NaifId::EARTH_MOON_BARYCENTER,
        NaifId::EARTH,
        NaifId::MOON,
        NaifId::MARS_BARYCENTER,
        NaifId::MARS,
        NaifId::JUPITER_BARYCENTER,
        NaifId::SATURN_BARYCENTER,
    ];
    for t_s in [
        820_500_000_i64,
        835_000_000,
        850_123_456,
        865_000_000,
        880_000_000,
    ] {
        verify_2026_epoch(&solar, &mut pool, t_s, &bodies_2026).expect("2026 epoch");
    }

    let bodies_1969 = [
        NaifId::SUN,
        NaifId::EARTH_MOON_BARYCENTER,
        NaifId::EARTH,
        NaifId::MOON,
        NaifId::MARS_BARYCENTER,
    ];
    for t_s in [-961_200_000_i64, -960_910_900, -960_500_000] {
        let ep = TdbEpoch::new(t_s, 0);
        for &body in &bodies_1969 {
            let sim_state = solar
                .ephemeris
                .geometric_state_icrf(body, NaifId::EARTH, ep, DerivativeOrder::Velocity)
                .expect("1969 sim-orbital state");
            let [rx, ry, rz, vx, vy, vz] = pool
                .spk_state(body.0, NaifId::EARTH.0, t_s as f64)
                .expect("1969 spicekit state");
            let spice_pos_m = DVec3::new(rx, ry, rz) * 1_000.0;
            let spice_vel_m_s = DVec3::new(vx, vy, vz) * 1_000.0;
            assert!((sim_state.pos_m - spice_pos_m).length() < 1e-4);
            assert!((sim_state.vel_m_s - spice_vel_m_s).length() < 1e-7);
        }
    }
}

fn integrate_spicekit_rk4(
    solar: &BuiltinSolarSystem2026_2028,
    pool: &mut KernelPool,
    t0_s: i64,
    duration_s: i64,
    r0: DVec3,
    v0: DVec3,
) -> TestResult<(DVec3, DVec3)> {
    let gm_earth = solar.constants.require_mu(NaifId::EARTH)?;
    let third_bodies = [
        (NaifId::SUN.0, solar.constants.require_mu(NaifId::SUN)?),
        (NaifId::MOON.0, solar.constants.require_mu(NaifId::MOON)?),
        (NaifId::MARS.0, solar.constants.require_mu(NaifId::MARS)?),
    ];
    let dt = 5.0_f64;
    let n_steps = (duration_s as usize) / 5;
    let mut r = r0;
    let mut v = v0;
    for step in 0..n_steps {
        let t = (t0_s as f64) + (step as f64) * dt;
        let k1_r = v;
        let k1_v = spicekit_accel(pool, gm_earth, &third_bodies, t, r)?;
        let k2_r = v + 0.5 * dt * k1_v;
        let k2_v = spicekit_accel(
            pool,
            gm_earth,
            &third_bodies,
            t + 0.5 * dt,
            r + 0.5 * dt * k1_r,
        )?;
        let k3_r = v + 0.5 * dt * k2_v;
        let k3_v = spicekit_accel(
            pool,
            gm_earth,
            &third_bodies,
            t + 0.5 * dt,
            r + 0.5 * dt * k2_r,
        )?;
        let k4_r = v + dt * k3_v;
        let k4_v = spicekit_accel(pool, gm_earth, &third_bodies, t + dt, r + dt * k3_r)?;
        r += (dt / 6.0) * (k1_r + 2.0 * k2_r + 2.0 * k3_r + k4_r);
        v += (dt / 6.0) * (k1_v + 2.0 * k2_v + 2.0 * k3_v + k4_v);
    }
    Ok((r, v))
}

fn spicekit_accel(
    pool: &mut KernelPool,
    gm_earth: f64,
    third_bodies: &[(i32, f64)],
    ts: f64,
    pos: DVec3,
) -> TestResult<DVec3> {
    let r2 = pos.length_squared();
    let mut a = -gm_earth * pos / (r2 * r2.sqrt());
    for &(body_id, gm_j) in third_bodies {
        let [sx, sy, sz, ..] = pool.spk_state(body_id, NaifId::EARTH.0, ts)?;
        let s_j = DVec3::new(sx, sy, sz) * 1_000.0;
        let d = s_j - pos;
        let d2 = d.length_squared();
        let s2 = s_j.length_squared();
        a += gm_j * (d / (d2 * d2.sqrt()) - s_j / (s2 * s2.sqrt()));
    }
    Ok(a)
}

#[test]
fn dop853_nbody_trajectory_matches_spicekit_driven_rk4_reference() {
    let solar = BuiltinSolarSystem2026_2028::load_verified().expect("solar");
    let manifest_dir = Path::new(env!("CARGO_MANIFEST_DIR"));
    let mut pool = KernelPool::new();
    pool.furnish(manifest_dir.join("data/solar_de440s_mar099s_2026_2028.bsp"))
        .expect("spicekit 2026 bsp");

    let attractors = vec![NaifId::SUN, NaifId::EARTH, NaifId::MOON, NaifId::MARS];
    let force_model = ForceModel::new(
        solar.constants.clone(),
        attractors,
        Vec::new(),
        DynamicalMode::SelfConsistentNBody,
        0,
    )
    .expect("force model");

    let t0_s = 836_222_400_i64;
    let duration_s = 12 * 3_600_i64;
    let t0 = TdbEpoch::new(t0_s, 0);
    let t1 = TdbEpoch::new(t0_s + duration_s, 0);
    let r0 = DVec3::new(20_000_000.0, 2_500_000.0, 1_000_000.0);
    let v0 = DVec3::new(-800.0, 6_200.0, 1_400.0);

    let y0 = ContinuousPlantState {
        primary_body: NaifId::EARTH,
        pos_rel_icrf_m: r0,
        vel_rel_icrf_m_s: v0,
        q_body_to_icrf: DQuat::IDENTITY,
        omega_body_rad_s: DVec3::ZERO,
        propellant_mass_kg: 0.0,
        h_rw_body_nms: DVec3::ZERO,
    };
    let mass = MassPropertiesModel::diagonal(1_000.0, [500.0; 3], [0.0; 3]);
    let out = propagate_plant_interval(
        &y0,
        t0,
        t1,
        None,
        &Dop853Tolerances::default(),
        &mass,
        PropellantMode::FiniteVariableMass,
        ReactionWheelMode::ConserveMomentum {
            h_max_nms: 100.0,
            tau_max_nm: 10.0,
        },
        &force_model,
        &solar.ephemeris,
        &solar.orientation,
        &CoastControlLaw,
    )
    .expect("dop853 propagation");

    let (r, v) = integrate_spicekit_rk4(&solar, &mut pool, t0_s, duration_s, r0, v0).expect("rk4");
    assert!((out.final_state.pos_rel_icrf_m - r).length() < 0.05);
    assert!((out.final_state.vel_rel_icrf_m_s - v).length() < 1e-5);
}
