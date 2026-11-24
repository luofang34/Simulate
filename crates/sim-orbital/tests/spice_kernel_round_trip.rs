//! End-to-end Simulate -> SPICE SPK (Types 2, 3, 9, 13) and CK (Types 2, 3) export
//! and round-trip verification against `OrbitalTrajectoryProduct`, `SpkEphemerisCatalog`,
//! `spice_kernels`, and `spicekit`.

use std::error::Error;

use glam::{DQuat, DVec3};
use sim_orbital::{
    BuiltinSolarSystem2026_2028, CkExportConfig, CkExportKind, CoastControlLaw,
    ContinuousPlantState, DerivativeOrder, Dop853Tolerances, DynamicalMode, EphemerisCatalog,
    ForceModel, KinematicStateIcrf, LinearSclkMapping, MarsCenterApproximation,
    MassPropertiesModel, NaifId, OrbitalTrajectoryProduct, PlanningInputFingerprint,
    PropellantMode, ReactionWheelMode, SpkEphemerisCatalog, SpkExportConfig, SpkExportKind,
    SpkFile as SimSpkFile, TdbEpoch, VesselConfig, export_attitude_ck, export_sampled_attitude_ck,
    export_sampled_states_spk, export_translational_spk, propagate_plant_interval,
    quat_geodesic_angle_rad, quat_normalize_portable, quat_rotate_vec_portable,
};
use sim_spacetime::{
    AttitudeTrajectorySource, DynamicalModelIdentity, TranslationalTrajectorySource,
};
use spice_kernels::{CkFile, SpkFile};

type TestResult<T = ()> = Result<T, Box<dyn Error>>;

const TARGET_NAIF: NaifId = NaifId(-999_001);
const INST_NAIF: i32 = -999_000;

fn build_leo_6dof_product(t0: TdbEpoch, duration_s: i64) -> TestResult<OrbitalTrajectoryProduct> {
    let solar = BuiltinSolarSystem2026_2028::load_verified()?;
    let t1 = TdbEpoch::new(t0.j2000_seconds() + duration_s, t0.subsec_ticks());
    let r0 = 6_900_000.0;
    let v0 = libm::sqrt(3.986_004_418e14 / r0);
    let q0 = quat_normalize_portable(DQuat::from_xyzw(0.18, -0.31, 0.42, 0.83));
    let plant0 = ContinuousPlantState {
        primary_body: NaifId::EARTH,
        pos_rel_icrf_m: DVec3::new(r0, 420_000.0, -310_000.0),
        vel_rel_icrf_m_s: DVec3::new(-320.0, v0 * 0.92, v0 * 0.39),
        q_body_to_icrf: q0,
        omega_body_rad_s: DVec3::new(0.008, -0.004, 0.012),
        propellant_mass_kg: 120.0,
        h_rw_body_nms: DVec3::new(0.4, -0.2, 0.7),
    };
    let force_model = ForceModel::new(
        solar.constants,
        vec![NaifId::SUN, NaifId::EARTH, NaifId::MOON],
        Vec::new(),
        DynamicalMode::PrescribedEphemeris,
        0,
    )?;
    let vessel_cfg = VesselConfig {
        mass_model: MassPropertiesModel::diagonal(1_250.0, [500.0, 620.0, 710.0], [1.0; 3]),
        propellant_mode: PropellantMode::FiniteVariableMass,
        rw_mode: ReactionWheelMode::ConserveMomentum {
            h_max_nms: 100.0,
            tau_max_nm: 10.0,
        },
        tolerances: Dop853Tolerances::default(),
        bounding_radius_m: 4.5,
        auto_soi_switch: false,
    };
    let prop_out = propagate_plant_interval(
        &plant0,
        t0,
        t1,
        Some(10.0),
        &vessel_cfg.tolerances,
        &vessel_cfg.mass_model,
        vessel_cfg.propellant_mode,
        vessel_cfg.rw_mode,
        &force_model,
        &solar.ephemeris,
        &solar.orientation,
        &CoastControlLaw,
    )?;
    let model_id = DynamicalModelIdentity::SolarSystemEphemerisNBody {
        force_model_hash: force_model.content_hash(),
    };
    let fp = PlanningInputFingerprint::compute(
        model_id,
        &force_model,
        &solar.ephemeris,
        &solar.orientation,
        &vessel_cfg,
    );
    Ok(OrbitalTrajectoryProduct::from_segments(
        42,
        fp,
        vessel_cfg.mass_model.dry_mass_kg,
        plant0,
        prop_out.final_state,
        t0,
        prop_out.segments,
    ))
}

fn check_spk_against_truth(
    product: &OrbitalTrajectoryProduct,
    bytes: &[u8],
    t0: TdbEpoch,
    duration_s: i64,
    kind_label: &str,
    pos_tol_m: f64,
    vel_tol_m_s: f64,
) -> TestResult {
    let parsed = SpkFile::parse(bytes)?;
    let path = std::env::temp_dir().join(format!("sim_orbital_rt_{kind_label}.bsp"));
    std::fs::write(&path, bytes)?;
    let other = spicekit::SpkFile::open(&path)?;
    std::fs::remove_file(&path)?;

    let mut worst_pos = 0.0_f64;
    let mut worst_vel = 0.0_f64;
    for k in 0..=80 {
        let off = (duration_s as f64) * (f64::from(k) / 80.0);
        let ep = t0.advanced_seconds(off);
        let clamped = if ep > product.valid_interval.end_epoch {
            product.valid_interval.end_epoch
        } else {
            ep
        };
        let truth = product.sample_translational_icrf(clamped)?.to_si_metric();
        let et = clamped.as_j2000_seconds_f64();
        let sp_state = parsed.state(TARGET_NAIF.0, NaifId::EARTH.0, et)?;
        let kit_state = other.state(TARGET_NAIF.0, NaifId::EARTH.0, et)?;
        let rt = SpkExportConfig::spice_to_kinematic(&sp_state);
        let kit = SpkExportConfig::spice_to_kinematic(&kit_state);

        worst_pos = worst_pos
            .max((rt.pos_m - truth.pos).length())
            .max((kit.pos_m - truth.pos).length());
        worst_vel = worst_vel
            .max((rt.vel_m_s - truth.vel).length())
            .max((kit.vel_m_s - truth.vel).length());
    }
    assert!(worst_pos < pos_tol_m, "{kind_label} pos err {worst_pos} m");
    assert!(
        worst_vel < vel_tol_m_s,
        "{kind_label} vel err {worst_vel} m/s"
    );
    Ok(())
}

#[test]
fn orbital_product_spk_round_trip_matches_truth_under_millimetre_across_types_2_3_9_13()
-> TestResult {
    let t0 = TdbEpoch::new(836_222_400, 0);
    let duration_s = 1_800;
    let product = build_leo_6dof_product(t0, duration_s)?;
    let configs = [
        (
            "t2",
            SpkExportKind::Type2Chebyshev {
                granule_duration_s: 300.0,
                degree: 14,
            },
        ),
        (
            "t3",
            SpkExportKind::Type3Chebyshev {
                granule_duration_s: 300.0,
                degree: 14,
            },
        ),
        (
            "t9",
            SpkExportKind::Type9Lagrange {
                step_s: 20.0,
                window: 8,
            },
        ),
        (
            "t13",
            SpkExportKind::Type13Hermite {
                step_s: 30.0,
                window: 4,
            },
        ),
    ];

    for (label, kind) in configs {
        let cfg = SpkExportConfig::new("SIM-ORBITAL-RT", label, TARGET_NAIF, NaifId::EARTH, kind);
        let spk = export_translational_spk(&product, &cfg)?;
        let spk_via_cfg = cfg.export_orbital_product(&product)?;
        assert_eq!(spk.to_bytes()?, spk_via_cfg.to_bytes()?);
        let bytes = spk.to_bytes()?;
        check_spk_against_truth(&product, &bytes, t0, duration_s, label, 1.0e-3, 1.0e-5)?;

        if matches!(
            kind,
            SpkExportKind::Type2Chebyshev { .. } | SpkExportKind::Type3Chebyshev { .. }
        ) {
            let sim_spk = SimSpkFile::from_spice_kernels(&spk)?;
            let cat = SpkEphemerisCatalog::from_files(
                vec![sim_spk],
                MarsCenterApproximation::StrictRequire499,
            );
            let probe = t0.advanced_seconds(777.0);
            let truth = product.sample_translational_icrf(probe)?.to_si_metric();
            let got = cat.geometric_state_icrf(
                TARGET_NAIF,
                NaifId::EARTH,
                probe,
                DerivativeOrder::Velocity,
            )?;
            assert!((got.pos_m - truth.pos).length() < 1.0e-3);
            assert!((got.vel_m_s - truth.vel).length() < 1.0e-5);
        }
    }
    Ok(())
}

#[test]
fn orbital_product_ck_round_trip_verifies_sclk_quaternions_and_angular_velocity() -> TestResult {
    let t0 = TdbEpoch::new(836_222_400, 0);
    let duration_s = 30;
    let product = build_leo_6dof_product(t0, duration_s)?;
    let sclk = LinearSclkMapping::new(t0, 1_000.0, 65_536.0);

    for (label, kind, expect_av) in [
        (
            "ck2",
            CkExportKind::Type2ConstantRate { step_s: 0.25 },
            true,
        ),
        (
            "ck3_av",
            CkExportKind::Type3Slerp {
                step_s: 0.25,
                include_angular_velocity: true,
            },
            true,
        ),
        (
            "ck3_noav",
            CkExportKind::Type3Slerp {
                step_s: 0.25,
                include_angular_velocity: false,
            },
            false,
        ),
    ] {
        let cfg = CkExportConfig::new("SIM-ORBITAL-CK", label, INST_NAIF, kind, sclk);
        let ck = export_attitude_ck(&product, &cfg)?;
        assert_eq!(
            ck.to_bytes()?,
            cfg.export_orbital_product(&product)?.to_bytes()?
        );
        let parsed = CkFile::parse(&ck.to_bytes()?)?;

        for step in 0..120 {
            let knot = t0.advanced_seconds(f64::from(step) * 0.25);
            let truth = product.sample_attitude_icrf(knot)?;
            let sc = sclk.epoch_to_sclk(knot)?;
            let pt = parsed.pointing_and_angular_velocity(INST_NAIF, sc, 0.0)?;
            let got = sclk
                .ck_to_attitude(pt)
                .ok_or("sclk_to_epoch failed on knot")?;
            let angle = quat_geodesic_angle_rad(got.q_body_to_icrf, truth.q_body_to_icrf);
            assert!(angle < 1.0e-11, "{label} knot angle {angle:e}");
            if expect_av {
                let dv = (got.omega_body_rad_s - truth.omega_body_rad_s).length();
                assert!(dv < 1.0e-11, "{label} knot av err {dv:e}");
            }
        }

        let mid = t0.advanced_seconds(14.375);
        let mid_truth = product.sample_attitude_icrf(mid)?;
        let mid_pt =
            parsed.pointing_and_angular_velocity(INST_NAIF, sclk.epoch_to_sclk(mid)?, 0.0)?;
        let mid_got = CkExportConfig::ck_to_sim_attitude(mid, mid_pt);
        let mid_angle = quat_geodesic_angle_rad(mid_got.q_body_to_icrf, mid_truth.q_body_to_icrf);
        assert!(mid_angle < 5.0e-6, "{label} mid angle {mid_angle:e}");
    }
    Ok(())
}

#[test]
fn sampled_states_spk_round_trip_under_millimetre() -> TestResult {
    let t0 = TdbEpoch::new(825_000_000, 0);
    let mu = 3.986_004_418e14_f64;
    let r0 = 7_100_000.0_f64;
    let n = libm::sqrt(mu / (r0 * r0 * r0));
    let orbit_samples: Vec<(TdbEpoch, KinematicStateIcrf)> = (0..=120)
        .map(|k| {
            let dt = f64::from(k) * 5.0;
            let (s, c) = libm::sincos(n * dt);
            let kin = KinematicStateIcrf {
                pos_m: DVec3::new(r0 * c, r0 * s * 0.8, r0 * s * 0.6),
                vel_m_s: DVec3::new(-r0 * n * s, r0 * n * c * 0.8, r0 * n * c * 0.6),
                acc_m_s2: DVec3::ZERO,
            };
            (t0.advanced_seconds(dt), kin)
        })
        .collect();

    for (label, kind) in [
        (
            "ls_t2",
            SpkExportKind::Type2Chebyshev {
                granule_duration_s: 300.0,
                degree: 14,
            },
        ),
        (
            "ls_t3",
            SpkExportKind::Type3Chebyshev {
                granule_duration_s: 300.0,
                degree: 14,
            },
        ),
        (
            "ls_t9",
            SpkExportKind::Type9Lagrange {
                step_s: 5.0,
                window: 8,
            },
        ),
        (
            "ls_t13",
            SpkExportKind::Type13Hermite {
                step_s: 5.0,
                window: 4,
            },
        ),
    ] {
        let cfg = SpkExportConfig::new("SAMPLED", label, TARGET_NAIF, NaifId::EARTH, kind);
        let parsed = SpkFile::parse(&export_sampled_states_spk(&orbit_samples, &cfg)?.to_bytes()?)?;
        let probe = t0.advanced_seconds(237.3);
        let (s, c) = libm::sincos(n * 237.3);
        let truth_p = DVec3::new(r0 * c, r0 * s * 0.8, r0 * s * 0.6);
        let rt = SpkExportConfig::spice_to_kinematic(&parsed.state(
            TARGET_NAIF.0,
            NaifId::EARTH.0,
            probe.as_j2000_seconds_f64(),
        )?);
        assert!(
            (rt.pos_m - truth_p).length() < 1.0e-3,
            "{label} sampled pos err"
        );
    }
    Ok(())
}

#[test]
fn constant_spin_ck_round_trip_to_sub_microarcsecond() -> TestResult {
    let t0 = TdbEpoch::new(825_000_000, 0);

    let q_init = quat_normalize_portable(DQuat::from_xyzw(-0.2, 0.5, 0.3, 0.78));
    let axis_icrf = DVec3::new(0.25, -0.55, 0.796).normalize();
    let omega_rate = 0.015_f64;
    let omega_icrf = axis_icrf * omega_rate;
    let att_samples: Vec<(TdbEpoch, DQuat, DVec3)> = (0..=10)
        .map(|k| {
            let dt = f64::from(k) * 6.0;
            let q_k = DQuat::from_axis_angle(axis_icrf, omega_rate * dt) * q_init;
            let omega_body = quat_rotate_vec_portable(q_k.conjugate(), omega_icrf);
            (t0.advanced_seconds(dt), q_k, omega_body)
        })
        .collect();
    let sclk = LinearSclkMapping::seconds_since(t0);
    let ck_cfg = CkExportConfig::new(
        "CONST-SPIN",
        "CONST",
        INST_NAIF,
        CkExportKind::Type2ConstantRate { step_s: 6.0 },
        sclk,
    );
    let ck = CkFile::parse(&export_sampled_attitude_ck(&att_samples, &ck_cfg)?.to_bytes()?)?;
    for dt in [0.0, 2.75, 19.4, 44.1, 59.99] {
        let ep = t0.advanced_seconds(dt);
        let dt_ep = ep.signed_offset_seconds_from(t0);
        let q_truth = DQuat::from_axis_angle(axis_icrf, omega_rate * dt_ep) * q_init;
        let pt = ck.pointing_and_angular_velocity(INST_NAIF, sclk.epoch_to_sclk(ep)?, 0.0)?;
        let got = sclk.ck_to_attitude(pt).ok_or("sclk_to_epoch")?;
        let ang = quat_geodesic_angle_rad(got.q_body_to_icrf, q_truth);
        assert!(ang < 1.0e-12, "constant-spin dt={dt} angle {ang:e}");
    }
    Ok(())
}
