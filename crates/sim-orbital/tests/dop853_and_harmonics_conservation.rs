//! Multi-origin equivalence and conservation verification: Earth/Moon/SSB 3-frame equivalence with
//! origin-acceleration defect $\delta\mathbf{a}_{\text{origin},P}(t)$, vacuum Tsiolkovsky
//! mass/velocity balance, $\mathbf{H}_{\text{ICRF}}$ conservation, and specific power.

use glam::{DQuat, DVec3};
use sim_orbital::{
    BodyGravitySpec, BuiltinSolarSystem2026_2028, CoastControlLaw, ConstantWrenchLaw,
    ContinuousPlantState, DerivativeOrder, Dop853GenericConfig, Dop853Tolerances, DynamicalMode,
    EphemerisCatalog, ForceModel, GravityConstantsCatalog, HarmonicTerm, KinematicStateIcrf,
    MassPropertiesModel, NaifId, PlantStageWrench, PropellantMode, ReactionWheelMode, TdbEpoch,
    dop853_integrate_generic, propagate_plant_interval,
};
use sim_spacetime::{TickSpan, ln};

#[cfg(test)]
fn propagate_cislunar_barycentric_endpoint(
    solar: &BuiltinSolarSystem2026_2028,
    model: &ForceModel,
    prim: NaifId,
    rp: DVec3,
    vp: DVec3,
    t0: TdbEpoch,
    t_end: TdbEpoch,
) -> KinematicStateIcrf {
    let tol = Dop853Tolerances {
        rtol: 1.0e-12,
        atol_pos_m: 1.0e-6,
        atol_vel_m_s: 1.0e-9,
        h_max_s: 1_800.0,
        ..Dop853Tolerances::default()
    };
    let mass = MassPropertiesModel::diagonal(12_000.0, [40_000.0, 40_000.0, 8_000.0], [0.0; 3]);
    let rw = ReactionWheelMode::ConserveMomentum {
        h_max_nms: 5.0e6,
        tau_max_nm: 5.0e4,
    };
    let init = ContinuousPlantState {
        primary_body: prim,
        pos_rel_icrf_m: rp,
        vel_rel_icrf_m_s: vp,
        q_body_to_icrf: DQuat::IDENTITY,
        omega_body_rad_s: DVec3::ZERO,
        propellant_mass_kg: 0.0,
        h_rw_body_nms: DVec3::ZERO,
    };
    let out = propagate_plant_interval(
        &init,
        t0,
        t_end,
        Some(60.0),
        &tol,
        &mass,
        PropellantMode::InfiniteConstantMass,
        rw,
        model,
        &solar.ephemeris,
        &solar.orientation,
        &CoastControlLaw,
    )
    .expect("prop");
    let af = model
        .relative_gravitational_accel_icrf(
            prim,
            out.final_state.pos_rel_icrf_m,
            t_end,
            &solar.ephemeris,
            &solar.orientation,
        )
        .expect("af");
    model
        .reconstruct_barycentric_state(
            prim,
            out.final_state.pos_rel_icrf_m,
            out.final_state.vel_rel_icrf_m_s,
            af,
            t_end,
            &solar.ephemeris,
        )
        .expect("reconstruct")
}

#[test]
fn earth_moon_ssb_instantaneous_and_roundtrip_frame_contracts() {
    let solar = BuiltinSolarSystem2026_2028::load_verified().expect("solar");
    let fm = ForceModel::standard_solar_system(solar.constants.clone()).expect("force model");
    let t0 = TdbEpoch::new(845_000_000, 0);
    let r_e0 = DVec3::new(95_000_000.0, 68_000_000.0, 22_000_000.0);
    let v_e0 = DVec3::new(-920.0, 1_350.0, 240.0);

    let (r_m0, v_m0) = fm
        .switch_primary_frame(
            NaifId::EARTH,
            NaifId::MOON,
            r_e0,
            v_e0,
            t0,
            &solar.ephemeris,
        )
        .expect("E->M");
    let (r_e_rt, v_e_rt) = fm
        .switch_primary_frame(
            NaifId::MOON,
            NaifId::EARTH,
            r_m0,
            v_m0,
            t0,
            &solar.ephemeris,
        )
        .expect("M->E");
    assert!((r_e_rt - r_e0).length() < 5.0e-7);
    assert!((v_e_rt - v_e0).length() < 1.0e-12);

    let (r_b0, _) = fm
        .switch_primary_frame(NaifId::EARTH, NaifId::SSB, r_e0, v_e0, t0, &solar.ephemeris)
        .expect("E->SSB");
    let a_ssb = fm
        .relative_gravitational_accel_icrf(
            NaifId::SSB,
            r_b0,
            t0,
            &solar.ephemeris,
            &solar.orientation,
        )
        .expect("ssb acc");
    for &(prim, r_p, v_p) in &[(NaifId::EARTH, r_e0, v_e0), (NaifId::MOON, r_m0, v_m0)] {
        let a_rel = fm
            .relative_gravitational_accel_icrf(prim, r_p, t0, &solar.ephemeris, &solar.orientation)
            .expect("rel acc");
        let rec = fm
            .reconstruct_barycentric_state(prim, r_p, v_p, a_rel, t0, &solar.ephemeris)
            .expect("rec");
        assert!((rec.acc_m_s2 - a_ssb).length() < 1.0e-13);
    }
}

#[test]
fn earth_moon_ssb_multi_day_trajectory_equivalence_verified() {
    let solar = BuiltinSolarSystem2026_2028::load_verified().expect("solar");
    let fm = ForceModel::standard_solar_system(solar.constants.clone()).expect("force model");
    let t0 = TdbEpoch::new(845_000_000, 0);
    let t_end = t0.add_span(TickSpan::from_seconds_and_subsec_ticks(4 * 86_400, 0));
    let r_e0 = DVec3::new(95_000_000.0, 68_000_000.0, 22_000_000.0);
    let v_e0 = DVec3::new(-920.0, 1_350.0, 240.0);

    let (r_m0, v_m0) = fm
        .switch_primary_frame(
            NaifId::EARTH,
            NaifId::MOON,
            r_e0,
            v_e0,
            t0,
            &solar.ephemeris,
        )
        .expect("E->M");
    let (r_b0, v_b0) = fm
        .switch_primary_frame(NaifId::EARTH, NaifId::SSB, r_e0, v_e0, t0, &solar.ephemeris)
        .expect("E->SSB");

    let fin_e =
        propagate_cislunar_barycentric_endpoint(&solar, &fm, NaifId::EARTH, r_e0, v_e0, t0, t_end);
    let fin_m =
        propagate_cislunar_barycentric_endpoint(&solar, &fm, NaifId::MOON, r_m0, v_m0, t0, t_end);
    let fin_b =
        propagate_cislunar_barycentric_endpoint(&solar, &fm, NaifId::SSB, r_b0, v_b0, t0, t_end);

    assert!((fin_e.pos_m - fin_m.pos_m).length() < 5.0e-3);
    assert!((fin_e.pos_m - fin_b.pos_m).length() < 1.0e-2);
    assert!((fin_e.vel_m_s - fin_m.vel_m_s).length() < 1.0e-7);
    assert!((fin_e.acc_m_s2 - fin_m.acc_m_s2).length() < 1.0e-12);

    let fm_no_defect = ForceModel::new(
        solar.constants.clone(),
        vec![NaifId::SUN, NaifId::EARTH, NaifId::MOON],
        Vec::new(),
        DynamicalMode::SelfConsistentNBody,
        0,
    )
    .expect("no defect");
    let bad_e = propagate_cislunar_barycentric_endpoint(
        &solar,
        &fm_no_defect,
        NaifId::EARTH,
        r_e0,
        v_e0,
        t0,
        t_end,
    );
    let bad_b = propagate_cislunar_barycentric_endpoint(
        &solar,
        &fm_no_defect,
        NaifId::SSB,
        r_b0,
        v_b0,
        t0,
        t_end,
    );
    assert!((bad_e.pos_m - bad_b.pos_m).length() > 1.0);
}

#[test]
fn vacuum_tsiolkovsky_and_reaction_wheel_momentum_conservation_verified() {
    let solar = BuiltinSolarSystem2026_2028::load_verified().expect("solar");
    let t0 = TdbEpoch::new(850_000_000, 0);
    let tiny_mu = GravityConstantsCatalog::from_specs([BodyGravitySpec {
        naif_id: NaifId::EARTH,
        mu_m3_s2: 1.0e-18,
        equatorial_radius_m: 1.0,
        polar_radius_m: 1.0,
        harmonics: Vec::new(),
    }]);
    let fm_free = ForceModel::new(
        tiny_mu,
        vec![NaifId::EARTH],
        Vec::new(),
        DynamicalMode::SelfConsistentNBody,
        0,
    )
    .expect("free space");
    let mass =
        MassPropertiesModel::diagonal(8_000.0, [25_000.0, 32_000.0, 9_000.0], [1.2, 1.5, 0.4]);
    let init_vac = ContinuousPlantState {
        primary_body: NaifId::SSB,
        pos_rel_icrf_m: DVec3::new(1.0e9, 0.0, 0.0),
        vel_rel_icrf_m_s: DVec3::ZERO,
        q_body_to_icrf: DQuat::IDENTITY,
        omega_body_rad_s: DVec3::ZERO,
        propellant_mass_kg: 4_000.0,
        h_rw_body_nms: DVec3::ZERO,
    };
    let burn_law = ConstantWrenchLaw {
        wrench: PlantStageWrench {
            main_thrust_body_n: DVec3::new(0.0, 0.0, 70_000.0),
            main_exhaust_vel_m_s: 3_500.0,
            ..PlantStageWrench::default()
        },
    };
    let tol = Dop853Tolerances {
        rtol: 1.0e-13,
        atol_pos_m: 1.0e-10,
        atol_vel_m_s: 1.0e-13,
        atol_att_rad: 1.0e-13,
        atol_omega_rad_s: 1.0e-13,
        atol_mass_kg: 1.0e-13,
        atol_h_rw_nms: 1.0e-10,
        ..Dop853Tolerances::default()
    };
    let vac_out = propagate_plant_interval(
        &init_vac,
        t0,
        t0.add_span(TickSpan::from_seconds_and_subsec_ticks(100, 0)),
        Some(1.0),
        &tol,
        &mass,
        PropellantMode::FiniteVariableMass,
        ReactionWheelMode::ConserveMomentum {
            h_max_nms: 5.0e6,
            tau_max_nm: 5.0e4,
        },
        &fm_free,
        &solar.ephemeris,
        &solar.orientation,
        &burn_law,
    )
    .expect("tsiolkovsky");
    assert!(!vac_out.audit.mass_conservation_exempt);
    assert!((vac_out.final_state.propellant_mass_kg - 2_000.0).abs() < 1.0e-11);
    assert!((vac_out.final_state.vel_rel_icrf_m_s.length() - 3_500.0 * ln(1.2)).abs() < 1.0e-11);

    verify_rw_saturation_momentum(&solar, &fm_free, &mass, &init_vac, &tol, t0);
}

#[cfg(test)]
fn verify_rw_saturation_momentum(
    solar: &BuiltinSolarSystem2026_2028,
    fm_free: &ForceModel,
    mass: &MassPropertiesModel,
    base: &ContinuousPlantState,
    tol: &Dop853Tolerances,
    t0: TdbEpoch,
) {
    let init_rot = ContinuousPlantState {
        omega_body_rad_s: DVec3::new(0.15, -0.22, 0.31),
        h_rw_body_nms: DVec3::new(1_200.0, -800.0, 500.0),
        propellant_mass_kg: 0.0,
        ..*base
    };
    let h0_icrf = mass.total_angular_momentum_icrf(&init_rot);
    let slew_law = ConstantWrenchLaw {
        wrench: PlantStageWrench {
            rw_torque_cmd_body_nm: DVec3::new(-400.0, 650.0, -300.0),
            ..PlantStageWrench::default()
        },
    };
    let rot_out = propagate_plant_interval(
        &init_rot,
        t0,
        t0.add_span(TickSpan::from_seconds_and_subsec_ticks(30, 0)),
        Some(0.5),
        tol,
        mass,
        PropellantMode::InfiniteConstantMass,
        ReactionWheelMode::ConserveMomentum {
            h_max_nms: 8_000.0,
            tau_max_nm: 2_000.0,
        },
        fm_free,
        &solar.ephemeris,
        &solar.orientation,
        &slew_law,
    )
    .expect("rw slew");
    let hf_icrf = mass.total_angular_momentum_icrf(&rot_out.final_state);
    assert!((hf_icrf - h0_icrf).length() / h0_icrf.length() < 1.0e-12);
}

#[test]
fn time_dependent_specific_mechanical_power_identity_verified() {
    let solar = BuiltinSolarSystem2026_2028::load_verified().expect("solar");
    let t0 = TdbEpoch::new(850_000_000, 0);
    let mut specs: Vec<_> = solar.constants.bodies().values().cloned().collect();
    for sp in &mut specs {
        if sp.naif_id == NaifId::EARTH {
            sp.harmonics.push(HarmonicTerm {
                degree: 2,
                order: 2,
                c_bar: 2.439e-6,
                s_bar: -1.400e-6,
            });
        }
    }
    let fm =
        ForceModel::standard_solar_system(GravityConstantsCatalog::from_specs(specs)).expect("fm");
    let e0 = solar
        .ephemeris
        .geometric_state_icrf(NaifId::EARTH, NaifId::SSB, t0, DerivativeOrder::Velocity)
        .expect("e0");
    let x0 = e0.pos_m + DVec3::new(6_850_000.0, 1_100_000.0, 2_200_000.0);
    let v0 = e0.vel_m_s + DVec3::new(-1_600.0, 6_900.0, 2_400.0);
    let a_ng = DVec3::new(3.5, -2.0, 1.25);

    let p0 = fm
        .barycentric_potential_and_power(x0, t0, &solar.ephemeris, &solar.orientation)
        .expect("p0");
    let cfg = Dop853GenericConfig {
        rtol: 1.0e-13,
        atol: 1.0e-10,
        h_min_s: 1.0e-8,
        h_max_s: 60.0,
        h_init_s: 5.0,
        max_steps: 50_000,
    };
    let dur_s = 900.0_f64;
    let (yf, _, _) = dop853_integrate_generic(
        [x0.x, x0.y, x0.z, v0.x, v0.y, v0.z, 0.0],
        dur_s,
        &cfg,
        &[],
        |tau, y| -> Result<[f64; 7], sim_orbital::GravityError> {
            let ep = t0.add_span(TickSpan::from_ticks_u128(
                (tau.max(0.0) * (sim_spacetime::TICKS_PER_SECOND as f64)).round() as u128,
            ));
            let (pos, vel) = (DVec3::new(y[0], y[1], y[2]), DVec3::new(y[3], y[4], y[5]));
            let samp =
                fm.barycentric_potential_and_power(pos, ep, &solar.ephemeris, &solar.orientation)?;
            let acc = samp.accel_icrf_m_s2 + a_ng;
            Ok([
                vel.x,
                vel.y,
                vel.z,
                acc.x,
                acc.y,
                acc.z,
                a_ng.dot(vel) + samp.partial_u_dt_w_kg,
            ])
        },
    )
    .expect("power ode");

    let tf = t0.add_span(TickSpan::from_seconds_and_subsec_ticks(900, 0));
    let (xf, vf) = (
        DVec3::new(yf[0], yf[1], yf[2]),
        DVec3::new(yf[3], yf[4], yf[5]),
    );
    let pf = fm
        .barycentric_potential_and_power(xf, tf, &solar.ephemeris, &solar.orientation)
        .expect("pf");
    let delta_e = (0.5 * vf.length_squared() + pf.potential_j_kg)
        - (0.5 * v0.length_squared() + p0.potential_j_kg);
    assert!((delta_e - yf[6]).abs() / delta_e.abs().max(1.0) < 1.0e-10);
}
