//! Verification suite for `sim-orbital` Kepler/Encke propagation, warp governor, and GNC:
//! - Universal-variable Stumpff Kepler & rectified `EnckeKeplerPropagator` vs `DOP853`,
//! - `ReducedManifoldThrust` entry/exit gates & trajectory agreement vs `FullCoupled6Dof` (`100 s` `4.0G` burn),
//! - Conservative `WarpGovernor` (`14.14 s` from rest at `1 km` under `10 m/s^2`, never `1,000 s`) & global bottleneck reporting,
//! - Universal-variable Lambert boundary-value solver & closed-loop GNC laws.

use glam::{DQuat, DVec3};
use sim_orbital::{
    BuiltinSolarSystem2026_2028, CoastControlLaw, CollinearLagrangePoint, ContinuousPlantState,
    Dop853Tolerances, EnckeConfig, FOUR_G_PROPER_ACCEL_M_S2, ForceModel,
    FullCoupledPidFeedForwardSasLaw, L2BoundedArcTargetSpec, L2OrbitRegimeKind,
    L2SingleShootingStatus, LambertError, MassPropertiesModel, NaifId, PropellantMode,
    ReactionWheelMode, ReducedManifoldSteeringLaw, ReducedThrustGateConfig,
    ReducedThrustGateRejection, SmoothSteeringProfile, SynodicFrame, TdbEpoch, VesselId,
    VesselWarpConstraintInput, WarpBottleneckReason, check_reduced_manifold_gate,
    conservative_hazard_time_to_go_s, evaluate_global_warp_status, propagate_encke_interval,
    propagate_kepler_universal, propagate_plant_interval, quat_geodesic_angle_rad,
    solve_collinear_lagrange_gamma, solve_lambert_universal, stumpff_c2_c3,
    target_l2_terminal_x_arc_dop853,
};
use sim_spacetime::TickSpan;

#[test]
fn universal_kepler_stumpff_and_encke_rectification_verified() {
    let (c2_0, c3_0) = stumpff_c2_c3(0.0);
    assert!((c2_0 - 0.5).abs() < 1.0e-15);
    assert!((c3_0 - 1.0 / 6.0).abs() < 1.0e-15);

    let (c2_h, c3_h) = stumpff_c2_c3(-25.0);
    assert!((c2_h - (5.0_f64.cosh() - 1.0) / 25.0).abs() < 1.0e-13);
    assert!((c3_h - (5.0_f64.sinh() - 5.0) / 125.0).abs() < 1.0e-13);

    let mu = 3.986_004_418e14_f64;
    let (r0, v0) = (
        DVec3::new(7_000_000.0, 0.0, 0.0),
        DVec3::new(0.0, 12_500.0, 1_200.0),
    );
    let (r1, v1) = propagate_kepler_universal(r0, v0, 1_800.0, mu);
    let (r_back, v_back) = propagate_kepler_universal(r1, v1, -1_800.0, mu);
    assert!((r_back - r0).length() < 1.0e-5);
    assert!((v_back - v0).length() < 1.0e-8);

    let solar = BuiltinSolarSystem2026_2028::load_verified().expect("solar");
    let fm = ForceModel::standard_solar_system(solar.constants.clone()).expect("fm");
    let t0 = TdbEpoch::new(835_000_000, 0);
    let t1 = t0.add_span(TickSpan::from_seconds_and_subsec_ticks(600, 0));
    let init = ContinuousPlantState {
        primary_body: NaifId::EARTH,
        pos_rel_icrf_m: DVec3::new(8_500_000.0, 1_200_000.0, 400_000.0),
        vel_rel_icrf_m_s: DVec3::new(-1_100.0, 6_600.0, 800.0),
        q_body_to_icrf: DQuat::IDENTITY,
        omega_body_rad_s: DVec3::ZERO,
        propellant_mass_kg: 0.0,
        h_rw_body_nms: DVec3::ZERO,
    };
    let mass = MassPropertiesModel::diagonal(12_000.0, [85_000.0, 18_000.0, 85_000.0], [0.0; 3]);
    let rw = ReactionWheelMode::ConserveMomentum {
        h_max_nms: 5.0e6,
        tau_max_nm: 5.0e4,
    };
    let dop = propagate_plant_interval(
        &init,
        t0,
        t1,
        None,
        &Dop853Tolerances::default(),
        &mass,
        PropellantMode::InfiniteConstantMass,
        rw,
        &fm,
        &solar.ephemeris,
        &solar.orientation,
        &CoastControlLaw,
    )
    .expect("dop853");
    let encke = propagate_encke_interval(
        &init,
        t0,
        t1,
        &EnckeConfig {
            rectification_ratio: 1.0e-5,
            step_seconds: 5.0,
            max_perturbation_ratio: 5.0e-2,
        },
        &mass,
        PropellantMode::InfiniteConstantMass,
        rw,
        &fm,
        &solar.ephemeris,
        &solar.orientation,
        &CoastControlLaw,
    )
    .expect("encke");

    assert!((encke.final_state.pos_rel_icrf_m - dop.final_state.pos_rel_icrf_m).length() < 0.02);
    assert!(
        (encke.final_state.vel_rel_icrf_m_s - dop.final_state.vel_rel_icrf_m_s).length() < 1.0e-4
    );
    assert!(encke.rectifications >= 1);
}

#[cfg(test)]
fn eval_gate_helper(
    st: &ContinuousPlantState,
    q_ref: DQuat,
    om_ref: DVec3,
    al_ref: DVec3,
    q_inf: f64,
    prox: bool,
) -> Result<(), ReducedThrustGateRejection> {
    let mass = MassPropertiesModel::diagonal(12_000.0, [85_000.0, 18_000.0, 85_000.0], [0.0; 3]);
    check_reduced_manifold_gate(
        st,
        q_ref,
        om_ref,
        al_ref,
        q_inf,
        prox,
        &mass,
        5.0e4,
        5.0e6,
        &ReducedThrustGateConfig::default(),
    )
}

#[test]
fn reduced_manifold_thrust_all_six_rejection_gates_verified() {
    let base_state = ContinuousPlantState {
        primary_body: NaifId::EARTH,
        pos_rel_icrf_m: DVec3::new(42_164_000.0, 0.0, 0.0),
        vel_rel_icrf_m_s: DVec3::new(0.0, 3_074.6, 0.0),
        q_body_to_icrf: DQuat::IDENTITY,
        omega_body_rad_s: DVec3::new(0.0, 0.0, 0.004),
        propellant_mass_kg: 0.0,
        h_rw_body_nms: DVec3::ZERO,
    };
    let om = base_state.omega_body_rad_s;
    assert_eq!(
        eval_gate_helper(&base_state, DQuat::IDENTITY, om, DVec3::ZERO, 2.0e-3, false),
        Err(ReducedThrustGateRejection::DynamicPressureExceeded)
    );
    assert_eq!(
        eval_gate_helper(&base_state, DQuat::IDENTITY, om, DVec3::ZERO, 0.0, true),
        Err(ReducedThrustGateRejection::ProximityEnvelopeActive)
    );
    assert_eq!(
        eval_gate_helper(
            &base_state,
            DQuat::from_rotation_z(5.0e-4),
            om,
            DVec3::ZERO,
            0.0,
            false
        ),
        Err(ReducedThrustGateRejection::AttitudeErrorExceeded)
    );
    assert_eq!(
        eval_gate_helper(
            &base_state,
            DQuat::IDENTITY,
            DVec3::ZERO,
            DVec3::ZERO,
            0.0,
            false
        ),
        Err(ReducedThrustGateRejection::RateErrorExceeded)
    );
    assert_eq!(
        eval_gate_helper(
            &base_state,
            DQuat::IDENTITY,
            om,
            DVec3::new(0.0, 0.0, 1.0),
            0.0,
            false
        ),
        Err(ReducedThrustGateRejection::FeedForwardTorqueExceeded)
    );
    let mut sat_state = base_state;
    sat_state.h_rw_body_nms = DVec3::new(0.0, 0.0, 4.5e6);
    assert_eq!(
        eval_gate_helper(&sat_state, DQuat::IDENTITY, om, DVec3::ZERO, 0.0, false),
        Err(ReducedThrustGateRejection::WheelMomentumSaturationProximity)
    );
}

#[cfg(test)]
fn run_100s_burn_with_law(
    solar: &BuiltinSolarSystem2026_2028,
    fm: &ForceModel,
    init: &ContinuousPlantState,
    law: &dyn sim_orbital::ContinuousControlLaw,
) -> ContinuousPlantState {
    let t0 = TdbEpoch::new(835_000_000, 0);
    let t1 = t0.add_span(TickSpan::from_seconds_and_subsec_ticks(100, 0));
    let mass = MassPropertiesModel::diagonal(12_000.0, [85_000.0, 18_000.0, 85_000.0], [0.0; 3]);
    let rw = ReactionWheelMode::ConserveMomentum {
        h_max_nms: 5.0e6,
        tau_max_nm: 5.0e4,
    };
    propagate_plant_interval(
        init,
        t0,
        t1,
        None,
        &Dop853Tolerances::default(),
        &mass,
        PropellantMode::InfiniteConstantMass,
        rw,
        fm,
        &solar.ephemeris,
        &solar.orientation,
        law,
    )
    .expect("100 s 4G burn")
    .final_state
}

#[test]
fn reduced_manifold_4g_burn_equivalence_vs_full_coupled_6dof_verified() {
    let solar = BuiltinSolarSystem2026_2028::load_verified().expect("solar");
    let fm = ForceModel::standard_solar_system(solar.constants.clone()).expect("fm");
    let mass = MassPropertiesModel::diagonal(12_000.0, [85_000.0, 18_000.0, 85_000.0], [0.0; 3]);
    let profile = SmoothSteeringProfile {
        q0_body_to_icrf: DQuat::IDENTITY,
        pitch_rate_rad_s: 0.003,
        harmonic_amp_rad: 0.04,
        harmonic_freq_rad_s: 0.05,
        thrust_n: 12_000.0 * FOUR_G_PROPER_ACCEL_M_S2,
        exhaust_vel_m_s: 4_500.0,
    };
    let (q0, om0, al0) = profile.evaluate_reference(0.0);
    let init_burn = ContinuousPlantState {
        primary_body: NaifId::EARTH,
        pos_rel_icrf_m: DVec3::new(42_164_000.0, 0.0, 0.0),
        vel_rel_icrf_m_s: DVec3::new(0.0, 3_074.6, 0.0),
        q_body_to_icrf: q0,
        omega_body_rad_s: om0,
        propellant_mass_kg: 0.0,
        h_rw_body_nms: DVec3::ZERO,
    };
    assert!(eval_gate_helper(&init_burn, q0, om0, al0, 0.0, false).is_ok());

    let red_law = ReducedManifoldSteeringLaw {
        profile,
        mass_model: mass,
    };
    let full_law = FullCoupledPidFeedForwardSasLaw {
        profile,
        mass_model: mass,
        kp_nm_rad: 8_500.0,
        kd_nms_rad: 17_000.0,
        integral_bias_nm: DVec3::ZERO,
    };
    let red_st = run_100s_burn_with_law(&solar, &fm, &init_burn, &red_law);
    let full_st = run_100s_burn_with_law(&solar, &fm, &init_burn, &full_law);

    assert!((red_st.pos_rel_icrf_m - full_st.pos_rel_icrf_m).length() < 1.0e-3);
    assert!((red_st.vel_rel_icrf_m_s - full_st.vel_rel_icrf_m_s).length() < 1.0e-5);
    assert!(quat_geodesic_angle_rad(red_st.q_body_to_icrf, full_st.q_body_to_icrf) < 1.0e-7);
    assert!((red_st.h_rw_body_nms - full_st.h_rw_body_nms).length() < 1.0e-2);
}

#[test]
fn warp_governor_rest_hover_counterexample_and_global_bottleneck_verified() {
    // Counterexample from review: h = 1,000 m from rest (v_down = 0) under a_in,max = 10 m/s^2.
    // Unsafe heuristic h / max(v_down, 1) gave 1,000 s; physical impact occurs in sqrt(2 * 1000 / 10) = 14.1421356 s!
    let tgo_s = conservative_hazard_time_to_go_s(1_000.0, 0.0, 10.0, None);
    let expected_s = 200.0_f64.sqrt();
    assert!((tgo_s - expected_s).abs() < 1.0e-12);
    assert!(tgo_s < 14.15);

    // When a speculative Bernstein root is even earlier (e.g. cliff edge at 9.5 s), it takes precedence:
    let tgo_cliff = conservative_hazard_time_to_go_s(1_000.0, 0.0, 10.0, Some(9.5));
    assert!((tgo_cliff - 9.5).abs() < 1.0e-12);

    // PinnedSurfaceRest vessel (Vessel 1) does NOT block 100,000x cruise of orbital vessel (Vessel 2):
    let v_pinned = VesselWarpConstraintInput {
        vessel_id: VesselId(1),
        is_pinned_surface_rest: true,
        is_active_ground_contact: false,
        q_inf_pa: 101_325.0,
        in_proximity_group: false,
        hazard_tgo_s: 0.0,
        on_reduced_manifold: false,
        plan_wait_s: f64::INFINITY,
    };
    let v_cruise = VesselWarpConstraintInput {
        vessel_id: VesselId(2),
        is_pinned_surface_rest: false,
        is_active_ground_contact: false,
        q_inf_pa: 0.0,
        in_proximity_group: false,
        hazard_tgo_s: 1.0e9,
        on_reduced_manifold: true,
        plan_wait_s: f64::INFINITY,
    };
    let status_free = evaluate_global_warp_status(100_000.0, 1.0 / 60.0, &[v_pinned, v_cruise]);
    assert_eq!(status_free.achieved_warp, 100_000.0);
    assert_eq!(status_free.limiting_vessel, None);
    assert_eq!(
        status_free.bottleneck_reason,
        WarpBottleneckReason::Unconstrained
    );

    // Adding Vessel 3 in active atmospheric entry caps the universe warp and reports Vessel 3 as the bottleneck:
    let v_entry = VesselWarpConstraintInput {
        vessel_id: VesselId(3),
        is_pinned_surface_rest: false,
        is_active_ground_contact: false,
        q_inf_pa: 450.0,
        in_proximity_group: false,
        hazard_tgo_s: 120.0,
        on_reduced_manifold: false,
        plan_wait_s: f64::INFINITY,
    };
    let status_capped =
        evaluate_global_warp_status(100_000.0, 1.0 / 60.0, &[v_pinned, v_cruise, v_entry]);
    assert_eq!(status_capped.achieved_warp, 4.0);
    assert_eq!(status_capped.limiting_vessel, Some(VesselId(3)));
    assert_eq!(
        status_capped.bottleneck_reason,
        WarpBottleneckReason::AtmosphericFlight
    );
}

#[test]
fn universal_lambert_short_and_long_way_boundary_value_verified() {
    let mu = 3.986_004_418e14_f64;
    let r1 = DVec3::new(6_563_137.0, 0.0, 0.0);
    let r2 = DVec3::new(-42_164_000.0 * 0.5, 42_164_000.0 * 0.866_025_4, 2_500_000.0);
    let tof_s = 14_400.0;

    let sol_short = solve_lambert_universal(r1, r2, tof_s, mu, false).expect("short-way lambert");
    let (r2_prop, v2_prop) = propagate_kepler_universal(r1, sol_short.v1_m_s, tof_s, mu);
    assert!(
        (r2_prop - r2).length() < 0.05,
        "Lambert short-way endpoint err: {} m",
        (r2_prop - r2).length()
    );
    assert!((v2_prop - sol_short.v2_m_s).length() < 1.0e-4);
    assert!(sol_short.transfer_angle_rad < std::f64::consts::PI);

    let sol_long = solve_lambert_universal(r1, r2, tof_s, mu, true).expect("long-way lambert");
    let (r2_long, v2_long) = propagate_kepler_universal(r1, sol_long.v1_m_s, tof_s, mu);
    assert!(
        (r2_long - r2).length() < 0.05,
        "Lambert long-way endpoint err: {} m",
        (r2_long - r2).length()
    );
    assert!((v2_long - sol_long.v2_m_s).length() < 1.0e-4);
    assert!(sol_long.transfer_angle_rad > std::f64::consts::PI);

    // Verify distinct trajectories, mid-flight path directions, and opposite angular momentum:
    assert!((sol_short.v1_m_s - sol_long.v1_m_s).length() > 100.0);
    let h_short = r1.cross(sol_short.v1_m_s);
    let h_long = r1.cross(sol_long.v1_m_s);
    assert!(
        h_short.z > 0.0 && h_long.z < 0.0,
        "short-way and long-way angular momentum z must have opposite signs: {} vs {}",
        h_short.z,
        h_long.z
    );
    assert!(sol_short.v1_m_s.y > 0.0 && sol_long.v1_m_s.y < 0.0);
    let (r_early_short, _) = propagate_kepler_universal(r1, sol_short.v1_m_s, 600.0, mu);
    let (r_early_long, _) = propagate_kepler_universal(r1, sol_long.v1_m_s, 600.0, mu);
    assert!(r_early_short.y > 0.0 && r_early_long.y < 0.0);

    // Explicit error returns on degenerate/collinear/invalid inputs:
    assert_eq!(
        solve_lambert_universal(r1, r1, tof_s, mu, false),
        Err(LambertError::CollinearTransferGeometry { cos_dnu: 1.0 })
    );
    assert!(matches!(
        solve_lambert_universal(r1, -r1, tof_s, mu, false),
        Err(LambertError::CollinearTransferGeometry { .. })
    ));
    assert!(matches!(
        solve_lambert_universal(r1, r2, 0.0, mu, false),
        Err(LambertError::InvalidInput { .. })
    ));
}

#[test]
fn jwst_sun_earth_l2_bounded_targeted_arc_and_patched_conics_failure_verified() {
    let solar = BuiltinSolarSystem2026_2028::load_verified().expect("solar");
    let fm = ForceModel::standard_solar_system(solar.constants.clone()).expect("fm");
    let t0 = TdbEpoch::new(835_000_000, 0);

    // 1. Verify Sun-Earth L2 quintic equilibrium ratio (~0.010037 => ~1.50e9 m anti-Sun of Earth):
    let mu_sun = fm.constants().require_mu(NaifId::SUN).expect("mu sun");
    let mu_earth = fm.constants().require_mu(NaifId::EARTH).expect("mu earth");
    let gamma_l2 =
        solve_collinear_lagrange_gamma(mu_earth / (mu_sun + mu_earth), CollinearLagrangePoint::L2);
    assert!(gamma_l2 > 0.009_9 && gamma_l2 < 0.010_2);

    let frame0 = SynodicFrame::from_ephemeris(NaifId::SUN, NaifId::EARTH, t0, &solar.ephemeris)
        .expect("synodic");
    let l2_syn = frame0.lagrange_point_synodic_m(mu_sun, mu_earth, CollinearLagrangePoint::L2);
    assert!(l2_syn.x > 1.47e9 && l2_syn.x < 1.53e9);

    // 2. Target a finite-horizon 60-day bounded arc around Sun-Earth L2 (Az = 400,000 km)
    // via 1D single-shooting on initial synodic vy_0 using DOP853 N-body:
    let horizon_s = 60.0 * 86_400.0;
    let spec = L2BoundedArcTargetSpec::sun_earth_southern(t0, horizon_s, 4.0e8);
    let halo = target_l2_terminal_x_arc_dop853(&spec, &fm, &solar.ephemeris, &solar.orientation)
        .expect("target L2 bounded arc");
    assert_eq!(halo.regime, L2OrbitRegimeKind::FiniteHorizonTargetedArc);
    assert_eq!(halo.status, L2SingleShootingStatus::Converged);
    assert!(halo.terminal_x_residual_m.abs() <= spec.tolerance_x_m);
    assert!(halo.max_l2_distance_m < 1.2e9);
    assert!(halo.min_l2_distance_m > 2.0e8);

    // 3. Demonstrate why KSP single-body patched conics fails on the exact same initial state:
    let (p_icrf0, v_icrf0) =
        frame0.synodic_to_icrf_rel_secondary(halo.initial_pos_syn_m, halo.initial_vel_syn_m_s);
    let (r_ksp_end, v_ksp_end) = propagate_kepler_universal(p_icrf0, v_icrf0, horizon_s, mu_earth);
    let t_end = t0.add_span(TickSpan::from_seconds_and_subsec_ticks(horizon_s as u64, 0));
    let frame_end =
        SynodicFrame::from_ephemeris(NaifId::SUN, NaifId::EARTH, t_end, &solar.ephemeris)
            .expect("synodic end");
    let (p_ksp_syn, _) = frame_end.icrf_rel_secondary_to_synodic(r_ksp_end, v_ksp_end);
    let ksp_l2_err = (p_ksp_syn - l2_syn).length();
    // Single-body Keplerian propagation around Earth lacks Solar gravity & centrifugal balance,
    // drifting > 1,500,000 km away from the L2 region!
    assert!(
        ksp_l2_err > 1.5e9,
        "expected KSP patched conic to diverge from L2, got {ksp_l2_err}"
    );
}
