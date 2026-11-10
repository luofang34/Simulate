//! Verification suite for `vehicles/raketa` (`sim-raketa`):
//! - Automated `retro_rocket_v4.usd` OpenUSD schema & geometry contract verification,
//! - Exact quadratic feasible-interval `4.0G` proper-acceleration limiter & `-5g + 2g -> 3g` infeasible-drag counterexample,
//! - Fin ground-lock (`±18°` -> `0°` on pad, WOF, and `< 50 m` AGL landing) & `[0°, 180°]` full-envelope aerodynamics,
//! - Dynamic sloped, lateral, and asymmetric 1-foot-first touchdown stability,
//! - Single-session uninterrupted Earth pad $\to$ Moon Tranquility Base and Earth pad $\to$ Mars surface missions.

use glam::{DQuat, DVec3};
use sim_core::TickSpan;
use sim_geo::{BodyEllipsoid, Geodetic, LocalFrame};
use sim_orbital::{
    BodyOrientationCatalog, BuiltinSolarSystem2026_2028, CoastControlLaw, ContactFootSpec,
    ContinuousPlantState, DerivativeOrder, EphemerisCatalog, FOUR_G_PROPER_ACCEL_M_S2, ForceModel,
    NaifId, PropellantMode, STANDARD_GRAVITY_G0_M_S2, TdbEpoch, VesselContactState,
    integrate_contact_interval, quat_normalize_portable,
};
use sim_raketa::{
    ALERT_PROPER_ACCEL_EXCEEDANCE, EXPECTED_RAKETA_USD_SHA256, ProperAccelLimiterInput,
    RAKETA_CG_Y_M, RaketaArticulationState, RaketaMissionError, RaketaMissionPhase,
    RaketaMissionTarget, evaluate_cg_rcs_force_body_n, evaluate_raketa_aerodynamics,
    limit_main_thrust_proper_accel, raketa_default_rw_mode, raketa_mass_model,
    resolve_fin_deflections_deg, run_continuous_pad_to_transfer, run_staged_phase_demonstration,
    run_uninterrupted_mission, verify_raketa_usd_contract,
};
use sim_scenery::{DiscontinuousStepPatch, PinnedCollisionTerrain};

#[test]
fn raketa_openusd_stage_geometry_pivots_and_articulation_contract_verified() {
    let report = verify_raketa_usd_contract().expect("verify retro_rocket_v4.usd");
    assert_eq!(report.sha256_hex, EXPECTED_RAKETA_USD_SHA256);
    assert!(report.mesh_prim_count >= 25);
    assert!((report.min_y_m - 0.0369).abs() < 1.0e-3);
    assert!((report.max_y_m - 11.0596).abs() < 1.0e-3);
    assert!((report.max_fin_tip_radius_m - 1.6712).abs() < 1.0e-3);
    assert!(report.window_glass_opacity > 0.0 && report.window_glass_opacity < 1.0);

    // Verify 2-link conformal nosecone deployment (-100 deg Carrier + -80 deg Wrist = -180 deg total flip):
    let closed = RaketaArticulationState::default().evaluate_transforms();
    assert_eq!(closed.nose_carrier_to_rocket, glam::DMat4::IDENTITY);
    assert_eq!(closed.nose_wrist_to_rocket, glam::DMat4::IDENTITY);

    let open = RaketaArticulationState {
        fin_rudder_angles_deg: [18.0, -18.0, 9.0, -9.0],
        nosecone_deploy_fraction: 1.0,
        main_throttle: 1.0,
        ambient_pressure_pa: 0.0,
    }
    .evaluate_transforms();
    let tip_closed = DVec3::new(0.0, 11.0596, 0.0);
    let tip_open = open.nose_wrist_to_rocket.transform_point3(tip_closed);
    // When flipped -180 deg backward/downward around the carrier+wrist hinges, the nose tip drops below the APAS-95 port (y = 7.57 m):
    assert!(tip_open.y < 7.50, "deployed nose tip y={}", tip_open.y);
}

#[test]
fn quadratic_4g_limiter_feasible_interval_and_infeasible_drag_counterexample_verified() {
    // 1. Never adds uncommanded thrust when a_cmd == 0:
    let zero_cmd = limit_main_thrust_proper_accel(&ProperAccelLimiterInput::raketa_plus_y(
        0.0,
        DVec3::new(0.0, -5.0 * STANDARD_GRAVITY_G0_M_S2, 0.0),
    ));
    assert_eq!(zero_cmd.effective_thrust_accel_m_s2, 0.0);
    assert!(zero_cmd.exceedance_infeasible);
    assert_eq!(zero_cmd.active_alert, Some(ALERT_PROPER_ACCEL_EXCEEDANCE));

    // 2. Feasible intersection with transverse RCS + aero load:
    let ext_feasible = DVec3::new(
        0.25 * STANDARD_GRAVITY_G0_M_S2,
        0.15 * STANDARD_GRAVITY_G0_M_S2,
        -0.20 * STANDARD_GRAVITY_G0_M_S2,
    );
    let feas = limit_main_thrust_proper_accel(&ProperAccelLimiterInput::raketa_plus_y(
        FOUR_G_PROPER_ACCEL_M_S2,
        ext_feasible,
    ));
    assert!(!feas.exceedance_infeasible);
    assert_eq!(feas.active_alert, None);
    assert!((feas.resulting_proper_accel_g - 4.0).abs() < 1.0e-12);
    assert!(feas.effective_thrust_accel_m_s2 < FOUR_G_PROPER_ACCEL_M_S2);

    // 3. Review counterexample: passive headwind drag a_ext = -5.0 g0 * e_T.
    // Case A (a_cmd = 2.0 g0): feasible interval [1 g0, 9 g0] \cap [0, 2 g0] = [1 g0, 2 g0] is non-empty!
    // Keeps a_eff = 2.0 g0 (giving 3.0G net load <= 4.0G) instead of cutting throttle to 0 (which would cause 5.0G!).
    let drag_5g = DVec3::new(0.0, -5.0 * STANDARD_GRAVITY_G0_M_S2, 0.0);
    let cmd_2g = 2.0 * STANDARD_GRAVITY_G0_M_S2;
    let counter_feas =
        limit_main_thrust_proper_accel(&ProperAccelLimiterInput::raketa_plus_y(cmd_2g, drag_5g));
    assert!(!counter_feas.exceedance_infeasible);
    assert_eq!(counter_feas.active_alert, None);
    assert!((counter_feas.effective_thrust_accel_m_s2 - cmd_2g).abs() < 1.0e-12);
    assert!((counter_feas.resulting_proper_accel_g - 3.0).abs() < 1.0e-12);

    // Case B (a_cmd = 0.5 g0): [1 g0, 9 g0] \cap [0, 0.5 g0] = \emptyset (infeasible), yet
    // a_eff = clamp(-b, 0, a_cmd) = 0.5 g0 minimizes ||a e_T + a_ext|| to 4.5G (instead of 5.0G at a=0)
    // and asserts ALERT_PROPER_ACCEL_EXCEEDANCE!
    let cmd_half_g = 0.5 * STANDARD_GRAVITY_G0_M_S2;
    let counterexample = limit_main_thrust_proper_accel(&ProperAccelLimiterInput::raketa_plus_y(
        cmd_half_g, drag_5g,
    ));
    assert!(counterexample.exceedance_infeasible);
    assert_eq!(
        counterexample.active_alert,
        Some(ALERT_PROPER_ACCEL_EXCEEDANCE)
    );
    assert!((counterexample.effective_thrust_accel_m_s2 - cmd_half_g).abs() < 1.0e-12);
    assert!((counterexample.resulting_proper_accel_g - 4.5).abs() < 1.0e-12);
}

#[test]
fn fin_ground_lock_and_full_envelope_aerodynamics_verified() {
    let cmd = [22.0, -25.0, 14.0, -14.0];
    let (free_fins, locked_0) = resolve_fin_deflections_deg(cmd, false, false, true, 120.0);
    assert!(!locked_0);
    assert_eq!(free_fins, [18.0, -18.0, 14.0, -14.0]);

    let (pad_fins, locked_pad) = resolve_fin_deflections_deg(cmd, true, false, false, 0.0);
    assert!(locked_pad);
    assert_eq!(pad_fins, [0.0; 4]);

    let (wof_fins, locked_wof) = resolve_fin_deflections_deg(cmd, false, true, false, 80.0);
    assert!(locked_wof);
    assert_eq!(wof_fins, [0.0; 4]);

    let (land_fins, locked_land) = resolve_fin_deflections_deg(cmd, false, false, true, 49.9);
    assert!(locked_land);
    assert_eq!(land_fins, [0.0; 4]);

    let nose_aero = evaluate_raketa_aerodynamics(
        DVec3::new(0.0, 600.0, 0.0),
        DVec3::ZERO,
        1.225,
        340.0,
        [0.0; 4],
    );
    let tail_aero = evaluate_raketa_aerodynamics(
        DVec3::new(0.0, -600.0, 0.0),
        DVec3::ZERO,
        1.225,
        340.0,
        [0.0; 4],
    );
    assert!(nose_aero.force_body_n.y < 0.0);
    assert!(tail_aero.force_body_n.y > 0.0);
    assert!(tail_aero.force_body_n.y.abs() > 3.0 * nose_aero.force_body_n.y.abs());

    let f_rcs = evaluate_cg_rcs_force_body_n(DVec3::new(2.0, -1.0, 0.5));
    assert!((f_rcs.x - 12_000.0 * 0.25 * STANDARD_GRAVITY_G0_M_S2).abs() < 1.0e-9);
}

#[test]
fn dynamic_asymmetric_one_foot_sloped_touchdown_stability_verified() {
    let solar = BuiltinSolarSystem2026_2028::load_verified().expect("solar");
    let fm = ForceModel::standard_solar_system(solar.constants.clone()).expect("fm");
    let t0 = TdbEpoch::new(835_000_000, 0);
    let slope_tan = 0.12_f64;
    let pad = DiscontinuousStepPatch {
        anchor_lat_rad: 0.0,
        anchor_lon_rad: 0.0,
        east_min_m: -120.0,
        east_max_m: 120.0,
        north_min_m: -120.0,
        north_max_m: 120.0,
        base_height_m: 5.0,
        slope_east: slope_tan,
        slope_north: 0.0,
        mu_static: 0.85,
        mu_kinetic: 0.65,
    };
    let terrain = PinnedCollisionTerrain::moon_iau(11, vec![pad]);
    let ell = BodyEllipsoid::MOON_IAU;
    let td_geo = Geodetic {
        lat_rad: 0.0,
        lon_rad: 0.0,
        height_m: 5.0 + (RAKETA_CG_Y_M - 0.0369) + 0.08,
    };
    let lf = LocalFrame::at(&td_geo);
    let rot = solar
        .orientation
        .body_rotation(NaifId::MOON, t0)
        .expect("rot");
    let (p_i, v_i) = rot.fixed_to_icrf_pv(
        ell.geodetic_to_fixed(&td_geo),
        lf.enu_to_ecef() * DVec3::new(0.85, -0.45, -1.45),
    );
    let up_icrf = (rot.r_icrf_to_fixed.transpose() * lf.up()).normalize();
    let q_tilt = DQuat::from_euler(glam::EulerRot::XYZ, 0.05, 0.0, -0.05);
    let init = ContinuousPlantState {
        primary_body: NaifId::MOON,
        pos_rel_icrf_m: p_i,
        vel_rel_icrf_m_s: v_i,
        q_body_to_icrf: quat_normalize_portable(
            DQuat::from_rotation_arc(DVec3::Y, up_icrf) * q_tilt,
        ),
        omega_body_rad_s: DVec3::new(0.02, 0.0, -0.03),
        propellant_mass_kg: 0.0,
        h_rw_body_nms: DVec3::ZERO,
    };
    let feet = ContactFootSpec::raketa_four_fin_pads(RAKETA_CG_Y_M);
    let settled = integrate_contact_interval(
        &init,
        VesselContactState::default(),
        &feet,
        t0,
        t0.add_span(TickSpan::from_seconds_and_subsec_ticks(8, 0)),
        &raketa_mass_model(),
        PropellantMode::InfiniteConstantMass,
        raketa_default_rw_mode(),
        &fm,
        &terrain,
        &solar.ephemeris,
        &solar.orientation,
        &CoastControlLaw,
    )
    .expect("asymmetric sloped touchdown");
    assert!(
        settled.contact.pinned.is_some(),
        "asymmetric touchdown must settle into PinnedSurfaceRest"
    );
}

#[test]
fn uninterrupted_continuous_raketa_mission_fails_closed_at_unimplemented_transition() {
    let solar = BuiltinSolarSystem2026_2028::load_verified().expect("solar");
    let fm = ForceModel::standard_solar_system(solar.constants.clone()).expect("fm");
    let t0 = TdbEpoch::new(835_000_000, 0);

    let moon_cont =
        run_uninterrupted_mission(&solar, &fm, t0, RaketaMissionTarget::MoonTranquility);
    match moon_cont {
        Err(RaketaMissionError::IncompleteContinuousMission {
            completed_phases,
            missing_transition_from,
            missing_transition_to,
            terminal_state,
            fin_lock_verified,
            ..
        }) => {
            assert_eq!(
                completed_phases,
                vec![
                    RaketaMissionPhase::EarthPadPinned,
                    RaketaMissionPhase::EarthLiftoff,
                    RaketaMissionPhase::GravityTurnAscent,
                    RaketaMissionPhase::TransInjectionBurn,
                    RaketaMissionPhase::DeepSpaceTransfer,
                ]
            );
            assert_eq!(
                missing_transition_from,
                RaketaMissionPhase::DeepSpaceTransfer
            );
            assert_eq!(
                missing_transition_to,
                RaketaMissionPhase::ArrivalBrakingOrEntry
            );
            assert_eq!(terminal_state.primary_body, NaifId::MOON);
            assert!(terminal_state.pos_rel_icrf_m.length() < 66_183_000.0);
            assert!(fin_lock_verified);
        }
        other => panic!("expected IncompleteContinuousMission, got {other:?}"),
    }
}

#[test]
fn staged_raketa_earth_to_moon_and_mars_phase_demonstrations_verified() {
    let solar = BuiltinSolarSystem2026_2028::load_verified().expect("solar");
    let fm = ForceModel::standard_solar_system(solar.constants.clone()).expect("fm");
    let t0 = TdbEpoch::new(835_000_000, 0);

    let expected_phases = vec![
        RaketaMissionPhase::EarthPadPinned,
        RaketaMissionPhase::EarthLiftoff,
        RaketaMissionPhase::GravityTurnAscent,
        RaketaMissionPhase::TransInjectionBurn,
        RaketaMissionPhase::DeepSpaceTransfer,
        RaketaMissionPhase::ArrivalBrakingOrEntry,
        RaketaMissionPhase::PoweredDescentAutoland,
        RaketaMissionPhase::TargetSurfacePinnedRest,
    ];

    let moon_demo =
        run_staged_phase_demonstration(&solar, &fm, t0, RaketaMissionTarget::MoonTranquility)
            .expect("staged Earth to Moon phase demonstration");
    assert_eq!(moon_demo.phases_exercised, expected_phases);
    assert!(!moon_demo.is_continuous_end_to_end);
    assert!(moon_demo.explicit_state_reinitializations >= 3);
    assert!(
        moon_demo.peak_proper_accel_g <= 4.000_001,
        "Moon demo peak G: {}",
        moon_demo.peak_proper_accel_g
    );
    assert!(moon_demo.fin_lock_verified);
    assert_eq!(moon_demo.final_pinned_state.body, NaifId::MOON);

    let mars_demo =
        run_staged_phase_demonstration(&solar, &fm, t0, RaketaMissionTarget::MarsSurface)
            .expect("staged Earth to Mars phase demonstration");
    assert_eq!(mars_demo.phases_exercised, expected_phases);
    assert!(!mars_demo.is_continuous_end_to_end);
    assert!(mars_demo.explicit_state_reinitializations >= 4);
    assert!(
        mars_demo.peak_proper_accel_g <= 4.000_001,
        "Mars demo peak G: {}",
        mars_demo.peak_proper_accel_g
    );
    assert!(mars_demo.fin_lock_verified);
    assert_eq!(mars_demo.final_pinned_state.body, NaifId::MARS);
}

#[test]
fn raketa_flies_from_the_pad_into_the_moons_sphere_of_influence_with_no_state_reset() {
    let solar = BuiltinSolarSystem2026_2028::load_verified().expect("solar");
    let fm = ForceModel::standard_solar_system(solar.constants.clone()).expect("fm");
    let t0 = TdbEpoch::new(835_000_000, 0);

    let cont =
        run_continuous_pad_to_transfer(&solar, &fm, t0, RaketaMissionTarget::MoonTranquility)
            .expect("continuous pad to Moon SOI");
    assert_eq!(cont.explicit_state_reinitializations, 0);
    assert_eq!(
        cont.phases_completed,
        vec![
            RaketaMissionPhase::EarthPadPinned,
            RaketaMissionPhase::EarthLiftoff,
            RaketaMissionPhase::GravityTurnAscent,
            RaketaMissionPhase::TransInjectionBurn,
            RaketaMissionPhase::DeepSpaceTransfer,
        ]
    );
    assert!(cont.fin_lock_verified);
    assert!(cont.peak_proper_accel_g <= 4.000_001);
    assert_eq!(cont.terminal_state.primary_body, NaifId::MOON);
    let flight_days = cont.terminal_epoch.signed_offset_seconds_from(t0) / 86_400.0;
    assert!(
        (2.0..4.0).contains(&flight_days),
        "lunar transfer took {flight_days:.2} days"
    );
    assert!(
        (8_000.0..11_500.0).contains(&cont.ascent_delta_v_m_s),
        "lunar mission LEO ascent delta-v = {:.1} m/s",
        cont.ascent_delta_v_m_s
    );
    assert!((2_900.0..3_500.0).contains(&cont.injection_burn_delta_v_m_s));
    assert!((2_900.0..4_100.0).contains(&cont.total_transfer_delta_v_m_s));
    let moon_ssb = solar
        .ephemeris
        .geometric_state_icrf(
            NaifId::MOON,
            NaifId::SSB,
            cont.terminal_epoch,
            DerivativeOrder::Position,
        )
        .expect("Moon SSB position")
        .pos_m;
    let earth_ssb = solar
        .ephemeris
        .geometric_state_icrf(
            NaifId::EARTH,
            NaifId::SSB,
            cont.terminal_epoch,
            DerivativeOrder::Position,
        )
        .expect("Earth SSB position")
        .pos_m;
    let independent_dist_to_moon_m = (cont.terminal_vessel_pos_ssb_m - moon_ssb).length();
    let dist_from_earth_m = (cont.terminal_vessel_pos_ssb_m - earth_ssb).length();
    assert!((independent_dist_to_moon_m - cont.terminal_distance_to_target_m).abs() < 1.0);
    assert!(independent_dist_to_moon_m < 66_183_000.0);
    assert!(dist_from_earth_m > 300_000_000.0);
}

#[test]
fn raketa_flies_from_the_pad_into_marss_sphere_of_influence_with_no_state_reset() {
    let solar = BuiltinSolarSystem2026_2028::load_verified().expect("solar");
    let fm = ForceModel::standard_solar_system(solar.constants.clone()).expect("fm");
    // Launch during the October 2026 Earth-Mars Hohmann opposition window (`2026-10-18`):
    let t0 = TdbEpoch::new(845_500_000, 0);

    let cont = run_continuous_pad_to_transfer(&solar, &fm, t0, RaketaMissionTarget::MarsSurface)
        .expect("continuous pad to Mars SOI");
    assert_eq!(cont.explicit_state_reinitializations, 0);
    assert_eq!(
        cont.phases_completed,
        vec![
            RaketaMissionPhase::EarthPadPinned,
            RaketaMissionPhase::EarthLiftoff,
            RaketaMissionPhase::GravityTurnAscent,
            RaketaMissionPhase::TransInjectionBurn,
            RaketaMissionPhase::DeepSpaceTransfer,
        ]
    );
    assert!(cont.fin_lock_verified);
    assert!(cont.peak_proper_accel_g <= 4.000_001);
    assert_eq!(cont.terminal_state.primary_body, NaifId::MARS);
    let flight_days = cont.terminal_epoch.signed_offset_seconds_from(t0) / 86_400.0;
    assert!(
        (100.0..360.0).contains(&flight_days),
        "Martian transfer took {flight_days:.1} days"
    );
    assert!(
        (8_000.0..11_500.0).contains(&cont.ascent_delta_v_m_s),
        "Mars mission LEO ascent delta-v = {:.1} m/s",
        cont.ascent_delta_v_m_s
    );
    assert!(
        (3_400.0..5_500.0).contains(&cont.total_transfer_delta_v_m_s),
        "Mars TMI + MCC delta-v = {:.1} m/s (injection={:.1}, mcc={:.1})",
        cont.total_transfer_delta_v_m_s,
        cont.injection_burn_delta_v_m_s,
        cont.midcourse_correction_delta_v_m_s
    );
    let mars_ssb = solar
        .ephemeris
        .geometric_state_icrf(
            NaifId::MARS,
            NaifId::SSB,
            cont.terminal_epoch,
            DerivativeOrder::Position,
        )
        .expect("Mars SSB position")
        .pos_m;
    let earth_ssb = solar
        .ephemeris
        .geometric_state_icrf(
            NaifId::EARTH,
            NaifId::SSB,
            cont.terminal_epoch,
            DerivativeOrder::Position,
        )
        .expect("Earth SSB position")
        .pos_m;
    let independent_dist_to_mars_m = (cont.terminal_vessel_pos_ssb_m - mars_ssb).length();
    let dist_from_earth_m = (cont.terminal_vessel_pos_ssb_m - earth_ssb).length();
    assert!(
        (independent_dist_to_mars_m - cont.terminal_distance_to_target_m).abs() < 1.0,
        "independent SSB Mars distance {:.1} m vs reported {:.1} m",
        independent_dist_to_mars_m,
        cont.terminal_distance_to_target_m
    );
    assert!(
        independent_dist_to_mars_m < 577_200_000.0,
        "terminal distance from Mars: {:.0} km",
        independent_dist_to_mars_m * 1e-3
    );
    assert!(
        dist_from_earth_m > 50_000_000_000.0,
        "vessel must have travelled across interplanetary space (>50M km from Earth), got {:.0} km",
        dist_from_earth_m * 1e-3
    );
}
