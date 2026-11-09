//! End-to-end verification of the reusable multi-model closed-loop POH & empirical
//! calibration workflow (`N642SP` Cessna 172S Skyhawk SP + Jacobian Gauss-Newton
//! parameter identification).

use sim_pack::{
    PITOT_STATIC_CALIBRATION_ASSET, PackError, PackageSource, PitotStaticCalibration,
    build_ace_feedback_report, builtin_c172_package, builtin_c172_source, default_design_trades,
    estimate_design_performance, evaluate_package, identify_package_sensitivities,
    load_syspack_from_ace_vehicle_json, run_design_trade_study,
};

#[test]
fn pitot_static_calibration_round_trips_and_rejects_non_monotonic_tables()
-> Result<(), Box<dyn std::error::Error>> {
    let c172 = builtin_c172_package()?;
    let pec = c172
        .pitot_static_calibration()
        .ok_or("C172 must ship pitot_static_calibration.json")?;
    assert_eq!(pec.type_designator, "C172");

    for (flap_norm, kias_expected, kcas_expected) in [
        (0.0, 48.0, 53.0),
        (0.0, 70.0, 70.0),
        (0.0, 120.0, 117.0),
        (0.33333, 42.0, 50.0),
        (1.0, 40.0, 48.0),
        (1.0, 85.0, 86.0),
    ] {
        let kias = pec.kcas_to_kias(flap_norm, kcas_expected);
        let kcas = pec.kias_to_kcas(flap_norm, kias);
        assert!(
            (kias - kias_expected).abs() < 0.05,
            "flap={flap_norm} kias={kias}"
        );
        assert!(
            (kcas - kcas_expected).abs() < 0.05,
            "flap={flap_norm} kcas={kcas}"
        );
    }

    let raw = builtin_c172_source().read_asset_bytes(PITOT_STATIC_CALIBRATION_ASSET)?;
    let mut broken: PitotStaticCalibration = serde_json::from_slice(&raw)?;
    broken.curves[0].points[3].kcas = broken.curves[0].points[2].kcas;
    assert!(matches!(
        broken.validate(),
        Err(PackError::InvalidCalibration { .. })
    ));
    Ok(())
}

#[test]
fn n642sp_c172_passes_reusable_closed_loop_calibration() -> Result<(), Box<dyn std::error::Error>> {
    let c172 = builtin_c172_package()?;
    let report = evaluate_package(&c172)?;
    let md = report.to_markdown();
    assert!(report.all_passed, "C172 calibration failed:\n{md}");
    let tail = report.tail_profile.as_ref().ok_or("missing N642SP tail")?;
    assert_eq!(tail.registration, "N642SP");
    assert_eq!(tail.serial_number, "172S8050");
    assert_eq!(tail.home_base_icao, "N51");
    assert!(report.stall_kcas_rmse_kt < 0.5);
    assert!(report.stall_kias_rmse_kt < 0.5);
    assert!(report.climb_rmse_fpm < 15.0);
    assert!(report.cruise_rmse_ktas < 1.5);
    assert!(report.cruise_rmse_gph < 0.4);
    assert!(report.empirical_gs_rmse_kt < 1.5);
    Ok(())
}

#[test]
fn jacobian_gauss_newton_identifier_computes_physical_sensitivities()
-> Result<(), Box<dyn std::error::Error>> {
    let c172 = builtin_c172_package()?;
    let sens = identify_package_sensitivities(&c172)?;
    assert!(sens.baseline.all_passed);
    assert!(sens.linearized_predicted_cost <= sens.baseline_cost + 1e-9);
    let ve = sens
        .adjustments
        .iter()
        .find(|a| a.parameter == "volumetric_efficiency_scale")
        .ok_or("missing ve sensitivity")?;
    let bsfc = sens
        .adjustments
        .iter()
        .find(|a| a.parameter == "bsfc_scale")
        .ok_or("missing bsfc sensitivity")?;
    assert!(
        ve.mean_d_climb_fpm > 500.0,
        "d_climb={}",
        ve.mean_d_climb_fpm
    );
    assert!(
        bsfc.mean_d_cruise_gph > 1.0,
        "d_gph={}",
        bsfc.mean_d_cruise_gph
    );
    Ok(())
}

#[test]
fn new_design_performance_estimator_synthesizes_vspeeds_ceiling_and_payload_range()
-> Result<(), Box<dyn std::error::Error>> {
    let c172 = builtin_c172_package()?;
    let est = estimate_design_performance(&c172)?;
    let md = est.to_markdown();
    assert!(est.vs0_kcas < est.vs1_kcas, "{md}");
    assert!(est.vx_kcas <= est.vy_kcas, "{md}");
    assert!((58.0..=66.0).contains(&est.vx_kcas), "{md}");
    assert!((70.0..=78.0).contains(&est.vy_kcas), "{md}");
    assert!((64.0..=72.0).contains(&est.vg_kcas), "{md}");
    assert!((8.5..=10.5).contains(&est.max_lift_to_drag), "{md}");
    assert!(
        (12_500.0..=16_500.0).contains(&est.service_ceiling_ft),
        "{md}"
    );
    assert_eq!(est.cruise_polar.len(), 3);
    assert_eq!(est.payload_range.len(), 3);
    assert_eq!(est.mission_profiles.len(), 4, "{md}");
    assert!(est.cruise_polar[1].range_45min_res_nm > 450.0, "{md}");
    let hilo = est
        .mission_profiles
        .iter()
        .find(|m| m.id == "hi_lo_drop_lo_hi")
        .ok_or("missing hi_lo_drop_lo_hi profile")?;
    assert!(hilo.total_payload_dropped_kg > 100.0, "{md}");
    assert!(hilo.action_radius_nm.unwrap_or(0.0) > 200.0, "{md}");
    assert!(
        hilo.drop_radius_bonus_nm.unwrap_or(0.0) > 10.0,
        "mid-mission payload drop must increase action radius: {md}"
    );
    assert!(
        hilo.post_drop_roc_gain_fpm.unwrap_or(0.0) > 40.0,
        "post-drop 6-DoF climb rate must exceed pre-drop climb rate: {md}"
    );
    est.synthesized_envelope.validate()?;
    Ok(())
}

#[test]
fn design_trade_study_and_ace_rs_codesign_bridge_evaluate_closed_loop()
-> Result<(), Box<dyn std::error::Error>> {
    let c172 = builtin_c172_package()?;
    let study = run_design_trade_study(&c172, &default_design_trades())?;
    let study_md = study.to_markdown();
    assert_eq!(study.candidates.len(), 4, "{study_md}");
    let base = &study.candidates[0];
    let high_ar = &study.candidates[1];
    let uprated = &study.candidates[2];
    let long_tank = &study.candidates[3];

    assert!(
        high_ar.report.max_lift_to_drag > base.report.max_lift_to_drag,
        "high-AR clean airframe should increase (L/D)_max: {study_md}"
    );
    assert!(
        uprated.report.sea_level_roc_fpm > base.report.sea_level_roc_fpm + 40.0,
        "uprated powerplant should increase sea-level ROC: {study_md}"
    );
    assert!(
        long_tank.ferry_range_nm > base.ferry_range_nm + 80.0,
        "extended fuel tank should increase ferry range: {study_md}"
    );

    let ace_sample = br#"{
      "schema_version": 1,
      "id": "ace_c172_variant",
      "name": "ACE Synthesized C172 Variant",
      "domain": "fixed_wing",
      "parameters": {
        "wing.area": { "value": { "value": 16.5, "unit": "m^2" } },
        "wing.span": { "value": { "value": 11.4, "unit": "m" } },
        "aero.cd0": { "value": { "value": 0.030, "unit": "1" } },
        "aero.oswald": { "value": { "value": 0.82, "unit": "1" } },
        "weight.oew": { "value": { "value": 775.0, "unit": "kg" } },
        "weight.maximum_fuel": { "value": { "value": 155.0, "unit": "kg" } },
        "engine.rated_power": { "value": { "value": 145000.0, "unit": "W" } },
        "engine.sizing_factor": { "value": { "value": 1.0, "unit": "1" } },
        "engine.bsfc": { "value": { "value": 0.240, "unit": "kg/kWh" } },
        "propeller.efficiency": { "value": { "value": 0.81, "unit": "1" } }
      }
    }"#;
    let ace_pack = load_syspack_from_ace_vehicle_json(ace_sample)?;
    let ace_report = estimate_design_performance(&ace_pack)?;
    let feedback = build_ace_feedback_report(&ace_pack, &ace_report, Some(ace_sample));
    assert!(feedback.aex_cli_command.contains("aex -- synthesize"));
    assert!(feedback.hi_lo_drop_action_radius_nm > 200.0);
    Ok(())
}
