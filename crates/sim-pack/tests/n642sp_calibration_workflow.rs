//! End-to-end verification of the reusable multi-model closed-loop POH & empirical
//! calibration workflow (`N642SP` Cessna 172S Skyhawk SP + Jacobian Gauss-Newton
//! parameter identification).

use sim_pack::{
    CALIBRATION_SUITE_ASSET, CalibrationReport, CalibrationSuite, PITOT_STATIC_CALIBRATION_ASSET,
    PackError, PackageSource, PitotStaticCalibration, build_ace_feedback_report,
    builtin_c172_package, builtin_c172_source, default_design_trades, estimate_design_performance,
    evaluate_package, identify_package_sensitivities, load_syspack_from_ace_vehicle_json,
    run_design_trade_study,
};

#[path = "support/c172_validation_record.rs"]
mod c172_validation_record;

use c172_validation_record::{graded, listed};

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

/// Every row of the card as `(name, passed)`: the static run-up, stalls and
/// take-offs by label, climbs as `climb <alt> ft`, level points as
/// `cruise <alt> ft <rpm> rpm`, recorded segments by id.
#[cfg(test)]
fn card_rows(report: &CalibrationReport) -> Vec<(String, bool)> {
    let mut rows: Vec<(String, bool)> = report
        .static_runup
        .iter()
        .map(|&(_, _, _, passed)| ("static run-up".to_owned(), passed))
        .collect();
    rows.extend(report.stalls.iter().map(|r| (r.label.clone(), r.passed)));
    rows.extend(
        report
            .climbs
            .iter()
            .map(|r| (format!("climb {:.0} ft", r.pressure_alt_ft), r.passed)),
    );
    rows.extend(report.cruises.iter().map(|r| {
        let name = format!(
            "cruise {:.0} ft {:.0} rpm",
            r.pressure_alt_ft, r.commanded_rpm
        );
        (name, r.passed)
    }));
    rows.extend(
        report
            .empirical_segments
            .iter()
            .map(|r| (r.segment_id.clone(), r.passed)),
    );
    rows.extend(report.takeoffs.iter().map(|r| (r.label.clone(), r.passed)));
    rows
}

/// The card passes exactly the rows `performance/validation.json` lists as
/// passing and fails exactly those it lists as failing, so a fix (a known
/// failure starts passing) or a regression (a passing row fails) both show.
#[cfg(test)]
fn matches_the_validation_record(
    report: &CalibrationReport,
) -> Result<(), Box<dyn std::error::Error>> {
    let record: serde_json::Value = serde_json::from_str(c172_validation_record::RECORD)?;
    let rows = card_rows(report);
    let flown = |passed: bool| -> Vec<String> {
        let mut names: Vec<String> = rows
            .iter()
            .filter(|r| r.1 == passed)
            .map(|r| r.0.clone())
            .collect();
        names.sort();
        names
    };
    let md = report.to_markdown();
    assert_eq!(flown(true), listed("passing"), "passing rows\n{md}");
    assert_eq!(flown(false), listed("failing"), "failing rows\n{md}");
    let status = record.get("status").and_then(serde_json::Value::as_str);
    let calibrated = listed("failing").is_empty();
    assert_eq!(
        status == Some("calibrated"),
        calibrated,
        "status {status:?}"
    );
    Ok(())
}

/// Every row that gives a CG is flown within 0.2 in of it, and the stall rows
/// are at POH Figure 5-3's most-rearward CG.
#[cfg(test)]
fn flies_the_card_cg(report: &CalibrationReport) -> Result<(), Box<dyn std::error::Error>> {
    let raw = builtin_c172_source().read_asset_bytes(CALIBRATION_SUITE_ASSET)?;
    let card = CalibrationSuite::from_json_bytes(&raw)?;
    assert!(card.stalls.iter().all(|s| s.cg_fs_in == Some(47.3)));
    let stalls = card.stalls.iter().map(|s| s.cg_fs_in);
    let takeoffs = card.takeoff_points.iter().map(|t| t.cg_fs_in);
    let flown_stalls = report.stalls.iter().map(|s| (&s.label, s.flown_cg_fs_in));
    let flown_takeoffs = report.takeoffs.iter().map(|t| (&t.label, t.flown_cg_fs_in));
    for (card_cg, (label, flown)) in stalls
        .chain(takeoffs)
        .zip(flown_stalls.chain(flown_takeoffs))
    {
        let Some(card_cg) = card_cg else {
            continue;
        };
        let flown = flown.ok_or("a row with a card CG reported none")?;
        assert!(
            (flown - card_cg).abs() <= 0.2,
            "{label}: flown FS {flown:.2} against {card_cg}"
        );
    }
    Ok(())
}

#[test]
fn n642sp_c172_passes_reusable_closed_loop_calibration() -> Result<(), Box<dyn std::error::Error>> {
    let c172 = builtin_c172_package()?;
    let report = evaluate_package(&c172)?;
    matches_the_validation_record(&report)?;
    // AC 23-8C ends the AFM ground run at the scheduled lift-off speed, so the
    // roll is graded only if the airplane leaves the runway at it.
    for takeoff in &report.takeoffs {
        let early = takeoff.sim_lift_off_kcas - takeoff.target_lift_off_kcas;
        let check = format!("lift-off {}", takeoff.label);
        graded(&check, early.abs() <= 1.0, &format!("{early:+.1} kt"));
    }
    flies_the_card_cg(&report)?;
    let tail = report.tail_profile.as_ref().ok_or("missing N642SP tail")?;
    assert_eq!(tail.registration, "N642SP");
    assert_eq!(tail.serial_number, "172S8050");
    assert_eq!(tail.home_base_icao, "N51");
    assert!(report.stall_kcas_rmse_kt < 0.5);
    assert!(report.stall_kias_rmse_kt < 0.5);
    for (check, value, bound) in [
        ("climb RMSE", report.climb_rmse_fpm, 15.0),
        ("cruise KTAS RMSE", report.cruise_rmse_ktas, 1.5),
        ("cruise GPH RMSE", report.cruise_rmse_gph, 0.4),
        ("empirical GS RMSE", report.empirical_gs_rmse_kt, 1.5),
    ] {
        graded(check, value < bound, &format!("{value:.2} against {bound}"));
    }
    Ok(())
}

#[test]
fn jacobian_gauss_newton_identifier_computes_physical_sensitivities()
-> Result<(), Box<dyn std::error::Error>> {
    let c172 = builtin_c172_package()?;
    let sens = identify_package_sensitivities(&c172)?;
    matches_the_validation_record(&sens.baseline)?;
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
    for (check, value, low, high) in [
        ("estimator Vx", est.vx_kcas, 58.0, 66.0),
        ("estimator Vy", est.vy_kcas, 70.0, 78.0),
        ("estimator Vg", est.vg_kcas, 64.0, 72.0),
        ("estimator L/D", est.max_lift_to_drag, 8.5, 10.5),
        (
            "estimator ceiling",
            est.service_ceiling_ft,
            12_500.0,
            16_500.0,
        ),
    ] {
        let detail = format!("{value:.2} against {low}-{high}\n{md}");
        graded(check, (low..=high).contains(&value), &detail);
    }
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

    graded(
        "trade study high-AR L/D",
        high_ar.report.max_lift_to_drag > base.report.max_lift_to_drag,
        &format!("high-AR clean airframe should increase (L/D)_max: {study_md}"),
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
