//! Closed-loop 6-DoF JSBSim performance, V-speed, ceiling, glide, specific-range,
//! endurance, and payload-range estimator for newly designed or uncalibrated aircraft.

use jsbsim::Fdm;

use crate::calibration::cruise::{
    configure_mass_and_overrides, fdm_err, fly_level_cruise, set_if_exists,
};
use crate::calibration::glide::{best_glide, measure_reference_geometry, sweep_glide_polar};
use crate::calibration::mission::{default_manual_mission_profiles, evaluate_mission_profile};
use crate::calibration::results::{
    DesignClimbPoint, DesignCruisePoint, DesignPerformanceReport, PayloadRangePoint,
};
use crate::calibration::solver::{
    FPS_PER_KT, POUNDS_PER_KG, ParameterOverrides, build_fdm, fly_steady_climb_kcas,
    probe_non_payload_lbs,
};
use crate::{AirspeedEnvelope, LoadedSyspack, PackError};

const KG_PER_US_GAL_100LL: f64 = 2.721_554_22;
const FPM_PER_KT: f64 = 101.268_591_4;
const FT_PER_NM: f64 = 6_076.115_486;
const KW_PER_HP: f64 = 0.745_699_872;
const N_PER_LBF: f64 = 4.448_221_615;

/// Evaluates a fixed-wing [`LoadedSyspack`] in closed-loop 6-DoF JSBSim simulation
/// to estimate its V-speed schedule, climb ceiling, glide ratio, cruise specific range,
/// endurance, and payload–range envelope without requiring prior POH tables.
pub fn estimate_design_performance(
    package: &LoadedSyspack,
) -> Result<DesignPerformanceReport, PackError> {
    let mut fdm = build_fdm(package)?;
    let base_lbs = probe_non_payload_lbs(&mut fdm)?;
    let (empty_mass_kg, usable_fuel_kg, nominal_payload_kg, ref_mass_kg) =
        extract_mass_breakdown(package);
    let usable_fuel_gal = usable_fuel_kg / KG_PER_US_GAL_100LL;
    let vs1_kcas = measure_stall_kcas(&mut fdm, ref_mass_kg, 0.0, base_lbs)?;
    let vs0_kcas = measure_stall_kcas(&mut fdm, ref_mass_kg, 1.0, base_lbs)?.min(vs1_kcas - 0.5);
    let (vx_kcas, max_climb_angle_deg, vy_kcas, sea_level_roc_fpm) =
        sweep_sea_level_vx_vy(&mut fdm, ref_mass_kg, vs1_kcas, base_lbs)?;
    let glide_polar = sweep_glide_polar(&mut fdm, ref_mass_kg, vs1_kcas, base_lbs)?;
    let (vg_kcas, max_lift_to_drag) = best_glide(&glide_polar, vs1_kcas);
    let reference = measure_reference_geometry(&mut fdm)?;
    let climb_profile = build_climb_profile(&mut fdm, ref_mass_kg, vy_kcas, base_lbs)?;
    let (service_ceiling_ft, absolute_ceiling_ft) = estimate_ceilings(&climb_profile);
    let v_max_sl = max_level_speed_sl(&mut fdm, ref_mass_kg, vy_kcas, base_lbs)?;
    let cruise_polar =
        build_cruise_polar(&mut fdm, ref_mass_kg, usable_fuel_gal, v_max_sl, base_lbs)?;
    let payload_range = build_payload_range(
        empty_mass_kg,
        usable_fuel_kg,
        nominal_payload_kg,
        ref_mass_kg,
        &cruise_polar,
    );
    let pos_g = package
        .airspeed_envelope()
        .map_or(3.8, |e| e.positive_limit_g);
    let va_kcas = vs1_kcas * pos_g.sqrt();
    let synthesized_envelope = synthesize_airspeed_envelope(
        package,
        ref_mass_kg,
        pos_g,
        (vs0_kcas, vs1_kcas, vx_kcas, vy_kcas, vg_kcas, va_kcas),
        v_max_sl,
    );
    let mut report = DesignPerformanceReport {
        package_id: package.manifest().id.clone(),
        package_name: package.manifest().name.clone(),
        empty_mass_kg,
        usable_fuel_kg,
        usable_fuel_gal,
        nominal_payload_kg,
        reference_gross_mass_kg: ref_mass_kg,
        vs0_kcas,
        vs1_kcas,
        vx_kcas,
        max_climb_angle_deg,
        vy_kcas,
        sea_level_roc_fpm,
        vg_kcas,
        max_lift_to_drag,
        glide_nm_per_1000ft: max_lift_to_drag * (1_000.0 / FT_PER_NM),
        va_kcas,
        max_level_speed_sl_ktas: v_max_sl,
        service_ceiling_ft,
        absolute_ceiling_ft,
        climb_profile,
        cruise_polar,
        payload_range,
        mission_profiles: Vec::new(),
        synthesized_envelope,
        glide_polar,
        reference,
    };
    for spec in default_manual_mission_profiles(empty_mass_kg, usable_fuel_kg, nominal_payload_kg) {
        report
            .mission_profiles
            .push(evaluate_mission_profile(package, &report, &spec)?);
    }
    Ok(report)
}

fn extract_mass_breakdown(package: &LoadedSyspack) -> (f64, f64, f64, f64) {
    let comps = &package.manifest().components;
    let dry_sum: f64 = comps.iter().map(|c| c.dry_mass_kg).sum();
    let fluid_sum: f64 = comps.iter().map(|c| c.initial_fluid_mass_kg).sum();
    let payload_sum: f64 = comps.iter().map(|c| c.initial_payload_mass_kg).sum();

    let tail = package
        .calibration_suite()
        .and_then(|s| s.tail_profile.as_ref());
    let empty_kg = tail.map_or(dry_sum.max(400.0), |t| t.basic_empty_mass_kg);
    let fuel_kg = tail.map_or_else(
        || {
            if fluid_sum > 1.0 {
                fluid_sum
            } else {
                (empty_kg * 0.14).max(50.0)
            }
        },
        |t| t.usable_fuel_kg,
    );
    let payload_kg = if payload_sum > 1.0 {
        payload_sum
    } else {
        170.0
    };
    let ref_kg = package
        .airspeed_envelope()
        .map_or(empty_kg + fuel_kg + payload_kg, |e| e.reference_mass_kg);
    (empty_kg, fuel_kg, payload_kg, ref_kg)
}

fn measure_stall_kcas(
    fdm: &mut Fdm,
    gross_mass_kg: f64,
    flap_norm: f64,
    base_non_payload_lbs: f64,
) -> Result<f64, PackError> {
    configure_mass_and_overrides(
        fdm,
        gross_mass_kg,
        base_non_payload_lbs,
        ParameterOverrides::default(),
    )?;
    let ref_kt = 55.0_f64;
    let mut max_lift_lbs = 0.0_f64;
    let mut weight_lbs = gross_mass_kg * POUNDS_PER_KG;
    for step in 16..=36_u32 {
        let alpha_rad = f64::from(step) * 0.01;
        for (prop, val) in [
            ("ic/h-sl-ft", 100.0),
            ("ic/terrain-elevation-ft", 0.0),
            ("ic/vt-fps", ref_kt * FPS_PER_KT),
            ("ic/alpha-rad", alpha_rad),
            ("ic/theta-rad", alpha_rad),
            ("ic/beta-rad", 0.0),
            ("fcs/throttle-cmd-norm", 0.0),
        ] {
            fdm.set_property(prop, val).map_err(fdm_err)?;
        }
        set_if_exists(fdm, "fcs/flap-cmd-norm", flap_norm)?;
        set_if_exists(fdm, "fcs/flap-pos-norm", flap_norm)?;
        fdm.run_ic().map_err(fdm_err)?;
        let lift = fdm.get_property("forces/fwz-aero-lbs").map_err(fdm_err)?;
        if lift > max_lift_lbs {
            max_lift_lbs = lift;
        }
        weight_lbs = fdm.get_property("inertia/weight-lbs").map_err(fdm_err)?;
    }
    Ok((ref_kt * ref_kt * (weight_lbs / max_lift_lbs.max(1.0))).sqrt())
}

fn sweep_sea_level_vx_vy(
    fdm: &mut Fdm,
    gross_mass_kg: f64,
    vs1_kcas: f64,
    base_non_payload_lbs: f64,
) -> Result<(f64, f64, f64, f64), PackError> {
    let ratios = [1.16, 1.205, 1.26, 1.33, 1.40, 1.47];
    let mut best_angle = (vs1_kcas * 1.205, -90.0_f64);
    let mut best_roc = (vs1_kcas * 1.40, -10_000.0_f64);
    for ratio in ratios {
        let cmd_kcas = (vs1_kcas * ratio).max(45.0);
        let (sim_kcas, sim_ktas, sim_fpm, _) = fly_steady_climb_kcas(
            fdm,
            0.0,
            gross_mass_kg,
            cmd_kcas,
            false,
            base_non_payload_lbs,
            ParameterOverrides::default(),
            None,
        )?;
        let sin_gamma = (sim_fpm / (sim_ktas.max(25.0) * FPM_PER_KT)).clamp(-0.9, 0.9);
        let gamma_deg = sin_gamma.asin().to_degrees();
        if gamma_deg > best_angle.1 {
            best_angle = (sim_kcas, gamma_deg);
        }
        if sim_fpm > best_roc.1 {
            best_roc = (sim_kcas, sim_fpm);
        }
    }
    let vx = best_angle.0.min(best_roc.0);
    Ok((vx, best_angle.1, best_roc.0, best_roc.1))
}

fn build_climb_profile(
    fdm: &mut Fdm,
    gross_mass_kg: f64,
    vy_kcas: f64,
    base_non_payload_lbs: f64,
) -> Result<Vec<DesignClimbPoint>, PackError> {
    let mut profile = Vec::with_capacity(4);
    for alt_ft in [0.0, 4_000.0, 8_000.0, 12_000.0] {
        let cmd_kcas = (vy_kcas - 0.000_25 * alt_ft).max(48.0);
        let (kcas, ktas, roc_fpm, _) = fly_steady_climb_kcas(
            fdm,
            alt_ft,
            gross_mass_kg,
            cmd_kcas,
            alt_ft >= 3_000.0,
            base_non_payload_lbs,
            ParameterOverrides::default(),
            None,
        )?;
        let sin_gamma = (roc_fpm / (ktas.max(25.0) * FPM_PER_KT)).clamp(-0.9, 0.9);
        let climb_angle_deg = sin_gamma.asin().to_degrees();
        let gradient_ft_per_nm = (roc_fpm * 60.0) / ktas.max(25.0);
        profile.push(DesignClimbPoint {
            pressure_alt_ft: alt_ft,
            kcas,
            ktas,
            roc_fpm,
            climb_angle_deg,
            gradient_ft_per_nm,
        });
    }
    Ok(profile)
}

fn estimate_ceilings(profile: &[DesignClimbPoint]) -> (f64, f64) {
    let n = profile.len() as f64;
    if n < 2.0 {
        return (12_000.0, 14_000.0);
    }
    let mean_h = profile.iter().map(|p| p.pressure_alt_ft).sum::<f64>() / n;
    let mean_rc = profile.iter().map(|p| p.roc_fpm).sum::<f64>() / n;
    let mut num = 0.0_f64;
    let mut den = 0.0_f64;
    for p in profile {
        let dh = p.pressure_alt_ft - mean_h;
        num += dh * (p.roc_fpm - mean_rc);
        den += dh * dh;
    }
    let slope_per_ft = (num / den.max(1.0)).min(-0.01);
    let rc0 = mean_rc - slope_per_ft * mean_h;
    let service = ((100.0 - rc0) / slope_per_ft).clamp(1_000.0, 45_000.0);
    let absolute = ((-rc0) / slope_per_ft).clamp(service + 500.0, 50_000.0);
    (service, absolute)
}

/// Sea-level true airspeed at full throttle (`KTAS`).
fn max_level_speed_sl(
    fdm: &mut Fdm,
    gross_mass_kg: f64,
    vy_kcas: f64,
    base_non_payload_lbs: f64,
) -> Result<f64, PackError> {
    Ok(fly_level_cruise(
        fdm,
        0.0,
        gross_mass_kg,
        None,
        Some(1.0),
        (vy_kcas * 1.45).max(95.0),
        base_non_payload_lbs,
        ParameterOverrides::default(),
        None,
    )?
    .ktas)
}

fn build_cruise_polar(
    fdm: &mut Fdm,
    gross_mass_kg: f64,
    usable_fuel_gal: f64,
    seed_ktas: f64,
    base_non_payload_lbs: f64,
) -> Result<Vec<DesignCruisePoint>, PackError> {
    let settings = [
        ("Economy Cruise (4,000 ft)", 4_000.0, 0.68),
        ("Normal Cruise (6,000 ft)", 6_000.0, 0.78),
        ("High-Speed Cruise (8,000 ft)", 8_000.0, 0.88),
    ];
    let mut out = Vec::with_capacity(settings.len());
    for (label, alt_ft, thr) in settings {
        let point = fly_level_cruise(
            fdm,
            alt_ft,
            gross_mass_kg,
            None,
            Some(thr),
            seed_ktas * 0.9,
            base_non_payload_lbs,
            ParameterOverrides::default(),
            None,
        )?;
        let (ktas, rpm, hp_opt) = (point.ktas, point.rpm, point.total_hp);
        let (fuel_flow_gph, direct) = match point.total_gph {
            Some(g) if g > 0.2 => (g, true),
            _ => {
                let est_hp = hp_opt.unwrap_or_else(|| 0.000_07 * ktas.powi(3));
                ((est_hp.max(35.0) * 0.44) / 6.0, false)
            }
        };
        let fuel_flow_kg_h = fuel_flow_gph * KG_PER_US_GAL_100LL;
        let specific_range_nm_per_gal = ktas / fuel_flow_gph.max(0.1);
        let specific_range_nm_per_kg = ktas / fuel_flow_kg_h.max(0.1);
        let total_hr = usable_fuel_gal / fuel_flow_gph.max(0.1);
        let endurance_45min_res_hr = (total_hr - 0.75).max(0.0);
        out.push(DesignCruisePoint {
            label: label.to_owned(),
            pressure_alt_ft: alt_ft,
            throttle_cmd: thr,
            rpm,
            ktas,
            fuel_flow_gph,
            fuel_flow_kg_h,
            direct_engine_fuel_flow: direct,
            specific_range_nm_per_gal,
            specific_range_nm_per_kg,
            endurance_45min_res_hr,
            range_45min_res_nm: ktas * endurance_45min_res_hr,
            range_no_res_nm: ktas * total_hr,
            shaft_power_kw: hp_opt.map(|hp| hp * KW_PER_HP),
            thrust_n: point.total_thrust_lbf * N_PER_LBF,
        });
    }
    Ok(out)
}

fn build_payload_range(
    empty_mass_kg: f64,
    usable_fuel_kg: f64,
    nominal_payload_kg: f64,
    ref_mass_kg: f64,
    polar: &[DesignCruisePoint],
) -> Vec<PayloadRangePoint> {
    let econ = polar.first();
    let norm = polar.get(1).or(econ);
    let mut pts = Vec::with_capacity(3);
    if let Some(n) = norm {
        let max_pay = (ref_mass_kg - empty_mass_kg - usable_fuel_kg * 0.65).max(nominal_payload_kg);
        let fuel_a =
            (ref_mass_kg - empty_mass_kg - max_pay).clamp(usable_fuel_kg * 0.4, usable_fuel_kg);
        let hr_a = (fuel_a / n.fuel_flow_kg_h.max(0.1) - 0.75).max(0.0);
        pts.push(PayloadRangePoint {
            label: "Point A — Max Payload / Reduced Fuel".to_owned(),
            payload_kg: max_pay,
            fuel_kg: fuel_a,
            takeoff_mass_kg: empty_mass_kg + max_pay + fuel_a,
            cruise_ktas: n.ktas,
            range_45min_res_nm: n.ktas * hr_a,
        });
        pts.push(PayloadRangePoint {
            label: "Point B — Full Usable Fuel (Normal Cruise)".to_owned(),
            payload_kg: (ref_mass_kg - empty_mass_kg - usable_fuel_kg).max(0.0),
            fuel_kg: usable_fuel_kg,
            takeoff_mass_kg: empty_mass_kg
                + usable_fuel_kg
                + (ref_mass_kg - empty_mass_kg - usable_fuel_kg).max(0.0),
            cruise_ktas: n.ktas,
            range_45min_res_nm: n.range_45min_res_nm,
        });
    }
    if let Some(e) = econ {
        pts.push(PayloadRangePoint {
            label: "Point C — Zero-Payload Ferry (Economy Cruise)".to_owned(),
            payload_kg: 0.0,
            fuel_kg: usable_fuel_kg,
            takeoff_mass_kg: empty_mass_kg + usable_fuel_kg,
            cruise_ktas: e.ktas * 1.015,
            range_45min_res_nm: e.range_45min_res_nm * 1.03,
        });
    }
    pts
}

fn synthesize_airspeed_envelope(
    package: &LoadedSyspack,
    ref_mass_kg: f64,
    pos_g: f64,
    speeds_kcas: (f64, f64, f64, f64, f64, f64),
    max_level_ktas: f64,
) -> AirspeedEnvelope {
    let (vs0_kcas, vs1_kcas, vx_kcas, vy_kcas, vg_kcas, va_kcas) = speeds_kcas;
    if let Some(existing) = package.airspeed_envelope() {
        return existing.clone();
    }
    let vs0 = (vs0_kcas.round() as f32).max(30.0);
    let vs1 = (vs1_kcas.round() as f32).max(vs0 + 2.0);
    let vfe = ((vs0_kcas * 1.80).round() as f32).max(vs1 + 15.0);
    let vno = ((max_level_ktas * 1.03).round() as f32).max(vfe + 15.0);
    let vne = ((vno as f64 * 1.25).round() as f32).max(vno + 15.0);
    let vx = (vx_kcas.round() as f32).clamp(vs1 + 2.0, vno - 5.0);
    let vy = (vy_kcas.round() as f32).clamp(vx, vno - 2.0);
    let vg = (vg_kcas.round() as f32).clamp(vs1 + 2.0, vno - 2.0);
    let va = (va_kcas.round() as f32).clamp(vs1 + 5.0, vno);
    let vr = ((vs1_kcas * 1.12).round() as f32).clamp(vs1 + 1.0, vx);
    let vref = ((vs0_kcas * 1.30).round() as f32).clamp(vs0 + 5.0, vfe);
    AirspeedEnvelope {
        type_designator: package.manifest().id.clone(),
        reference_document: "Synthesized by sim_pack::estimate_design_performance".to_owned(),
        reference_mass_kg: ref_mass_kg,
        positive_limit_g: pos_g,
        negative_limit_g: -0.4 * pos_g,
        vs0_kt: vs0,
        vs1_kt: vs1,
        vfe_kt: vfe,
        vfe_takeoff_kt: Some(((vfe + vno) * 0.5).round()),
        vr_kt: Some(vr),
        vx_kt: Some(vx),
        vy_kt: Some(vy),
        vg_kt: Some(vg),
        va_ref_kt: Some(va),
        vref_kt: Some(vref),
        vno_kt: vno,
        vne_kt: vne,
    }
}
