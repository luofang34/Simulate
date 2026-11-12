//! Empirical 6-DoF JSBSim flight-test envelope validation of `vehicles/c172`
//! against the Cessna 172S Skyhawk Pilot's Operating Handbook (`180 HP`
//! Lycoming IO-360-L2A) and its verified `performance/airspeed_envelope.json`.

use std::collections::BTreeMap;

use jsbsim::{Fdm, LoadOptions};
use sim_core::{
    Block, ControlInputs, EngineStartMode, JSBSIM_DEFAULT_PERIOD, JsbsimFdmBlock, JsbsimFdmConfig,
    KinematicInput,
};
use sim_geo::Geodetic;
use sim_pack::{
    AIRSPEED_ENVELOPE_ASSET, AirspeedEnvelope, LoadedSyspack, MemoryPackageSource, PackError,
    PackageSource, SyspackError, builtin_c172_package, builtin_c172_source,
};

#[path = "support/c172_validation_record.rs"]
mod c172_validation_record;

use c172_validation_record::graded;

const DT_S: f64 = 1.0 / 120.0;
const FPS_PER_KT: f64 = 1.687_809_857_1;
const POUNDS_PER_KG: f64 = 2.204_622_621_8;
const PAYLOAD_LOADED_LBS: f64 = 500.0;
const MTOW_LBS: f64 = 2_550.0;
const PAYLOAD_MTOW_LBS: f64 = PAYLOAD_LOADED_LBS + (MTOW_LBS - 2_448.746_8);

struct SteadyFlightSample {
    speed_kcas: f64,
    gamma_deg: f64,
    climb_fpm: f64,
    glide_ratio: f64,
    rpm: f64,
}

fn new_c172_fdm(package: &LoadedSyspack) -> Result<Fdm, Box<dyn std::error::Error>> {
    let files = package.build_jsbsim_resolver()?;
    let (aircraft_dir, model_name) = package.jsbsim_model()?;
    let mut fdm = Fdm::with_options(LoadOptions::new().with_strict(true))?;
    fdm.set_dt(DT_S);
    fdm.load_model(&files, aircraft_dir, model_name)?;
    Ok(fdm)
}

fn seed_flight_ic(
    fdm: &mut Fdm,
    target_kcas: f64,
    throttle: f64,
    flaps_norm: f64,
    payload_lbs: f64,
) -> Result<(), Box<dyn std::error::Error>> {
    let start_h_ft = if throttle > 0.5 { 150.0 } else { 1_500.0 };
    let est_cl = (MTOW_LBS / (0.589_27 * target_kcas * target_kcas)).clamp(0.15, 1.45);
    let est_alpha = (est_cl / 6.35) - 0.04;
    let est_theta = est_alpha + if throttle > 0.5 { 0.09 } else { -0.09 };
    fdm.set_property("inertia/pointmass-weight-lbs[0]", payload_lbs)?;
    fdm.set_property("inertia/pointmass-weight-lbs[1]", 0.0)?;
    fdm.set_property("propulsion/tank[0]/contents-lbs", 125.663_4)?;
    fdm.set_property("propulsion/tank[1]/contents-lbs", 125.663_4)?;
    for (prop, val) in [
        ("ic/h-sl-ft", start_h_ft),
        ("ic/vt-fps", target_kcas * FPS_PER_KT),
        ("ic/alpha-deg", est_alpha.to_degrees()),
        ("ic/theta-deg", est_theta.to_degrees()),
        ("ic/beta-deg", 0.0),
        ("fcs/flap-cmd-norm", flaps_norm),
        ("fcs/flap-pos-norm", flaps_norm),
        ("fcs/throttle-cmd-norm", throttle),
        ("fcs/mixture-cmd-norm", 1.0),
        ("fcs/mixture-pos-norm", 1.0),
        ("propulsion/magneto_cmd", 3.0),
        ("propulsion/set-running", -1.0),
    ] {
        fdm.set_property(prop, val)?;
    }
    fdm.run_ic()?;
    Ok(())
}

/// Flies a genuine 6-DoF closed-loop airspeed hold (`30 s` at `120 Hz`) so the
/// aircraft settles cleanly before the `[24 s, 30 s]` measurement window.
fn fly_steady_airspeed(
    fdm: &mut Fdm,
    target_kcas: f64,
    throttle: f64,
    flaps_norm: f64,
    payload_lbs: f64,
) -> Result<SteadyFlightSample, Box<dyn std::error::Error>> {
    seed_flight_ic(fdm, target_kcas, throttle, flaps_norm, payload_lbs)?;
    let base_theta = if throttle > 0.5 { 0.14 } else { -0.06 };
    let mut prev_vc = fdm.get_property("velocities/vc-kts")?;
    let mut int_err = 0.0_f64;
    let mut h_start = 0.0_f64;
    let (mut vc_sum, mut vt_sum, mut rpm_sum) = (0.0_f64, 0.0_f64, 0.0_f64);
    let mut samples = 0_u32;

    for step in 0..3_600_u32 {
        let vc = fdm.get_property("velocities/vc-kts")?;
        let vdot = (vc - prev_vc) / DT_S;
        prev_vc = vc;
        let err = vc - target_kcas;
        int_err = (int_err + err * DT_S).clamp(-35.0, 35.0);
        let theta = fdm.get_property("attitude/theta-rad")?;
        let q = fdm.get_property("velocities/q-rad_sec")?;
        let p = fdm.get_property("velocities/p-rad_sec")?;
        let r = fdm.get_property("velocities/r-rad_sec")?;
        let phi = fdm.get_property("attitude/phi-rad")?;
        let beta = fdm.get_property("aero/beta-rad")?;

        let wanted_theta =
            (base_theta + 0.018 * err + 0.012 * int_err + 0.008 * vdot).clamp(-0.25, 0.55);
        let nose_up = (wanted_theta - theta) * 2.5 - 0.55 * q;
        fdm.set_property("fcs/elevator-cmd-norm", (-nose_up).clamp(-1.0, 1.0))?;
        fdm.set_property(
            "fcs/aileron-cmd-norm",
            (-2.0 * phi - 0.5 * p).clamp(-1.0, 1.0),
        )?;
        fdm.set_property(
            "fcs/rudder-cmd-norm",
            (-1.5 * beta + 0.4 * r).clamp(-1.0, 1.0),
        )?;
        fdm.set_property("fcs/throttle-cmd-norm", throttle)?;
        fdm.set_property("fcs/flap-pos-norm", flaps_norm)?;
        fdm.run()?;

        if step == 2_880 {
            h_start = fdm.get_property("position/h-sl-ft")?;
        } else if step > 2_880 {
            vc_sum += fdm.get_property("velocities/vc-kts")?;
            vt_sum += fdm.get_property("velocities/vt-fps")?;
            rpm_sum += fdm.get_property("propulsion/engine[0]/propeller-rpm")?;
            samples = samples.wrapping_add(1);
        }
    }
    let win_s = f64::from(samples) * DT_S;
    let h_end = fdm.get_property("position/h-sl-ft")?;
    let h_dot_fps = (h_end - h_start) / win_s;
    let mean_vt_fps = vt_sum / f64::from(samples);
    let gamma_rad = (h_dot_fps / mean_vt_fps).clamp(-0.8, 0.8).asin();
    let glide_ratio = if gamma_rad < -1e-4 {
        (-gamma_rad).cos() / (-gamma_rad).sin()
    } else {
        0.0
    };
    Ok(SteadyFlightSample {
        speed_kcas: vc_sum / f64::from(samples),
        gamma_deg: gamma_rad.to_degrees(),
        climb_fpm: h_dot_fps * 60.0,
        glide_ratio,
        rpm: rpm_sum / f64::from(samples),
    })
}

/// Measures 1g power-off stall speed (`KCAS`) and peak aerodynamic lift at `(ref_kt, flaps_norm)`.
fn measure_stall_and_peak_lift(
    fdm: &mut Fdm,
    ref_speed_kt: f64,
    flaps_norm: f64,
    payload_lbs: f64,
) -> Result<(f64, f64, f64), Box<dyn std::error::Error>> {
    let mut max_lift_lbs = 0.0_f64;
    let mut weight_lbs = MTOW_LBS;
    fdm.set_property("inertia/pointmass-weight-lbs[0]", payload_lbs)?;
    fdm.set_property("inertia/pointmass-weight-lbs[1]", 0.0)?;
    fdm.set_property("propulsion/tank[0]/contents-lbs", 125.663_4)?;
    fdm.set_property("propulsion/tank[1]/contents-lbs", 125.663_4)?;
    for step in 22..=33_u32 {
        let alpha_rad = f64::from(step) * 0.01;
        fdm.set_property("ic/h-sl-ft", 100.0)?;
        fdm.set_property("ic/vt-fps", ref_speed_kt * FPS_PER_KT)?;
        fdm.set_property("ic/alpha-rad", alpha_rad)?;
        fdm.set_property("ic/theta-rad", alpha_rad)?;
        fdm.set_property("ic/beta-rad", 0.0)?;
        fdm.set_property("fcs/flap-cmd-norm", flaps_norm)?;
        fdm.set_property("fcs/flap-pos-norm", flaps_norm)?;
        fdm.set_property("fcs/throttle-cmd-norm", 0.0)?;
        fdm.run_ic()?;
        let lift_lbs = fdm.get_property("forces/fwz-aero-lbs")?;
        if lift_lbs > max_lift_lbs {
            max_lift_lbs = lift_lbs;
        }
        weight_lbs = fdm.get_property("inertia/weight-lbs")?;
    }
    let stall_kcas = (ref_speed_kt * ref_speed_kt * (weight_lbs / max_lift_lbs)).sqrt();
    Ok((stall_kcas, max_lift_lbs, weight_lbs))
}

/// Measures static full-throttle brake-hold RPM and sea-level full-throttle `Vmax` via `JsbsimFdmBlock`.
fn measure_static_rpm_and_sl_vmax(
    package: &LoadedSyspack,
) -> Result<(f64, f64, f64), Box<dyn std::error::Error>> {
    let files = package.build_jsbsim_resolver()?;
    let (aircraft_path, model) = package.jsbsim_model()?;
    let mut parked = JsbsimFdmBlock::new(&JsbsimFdmConfig {
        resolver: &files,
        aircraft_path,
        model,
        period: JSBSIM_DEFAULT_PERIOD,
        initial: Geodetic::from_degrees(39.1, -75.6, 1.30),
        heading_rad: 0.0,
        speed_m_s: 0.0,
        alpha_rad: 0.0,
        engine_start: EngineStartMode::Running,
    })?;
    let mut sys = parked.system_controls();
    sys.wheel_brake_norm = 1.0;
    parked.set_system_controls(sys);
    for _ in 0..600_u32 {
        parked.step(&KinematicInput {
            controls: ControlInputs {
                throttle: 1.0,
                ..ControlInputs::default()
            },
        })?;
    }
    let static_rpm = parked.engine_status().propeller_rpm;

    let mut fdm = new_c172_fdm(package)?;
    seed_flight_ic(&mut fdm, 124.0, 0.90, 0.0, PAYLOAD_LOADED_LBS)?;
    fdm.set_property("ic/h-sl-ft", 50.0)?;
    fdm.run_ic()?;
    let mut throttle = 0.90_f64;
    for _ in 0..3_600_u32 {
        let h_ft = fdm.get_property("position/h-sl-ft")?;
        let h_dot = fdm.get_property("velocities/h-dot-fps")?;
        let theta = fdm.get_property("attitude/theta-rad")?;
        let q = fdm.get_property("velocities/q-rad_sec")?;
        let phi = fdm.get_property("attitude/phi-rad")?;
        let p = fdm.get_property("velocities/p-rad_sec")?;
        let rpm = fdm.get_property("propulsion/engine[0]/propeller-rpm")?;
        throttle = (throttle + 0.002 * (2_700.0 - rpm) * DT_S).clamp(0.4, 1.0);
        let prop_throttled = (throttle + 0.0004 * (2_700.0 - rpm)).clamp(0.4, 1.0);
        let target_theta = ((50.0 - h_ft) * 0.002 - h_dot * 0.015).clamp(-0.08, 0.08);
        let nose_up = (target_theta - theta) * 2.5 - 0.5 * q;
        fdm.set_property("fcs/elevator-cmd-norm", (-nose_up).clamp(-1.0, 1.0))?;
        fdm.set_property(
            "fcs/aileron-cmd-norm",
            (-2.0 * phi - 0.5 * p).clamp(-1.0, 1.0),
        )?;
        fdm.set_property("fcs/throttle-cmd-norm", prop_throttled)?;
        fdm.run()?;
    }
    let vmax_kcas = fdm.get_property("velocities/vc-kts")?;
    let vmax_rpm = fdm.get_property("propulsion/engine[0]/propeller-rpm")?;
    Ok((static_rpm, vmax_kcas, vmax_rpm))
}

#[test]
fn c172_package_ships_a_verified_airspeed_envelope() -> Result<(), Box<dyn std::error::Error>> {
    let c172 = builtin_c172_package()?;
    let env = c172.airspeed_envelope().ok_or("missing C172 envelope")?;
    assert_eq!(env.type_designator, "C172S");
    assert_eq!((env.vs0_kt, env.vs1_kt, env.vfe_kt), (40.0, 48.0, 85.0));
    assert_eq!((env.vno_kt, env.vne_kt), (129.0, 163.0));
    assert_eq!(
        (env.vr_kt, env.vx_kt, env.vy_kt),
        (Some(55.0), Some(62.0), Some(74.0))
    );
    assert_eq!(
        (env.vg_kt, env.va_ref_kt, env.vref_kt),
        (Some(68.0), Some(105.0), Some(61.0))
    );

    let va_mtow = env
        .va_at_mass_kt(2_550.0 / POUNDS_PER_KG)
        .ok_or("missing Va")?;
    let va_loaded = c172
        .resolved_airspeed_envelope()
        .and_then(|r| r.va_kt)
        .ok_or("missing resolved Va")?;
    let va_2200 = env
        .va_at_mass_kt(2_200.0 / POUNDS_PER_KG)
        .ok_or("missing Va")?;
    let va_1900 = env
        .va_at_mass_kt(1_900.0 / POUNDS_PER_KG)
        .ok_or("missing Va")?;
    assert!((va_mtow - 105.0).abs() < 0.2, "va_mtow={va_mtow}");
    assert!((va_loaded - 102.9).abs() < 0.3, "va_loaded={va_loaded}");
    assert!((va_2200 - 97.5).abs() < 0.8, "va_2200={va_2200}");
    assert!((va_1900 - 90.6).abs() < 0.8, "va_1900={va_1900}");
    Ok(())
}

#[test]
fn tampered_or_unordered_airspeed_envelope_fails_closed() -> Result<(), Box<dyn std::error::Error>>
{
    let source = builtin_c172_source();
    let c172 = builtin_c172_package()?;
    let manifest = source.read_manifest_bytes()?;
    let mut mem_assets: BTreeMap<String, Vec<u8>> = BTreeMap::new();
    for key in c172.manifest().assets.keys() {
        mem_assets.insert(key.clone(), source.read_asset_bytes(key)?);
    }
    let mut env_bytes = source.read_asset_bytes(AIRSPEED_ENVELOPE_ASSET)?;
    if let Some(first) = env_bytes.iter_mut().find(|b| **b == b'4') {
        *first = b'5';
    }
    mem_assets.insert(AIRSPEED_ENVELOPE_ASSET.to_owned(), env_bytes);
    let tampered = MemoryPackageSource::new(manifest, mem_assets);
    assert!(matches!(
        LoadedSyspack::load(&tampered),
        Err(PackError::Syspack(SyspackError::AssetDigestMismatch { path, .. }))
            if path == AIRSPEED_ENVELOPE_ASSET
    ));

    let mut bad: AirspeedEnvelope =
        serde_json::from_slice(&source.read_asset_bytes(AIRSPEED_ENVELOPE_ASSET)?)?;
    bad.vx_kt = Some(80.0);
    bad.vy_kt = Some(74.0);
    assert!(matches!(
        bad.validate(),
        Err(PackError::InvalidAirspeedEnvelope { .. })
    ));
    Ok(())
}

#[test]
fn empirical_c172_stalls_and_pec_match_poh() -> Result<(), Box<dyn std::error::Error>> {
    let package = builtin_c172_package()?;
    let mut fdm = new_c172_fdm(&package)?;
    let (vs1_loaded, _, _) = measure_stall_and_peak_lift(&mut fdm, 55.0, 0.0, PAYLOAD_LOADED_LBS)?;
    let (vs0_loaded, _, _) = measure_stall_and_peak_lift(&mut fdm, 55.0, 1.0, PAYLOAD_LOADED_LBS)?;
    let (vs1_mtow, _, _) = measure_stall_and_peak_lift(&mut fdm, 55.0, 0.0, PAYLOAD_MTOW_LBS)?;
    let (vs0_mtow, _, _) = measure_stall_and_peak_lift(&mut fdm, 55.0, 1.0, PAYLOAD_MTOW_LBS)?;
    let pec = package
        .pitot_static_calibration()
        .ok_or("missing C172 PEC")?;
    let vs0_mtow_kias = pec.kcas_to_kias(1.0, vs0_mtow);
    let vs1_mtow_kias = pec.kcas_to_kias(0.0, vs1_mtow);

    assert!(
        (46.0..=48.2).contains(&vs0_loaded),
        "vs0_loaded={vs0_loaded}"
    );
    assert!(
        (51.0..=53.0).contains(&vs1_loaded),
        "vs1_loaded={vs1_loaded}"
    );
    assert!((47.2..=48.8).contains(&vs0_mtow), "vs0_mtow={vs0_mtow}");
    assert!((52.2..=53.8).contains(&vs1_mtow), "vs1_mtow={vs1_mtow}");
    assert!(
        (39.4..=40.8).contains(&vs0_mtow_kias),
        "vs0_kias={vs0_mtow_kias}"
    );
    assert!(
        (47.4..=48.8).contains(&vs1_mtow_kias),
        "vs1_kias={vs1_mtow_kias}"
    );
    Ok(())
}

#[test]
fn empirical_c172_vx_and_vy_climb_match_poh() -> Result<(), Box<dyn std::error::Error>> {
    let package = builtin_c172_package()?;
    let pec = package
        .pitot_static_calibration()
        .ok_or("missing C172 PEC")?;
    let mut fdm = new_c172_fdm(&package)?;
    let mut climb_loaded = Vec::new();
    let mut climb_mtow = Vec::new();
    for kt in 56..=86_u32 {
        let target = f64::from(kt);
        climb_loaded.push(fly_steady_airspeed(
            &mut fdm,
            target,
            1.0,
            0.0,
            PAYLOAD_LOADED_LBS,
        )?);
        climb_mtow.push(fly_steady_airspeed(
            &mut fdm,
            target,
            1.0,
            0.0,
            PAYLOAD_MTOW_LBS,
        )?);
    }
    let best_vx_loaded = climb_loaded
        .iter()
        .max_by(|a, b| a.gamma_deg.total_cmp(&b.gamma_deg))
        .ok_or("empty climb sweep")?;
    let best_vx_mtow = climb_mtow
        .iter()
        .max_by(|a, b| a.gamma_deg.total_cmp(&b.gamma_deg))
        .ok_or("empty MTOW climb sweep")?;
    let best_vy_loaded = climb_loaded
        .iter()
        .max_by(|a, b| a.climb_fpm.total_cmp(&b.climb_fpm))
        .ok_or("empty climb sweep")?;
    let best_vy_mtow = climb_mtow
        .iter()
        .max_by(|a, b| a.climb_fpm.total_cmp(&b.climb_fpm))
        .ok_or("empty MTOW climb sweep")?;

    let vx_loaded_kias = pec.kcas_to_kias(0.0, best_vx_loaded.speed_kcas);
    let vx_mtow_kias = pec.kcas_to_kias(0.0, best_vx_mtow.speed_kcas);
    graded(
        "envelope Vx loaded",
        (59.5..=64.5).contains(&vx_loaded_kias)
            && best_vx_loaded.gamma_deg > best_vx_mtow.gamma_deg,
        &format!("{vx_loaded_kias:.1} KIAS"),
    );
    graded(
        "envelope Vx MTOW",
        (59.5..=64.5).contains(&vx_mtow_kias) && best_vx_mtow.rpm > 2_350.0,
        &format!("{vx_mtow_kias:.1} KIAS at {:.0} rpm", best_vx_mtow.rpm),
    );
    graded(
        "envelope Vy MTOW",
        (71.0..=76.5).contains(&best_vy_mtow.speed_kcas)
            && best_vy_loaded.climb_fpm > best_vy_mtow.climb_fpm,
        &format!("{:.1} KCAS", best_vy_mtow.speed_kcas),
    );
    graded(
        "envelope climb rate MTOW",
        (690.0..=780.0).contains(&best_vy_mtow.climb_fpm),
        &format!("{:.0} fpm", best_vy_mtow.climb_fpm),
    );
    Ok(())
}

#[test]
fn empirical_c172_glide_maneuvering_and_vmax_match_poh() -> Result<(), Box<dyn std::error::Error>> {
    let package = builtin_c172_package()?;
    let env = package.airspeed_envelope().ok_or("missing C172 envelope")?;
    let mut fdm = new_c172_fdm(&package)?;

    let mut glides = Vec::new();
    for kt in 58..=82_u32 {
        glides.push(fly_steady_airspeed(
            &mut fdm,
            f64::from(kt),
            0.0,
            0.0,
            PAYLOAD_MTOW_LBS,
        )?);
    }
    let best_vg = glides
        .iter()
        .max_by(|a, b| a.glide_ratio.total_cmp(&b.glide_ratio))
        .ok_or("empty glide sweep")?;

    let (vs1_loaded, _, _) = measure_stall_and_peak_lift(&mut fdm, 55.0, 0.0, PAYLOAD_LOADED_LBS)?;
    let (vs1_mtow, _, _) = measure_stall_and_peak_lift(&mut fdm, 55.0, 0.0, PAYLOAD_MTOW_LBS)?;
    let va_loaded_kcas = vs1_loaded * env.positive_limit_g.sqrt();
    let va_mtow_kcas = vs1_mtow * env.positive_limit_g.sqrt();
    let (_, peak_lift_va, weight_va) =
        measure_stall_and_peak_lift(&mut fdm, va_mtow_kcas, 0.0, PAYLOAD_MTOW_LBS)?;
    let load_factor_at_va = peak_lift_va / weight_va;

    let (static_rpm, vmax_sl_kcas, vmax_rpm) = measure_static_rpm_and_sl_vmax(&package)?;

    graded(
        "envelope Vg",
        (65.0..=71.0).contains(&best_vg.speed_kcas) && best_vg.rpm > 600.0,
        &format!("{:.1} KCAS", best_vg.speed_kcas),
    );
    assert!(
        (8.5..=10.3).contains(&best_vg.glide_ratio),
        "L/D={}",
        best_vg.glide_ratio
    );
    assert!(
        (97.0..=102.5).contains(&va_loaded_kcas) && va_loaded_kcas < va_mtow_kcas,
        "Va_loaded={va_loaded_kcas}"
    );
    assert!(
        (99.0..=104.5).contains(&va_mtow_kcas),
        "Va_mtow={va_mtow_kcas}"
    );
    assert!(
        (3.75..=3.85).contains(&load_factor_at_va),
        "nz={load_factor_at_va}"
    );
    graded(
        "envelope static rpm",
        (2_280.0..=2_440.0).contains(&static_rpm),
        &format!("{static_rpm:.0} rpm"),
    );
    assert!(
        (121.0..129.0).contains(&vmax_sl_kcas) && vmax_rpm > 2_600.0,
        "vmax_sl_kcas={vmax_sl_kcas}"
    );
    Ok(())
}
