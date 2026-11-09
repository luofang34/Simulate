//! Closed-loop 6-DoF JSBSim level-cruise, empirical-segment, and shared FDM
//! flight-control primitives.

use jsbsim::Fdm;

use crate::PackError;
use crate::calibration::engines::{
    GovernorHold, ManifoldPressureHold, has_governor, set_each_engine,
};
use crate::calibration::mixture::{MixtureProcedure, set_mixture};
use crate::calibration::results::{CruiseResult, EmpiricalSegmentResult};
use crate::calibration::schema::{
    CalibrationTolerances, CruiseBenchmark, EmpiricalFlightSegment, EmpiricalSegmentMode,
};
use crate::calibration::solver::{
    DT_S, FPS_PER_KT, POUNDS_PER_KG, ParameterOverrides, fly_steady_climb_kcas,
};

pub(super) fn solve_cruise(
    fdm: &mut Fdm,
    bench: &CruiseBenchmark,
    rated_bhp: Option<f64>,
    tol: &CalibrationTolerances,
    base_non_payload_lbs: f64,
    overrides: ParameterOverrides,
) -> Result<CruiseResult, PackError> {
    let (sim_ktas, sim_rpm, sim_hp, sim_gph) = fly_level_cruise(
        fdm,
        bench.pressure_alt_ft,
        bench.gross_mass_kg,
        bench.target_rpm,
        bench.fixed_throttle,
        bench.target_ktas,
        base_non_payload_lbs,
        overrides,
        bench.target_map_inhg,
    )?;
    let residual_ktas = sim_ktas - bench.target_ktas;
    let sim_pct_bhp = match (sim_hp, rated_bhp) {
        (Some(hp), Some(max_hp)) if max_hp > 0.0 => Some(100.0 * hp / max_hp),
        _ => None,
    };
    let residual_gph = match (sim_gph, bench.target_gph) {
        (Some(s), Some(t)) => Some(s - t),
        _ => None,
    };
    let bhp_ok = match (sim_pct_bhp, bench.target_pct_bhp) {
        (Some(s), Some(t)) => (s - t).abs() <= tol.cruise_pct_bhp,
        _ => true,
    };
    let gph_ok = residual_gph.is_none_or(|r| r.abs() <= tol.cruise_gph);
    Ok(CruiseResult {
        pressure_alt_ft: bench.pressure_alt_ft,
        commanded_rpm: bench.target_rpm.unwrap_or(0.0),
        sim_rpm,
        sim_ktas,
        target_ktas: bench.target_ktas,
        residual_ktas,
        sim_pct_bhp,
        target_pct_bhp: bench.target_pct_bhp,
        sim_gph,
        target_gph: bench.target_gph,
        residual_gph,
        passed: residual_ktas.abs() <= tol.cruise_ktas && gph_ok && bhp_ok,
    })
}

#[derive(Debug, Clone, Copy)]
pub(super) struct BaseEngineTuning {
    pub ve: Option<f64>,
    pub bsfc: Option<f64>,
    pub fric: Option<f64>,
}

#[allow(clippy::too_many_arguments)]
pub(super) fn fly_level_cruise(
    fdm: &mut Fdm,
    pressure_alt_ft: f64,
    gross_mass_kg: f64,
    target_rpm: Option<f64>,
    fixed_throttle: Option<f64>,
    seed_ktas: f64,
    base_non_payload_lbs: f64,
    overrides: ParameterOverrides,
    map_hold_inhg: Option<f64>,
) -> Result<(f64, f64, Option<f64>, Option<f64>), PackError> {
    let base = capture_base_engine_tuning(fdm)?;
    configure_mass_and_overrides(fdm, gross_mass_kg, base_non_payload_lbs, overrides)?;
    let init_thr = fixed_throttle.unwrap_or(0.78).clamp(0.05, 1.0);
    for (prop, val) in [
        ("ic/h-sl-ft", pressure_alt_ft),
        ("ic/terrain-elevation-ft", -1_000.0),
        ("ic/vt-fps", seed_ktas * FPS_PER_KT),
        ("ic/alpha-deg", 1.8),
        ("ic/theta-deg", 1.8),
        ("ic/beta-deg", 0.0),
    ] {
        fdm.set_property(prop, val).map_err(fdm_err)?;
    }
    set_each_engine(fdm, "fcs/throttle-cmd-norm", init_thr)?;
    set_each_engine(fdm, "fcs/advance-cmd-norm", 1.0)?;
    set_if_exists(fdm, "fcs/flap-cmd-norm", 0.0)?;
    set_if_exists(fdm, "fcs/flap-pos-norm", 0.0)?;
    set_if_exists(fdm, "gear/gear-cmd-norm", 0.0)?;
    set_if_exists(fdm, "gear/gear-pos-norm", 0.0)?;
    set_each_engine(fdm, "fcs/mixture-cmd-norm", 1.0)?;
    set_each_engine(fdm, "fcs/mixture-pos-norm", 1.0)?;
    set_if_exists(fdm, "propulsion/magneto_cmd", 3.0)?;
    set_if_exists(fdm, "propulsion/set-running", -1.0)?;
    fdm.run_ic().map_err(fdm_err)?;
    apply_engine_overrides(fdm, base, overrides)?;
    // POH cruise tables assume the recommended lean mixture, 50°F rich of peak EGT.
    let mix = set_mixture(fdm, MixtureProcedure::RichOfPeakEgt(50.0), init_thr)?;

    let power = CruisePower {
        target_rpm,
        fixed_throttle,
        map_hold_inhg,
        governed: has_governor(fdm)?,
    };
    let out = integrate_cruise_hold(fdm, pressure_alt_ft, power, mix)?;
    restore_engine_tuning(fdm, base)?;
    Ok(out)
}

/// How a cruise point sets power: a fixed-pitch propeller reaches `target_rpm` with
/// the throttle; a governed one holds it with the propeller lever while the throttle
/// holds `map_hold_inhg` (or stays at `fixed_throttle`).
#[derive(Debug, Clone, Copy)]
struct CruisePower {
    target_rpm: Option<f64>,
    fixed_throttle: Option<f64>,
    map_hold_inhg: Option<f64>,
    governed: bool,
}

fn integrate_cruise_hold(
    fdm: &mut Fdm,
    pressure_alt_ft: f64,
    power: CruisePower,
    mixture: f64,
) -> Result<(f64, f64, Option<f64>, Option<f64>), PackError> {
    let CruisePower {
        target_rpm,
        fixed_throttle,
        map_hold_inhg,
        governed,
    } = power;
    let mut map_hold = ManifoldPressureHold::new(fixed_throttle.unwrap_or(0.75));
    let mut governor = GovernorHold::new();
    let has_hp = fdm
        .node("propulsion/engine[0]/power-hp")
        .map_err(fdm_err)?
        .is_some();
    let has_gph = fdm
        .node("propulsion/engine[0]/fuel-flow-rate-gph")
        .map_err(fdm_err)?
        .is_some();
    let mut thr_int = fixed_throttle.unwrap_or(0.75);
    let (mut vt_sum, mut rpm_sum, mut hp_sum, mut gph_sum) = (0.0, 0.0, 0.0, 0.0);
    let mut samples = 0_u32;
    for step in 0..3_600_u32 {
        let h_ft = fdm.get_property("position/h-sl-ft").map_err(fdm_err)?;
        let h_dot = fdm.get_property("velocities/h-dot-fps").map_err(fdm_err)?;
        let theta = fdm.get_property("attitude/theta-rad").map_err(fdm_err)?;
        let q = fdm.get_property("velocities/q-rad_sec").map_err(fdm_err)?;
        let wanted_theta = ((pressure_alt_ft - h_ft) * 0.0025 - h_dot * 0.018).clamp(-0.12, 0.22);
        let nose_up = (wanted_theta - theta) * 2.6 - 0.55 * q;
        let rpm = get_opt(fdm, "propulsion/engine[0]/propeller-rpm")?.unwrap_or(0.0);
        if governed {
            let lever = match target_rpm {
                Some(tgt) => governor.step(fdm, tgt)?,
                None => 1.0,
            };
            set_each_engine(fdm, "fcs/advance-cmd-norm", lever)?;
        }
        let thr = match (map_hold_inhg, target_rpm) {
            (Some(map), _) => map_hold.step(fdm, map)?,
            (None, Some(tgt)) if !governed => {
                let err = tgt - rpm;
                thr_int = (thr_int + 0.0014 * err * DT_S).clamp(0.05, 1.0);
                (thr_int + 0.0018 * err).clamp(0.05, 1.0)
            }
            _ => fixed_throttle.unwrap_or(0.75).clamp(0.0, 1.0),
        };
        step_wings_level(fdm, nose_up, thr, mixture)?;
        if step >= 2_880 {
            vt_sum += fdm.get_property("velocities/vt-fps").map_err(fdm_err)?;
            rpm_sum += rpm;
            if has_hp {
                hp_sum += fdm
                    .get_property("propulsion/engine[0]/power-hp")
                    .map_err(fdm_err)?;
            }
            if has_gph {
                gph_sum += fdm
                    .get_property("propulsion/engine[0]/fuel-flow-rate-gph")
                    .map_err(fdm_err)?;
            }
            samples = samples.wrapping_add(1);
        }
    }
    let n = f64::from(samples.max(1));
    Ok((
        (vt_sum / n) / FPS_PER_KT,
        rpm_sum / n,
        has_hp.then_some(hp_sum / n),
        has_gph.then_some(gph_sum / n),
    ))
}

pub(super) fn solve_empirical(
    fdm: &mut Fdm,
    seg: &EmpiricalFlightSegment,
    base_non_payload_lbs: f64,
    overrides: ParameterOverrides,
) -> Result<EmpiricalSegmentResult, PackError> {
    let seed_ktas = (seg.observed_gs_kt - seg.wind_along_track_kt).max(45.0);
    let (sim_ktas, sim_roc_fpm) = match seg.mode {
        EmpiricalSegmentMode::LevelCruiseRpm => {
            let (ktas, _, _, _) = fly_level_cruise(
                fdm,
                seg.pressure_alt_ft,
                seg.gross_mass_kg,
                Some(seg.command_setpoint),
                None,
                seed_ktas,
                base_non_payload_lbs,
                overrides,
                None,
            )?;
            (ktas, 0.0)
        }
        EmpiricalSegmentMode::LevelCruiseThrottle => {
            let (ktas, _, _, _) = fly_level_cruise(
                fdm,
                seg.pressure_alt_ft,
                seg.gross_mass_kg,
                None,
                Some(seg.command_setpoint),
                seed_ktas,
                base_non_payload_lbs,
                overrides,
                None,
            )?;
            (ktas, 0.0)
        }
        EmpiricalSegmentMode::FullThrottleClimbKcas => {
            let (_, ktas, fpm, _) = fly_steady_climb_kcas(
                fdm,
                seg.pressure_alt_ft,
                seg.gross_mass_kg,
                seg.command_setpoint,
                true,
                base_non_payload_lbs,
                overrides,
                None,
            )?;
            (ktas, fpm)
        }
    };
    let sim_gs_kt = sim_ktas + seg.wind_along_track_kt;
    let residual_gs_kt = sim_gs_kt - seg.observed_gs_kt;
    let residual_roc_fpm = seg.observed_roc_fpm.map(|obs| sim_roc_fpm - obs);
    let roc_ok = match (residual_roc_fpm, seg.roc_tolerance_fpm) {
        (Some(r), Some(tol)) => r.abs() <= tol,
        _ => true,
    };
    Ok(EmpiricalSegmentResult {
        segment_id: seg.segment_id.clone(),
        mode: seg.mode,
        pressure_alt_ft: seg.pressure_alt_ft,
        sim_gs_kt,
        observed_gs_kt: seg.observed_gs_kt,
        residual_gs_kt,
        sim_roc_fpm,
        observed_roc_fpm: seg.observed_roc_fpm,
        residual_roc_fpm,
        passed: residual_gs_kt.abs() <= seg.gs_tolerance_kt && roc_ok,
    })
}

pub(super) fn configure_mass_and_overrides(
    fdm: &mut Fdm,
    gross_mass_kg: f64,
    base_non_payload_lbs: f64,
    overrides: ParameterOverrides,
) -> Result<(), PackError> {
    let target_lbs = gross_mass_kg * POUNDS_PER_KG + overrides.extra_mass_lbs;
    let payload_lbs = (target_lbs - base_non_payload_lbs).max(0.0);
    set_if_exists(fdm, "inertia/pointmass-weight-lbs[0]", payload_lbs)
}

pub(super) fn capture_base_engine_tuning(fdm: &mut Fdm) -> Result<BaseEngineTuning, PackError> {
    Ok(BaseEngineTuning {
        ve: get_opt(fdm, "propulsion/engine[0]/volumetric-efficiency")?,
        bsfc: get_opt(fdm, "propulsion/engine[0]/bsfc-lbs_hphr")?,
        fric: get_opt(fdm, "propulsion/engine[0]/friction-hp")?,
    })
}

pub(super) fn apply_engine_overrides(
    fdm: &mut Fdm,
    base: BaseEngineTuning,
    overrides: ParameterOverrides,
) -> Result<(), PackError> {
    if let (Some(ve), Some(scale)) = (base.ve, overrides.volumetric_efficiency_scale) {
        set_if_exists(
            fdm,
            "propulsion/engine[0]/volumetric-efficiency",
            ve * scale,
        )?;
    }
    if let (Some(bsfc), Some(scale)) = (base.bsfc, overrides.bsfc_scale) {
        set_if_exists(fdm, "propulsion/engine[0]/bsfc-lbs_hphr", bsfc * scale)?;
    }
    if let (Some(fric), Some(delta)) = (base.fric, overrides.friction_hp_delta) {
        set_if_exists(
            fdm,
            "propulsion/engine[0]/friction-hp",
            (fric + delta).max(0.0),
        )?;
    }
    Ok(())
}

pub(super) fn restore_engine_tuning(
    fdm: &mut Fdm,
    base: BaseEngineTuning,
) -> Result<(), PackError> {
    apply_engine_overrides(
        fdm,
        base,
        ParameterOverrides {
            volumetric_efficiency_scale: Some(1.0),
            bsfc_scale: Some(1.0),
            friction_hp_delta: Some(0.0),
            extra_mass_lbs: 0.0,
        },
    )
}

pub(super) fn step_wings_level(
    fdm: &mut Fdm,
    nose_up_cmd: f64,
    throttle: f64,
    mixture: f64,
) -> Result<(), PackError> {
    let phi = fdm.get_property("attitude/phi-rad").map_err(fdm_err)?;
    let p = fdm.get_property("velocities/p-rad_sec").map_err(fdm_err)?;
    let beta = fdm.get_property("aero/beta-rad").map_err(fdm_err)?;
    let r = fdm.get_property("velocities/r-rad_sec").map_err(fdm_err)?;
    fdm.set_property("fcs/elevator-cmd-norm", (-nose_up_cmd).clamp(-1.0, 1.0))
        .map_err(fdm_err)?;
    fdm.set_property(
        "fcs/aileron-cmd-norm",
        (-2.0 * phi - 0.5 * p).clamp(-1.0, 1.0),
    )
    .map_err(fdm_err)?;
    fdm.set_property(
        "fcs/rudder-cmd-norm",
        (-1.5 * beta + 0.4 * r).clamp(-1.0, 1.0),
    )
    .map_err(fdm_err)?;
    set_each_engine(fdm, "fcs/throttle-cmd-norm", throttle)?;
    set_each_engine(fdm, "fcs/mixture-cmd-norm", mixture)?;
    set_each_engine(fdm, "fcs/mixture-pos-norm", mixture)?;
    let _keep_running = fdm.run().map_err(fdm_err)?;
    Ok(())
}

pub(super) fn set_if_exists(fdm: &mut Fdm, prop: &str, val: f64) -> Result<(), PackError> {
    if fdm.node(prop).map_err(fdm_err)?.is_some() {
        fdm.set_property(prop, val).map_err(fdm_err)?;
    }
    Ok(())
}

pub(super) fn get_opt(fdm: &mut Fdm, prop: &str) -> Result<Option<f64>, PackError> {
    if fdm.node(prop).map_err(fdm_err)?.is_some() {
        Ok(Some(fdm.get_property(prop).map_err(fdm_err)?))
    } else {
        Ok(None)
    }
}

pub(super) fn fdm_err(err: jsbsim::Error) -> PackError {
    PackError::CalibrationRun {
        detail: err.to_string(),
    }
}
