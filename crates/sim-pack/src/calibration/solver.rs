//! Closed-loop 6-DoF JSBSim flight-test solvers for static runup, stalls,
//! and full-throttle steady climbs, dispatching level cruise to `cruise.rs`.

use jsbsim::{Fdm, LoadOptions};

use crate::calibration::cruise::{
    apply_engine_overrides, capture_base_engine_tuning, configure_mass_and_overrides, fdm_err,
    get_opt, restore_engine_tuning, set_if_exists, solve_cruise, solve_empirical, step_wings_level,
};
use crate::calibration::engines::{ManifoldPressureHold, set_each_engine};
use crate::calibration::loading::{cg_target, load_point};
use crate::calibration::mixture::{MixtureProcedure, set_mixture};
use crate::calibration::results::{ClimbResult, RawSuiteEvaluation, StallResult};
use crate::calibration::schema::{
    CalibrationSuite, ClimbBenchmark, StallBenchmark, StaticRunupBenchmark,
};
use crate::calibration::takeoff::solve_takeoff;
use crate::{LoadedSyspack, PackError, PitotStaticCalibration};

pub(super) const DT_S: f64 = 1.0 / 120.0;
pub(super) const FPS_PER_KT: f64 = 1.687_809_857_1;
pub(super) const POUNDS_PER_KG: f64 = 2.204_622_621_8;
pub(super) const ISA_LAPSE_PER_FT: f64 = 6.875_585_6e-6;
/// A climb point flies 60 s and measures the last 15 s: long enough for a heavy
/// aircraft's speed to settle on the hold, so climb rate reflects excess power.
const CLIMB_STEPS: u32 = 7_200;
const CLIMB_WINDOW_START: u32 = 5_400;

/// Live in-memory perturbations applied to JSBSim engine/mass properties during
/// finite-difference Jacobian sensitivity evaluation.
#[derive(Debug, Clone, Copy, Default)]
pub(super) struct ParameterOverrides {
    pub volumetric_efficiency_scale: Option<f64>,
    pub bsfc_scale: Option<f64>,
    pub friction_hp_delta: Option<f64>,
    pub extra_mass_lbs: f64,
}

pub(super) fn run_suite_with_overrides(
    package: &LoadedSyspack,
    suite: &CalibrationSuite,
    overrides: ParameterOverrides,
) -> Result<RawSuiteEvaluation, PackError> {
    let mut fdm = handbook_fdm(package, suite)?;
    let base_non_payload_lbs = probe_non_payload_lbs(&mut fdm)?;
    let pec = package.pitot_static_calibration();

    let static_rpm = match &suite.static_runup {
        Some(runup) => {
            let rpm = solve_static_runup(&mut fdm, runup, overrides)?;
            let ok = rpm >= runup.min_rpm && rpm <= runup.max_rpm;
            Some((rpm, ok))
        }
        None => None,
    };

    let mut stalls = Vec::with_capacity(suite.stalls.len());
    for s in &suite.stalls {
        stalls.push(solve_stall(
            &mut fdm,
            s,
            pec,
            suite.tolerances.stall_kt,
            base_non_payload_lbs,
            overrides,
            suite.fuselage_station_origin_in,
        )?);
    }
    let powered = fly_powered_points(package, suite, base_non_payload_lbs, overrides)?;
    Ok(RawSuiteEvaluation {
        static_rpm,
        stalls,
        ..powered
    })
}

/// The model as the handbook's airplane was equipped.
fn handbook_fdm(package: &LoadedSyspack, suite: &CalibrationSuite) -> Result<Fdm, PackError> {
    let mut fdm = build_fdm(package)?;
    for setting in &suite.handbook_configuration {
        fdm.set_property(&setting.property, setting.value)
            .map_err(fdm_err)?;
    }
    Ok(fdm)
}

/// Climbs, cruises, takeoffs and recorded segments. Each flies a freshly loaded
/// model so no engine, governor or fuel state carries from one flight into the
/// next, as on separate test sorties.
fn fly_powered_points(
    package: &LoadedSyspack,
    suite: &CalibrationSuite,
    base_lbs: f64,
    overrides: ParameterOverrides,
) -> Result<RawSuiteEvaluation, PackError> {
    let tol = &suite.tolerances;
    let fresh = || handbook_fdm(package, suite);
    let climbs = (suite.climb_points.iter())
        .map(|c| solve_climb(&mut fresh()?, c, tol.climb_fpm, base_lbs, overrides))
        .collect::<Result<Vec<_>, _>>()?;
    let cruises = (suite.cruise_points.iter())
        .map(|cr| solve_cruise(&mut fresh()?, cr, suite.rated_bhp, tol, base_lbs, overrides))
        .collect::<Result<Vec<_>, _>>()?;
    let takeoffs = (suite.takeoff_points.iter())
        .map(|t| {
            let cg = cg_target(&t.label, t.cg_fs_in, suite.fuselage_station_origin_in)?;
            solve_takeoff(&mut fresh()?, t, tol.takeoff_pct, base_lbs, overrides, cg)
        })
        .collect::<Result<Vec<_>, _>>()?;
    // Recorded segments fly this tail as it is, not the handbook's airplane.
    let empirical = (suite.empirical_segments.iter())
        .map(|seg| solve_empirical(&mut build_fdm(package)?, seg, base_lbs, overrides))
        .collect::<Result<Vec<_>, _>>()?;
    Ok(RawSuiteEvaluation {
        static_rpm: None,
        stalls: Vec::new(),
        climbs,
        cruises,
        takeoffs,
        empirical,
    })
}

pub(super) fn build_fdm(package: &LoadedSyspack) -> Result<Fdm, PackError> {
    let files = package.build_jsbsim_resolver()?;
    let (aircraft_dir, model_name) = package.jsbsim_model()?;
    let mut fdm = Fdm::with_options(LoadOptions::new().with_strict(true)).map_err(fdm_err)?;
    fdm.set_dt(DT_S);
    fdm.load_model(&files, aircraft_dir, model_name)
        .map_err(fdm_err)?;
    Ok(fdm)
}

pub(super) fn probe_non_payload_lbs(fdm: &mut Fdm) -> Result<f64, PackError> {
    fdm.set_property("ic/h-sl-ft", 100.0).map_err(fdm_err)?;
    fdm.set_property("ic/vt-fps", 100.0).map_err(fdm_err)?;
    fdm.run_ic().map_err(fdm_err)?;
    let total_lbs = fdm.get_property("inertia/weight-lbs").map_err(fdm_err)?;
    let pm_lbs = get_opt(fdm, "inertia/pointmass-weight-lbs[0]")?.unwrap_or(0.0);
    Ok((total_lbs - pm_lbs).max(1.0))
}

fn solve_static_runup(
    fdm: &mut Fdm,
    runup: &StaticRunupBenchmark,
    overrides: ParameterOverrides,
) -> Result<f64, PackError> {
    let base = capture_base_engine_tuning(fdm)?;
    // The POH static run-up is a full-rich, full-throttle check.
    let mix = 1.0;
    for (prop, val) in [
        ("ic/h-sl-ft", runup.pressure_alt_ft + 4.265),
        ("ic/terrain-elevation-ft", runup.pressure_alt_ft),
        ("ic/vt-fps", 0.0),
        ("ic/alpha-deg", 0.0),
        ("ic/theta-deg", 0.0),
    ] {
        fdm.set_property(prop, val).map_err(fdm_err)?;
    }
    set_if_exists(fdm, "fcs/left-brake-cmd-norm", 1.0)?;
    set_if_exists(fdm, "fcs/right-brake-cmd-norm", 1.0)?;
    set_if_exists(fdm, "fcs/mixture-cmd-norm", mix)?;
    set_if_exists(fdm, "fcs/mixture-pos-norm", mix)?;
    set_if_exists(fdm, "propulsion/magneto_cmd", 3.0)?;
    set_if_exists(fdm, "propulsion/set-running", -1.0)?;
    fdm.set_property("fcs/throttle-cmd-norm", 1.0)
        .map_err(fdm_err)?;
    fdm.run_ic().map_err(fdm_err)?;
    apply_engine_overrides(fdm, base, overrides)?;
    for _ in 0..480_u32 {
        set_if_exists(fdm, "fcs/left-brake-cmd-norm", 1.0)?;
        set_if_exists(fdm, "fcs/right-brake-cmd-norm", 1.0)?;
        fdm.set_property("fcs/throttle-cmd-norm", 1.0)
            .map_err(fdm_err)?;
        if !fdm.run().map_err(fdm_err)? {
            break;
        }
    }
    set_if_exists(fdm, "fcs/left-brake-cmd-norm", 0.0)?;
    set_if_exists(fdm, "fcs/right-brake-cmd-norm", 0.0)?;
    let rpm = get_opt(fdm, "propulsion/engine[0]/propeller-rpm")?.unwrap_or(0.0);
    restore_engine_tuning(fdm, base)?;
    Ok(rpm)
}

fn solve_stall(
    fdm: &mut Fdm,
    bench: &StallBenchmark,
    pec: Option<&PitotStaticCalibration>,
    tol_kt: f64,
    base_non_payload_lbs: f64,
    overrides: ParameterOverrides,
    origin_in: Option<f64>,
) -> Result<StallResult, PackError> {
    let cg = cg_target(&bench.label, bench.cg_fs_in, origin_in)?;
    let flown_cg_fs_in = load_point(
        fdm,
        bench.gross_mass_kg,
        base_non_payload_lbs,
        overrides,
        cg,
    )?;
    let ref_kt = 55.0_f64;
    let mut max_lift_lbs = 0.0_f64;
    let mut weight_lbs = bench.gross_mass_kg * POUNDS_PER_KG;
    for step in 18..=36_u32 {
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
        set_if_exists(fdm, "fcs/flap-cmd-norm", bench.flap_norm)?;
        set_if_exists(fdm, "fcs/flap-pos-norm", bench.flap_norm)?;
        fdm.run_ic().map_err(fdm_err)?;
        let lift = fdm.get_property("forces/fwz-aero-lbs").map_err(fdm_err)?;
        if lift > max_lift_lbs {
            max_lift_lbs = lift;
        }
        weight_lbs = fdm.get_property("inertia/weight-lbs").map_err(fdm_err)?;
    }
    let load_factor = 1.0 / bench.bank_deg.to_radians().cos().max(0.2);
    let sim_kcas = (ref_kt * ref_kt * (weight_lbs * load_factor / max_lift_lbs.max(1.0))).sqrt();
    let residual_kcas = sim_kcas - bench.target_kcas;
    let sim_kias = pec.map(|p| p.kcas_to_kias(bench.flap_norm, sim_kcas));
    let residual_kias = match (sim_kias, bench.target_kias) {
        (Some(sim), Some(tgt)) => Some(sim - tgt),
        _ => None,
    };
    let kias_ok = residual_kias.is_none_or(|r| r.abs() <= tol_kt);
    Ok(StallResult {
        label: bench.label.clone(),
        flap_deg: bench.flap_deg,
        bank_deg: bench.bank_deg,
        sim_kcas,
        target_kcas: bench.target_kcas,
        residual_kcas,
        sim_kias,
        target_kias: bench.target_kias,
        residual_kias,
        flown_cg_fs_in,
        passed: residual_kcas.abs() <= tol_kt && kias_ok,
    })
}

fn solve_climb(
    fdm: &mut Fdm,
    bench: &ClimbBenchmark,
    tol_fpm: f64,
    base_non_payload_lbs: f64,
    overrides: ParameterOverrides,
) -> Result<ClimbResult, PackError> {
    let (sim_kcas, _sim_ktas, sim_fpm, sim_rpm) = fly_steady_climb_kcas(
        fdm,
        bench.pressure_alt_ft,
        bench.gross_mass_kg,
        bench.climb_kcas,
        bench.lean_best_power,
        base_non_payload_lbs,
        overrides,
        bench.target_map_inhg,
    )?;
    let residual_fpm = sim_fpm - bench.target_fpm;
    Ok(ClimbResult {
        pressure_alt_ft: bench.pressure_alt_ft,
        sim_kcas,
        sim_rpm,
        sim_fpm,
        target_fpm: bench.target_fpm,
        residual_fpm,
        passed: residual_fpm.abs() <= tol_fpm,
    })
}

#[allow(clippy::too_many_arguments)]
pub(super) fn fly_steady_climb_kcas(
    fdm: &mut Fdm,
    pressure_alt_ft: f64,
    gross_mass_kg: f64,
    climb_kcas: f64,
    lean_best_power: bool,
    base_non_payload_lbs: f64,
    overrides: ParameterOverrides,
    map_hold_inhg: Option<f64>,
) -> Result<(f64, f64, f64, f64), PackError> {
    let base = capture_base_engine_tuning(fdm)?;
    configure_mass_and_overrides(fdm, gross_mass_kg, base_non_payload_lbs, overrides)?;
    let sigma = (1.0 - ISA_LAPSE_PER_FT * pressure_alt_ft)
        .max(0.2)
        .powf(4.255_879_7);
    let init_ktas = climb_kcas / sigma.sqrt();
    for (prop, val) in [
        ("ic/h-sl-ft", (pressure_alt_ft - 120.0).max(20.0)),
        ("ic/terrain-elevation-ft", -2_000.0),
        ("ic/vt-fps", init_ktas * FPS_PER_KT),
        ("ic/alpha-deg", 5.2),
        ("ic/theta-deg", 10.2),
        ("ic/beta-deg", 0.0),
    ] {
        fdm.set_property(prop, val).map_err(fdm_err)?;
    }
    set_each_engine(fdm, "fcs/throttle-cmd-norm", 1.0)?;
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
    let procedure = MixtureProcedure::climb(pressure_alt_ft, lean_best_power);
    let mix = set_mixture(fdm, procedure, 1.0)?;

    let out = integrate_climb_hold(fdm, climb_kcas, mix, map_hold_inhg)?;
    restore_engine_tuning(fdm, base)?;
    Ok(out)
}

fn integrate_climb_hold(
    fdm: &mut Fdm,
    climb_kcas: f64,
    mixture: f64,
    map_hold_inhg: Option<f64>,
) -> Result<(f64, f64, f64, f64), PackError> {
    let mut map_hold = ManifoldPressureHold::new(1.0);
    let mut prev_vc = fdm.get_property("velocities/vc-kts").map_err(fdm_err)?;
    let mut int_err = 0.0_f64;
    let mut h_start = 0.0_f64;
    let (mut vc_sum, mut vt_sum, mut rpm_sum) = (0.0, 0.0, 0.0);
    let mut samples = 0_u32;
    for step in 0..CLIMB_STEPS {
        let vc = fdm.get_property("velocities/vc-kts").map_err(fdm_err)?;
        let vdot = (vc - prev_vc) / DT_S;
        prev_vc = vc;
        let err = vc - climb_kcas;
        int_err = (int_err + err * DT_S).clamp(-250.0, 250.0);
        let theta = fdm.get_property("attitude/theta-rad").map_err(fdm_err)?;
        let q = fdm.get_property("velocities/q-rad_sec").map_err(fdm_err)?;
        // Loose speed loop over a well-damped attitude loop: tighter gains set up a
        // speed/attitude oscillation in heavy aircraft at low air density whose drag
        // eats the climb being measured.
        let wanted_theta = (0.14 + 0.006 * err + 0.002 * int_err + 0.004 * vdot).clamp(-0.15, 0.55);
        let nose_up = (wanted_theta - theta) * 2.0 - 1.2 * q;
        set_if_exists(fdm, "fcs/flap-pos-norm", 0.0)?;
        let throttle = match map_hold_inhg {
            Some(target) => map_hold.step(fdm, target)?,
            None => 1.0,
        };
        step_wings_level(fdm, nose_up, throttle, mixture)?;
        if step == CLIMB_WINDOW_START {
            h_start = energy_height_ft(fdm)?;
        }
        if step > CLIMB_WINDOW_START {
            vc_sum += fdm.get_property("velocities/vc-kts").map_err(fdm_err)?;
            vt_sum += fdm.get_property("velocities/vt-fps").map_err(fdm_err)?;
            rpm_sum += get_opt(fdm, "propulsion/engine[0]/propeller-rpm")?.unwrap_or(0.0);
            samples = samples.wrapping_add(1);
        }
    }
    let h_end = energy_height_ft(fdm)?;
    let n = f64::from(samples.max(1));
    let fpm = ((h_end - h_start) / (n * DT_S)) * 60.0;
    Ok((vc_sum / n, (vt_sum / n) / FPS_PER_KT, fpm, rpm_sum / n))
}

/// Energy height `h + V²/2g` (ft). Its rate is the steady climb rate the excess
/// power supports, so a speed-for-height exchange still decaying in the measuring
/// window (a heavy aircraft's long phugoid) does not bias the result.
fn energy_height_ft(fdm: &mut Fdm) -> Result<f64, PackError> {
    const G_FPS2: f64 = 32.174_049;
    let h = fdm.get_property("position/h-sl-ft").map_err(fdm_err)?;
    let v = fdm.get_property("velocities/vt-fps").map_err(fdm_err)?;
    Ok(h + v * v / (2.0 * G_FPS2))
}
