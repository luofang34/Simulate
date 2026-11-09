//! POH takeoff-distance flight test: brakes held at full power, released, rotated
//! at the lift-off speed and climbed at the obstacle speed until 50 ft above the
//! runway. Ground roll and total distance price what no airborne point does:
//! static and low-advance-ratio thrust, rolling friction and the flap's lift.

use jsbsim::Fdm;

use crate::PackError;
use crate::calibration::cruise::{
    apply_engine_overrides, capture_base_engine_tuning, configure_mass_and_overrides, fdm_err,
    get_opt, restore_engine_tuning, set_if_exists,
};
use crate::calibration::engines::set_each_engine;
use crate::calibration::mixture::{MixtureProcedure, set_mixture};
use crate::calibration::results::TakeoffResult;
use crate::calibration::schema::TakeoffBenchmark;
use crate::calibration::solver::{DT_S, ParameterOverrides};

/// Brakes-on settling time before release, long enough for the gear to come to
/// rest and the engine to reach full-throttle RPM.
const SETTLE_STEPS: u32 = 1_200;
/// Longest run the solver flies before reporting the obstacle never cleared.
const MAX_RUN_STEPS: u32 = 12_000;
/// The obstacle height of the POH distance tables.
const OBSTACLE_FT: f64 = 50.0;
/// Fraction of the lift-off speed at which the pilot starts lowering the tail.
const ROTATE_FRACTION: f64 = 0.9;
/// Pitch attitude held on the roll before rotation: the short-field technique's
/// "Elevator Control -- SLIGHTLY TAIL LOW" from brake release (C172S POH
/// Section 4), taken as about 3.4 degrees, nose wheel light; the elevator only
/// reaches it once the slipstream and airspeed give it authority.
const TAIL_LOW_ATTITUDE_RAD: f64 = 0.06;
/// Pitch attitude held for lift-off: about 9 degrees, the lift coefficient a
/// short-field lift-off near 1.15 stall speeds needs, short of a tail strike.
const LIFT_OFF_ATTITUDE_RAD: f64 = 0.16;
const G_FPS2: f64 = 32.174_049;
/// The most gear units probed for the at-rest height.
const MAX_GEAR_UNITS: usize = 16;

/// A flown take-off from brake release to the obstacle.
#[derive(Debug, Clone, Copy)]
pub(super) struct TakeoffRun {
    /// Distance rolled before the wheels left the runway (`ft`).
    pub ground_roll_ft: f64,
    /// Distance to the obstacle (`ft`).
    pub total_ft: f64,
    /// Calibrated airspeed as the last wheel left the runway (`kt`).
    pub lift_off_kcas: f64,
    /// Time from brake release to the obstacle (`s`).
    pub elapsed_s: f64,
    /// Energy height above the runway at the obstacle (`ft`).
    pub energy_ft: f64,
}

/// Flies `bench` and grades both distances against `tolerance_pct` of target.
pub(super) fn solve_takeoff(
    fdm: &mut Fdm,
    bench: &TakeoffBenchmark,
    tolerance_pct: f64,
    base_non_payload_lbs: f64,
    overrides: ParameterOverrides,
) -> Result<TakeoffResult, PackError> {
    let run = fly_takeoff_from_brake_release(fdm, bench, base_non_payload_lbs, overrides)?;
    let within = |sim: f64, target: f64| (sim - target).abs() <= target * tolerance_pct / 100.0;
    Ok(TakeoffResult {
        label: bench.label.clone(),
        pressure_alt_ft: bench.pressure_alt_ft,
        sim_ground_roll_ft: run.ground_roll_ft,
        target_ground_roll_ft: bench.target_ground_roll_ft,
        sim_total_ft: run.total_ft,
        target_total_ft: bench.target_total_50ft_ft,
        sim_lift_off_kcas: run.lift_off_kcas,
        target_lift_off_kcas: bench.lift_off_kcas,
        passed: within(run.ground_roll_ft, bench.target_ground_roll_ft)
            && within(run.total_ft, bench.target_total_50ft_ft),
    })
}

/// Holds the brakes at full power, releases them and flies `bench`'s technique to
/// the obstacle.
pub(super) fn fly_takeoff_from_brake_release(
    fdm: &mut Fdm,
    bench: &TakeoffBenchmark,
    base_non_payload_lbs: f64,
    overrides: ParameterOverrides,
) -> Result<TakeoffRun, PackError> {
    let base = capture_base_engine_tuning(fdm)?;
    configure_mass_and_overrides(fdm, bench.gross_mass_kg, base_non_payload_lbs, overrides)?;
    let rest_height_ft = rest_height_ft(fdm)?;
    for (prop, val) in [
        ("ic/terrain-elevation-ft", bench.pressure_alt_ft),
        ("ic/h-sl-ft", bench.pressure_alt_ft + rest_height_ft),
        ("ic/vt-fps", 0.0),
        ("ic/alpha-deg", 0.0),
        ("ic/theta-deg", 0.0),
        ("ic/beta-deg", 0.0),
    ] {
        fdm.set_property(prop, val).map_err(fdm_err)?;
    }
    set_if_exists(fdm, "fcs/flap-cmd-norm", bench.flap_norm)?;
    set_if_exists(fdm, "fcs/flap-pos-norm", bench.flap_norm)?;
    set_each_engine(fdm, "fcs/throttle-cmd-norm", 1.0)?;
    set_each_engine(fdm, "fcs/advance-cmd-norm", 1.0)?;
    set_each_engine(fdm, "fcs/mixture-cmd-norm", 1.0)?;
    set_if_exists(fdm, "propulsion/magneto_cmd", 3.0)?;
    set_if_exists(fdm, "propulsion/set-running", -1.0)?;
    fdm.run_ic().map_err(fdm_err)?;
    apply_engine_overrides(fdm, base, overrides)?;
    let mixture = settle_on_brakes(fdm, bench)?;
    let run = fly_takeoff(fdm, bench, mixture)?;
    restore_engine_tuning(fdm, base)?;
    Ok(run)
}

/// Height of the CG above the runway with the gear just touching, from the
/// lowest wheel's structural position.
fn rest_height_ft(fdm: &mut Fdm) -> Result<f64, PackError> {
    let cg_z_in = get_opt(fdm, "inertia/cg-z-in")?.unwrap_or(0.0);
    let mut lowest_in = f64::INFINITY;
    for n in 0..MAX_GEAR_UNITS {
        match get_opt(fdm, &format!("gear/unit[{n}]/z-position"))? {
            Some(z) => lowest_in = lowest_in.min(z),
            None => break,
        }
    }
    if !lowest_in.is_finite() {
        return Err(PackError::CalibrationRun {
            detail: "takeoff test needs landing gear; the model declares none".to_owned(),
        });
    }
    Ok((cg_z_in - lowest_in) / 12.0)
}

/// Holds the brakes at full throttle until the aircraft is at rest, leaning for
/// maximum RPM where the POH says to (above 3,000 ft).
fn settle_on_brakes(fdm: &mut Fdm, bench: &TakeoffBenchmark) -> Result<f64, PackError> {
    for _ in 0..SETTLE_STEPS {
        brakes(fdm, 1.0)?;
        ground_step(fdm, 1.0, 0.0, 1.0)?;
    }
    let procedure = MixtureProcedure::climb(bench.pressure_alt_ft, false);
    let mixture = set_mixture(fdm, procedure, 1.0)?;
    Ok(mixture)
}

/// Flies from brake release to the obstacle.
fn fly_takeoff(
    fdm: &mut Fdm,
    bench: &TakeoffBenchmark,
    mixture: f64,
) -> Result<TakeoffRun, PackError> {
    let h0 = fdm.get_property("position/h-agl-ft").map_err(fdm_err)?;
    let mut distance_ft = 0.0_f64;
    let mut ground_roll_ft = None;
    let mut lift_off_kcas = 0.0_f64;
    for step in 0..MAX_RUN_STEPS {
        let vc = fdm.get_property("velocities/vc-kts").map_err(fdm_err)?;
        let theta = fdm.get_property("attitude/theta-rad").map_err(fdm_err)?;
        let q = fdm.get_property("velocities/q-rad_sec").map_err(fdm_err)?;
        let airborne = ground_roll_ft.is_some();
        // Short-field technique: slightly tail low on the roll, the tail lowered
        // further from 90% of the lift-off speed so the airplane flies itself off
        // at it, then the obstacle speed held with pitch.
        let wanted_theta = if airborne {
            (0.15 + 0.03 * (vc - bench.obstacle_kcas)).clamp(0.0, 0.30)
        } else if vc >= ROTATE_FRACTION * bench.lift_off_kcas {
            LIFT_OFF_ATTITUDE_RAD
        } else {
            TAIL_LOW_ATTITUDE_RAD
        };
        let nose_up = ((wanted_theta - theta) * 4.0 - 1.5 * q).clamp(-1.0, 1.0);
        brakes(fdm, 0.0)?;
        ground_step(fdm, 1.0, nose_up, mixture)?;
        distance_ft += fdm.get_property("velocities/vg-fps").map_err(fdm_err)? * DT_S;
        if ground_roll_ft.is_none() && !on_ground(fdm)? {
            ground_roll_ft = Some(distance_ft);
            lift_off_kcas = fdm.get_property("velocities/vc-kts").map_err(fdm_err)?;
        }
        let height = fdm.get_property("position/h-agl-ft").map_err(fdm_err)? - h0;
        if let Some(roll) = ground_roll_ft
            && height >= OBSTACLE_FT
        {
            let v_fps = fdm.get_property("velocities/vt-fps").map_err(fdm_err)?;
            return Ok(TakeoffRun {
                ground_roll_ft: roll,
                total_ft: distance_ft,
                lift_off_kcas,
                elapsed_s: f64::from(step.wrapping_add(1)) * DT_S,
                energy_ft: height + v_fps * v_fps / (2.0 * G_FPS2),
            });
        }
    }
    Err(PackError::CalibrationRun {
        detail: format!("{}: never cleared {OBSTACLE_FT} ft", bench.label),
    })
}

fn on_ground(fdm: &mut Fdm) -> Result<bool, PackError> {
    for n in 0..MAX_GEAR_UNITS {
        match get_opt(fdm, &format!("gear/unit[{n}]/WOW"))? {
            Some(wow) if wow > 0.5 => return Ok(true),
            Some(_) => {}
            None => break,
        }
    }
    Ok(false)
}

fn brakes(fdm: &mut Fdm, cmd: f64) -> Result<(), PackError> {
    set_if_exists(fdm, "fcs/left-brake-cmd-norm", cmd)?;
    set_if_exists(fdm, "fcs/right-brake-cmd-norm", cmd)
}

/// One frame with wings level and the nose wheel and rudder holding the runway
/// heading the run started on.
fn ground_step(fdm: &mut Fdm, throttle: f64, nose_up: f64, mixture: f64) -> Result<(), PackError> {
    let phi = fdm.get_property("attitude/phi-rad").map_err(fdm_err)?;
    let p = fdm.get_property("velocities/p-rad_sec").map_err(fdm_err)?;
    let r = fdm.get_property("velocities/r-rad_sec").map_err(fdm_err)?;
    let psi_err = fdm.get_property("attitude/psi-rad").map_err(fdm_err)?;
    let psi_err = psi_err.sin().atan2(psi_err.cos());
    // JSBSim's positive rudder yaws the nose left, its positive steering right.
    let yaw_cmd = (2.0 * psi_err + 0.8 * r).clamp(-1.0, 1.0);
    fdm.set_property("fcs/elevator-cmd-norm", -nose_up)
        .map_err(fdm_err)?;
    fdm.set_property(
        "fcs/aileron-cmd-norm",
        (-2.0 * phi - 0.5 * p).clamp(-1.0, 1.0),
    )
    .map_err(fdm_err)?;
    fdm.set_property("fcs/rudder-cmd-norm", yaw_cmd)
        .map_err(fdm_err)?;
    set_if_exists(fdm, "fcs/steer-cmd-norm", -yaw_cmd)?;
    set_each_engine(fdm, "fcs/throttle-cmd-norm", throttle)?;
    set_each_engine(fdm, "fcs/mixture-cmd-norm", mixture)?;
    set_each_engine(fdm, "fcs/mixture-pos-norm", mixture)?;
    let _keep_running = fdm.run().map_err(fdm_err)?;
    Ok(())
}
