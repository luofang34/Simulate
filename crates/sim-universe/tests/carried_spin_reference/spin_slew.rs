//! Spin-regime conservation and convergence reference tests:
//! `0.15`, `0.50`, `1.0 rad/s` on asymmetric non-diagonal spacecraft, plus
//! exact drift profiling at `2e-4`, `0.05`, `0.50`, `1.0 rad/s`.

use glam::{DQuat, DVec3};
use sim_core::VehicleIntegrator;
use sim_universe::{LocalPlant, Tier};

use super::models::{CraftSpec, ReferenceWorld, TEST_VESSEL, at_s, motor_input};

struct Outcome {
    attitude: DQuat,
    omega_body_rad_s: DVec3,
    gimbal_rad: f64,
    gimbal_rate_rad_s: f64,
    max_vec_drift: f64,
    max_mag_drift: f64,
    max_mbd_mismatch: f64,
}

fn fly_local(spec: &CraftSpec, end_s: u64, drive_fn: impl Fn(u64) -> f64) -> Outcome {
    let mut world = ReferenceWorld::spawn(spec);
    let l0 = world.local_ellipsoid_momentum_icrf();
    let step_s = spec.block_period.as_seconds() as u64;
    let mut max_vec_drift = 0.0_f64;
    let mut max_mag_drift = 0.0_f64;
    for t in (0..end_s).step_by(step_s as usize) {
        let cmd = motor_input(world.ship().input(), 0.0, drive_fn(t));
        world.ship_mut().set_input(cmd);
        world.universe.advance_to(at_s(t + step_s)).unwrap();
        let l = world.local_ellipsoid_momentum_icrf();
        max_vec_drift = max_vec_drift.max((l - l0).length() / l0.length());
        max_mag_drift = max_mag_drift.max((l.length() - l0.length()).abs() / l0.length());
    }
    let orb_state = world.universe.vehicle_state(TEST_VESSEL).unwrap();
    let tdb = world.epoch.tdb_at(world.universe.now());
    let rot = world
        .orientation
        .body_rotation(sim_orbital::NaifId::EARTH, tdb)
        .unwrap();
    let omega_icrf =
        orb_state.angular_rate_body + orb_state.attitude.conjugate() * rot.omega_body_rad_s;
    let (gimbal_rad, gimbal_rate_rad_s) = world.joint_state();
    Outcome {
        attitude: orb_state.attitude,
        omega_body_rad_s: omega_icrf,
        gimbal_rad,
        gimbal_rate_rad_s,
        max_vec_drift,
        max_mag_drift,
        max_mbd_mismatch: 0.0,
    }
}

fn fly_carried(
    spec: &CraftSpec,
    end_s: u64,
    exchange_s: u64,
    drive_fn: impl Fn(u64) -> f64,
) -> Outcome {
    let mut world = ReferenceWorld::spawn(spec);
    world.universe.promote_to_orbital(TEST_VESSEL).unwrap();
    let l0 = world.angular_momentum_icrf();
    let mut max_vec_drift = 0.0_f64;
    let mut max_mag_drift = 0.0_f64;
    let mut max_mbd_mismatch = 0.0_f64;
    let exchange_span = sim_core::TickSpan::from_ticks(exchange_s * sim_core::TICKS_PER_SECOND);
    for t in (0..end_s).step_by(exchange_s as usize) {
        let cmd = motor_input(world.ship().input(), 0.0, drive_fn(t));
        let ship = world.ship_mut();
        ship.set_carry_period(exchange_span);
        ship.set_input(cmd);
        world.universe.advance_to(at_s(t + exchange_s)).unwrap();
        assert_eq!(world.universe.tier(TEST_VESSEL).unwrap(), Tier::Orbital);
        assert_eq!(world.ship().now(), at_s(t + exchange_s));
        let l = world.angular_momentum_icrf();
        let l_mbd = world.plant_multibody_momentum_icrf();
        max_vec_drift = max_vec_drift.max((l - l0).length() / l0.length());
        max_mag_drift = max_mag_drift.max((l.length() - l0.length()).abs() / l0.length());
        max_mbd_mismatch = max_mbd_mismatch.max((l - l_mbd).length() / l0.length());
    }
    let orb = world.universe.inertial_state(TEST_VESSEL).unwrap();
    let vstate = world.universe.vehicle_state(TEST_VESSEL).unwrap();
    let (gimbal_rad, gimbal_rate_rad_s) = world.joint_state();
    Outcome {
        attitude: vstate.attitude,
        omega_body_rad_s: orb.omega_body_rad_s,
        gimbal_rad,
        gimbal_rate_rad_s,
        max_vec_drift,
        max_mag_drift,
        max_mbd_mismatch,
    }
}

fn classic_torque(t: u64) -> f64 {
    match t {
        0..1_200 => 2.0e-6,
        1_200..2_400 => -2.0e-6,
        _ => 0.0,
    }
}

fn coupled_torque(end_s: u64) -> impl Fn(u64) -> f64 {
    move |t| {
        if t < end_s / 2 { 2.5e-3 } else { -2.5e-3 }
    }
}

#[test]
fn classic_gimbal_momentum_drift_across_spin_regimes() {
    let cases = [
        (
            "tumble_2e-4",
            DVec3::new(2.0e-4, -1.0e-4, 1.5e-4),
            &[1_200_u64, 600, 300][..],
        ),
        (
            "slow_0.05",
            DVec3::new(0.004, 0.050, 0.0),
            &[1_200, 600, 300],
        ),
        (
            "fast_0.50",
            DVec3::new(0.020, 0.500, 0.0),
            &[1_200, 600, 300],
        ),
        ("rapid_1.0", DVec3::new(0.040, 1.000, -0.030), &[600, 300]),
    ];
    for (label, rate, periods) in cases {
        let spec = CraftSpec::classic_gimbal(rate, VehicleIntegrator::FixedRk4 { substeps: 6 });
        for &exchange_s in periods {
            let res = fly_carried(&spec, 3_600, exchange_s, classic_torque);
            tracing::info!(
                case = label,
                exchange_s,
                vec_drift = res.max_vec_drift,
                mag_drift = res.max_mag_drift,
                mbd_mismatch = res.max_mbd_mismatch,
                gimbal_rad = res.gimbal_rad,
                "classic gimbal drift measurement"
            );
            assert!(
                res.max_vec_drift < 1e-11 && res.max_mag_drift < 1e-11,
                "{label} exchange={exchange_s}: drift vec={:e}, mag={:e}",
                res.max_vec_drift,
                res.max_mag_drift
            );
            assert!(
                res.max_mbd_mismatch < 1e-12,
                "{label} exchange={exchange_s}: orbital vs multibody mismatch {:e}",
                res.max_mbd_mismatch
            );
        }
    }
}

fn assert_refined_errors(
    errs: &[(f64, f64, f64, f64)],
    max_att: f64,
    max_omega: f64,
    max_q: f64,
    max_qdot: f64,
) {
    let finest = errs[2];
    for (idx, limit) in [(0, max_att), (1, max_omega), (2, max_q), (3, max_qdot)] {
        let pick = |tuple: (f64, f64, f64, f64)| match idx {
            0 => tuple.0,
            1 => tuple.1,
            2 => tuple.2,
            _ => tuple.3,
        };
        let f = pick(finest);
        assert!(
            f < 0.25 * pick(errs[0]) && f < 0.25 * pick(errs[1]),
            "metric {idx} failed to converge: {:?} -> {:?} -> {:?}",
            pick(errs[0]),
            pick(errs[1]),
            f
        );
        assert!(f < limit, "metric {idx} finest {f} >= {limit}");
    }
}

fn check_asymmetric_regime(
    spin_mag: f64,
    end_s: u64,
    periods: [u64; 3],
    max_att: f64,
    max_omega: f64,
    max_q: f64,
    max_qdot: f64,
) {
    let rate = DVec3::new(0.25, 0.90, -0.35).normalize() * spin_mag;
    let local_spec = CraftSpec::asymmetric_coupled(rate, VehicleIntegrator::adaptive(1e-13, 1e-13));
    let torque_fn = coupled_torque(end_s);
    let local = fly_local(&local_spec, end_s, &torque_fn);
    assert!(
        local.gimbal_rad.abs() > 0.3,
        "joint slewed {} rad",
        local.gimbal_rad
    );
    assert!(
        local.max_vec_drift < 5e-11,
        "local drift {:e}",
        local.max_vec_drift
    );
    let carried_spec =
        CraftSpec::asymmetric_coupled(rate, VehicleIntegrator::FixedRk4 { substeps: 6 });
    let mut errs = Vec::new();
    for exchange_s in periods {
        let carried = fly_carried(&carried_spec, end_s, exchange_s, &torque_fn);
        let att_err = carried.attitude.angle_between(local.attitude);
        let w_err = (carried.omega_body_rad_s - local.omega_body_rad_s).length();
        let q_err = (carried.gimbal_rad - local.gimbal_rad).abs();
        let qdot_err = (carried.gimbal_rate_rad_s - local.gimbal_rate_rad_s).abs();
        tracing::info!(
            spin_mag,
            end_s,
            exchange_s,
            vec_drift = carried.max_vec_drift,
            mag_drift = carried.max_mag_drift,
            att_err,
            w_err,
            q_err,
            qdot_err,
            "asymmetric coupled slew"
        );
        assert!(
            carried.max_vec_drift < 1e-11 && carried.max_mag_drift < 1e-11,
            "spin={spin_mag} exchange={exchange_s}: momentum drift vec={:e}, mag={:e}",
            carried.max_vec_drift,
            carried.max_mag_drift
        );
        assert!(carried.max_mbd_mismatch < 1e-12);
        errs.push((att_err, w_err, q_err, qdot_err));
    }
    assert_refined_errors(&errs, max_att, max_omega, max_q, max_qdot);
}

#[test]
fn asymmetric_spinning_slew_conserves_and_converges_015() {
    check_asymmetric_regime(0.15, 600, [600, 150, 10], 1e-3, 1e-4, 2e-3, 5e-5);
}

#[test]
fn asymmetric_spinning_slew_conserves_and_converges_050() {
    check_asymmetric_regime(0.50, 180, [60, 20, 2], 1e-3, 5e-4, 2e-3, 2e-4);
}

#[test]
fn asymmetric_spinning_slew_conserves_and_converges_100() {
    check_asymmetric_regime(1.00, 120, [30, 10, 2], 2e-3, 5e-4, 2e-3, 2e-4);
}

#[test]
fn heavy_appendage_two_pass_ramp_converges_under_spin() {
    let rate = DVec3::new(0.12, 0.50, -0.09);
    let local_spec = CraftSpec::heavy_appendage(rate, VehicleIntegrator::adaptive(1e-12, 1e-12));
    let local = fly_local(&local_spec, 600, |t| if t < 300 { 2.0e-3 } else { -2.0e-3 });
    let carried_spec =
        CraftSpec::heavy_appendage(rate, VehicleIntegrator::FixedRk4 { substeps: 6 });
    let mut prev_att = f64::INFINITY;
    for exchange_s in [120_u64, 60, 30, 10] {
        let carried = fly_carried(&carried_spec, 600, exchange_s, |t| {
            if t < 300 { 2.0e-3 } else { -2.0e-3 }
        });
        let att_err = carried.attitude.angle_between(local.attitude);
        let w_err = (carried.omega_body_rad_s - local.omega_body_rad_s).length();
        let q_err = (carried.gimbal_rad - local.gimbal_rad).abs();
        let qdot_err = (carried.gimbal_rate_rad_s - local.gimbal_rate_rad_s).abs();
        tracing::info!(
            exchange_s,
            vec_drift = carried.max_vec_drift,
            att_err,
            w_err,
            q_err,
            qdot_err,
            "heavy appendage coupled slew"
        );
        assert!(carried.max_vec_drift < 1e-11 && carried.max_mag_drift < 1e-11);
        assert!(att_err < prev_att, "heavy att {prev_att} -> {att_err}");
        prev_att = att_err;
    }
}
