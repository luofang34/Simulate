//! Combined off-CG thrust spin-up + simultaneous slew, joint stop-limit
//! impacts on spinning buses, and rigid/fixed-joint baseline invariance.

use glam::DVec3;
use sim_core::{TICKS_PER_SECOND, TickSpan, VehicleIntegrator};
use sim_orbital::NaifId;
use sim_universe::{LocalPlant, Tier};

use super::models::{CraftSpec, ReferenceWorld, TEST_VESSEL, at_s, motor_input};

struct BurnState {
    pos_icrf: DVec3,
    vel_icrf: DVec3,
    attitude: glam::DQuat,
    omega_icrf_body: DVec3,
    l_icrf: DVec3,
    gimbal_rad: f64,
    gimbal_rate_rad_s: f64,
    xenon_kg: f64,
}

fn capture_burn_state(world: &mut ReferenceWorld) -> BurnState {
    let tdb = world.epoch.tdb_at(world.universe.now());
    let rot = world.orientation.body_rotation(NaifId::EARTH, tdb).unwrap();
    let vstate = world.universe.vehicle_state(TEST_VESSEL).unwrap();
    let (pos_icrf, vel_icrf, omega_icrf_body) = match world.universe.tier(TEST_VESSEL).unwrap() {
        Tier::Orbital => {
            let orb = world.universe.inertial_state(TEST_VESSEL).unwrap();
            (
                orb.pos_rel_icrf_m,
                orb.vel_rel_icrf_m_s,
                orb.omega_body_rad_s,
            )
        }
        Tier::Local => {
            let fixed_to_icrf = rot.r_icrf_to_fixed.transpose();
            let pos = fixed_to_icrf * vstate.position_ecef;
            let vel = fixed_to_icrf
                * (vstate.velocity_ecef + rot.omega_body_rad_s.cross(vstate.position_ecef));
            let w = vstate.angular_rate_body + vstate.attitude.conjugate() * rot.omega_body_rad_s;
            (pos, vel, w)
        }
    };
    let l_icrf = world.angular_momentum_icrf();
    let (gimbal_rad, gimbal_rate_rad_s) = world.joint_state();
    let xenon_kg = world.xenon_kg();
    BurnState {
        pos_icrf,
        vel_icrf,
        attitude: vstate.attitude,
        omega_icrf_body,
        l_icrf,
        gimbal_rad,
        gimbal_rate_rad_s,
        xenon_kg,
    }
}

fn run_thrust_local(spec: &CraftSpec, end_s: u64) -> (BurnState, BurnState) {
    let mut world = ReferenceWorld::spawn(spec);
    let step_s = spec.block_period.as_seconds() as u64;
    let first_cmd = motor_input(world.ship().input(), 1.0, 2.0e-4);
    world.ship_mut().set_input(first_cmd);
    world.universe.advance_to(at_s(step_s)).unwrap();
    let initial = capture_burn_state(&mut world);
    for t in (step_s..step_s + end_s).step_by(step_s as usize) {
        let torque = if t < step_s + 600 { 2.0e-4 } else { -1.0e-4 };
        let cmd = motor_input(world.ship().input(), 1.0, torque);
        world.ship_mut().set_input(cmd);
        world.universe.advance_to(at_s(t + step_s)).unwrap();
    }
    let ended = capture_burn_state(&mut world);
    (initial, ended)
}

fn run_thrust_carried(spec: &CraftSpec, end_s: u64, exchange_s: u64) -> (BurnState, BurnState) {
    let mut world = ReferenceWorld::spawn(spec);
    let step_s = spec.block_period.as_seconds() as u64;
    let first_cmd = motor_input(world.ship().input(), 1.0, 2.0e-4);
    world.ship_mut().set_input(first_cmd);
    world.universe.advance_to(at_s(step_s)).unwrap();
    world.universe.promote_to_orbital(TEST_VESSEL).unwrap();
    let initial = capture_burn_state(&mut world);
    let exchange = TickSpan::from_ticks(exchange_s * TICKS_PER_SECOND);
    for t in (step_s..step_s + end_s).step_by(exchange_s as usize) {
        let torque = if t < step_s + 600 { 2.0e-4 } else { -1.0e-4 };
        let cmd = motor_input(world.ship().input(), 1.0, torque);
        let ship = world.ship_mut();
        ship.set_carry_period(exchange);
        ship.set_input(cmd);
        world.universe.advance_to(at_s(t + exchange_s)).unwrap();
    }
    let ended = capture_burn_state(&mut world);
    (initial, ended)
}

#[test]
fn off_cg_thrust_spinup_with_simultaneous_joint_slew_matches_local() {
    let initial_rate = DVec3::new(0.04, 0.15, -0.03);
    let mut local_spec =
        CraftSpec::asymmetric_coupled(initial_rate, VehicleIntegrator::adaptive(1e-12, 1e-12));
    local_spec.thruster_offset_m = Some(DVec3::new(0.0, 0.18, 0.25));
    local_spec.xenon_kg = 20.0;
    let (l_init, l_end) = run_thrust_local(&local_spec, 1_200);
    let spin_gain = (l_end.omega_icrf_body - l_init.omega_icrf_body).length();
    let local_dl = l_end.l_icrf - l_init.l_icrf;
    assert!(
        spin_gain > 0.05,
        "off-CG thruster spun bus by {spin_gain} rad/s"
    );
    assert!(l_end.xenon_kg < l_init.xenon_kg, "propellant depleted");
    assert!(
        l_end.gimbal_rad.abs() > 0.3,
        "gimbal slewed {} rad",
        l_end.gimbal_rad
    );

    let mut carried_spec = local_spec.clone();
    carried_spec.integrator = VehicleIntegrator::FixedRk4 { substeps: 6 };
    let mut att_errs = Vec::new();
    for exchange_s in [300_u64, 60, 4] {
        let (c_init, c_end) = run_thrust_carried(&carried_spec, 1_200, exchange_s);
        let pos_err_m = (c_end.pos_icrf - l_end.pos_icrf).length();
        let vel_err_m_s = (c_end.vel_icrf - l_end.vel_icrf).length();
        let att_err = c_end.attitude.angle_between(l_end.attitude);
        let w_err = (c_end.omega_icrf_body - l_end.omega_icrf_body).length();
        let q_err = (c_end.gimbal_rad - l_end.gimbal_rad).abs();
        let qdot_err = (c_end.gimbal_rate_rad_s - l_end.gimbal_rate_rad_s).abs();
        let carried_dl = c_end.l_icrf - c_init.l_icrf;
        let dl_rel_err = (carried_dl - local_dl).length() / local_dl.length();
        let prop_err = (c_end.xenon_kg - l_end.xenon_kg).abs();
        tracing::info!(
            exchange_s,
            spin_gain,
            pos_err_m,
            vel_err_m_s,
            att_err,
            w_err,
            q_err,
            qdot_err,
            dl_rel_err,
            prop_err,
            "off-CG thrust spin-up + slew"
        );
        att_errs.push(att_err);
        if exchange_s == 4 {
            assert!(
                att_err < 0.2 * att_errs[0] && att_err < 0.2 * att_errs[1],
                "off-CG att failed to converge: {:?}",
                att_errs
            );
            assert!(pos_err_m < 1.0, "pos_err_m {pos_err_m}");
            assert!(vel_err_m_s < 1e-3, "vel_err_m_s {vel_err_m_s}");
            assert!(att_err < 2e-3, "att_err {att_err}");
            assert!(w_err < 5e-4, "w_err {w_err}");
            assert!(q_err < 5e-3, "q_err {q_err}");
            assert!(qdot_err < 5e-4, "qdot_err {qdot_err}");
            assert!(dl_rel_err < 2e-4, "dl_rel_err {dl_rel_err}");
            assert!(prop_err < 1e-4, "prop_err {prop_err}");
        }
    }
}

fn run_stop_carried(
    spec: &CraftSpec,
    drive_nm: f64,
    exchange_s: u64,
) -> (f64, f64, glam::DQuat, f64) {
    let mut carried = ReferenceWorld::spawn(spec);
    carried.universe.promote_to_orbital(TEST_VESSEL).unwrap();
    let l0 = carried.angular_momentum_icrf();
    let mut max_drift = 0.0_f64;
    let exchange = TickSpan::from_ticks(exchange_s * TICKS_PER_SECOND);
    for t in (0..240_u64).step_by(exchange_s as usize) {
        let torque = if t < 160 { drive_nm } else { -0.4 * drive_nm };
        let cmd = motor_input(carried.ship().input(), 0.0, torque);
        let ship = carried.ship_mut();
        ship.set_carry_period(exchange);
        ship.set_input(cmd);
        carried.universe.advance_to(at_s(t + exchange_s)).unwrap();
        let l = carried.angular_momentum_icrf();
        max_drift = max_drift.max((l - l0).length() / l0.length());
    }
    let (q, qdot) = carried.joint_state();
    let att = carried
        .universe
        .vehicle_state(TEST_VESSEL)
        .unwrap()
        .attitude;
    (q, qdot, att, max_drift)
}

fn check_stop_impact(spin_rate: DVec3, drive_nm: f64, max_q_err: f64, max_att_err: f64) {
    let mut local_spec =
        CraftSpec::asymmetric_coupled(spin_rate, VehicleIntegrator::adaptive(1e-11, 1e-11));
    local_spec.min_rad = -0.35;
    local_spec.max_rad = 0.35;
    let mut local = ReferenceWorld::spawn(&local_spec);
    let step_s = local_spec.block_period.as_seconds() as u64;
    for t in (0..240_u64).step_by(step_s as usize) {
        let torque = if t < 160 { drive_nm } else { -0.4 * drive_nm };
        let cmd = motor_input(local.ship().input(), 0.0, torque);
        local.ship_mut().set_input(cmd);
        local.universe.advance_to(at_s(t + step_s)).unwrap();
    }
    let (local_q, local_qdot) = local.joint_state();
    let local_att = local.universe.vehicle_state(TEST_VESSEL).unwrap().attitude;

    let mut carried_spec = local_spec;
    carried_spec.integrator = VehicleIntegrator::FixedRk4 { substeps: 6 };
    let mut q_errs = Vec::new();
    let mut att_errs = Vec::new();
    for exchange_s in [40_u64, 12, 2] {
        let (q, qdot, att, max_drift) = run_stop_carried(&carried_spec, drive_nm, exchange_s);
        let att_err = att.angle_between(local_att);
        let q_err = (q - local_q).abs();
        tracing::info!(
            spin = spin_rate.length(),
            exchange_s,
            max_drift,
            local_q,
            q,
            local_qdot,
            qdot,
            q_err,
            att_err,
            "joint stop-limit impact while spinning"
        );
        assert!(q.is_finite() && qdot.is_finite());
        assert!(
            q.abs() <= 0.36 && qdot.abs() < 0.5,
            "unbounded q={q} qdot={qdot}"
        );
        assert!(max_drift < 1e-11, "stop-impact drift {max_drift:e}");
        q_errs.push(q_err);
        att_errs.push(att_err);
        if exchange_s == 2 {
            assert!(q_err < max_q_err, "q {q} vs local {local_q}");
            assert!(att_err < max_att_err, "att_err {att_err}");
            assert!(
                att_err < 0.4 * att_errs[1] && att_err < 0.15 * att_errs[0],
                "stop att_err failed to converge: {:?}",
                att_errs
            );
            assert!(
                q_err <= q_errs[0].min(q_errs[1]) || q_err < 1e-5,
                "stop q_err failed to converge: {:?}",
                q_errs
            );
        }
    }
}

#[test]
fn joint_stop_limit_impact_while_spinning_stays_conservative_and_stable() {
    check_stop_impact(DVec3::new(0.05, 0.35, -0.04), 2.5e-3, 1e-3, 5e-2);
    check_stop_impact(DVec3::new(0.08, 0.60, -0.06), 3.5e-3, 6e-2, 1.2e-1);
}

#[test]
fn rigid_fixed_joint_vehicle_unaffected_under_spin_and_thrust() {
    let rate = DVec3::new(0.05, 0.60, -0.04);
    let mut local_spec =
        CraftSpec::asymmetric_coupled(rate, VehicleIntegrator::adaptive(1e-12, 1e-12));
    local_spec.joint_axis = None;
    local_spec.thruster_offset_m = Some(DVec3::new(0.0, 0.15, -0.20));
    local_spec.xenon_kg = 20.0;
    let (_, l_end) = run_thrust_local(&local_spec, 600);

    let mut carried_spec = local_spec;
    carried_spec.integrator = VehicleIntegrator::FixedRk4 { substeps: 6 };
    let mut world = ReferenceWorld::spawn(&carried_spec);
    let step_s = carried_spec.block_period.as_seconds() as u64;
    let cmd = motor_input(world.ship().input(), 1.0, 0.0);
    world.ship_mut().set_input(cmd);
    world.universe.advance_to(at_s(step_s)).unwrap();
    world.universe.promote_to_orbital(TEST_VESSEL).unwrap();
    assert!(!world.ship().has_moving_joints());
    let (i_orb, h_held) = world
        .universe
        .orbital_held_rotational_state_body(TEST_VESSEL)
        .unwrap();
    assert_eq!(h_held, DVec3::ZERO);
    assert_eq!(
        i_orb,
        world.ship().block().mass_properties().inertia_cg_body
    );
    let exchange = TickSpan::from_ticks(60 * TICKS_PER_SECOND);
    for t in (step_s..step_s + 600_u64).step_by(60) {
        let cmd = motor_input(world.ship().input(), 1.0, 0.0);
        let ship = world.ship_mut();
        ship.set_carry_period(exchange);
        ship.set_input(cmd);
        world.universe.advance_to(at_s(t + 60)).unwrap();
    }
    let c_end = capture_burn_state(&mut world);
    let att_err = c_end.attitude.angle_between(l_end.attitude);
    let w_err = (c_end.omega_icrf_body - l_end.omega_icrf_body).length();
    tracing::info!(att_err, w_err, "fixed-joint spinning thruster baseline");
    assert!(att_err < 1e-2 && w_err < 2e-3);
}
