//! Mid-arc carry period changes, checkpoint/restore determinism,
//! speculation-horizon continuation, and mid-arc demotion under active spin.

use glam::DVec3;
use sim_core::{TICKS_PER_SECOND, TickSpan, VehicleIntegrator};
use sim_universe::{LocalPlant, Tier};

use super::models::{CraftSpec, ReferenceWorld, TEST_VESSEL, at_s, motor_input};

fn snapshot_bits(world: &ReferenceWorld) -> ([u64; 13], u64, u64) {
    let orb = world.universe.inertial_state(TEST_VESSEL).unwrap();
    let (q, qdot) = world.joint_state();
    let p = orb.pos_rel_icrf_m;
    let v = orb.vel_rel_icrf_m_s;
    let qt = orb.q_body_to_icrf;
    let w = orb.omega_body_rad_s;
    (
        [
            p.x.to_bits(),
            p.y.to_bits(),
            p.z.to_bits(),
            v.x.to_bits(),
            v.y.to_bits(),
            v.z.to_bits(),
            qt.x.to_bits(),
            qt.y.to_bits(),
            qt.z.to_bits(),
            qt.w.to_bits(),
            w.x.to_bits(),
            w.y.to_bits(),
            w.z.to_bits(),
        ],
        q.to_bits(),
        qdot.to_bits(),
    )
}

fn assert_sliced_parity(
    world: &mut ReferenceWorld,
    snap_mid: &sim_universe::UniverseCheckpoint,
    bits_direct: ([u64; 13], u64, u64),
) {
    let long_period = sim_core::TickSpan::from_ticks(1_200 * TICKS_PER_SECOND);
    world.universe.restore(snap_mid).unwrap();
    world.ship_mut().set_carry_period(long_period);
    world.universe.advance_to(at_s(660)).unwrap();
    let b_direct_660 = snapshot_bits(world);

    world.universe.restore(snap_mid).unwrap();
    world.ship_mut().set_carry_period(long_period);
    world.universe.advance_to(at_s(600)).unwrap();
    world.universe.advance_to(at_s(660)).unwrap();
    let b_sliced_660 = snapshot_bits(world);
    assert_eq!(b_direct_660, b_sliced_660, "[660] vs [600, 660] mismatch");

    for slice in [
        &[600_u64, 1_200][..],
        &[900, 1_200],
        &[500, 1_200],
        &[500, 900, 1_200],
    ] {
        world.universe.restore(snap_mid).unwrap();
        world.ship_mut().set_carry_period(long_period);
        for &t in slice {
            world.universe.advance_to(at_s(t)).unwrap();
        }
        assert_eq!(
            snapshot_bits(world),
            bits_direct,
            "slice {slice:?} mismatch"
        );
    }
}

#[test]
fn mid_arc_carry_period_extension_and_checkpoint_roundtrip_under_spin() {
    let rate = DVec3::new(0.06, 0.65, -0.05);
    let mut spec = CraftSpec::asymmetric_coupled(rate, VehicleIntegrator::FixedRk4 { substeps: 6 });
    spec.carry_period = TickSpan::from_ticks(600 * TICKS_PER_SECOND);
    let mut world = ReferenceWorld::spawn(&spec);
    world.universe.promote_to_orbital(TEST_VESSEL).unwrap();
    let l0 = world.angular_momentum_icrf();
    let cmd = motor_input(world.ship().input(), 0.0, 3.0e-5);
    world.ship_mut().set_input(cmd);

    world.universe.advance_to(at_s(300)).unwrap();
    assert_eq!(world.ship().now(), at_s(0));
    let mid_l = world.angular_momentum_icrf();
    let mid_drift = (mid_l - l0).length() / l0.length();
    assert!(mid_drift < 1e-11, "mid-arc drift {mid_drift:e}");

    let snap_mid = world.universe.checkpoint();
    let long_period = TickSpan::from_ticks(1_200 * TICKS_PER_SECOND);
    world.ship_mut().set_carry_period(long_period);
    world.universe.advance_to(at_s(1_200)).unwrap();
    assert_eq!(world.ship().now(), at_s(1_200));
    let end_l = world.angular_momentum_icrf();
    let end_drift = (end_l - l0).length() / l0.length();
    let bits_direct = snapshot_bits(&world);

    world.universe.restore(&snap_mid).unwrap();
    world.ship_mut().set_carry_period(long_period);
    world.universe.advance_to(at_s(1_200)).unwrap();
    let bits_same_slice = snapshot_bits(&world);
    assert_eq!(bits_direct, bits_same_slice, "same-slice restore mismatch");

    assert_sliced_parity(&mut world, &snap_mid, bits_direct);
    let replay_l = world.angular_momentum_icrf();
    let replay_drift = (replay_l - l0).length() / l0.length();
    tracing::info!(
        mid_drift,
        end_drift,
        replay_drift,
        gimbal_rad = world.joint_state().0,
        "mid-arc carry extension + checkpoint replay"
    );
    assert!(end_drift < 1e-11, "end_drift {end_drift:e}");
}

#[test]
fn mid_arc_carry_period_shortening_under_spin() {
    let rate = DVec3::new(0.05, 0.55, -0.04);
    let mut spec = CraftSpec::asymmetric_coupled(rate, VehicleIntegrator::FixedRk4 { substeps: 6 });
    spec.carry_period = TickSpan::from_ticks(1_200 * TICKS_PER_SECOND);
    let mut world = ReferenceWorld::spawn(&spec);
    world.universe.promote_to_orbital(TEST_VESSEL).unwrap();
    let l0 = world.angular_momentum_icrf();
    let cmd = motor_input(world.ship().input(), 0.0, 2.5e-5);
    world.ship_mut().set_input(cmd);

    world.universe.advance_to(at_s(300)).unwrap();
    world
        .ship_mut()
        .set_carry_period(TickSpan::from_ticks(600 * TICKS_PER_SECOND));
    world.universe.advance_to(at_s(600)).unwrap();
    assert_eq!(world.ship().now(), at_s(600));
    let l1 = world.angular_momentum_icrf();
    let drift = (l1 - l0).length() / l0.length();
    tracing::info!(
        drift,
        gimbal_rad = world.joint_state().0,
        "mid-arc carry shortening"
    );
    assert!(drift < 1e-11, "shortened carry drift {drift:e}");
}

#[test]
fn mid_arc_demotion_under_fast_spin_preserves_momentum() {
    let rate = DVec3::new(0.06, 0.65, -0.05);
    let mut spec = CraftSpec::asymmetric_coupled(rate, VehicleIntegrator::FixedRk4 { substeps: 6 });
    spec.carry_period = TickSpan::from_ticks(600 * TICKS_PER_SECOND);
    let mut world = ReferenceWorld::spawn(&spec);
    world.universe.promote_to_orbital(TEST_VESSEL).unwrap();
    let l0 = world.angular_momentum_icrf();
    let cmd = motor_input(world.ship().input(), 0.0, 3.0e-5);
    world.ship_mut().set_input(cmd);

    world.universe.advance_to(at_s(300)).unwrap();
    let l_aloft = world.angular_momentum_icrf();
    world.universe.demote_to_local(TEST_VESSEL).unwrap();
    assert_eq!(world.universe.tier(TEST_VESSEL).unwrap(), Tier::Local);
    assert_eq!(world.ship().now(), at_s(300));
    let l_down = world.angular_momentum_icrf();
    let handoff_drift = (l_down - l_aloft).length() / l_aloft.length();
    let total_drift = (l_down - l0).length() / l0.length();
    tracing::info!(handoff_drift, total_drift, "mid-arc demotion under spin");
    assert!(
        handoff_drift < 1e-11,
        "demotion handoff drift {handoff_drift:e}"
    );
    assert!(total_drift < 1e-11, "demotion total drift {total_drift:e}");
}

#[test]
fn fast_spin_speculation_horizon_slicing_invariant() {
    let rate = DVec3::new(0.05, 0.75, -0.04);
    let mut spec = CraftSpec::asymmetric_coupled(rate, VehicleIntegrator::FixedRk4 { substeps: 6 });
    spec.carry_period = TickSpan::from_ticks(300 * TICKS_PER_SECOND);

    let mut coarse = ReferenceWorld::spawn(&spec);
    coarse.universe.promote_to_orbital(TEST_VESSEL).unwrap();
    let cmd = motor_input(coarse.ship().input(), 0.0, 2.0e-5);
    coarse.ship_mut().set_input(cmd.clone());
    for t in [300_u64, 600, 900] {
        coarse.universe.advance_to(at_s(t)).unwrap();
    }
    let coarse_bits = snapshot_bits(&coarse);

    let mut fine = ReferenceWorld::spawn(&spec);
    fine.universe.promote_to_orbital(TEST_VESSEL).unwrap();
    fine.ship_mut().set_input(cmd);
    for t in (30..=900_u64).step_by(30) {
        fine.universe.advance_to(at_s(t)).unwrap();
    }
    let fine_bits = snapshot_bits(&fine);
    assert_eq!(
        coarse_bits, fine_bits,
        "slicing changed orbital carried spin state"
    );
}
