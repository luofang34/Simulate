//! A vehicle carried by another integrator: its joints move under their
//! drives with the base prescribed, and what that motion does to the
//! vehicle's angular momentum is reported for the other integrator to apply.

use mbd_multibody::{Tether, TetherAnchor, TetherCommand, WinchTetherSpec, WinchTetherState};

use super::*;
use crate::{BlockError, CarriedReaction, CoupledMechanism};

/// Angular momentum of `block` about its centre of gravity in body axes,
/// from the tree and the base alone.
fn angular_momentum_body(block: &MultibodyVehicleBlock) -> DVec3 {
    let base = block.floating_base_fixed();
    let momentum = MultiBodyDynamics::system_momentum(block.tree(), &[], &base).unwrap();
    block.output().attitude.conjugate() * momentum.angular_about_com_world_nms
}

#[test]
fn a_carried_drive_moves_its_joint_and_reports_what_it_took_from_the_base() {
    let mut block = floating_arm_block();
    let prescribed = VehicleState {
        angular_rate_body: DVec3::new(0.01, -0.02, 0.015),
        ..block.output()
    };
    block.adopt_base_state(&prescribed).unwrap();
    let before = angular_momentum_body(&block);
    let input = MultibodyVehicleInput {
        joint_drives: vec![mbd_multibody::JointDriveInput::scalar(ComponentId(1), 15.0)],
        ..MultibodyVehicleInput::default()
    };
    let mut handed_over = DVec3::ZERO;
    let mut last = CarriedReaction::NONE;
    for _ in 0..40 {
        last = block.step_carried(&input, &prescribed).unwrap();
        handed_over += last.angular_impulse_cg_body_nms;
        assert_eq!(
            block.output().angular_rate_body,
            prescribed.angular_rate_body
        );
    }
    let joint = block.tree().state(ComponentId(1)).unwrap().joint;
    assert!(
        joint.q_scalar > 0.02 && joint.q_vel_scalar > 0.1,
        "the arm swings: {} rad at {} rad/s",
        joint.q_scalar,
        joint.q_vel_scalar
    );
    // The body rate was held, so the arm's swing changed the vehicle's
    // angular momentum; the impulses handed over are exactly that change,
    // owed to the base.
    let after = angular_momentum_body(&block);
    assert!((after - before).length() > 1.0);
    assert!(
        (after - before + handed_over).length() < 1e-10,
        "{before} to {after} with {handed_over} handed over"
    );
    // What the arm itself carries is the vehicle's momentum less the part
    // its body rate accounts for with the arm where it is.
    let inertia = block.mass_properties().inertia_cg_body;
    let carried_by_the_arm = after - inertia * prescribed.angular_rate_body;
    assert!((last.internal_momentum_cg_body_nms - carried_by_the_arm).length() < 1e-9);
    assert_eq!(
        block.internal_momentum_cg_body().unwrap(),
        last.internal_momentum_cg_body_nms
    );
}

#[test]
fn a_carried_vehicle_without_joints_asks_nothing_of_the_base() {
    let mut block = capsule_block(VehicleIntegrator::REAL_TIME);
    let prescribed = block.output();
    let reaction = block
        .step_carried(&MultibodyVehicleInput::default(), &prescribed)
        .unwrap();
    assert_eq!(reaction, CarriedReaction::NONE);
    assert_eq!(block.internal_momentum_cg_body().unwrap(), DVec3::ZERO);
}

#[test]
fn a_carried_vehicle_refuses_a_tether_to_another() {
    let mut block = floating_arm_block();
    let prescribed = block.output();
    let tether = Tether {
        spec: WinchTetherSpec {
            stiffness_n_m: 2_000.0,
            damping_n_s_m: 400.0,
            max_length_m: 20.0,
            breaking_tension_n: 50_000.0,
            burn_through_energy_j: None,
            auto_cut_low_tension_n: None,
            auto_cut_dwell_s: 0.0,
        },
        anchor_a: TetherAnchor::Link {
            component: ComponentId(0),
            offset_local_m: DVec3::ZERO,
        },
        anchor_b: TetherAnchor::World {
            position_world_m: prescribed.position_ecef + DVec3::X * 10.0,
            velocity_world_m_s: DVec3::ZERO,
        },
        initial_length_m: 9.5,
        command: TetherCommand {
            payout_rate_m_s: 0.0,
            heat_rate_w: 0.0,
            cut: false,
        },
    };
    let input = MultibodyVehicleInput {
        mechanisms: vec![CoupledMechanism::Tether(
            tether,
            WinchTetherState::intact_at(9.5),
        )],
        ..MultibodyVehicleInput::default()
    };
    let before = format!("{:?}", block.snapshot());
    assert!(matches!(
        block.step_carried(&input, &prescribed),
        Err(BlockError::InputOutOfRange {
            input: "mechanisms",
            ..
        })
    ));
    assert_eq!(format!("{:?}", block.snapshot()), before);
}
