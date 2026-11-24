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

#[cfg(test)]
fn spin_test_block(
    body: BodyEllipsoid,
    initial: VehicleState,
    integrator: VehicleIntegrator,
    min_rad: f64,
    max_rad: f64,
) -> MultibodyVehicleBlock {
    let tree = ComponentTree::new([
        rigid_part(
            "bus",
            Placement::ROOT,
            480.0,
            DVec3::new(200.0, 300.0, 250.0),
        ),
        (
            ComponentSpec {
                name: "antenna".to_owned(),
                placement: Placement::child_of(ComponentId(0), DVec3::new(0.0, 0.0, -1.2)),
                dry_mass_kg: 12.0,
                dry_inertia_body: DMat3::from_diagonal(DVec3::new(1.5, 1.5, 2.0)),
                thermal_capacity_j_k: 50.0,
                nominal_power_w: 0.0,
                joint: JointSpec::revolute(DVec3::Z, min_rad, max_rad),
            },
            ComponentState::at_temperature(290.0),
        ),
    ])
    .unwrap();
    MultibodyVehicleBlock::new(
        MultibodyVehicleConfig {
            body,
            period: TickSpan::from_ticks(60 * TICKS_PER_SECOND),
            epoch: UtcTime::from_unix_seconds(0.0),
            initial,
            integrator,
            time_scale: MotionTimeScale::Seconds(1.0),
        },
        tree,
        Vec::new(),
        MultibodySubsystems::default(),
        Box::new(VacuumAtmosphere),
        std::sync::Arc::new(body),
    )
    .unwrap()
}

#[test]
fn carried_steps_on_spinning_spacecraft_stay_stable_and_match_local_slew() {
    use mbd_multibody::Tolerances;
    let body = BodyEllipsoid {
        mu_m3_s2: 0.0,
        omega_rad_s: 0.0,
        ..BodyEllipsoid::WGS84
    };
    let tol = Tolerances::uniform(1e-13, 1e-13);
    for spin_y in [0.05, 0.2, 0.3, 0.5, 1.0] {
        let initial = VehicleState {
            position_ecef: DVec3::new(7_000_000.0, 0.0, 0.0),
            velocity_ecef: DVec3::ZERO,
            attitude: DQuat::IDENTITY,
            angular_rate_body: DVec3::new(0.1 * spin_y, spin_y, -0.08 * spin_y),
        };
        let input = MultibodyVehicleInput {
            joint_drives: vec![mbd_multibody::JointDriveInput::scalar(ComponentId(1), 2e-4)],
            ..MultibodyVehicleInput::default()
        };
        let mut ref_local = spin_test_block(
            body,
            initial,
            VehicleIntegrator::Adaptive(tol),
            -100.0,
            100.0,
        );
        let mut ref_states = vec![ref_local.output()];
        for _ in 0..10 {
            ref_local.step(&input).unwrap();
            ref_states.push(ref_local.output());
        }
        let ref_j = ref_local.tree().state(ComponentId(1)).unwrap().joint;

        for integ in [
            VehicleIntegrator::FixedRk4 { substeps: 6 },
            VehicleIntegrator::Adaptive(tol),
        ] {
            let mut carried = spin_test_block(body, initial, integ, -100.0, 100.0);
            let mut momentum_body_start = angular_momentum_body(&carried);
            let (i0, h0) = carried
                .effective_inertia_and_momentum_cg_body(&ref_states[0])
                .unwrap();
            let l0_inertial = ref_states[0].attitude * (i0 * ref_states[0].angular_rate_body + h0);
            for k in 0..10 {
                let out = carried
                    .step_carried_decoupled(
                        &input,
                        &ref_states[k],
                        &ref_states[k + 1],
                        &ref_states[k + 1],
                    )
                    .unwrap();
                momentum_body_start += out.reaction.angular_impulse_cg_body_nms;
            }
            let end_state = &ref_states[10];
            let (i1, h1) = carried
                .effective_inertia_and_momentum_cg_body(end_state)
                .unwrap();
            let l1_inertial = end_state.attitude * (i1 * end_state.angular_rate_body + h1);
            let drift = (l1_inertial - l0_inertial).length() / l0_inertial.length();
            assert!(
                drift < 1e-11,
                "spin_y={spin_y} {integ:?}: momentum drifted by {drift:e}"
            );
            let j = carried.tree().state(ComponentId(1)).unwrap().joint;
            assert!(
                (j.q_scalar - ref_j.q_scalar).abs() < 1e-6
                    && (j.q_vel_scalar - ref_j.q_vel_scalar).abs() < 1e-7,
                "spin_y={spin_y} {integ:?}: carried joint ({}, {}) drifted from local ({}, {})",
                j.q_scalar,
                j.q_vel_scalar,
                ref_j.q_scalar,
                ref_j.q_vel_scalar,
            );
            let _ = momentum_body_start;
        }
    }
}

#[test]
fn carried_fixed_rk4_stays_stable_across_joint_stop_while_spinning() {
    let body = BodyEllipsoid {
        mu_m3_s2: 0.0,
        omega_rad_s: 0.0,
        ..BodyEllipsoid::WGS84
    };
    let initial = VehicleState {
        position_ecef: DVec3::new(7_000_000.0, 0.0, 0.0),
        velocity_ecef: DVec3::ZERO,
        attitude: DQuat::IDENTITY,
        angular_rate_body: DVec3::new(0.05, 0.5, -0.04),
    };
    let input = MultibodyVehicleInput {
        joint_drives: vec![mbd_multibody::JointDriveInput::scalar(ComponentId(1), 5e-3)],
        ..MultibodyVehicleInput::default()
    };
    let mut local = spin_test_block(
        body,
        initial,
        VehicleIntegrator::Adaptive(mbd_multibody::Tolerances::uniform(1e-9, 1e-9)),
        -0.2,
        0.2,
    );
    let mut carried = spin_test_block(
        body,
        initial,
        VehicleIntegrator::FixedRk4 { substeps: 6 },
        -0.2,
        0.2,
    );
    for _ in 0..8 {
        let start = local.output();
        local.step(&input).unwrap();
        let end = local.output();
        carried
            .step_carried_between(&input, &start, &end, &end)
            .unwrap();
    }
    let j = carried.tree().state(ComponentId(1)).unwrap().joint;
    let ref_j = local.tree().state(ComponentId(1)).unwrap().joint;
    assert!(j.q_scalar.is_finite() && (j.q_scalar - ref_j.q_scalar).abs() < 1e-3);
}
