use sim_atmosphere::StandardAtmosphere;

use super::aero::{capsule_aero, capsule_tree};
use super::*;

fn busy_input() -> RigidBodyInput {
    RigidBodyInput {
        thrusters: vec![ThrusterWrench {
            application_point_body_m: DVec3::new(-3.0, 0.2, 0.0),
            force_body_n: DVec3::new(500.0, 0.0, 30.0),
            moment_body_nm: DVec3::new(2.0, 0.0, 0.0),
        }],
        burns: vec![PropellantBurn {
            tank: TANK,
            dm_dt_kg_s: -1.5,
        }],
        ..Default::default()
    }
}

#[test]
fn a_restored_block_replays_bit_identically_with_its_tree() {
    let body = BodyEllipsoid::MOON_IAU;
    let start = rest_at(&body, 100_000.0);
    let input = busy_input();
    let mut original = vacuum_block(config(body, TEN_MS, start), vehicle(100.0));
    run(&mut original, 200, &input);
    let saved = original.snapshot();
    assert_eq!(saved.component_states(), original.tree().states());
    let saved_site = original.site_kinematics(SITE).unwrap();

    run(&mut original, 200, &input);
    let expected = (
        original.output(),
        original.site_kinematics(SITE).unwrap(),
        original.tree().states().to_vec(),
        original.now(),
    );

    let mut replay = vacuum_block(config(body, TEN_MS, start), vehicle(100.0));
    replay.restore(&saved);
    assert_eq!(replay.site_kinematics(SITE).unwrap(), saved_site);
    run(&mut replay, 200, &input);
    assert_eq!(expected.0, replay.output());
    assert_eq!(expected.1, replay.site_kinematics(SITE).unwrap());
    assert_eq!(expected.2, replay.tree().states());
    assert_eq!(expected.3, replay.now());
}

#[test]
fn a_snapshot_from_a_different_tree_is_refused() {
    let body = BodyEllipsoid::MOON_IAU;
    let start = rest_at(&body, 100_000.0);
    let source = vacuum_block(
        config(body, TEN_MS, start),
        capsule_tree(DVec3::new(0.0, 0.0, 0.0)),
    );
    let mut target = vacuum_block(config(body, TEN_MS, start), vehicle(10.0));
    let before = target.snapshot();
    assert!(matches!(
        target.try_restore(&source.snapshot()),
        Err(MbdError::StateCountMismatch { .. })
    ));
    assert_eq!(target.snapshot(), before);
}

#[test]
fn a_failed_step_changes_nothing() {
    let body = no_spin(BodyEllipsoid::MOON_IAU);
    let start = rest_at(&body, 100_000.0);
    let mut block = vacuum_block(config(body, TEN_MS, start), vehicle(100.0));
    let before = block.snapshot();

    let unknown_tank = RigidBodyInput {
        burns: vec![
            PropellantBurn {
                tank: TANK,
                dm_dt_kg_s: -5.0,
            },
            PropellantBurn {
                tank: ComponentId(99),
                dm_dt_kg_s: -5.0,
            },
        ],
        ..Default::default()
    };
    assert!(matches!(
        block.step(&unknown_tank),
        Err(BlockError::Mbd {
            source: MbdError::UnknownComponent { .. },
            ..
        })
    ));
    assert_eq!(block.snapshot(), before);

    let nan_thrust = RigidBodyInput {
        thrusters: vec![ThrusterWrench {
            application_point_body_m: DVec3::ZERO,
            force_body_n: DVec3::NAN,
            moment_body_nm: DVec3::ZERO,
        }],
        ..Default::default()
    };
    assert!(matches!(
        block.step(&nan_thrust),
        Err(BlockError::InputOutOfRange { .. })
    ));
    assert_eq!(block.snapshot(), before);
}

#[test]
fn leaving_the_atmosphere_model_reports_the_environment_and_keeps_the_state() {
    let body = no_spin(BodyEllipsoid::WGS84);
    let start = VehicleState {
        velocity_ecef: DVec3::new(10_000.0, 0.0, 0.0),
        ..rest_at(&body, 30_000.0)
    };
    let mut block = RigidBody6DofBlock::new(
        config(body, TickSpan::SECOND, start),
        capsule_tree(DVec3::ZERO),
        capsule_aero(),
        Box::new(StandardAtmosphere),
    )
    .unwrap();
    let before = block.snapshot();
    assert!(matches!(
        block.step(&RigidBodyInput::default()),
        Err(BlockError::Environment { .. })
    ));
    assert_eq!(block.snapshot(), before);
}

#[test]
fn a_degenerate_inertia_is_refused_at_construction() {
    let body = BodyEllipsoid::MOON_IAU;
    let point_mass = ComponentTree::new([(
        spec("point", Placement::ROOT, 100.0, DVec3::ZERO),
        component(290.0),
    )])
    .unwrap();
    let result = RigidBody6DofBlock::new(
        config(body, TEN_MS, rest_at(&body, 1_000.0)),
        point_mass,
        Box::new(VacuumAero),
        Box::new(VacuumAtmosphere),
    );
    assert!(matches!(result, Err(BlockError::InputOutOfRange { .. })));
}
