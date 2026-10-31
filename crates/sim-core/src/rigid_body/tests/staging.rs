use super::*;

/// The vehicle in flight and turning, so a separation has a velocity to
/// transfer as well as a position.
fn spinning_block(body: BodyEllipsoid) -> RigidBody6DofBlock {
    let start = VehicleState {
        velocity_ecef: DVec3::new(0.0, 1_600.0, 0.0),
        angular_rate_body: DVec3::new(0.01, 0.02, 0.03),
        ..rest_at(&body, 100_000.0)
    };
    vacuum_block(config(body, TEN_MS, start), vehicle(600.0))
}

#[test]
fn separating_a_stage_keeps_the_remaining_airframe_on_its_rigid_path() {
    let body = no_spin(BodyEllipsoid::MOON_IAU);
    let mut block = spinning_block(body);
    run(&mut block, 10, &RigidBodyInput::default());
    let before = block.output();
    let omega = block.omega_inertial_body();
    let old_cg = block.mass_properties().cg_body_m;
    let new_cg = DVec3::ZERO;

    block.separate_stage(TANK).unwrap();

    let after = block.output();
    assert_eq!(block.mass_properties().cg_body_m, new_cg);
    assert_eq!(block.mass_properties().total_mass_kg, 1_000.0);
    let arm = before.attitude * (new_cg - old_cg);
    assert!((after.position_ecef - (before.position_ecef + arm)).length() < 1e-9);
    let rate = before.attitude * before.angular_rate_body;
    let rim_speed = rate.cross(arm);
    assert!((after.velocity_ecef - (before.velocity_ecef + rim_speed)).length() < 1e-9);
    assert_eq!(block.omega_inertial_body(), omega);
    assert_eq!(after.attitude, before.attitude);
    assert!(!block.tree().is_attached(TANK).unwrap());
}

#[test]
fn a_separated_stage_reports_no_site_and_cannot_separate_twice() {
    let mut block = spinning_block(no_spin(BodyEllipsoid::MOON_IAU));
    block.separate_stage(TANK).unwrap();
    assert!(block.site_kinematics(TANK).is_err());
    let snapshot = block.snapshot();
    assert_eq!(
        block.separate_stage(TANK),
        Err(BlockError::Mbd {
            block: BLOCK,
            source: MbdError::DetachedComponent { id: TANK },
        })
    );
    assert_eq!(block.snapshot(), snapshot);
}

#[test]
fn separating_everything_is_refused_and_changes_nothing() {
    let mut block = spinning_block(no_spin(BodyEllipsoid::MOON_IAU));
    let snapshot = block.snapshot();
    assert_eq!(
        block.separate_stage(CORE),
        Err(BlockError::Mbd {
            block: BLOCK,
            source: MbdError::NoAttachedMass,
        })
    );
    assert_eq!(block.snapshot(), snapshot);
}

#[test]
fn a_snapshot_rolls_a_separation_back() {
    let mut block = spinning_block(no_spin(BodyEllipsoid::MOON_IAU));
    let snapshot = block.snapshot();
    block.separate_stage(TANK).unwrap();
    block.restore(&snapshot);
    assert_eq!(block.snapshot(), snapshot);
    assert_eq!(block.mass_properties().total_mass_kg, 1_600.0);
}
