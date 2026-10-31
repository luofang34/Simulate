use super::*;

#[test]
fn thruster_moment_is_taken_about_the_live_cg() {
    let body = no_spin(BodyEllipsoid::MOON_IAU);
    let start = rest_at(&body, 100_000.0);
    let mut block = vacuum_block(config(body, TEN_MS, start), vehicle(1_000.0));
    let mass = *block.mass_properties();
    assert!((mass.cg_body_m - DVec3::new(1.0, 0.0, 0.0)).length() < 1e-12);

    let input = RigidBodyInput {
        thrusters: vec![ThrusterWrench {
            application_point_body_m: DVec3::ZERO,
            force_body_n: DVec3::new(0.0, 0.0, 400.0),
            moment_body_nm: DVec3::new(30.0, 0.0, 0.0),
        }],
        ..Default::default()
    };
    block.step(&input).unwrap();

    let arm = DVec3::new(-1.0, 0.0, 0.0);
    let moment = arm.cross(input.thrusters[0].force_body_n) + DVec3::new(30.0, 0.0, 0.0);
    let expected_alpha = mass.inertia_cg_body.inverse() * moment;
    assert!((block.alpha_body() - expected_alpha).length() < 1e-5);
    let expected_force = DVec3::new(0.0, 0.0, 400.0) / mass.total_mass_kg;
    assert!((block.specific_force_cg_body() - expected_force).length() < 1e-12);
}

#[test]
fn burning_propellant_moves_the_cg_and_shrinks_the_inertia() {
    let body = no_spin(BodyEllipsoid::MOON_IAU);
    let start = rest_at(&body, 100_000.0);
    let mut block = vacuum_block(config(body, TEN_MS, start), vehicle(1_000.0));
    let before = *block.mass_properties();

    let input = RigidBodyInput {
        burns: vec![PropellantBurn {
            tank: TANK,
            dm_dt_kg_s: -100.0,
        }],
        ..Default::default()
    };
    run(&mut block, 500, &input);

    let after = *block.mass_properties();
    assert!((after.total_mass_kg - 1_500.0).abs() < 1e-9);
    let expected_cg = 2.0 * 500.0 / 1_500.0;
    assert!((after.cg_body_m.x - expected_cg).abs() < 1e-9);
    assert!(after.cg_body_m.x < before.cg_body_m.x);
    assert!(after.inertia_cg_body.y_axis.y < before.inertia_cg_body.y_axis.y);
    assert!((block.tree().state(TANK).unwrap().fluid_mass_kg - 500.0).abs() < 1e-9);
}

#[test]
fn a_tank_stops_at_empty() {
    let body = no_spin(BodyEllipsoid::MOON_IAU);
    let start = rest_at(&body, 100_000.0);
    let mut block = vacuum_block(config(body, TEN_MS, start), vehicle(1.0));
    let input = RigidBodyInput {
        burns: vec![PropellantBurn {
            tank: TANK,
            dm_dt_kg_s: -1_000.0,
        }],
        ..Default::default()
    };
    run(&mut block, 5, &input);
    assert_eq!(block.tree().state(TANK).unwrap().fluid_mass_kg, 0.0);
}

fn vent(dm_dt_kg_s: f64) -> RigidBodyInput {
    RigidBodyInput {
        burns: vec![PropellantBurn {
            tank: TANK,
            dm_dt_kg_s,
        }],
        ..Default::default()
    }
}

/// Venting without thrust throws no impulse at the airframe, so a vehicle with
/// no rotation has to keep the airframe on the same trajectory.
#[test]
fn venting_propellant_leaves_the_airframe_path_undisturbed() {
    let body = no_spin(BodyEllipsoid::MOON_IAU);
    let start = rest_at(&body, 100_000.0);
    let mut vented = vacuum_block(config(body, TEN_MS, start), vehicle(1_000.0));
    let cg = vented.mass_properties().cg_body_m;
    let mut coasting = vacuum_block(config(body, TEN_MS, start), vehicle(1_000.0));
    run(&mut vented, 100, &vent(-400.0));
    run(&mut coasting, 100, &RigidBodyInput::default());

    let shifted = vented.mass_properties().cg_body_m;
    assert!((shifted - cg).length() > 0.1);
    let airframe = |block: &RigidBody6DofBlock, cg_body: DVec3| {
        let state = block.output();
        state.position_ecef - state.attitude * cg_body
    };
    let separation = (airframe(&vented, shifted) - airframe(&coasting, cg)).length();
    assert!(separation < 1e-6, "{separation}");
}

/// A spinning vehicle goes on to pivot about its new centre of gravity, but
/// the airframe's velocity cannot step at the instant that centre moves.
#[test]
fn a_cg_shift_while_spinning_keeps_the_airframe_velocity_continuous() {
    let body = no_spin(BodyEllipsoid::MOON_IAU);
    let start = VehicleState {
        angular_rate_body: DVec3::new(0.0, 0.0, 0.3),
        ..rest_at(&body, 100_000.0)
    };
    let mut vented = vacuum_block(config(body, TEN_MS, start), vehicle(1_000.0));
    let mut coasting = vacuum_block(config(body, TEN_MS, start), vehicle(1_000.0));
    vented.step(&vent(-10_000.0)).unwrap();
    coasting.step(&RigidBodyInput::default()).unwrap();

    let airframe_velocity = |block: &RigidBody6DofBlock| {
        let state = block.output();
        let arm = state.attitude * block.mass_properties().cg_body_m;
        state.velocity_ecef - (state.attitude * state.angular_rate_body).cross(arm)
    };
    let step = (airframe_velocity(&vented) - airframe_velocity(&coasting)).length();
    assert!(step < 1e-12, "{step}");
    assert!((vented.mass_properties().cg_body_m.x - 2.0 * 900.0 / 1_900.0).abs() < 1e-9);
}

#[test]
fn site_kinematics_carry_centripetal_acceleration_and_thrust() {
    let body = no_spin(BodyEllipsoid::MOON_IAU);
    let start = VehicleState {
        angular_rate_body: DVec3::new(0.0, 0.0, 2.0),
        ..rest_at(&body, 100_000.0)
    };
    let mut block = vacuum_block(config(body, TEN_MS, start), vehicle(0.0));
    let site = block.site_kinematics(SITE).unwrap();
    assert!((site.specific_force_site_m_s2 - DVec3::new(0.0, -4.0, 0.0)).length() < 1e-9);
    assert!((site.angular_rate_site_rad_s - DVec3::new(0.0, 0.0, 2.0)).length() < 1e-12);

    let input = RigidBodyInput {
        thrusters: vec![ThrusterWrench {
            application_point_body_m: DVec3::ZERO,
            force_body_n: DVec3::new(1_000.0, 0.0, 0.0),
            moment_body_nm: DVec3::ZERO,
        }],
        ..Default::default()
    };
    block.step(&input).unwrap();
    let thrusting = block.site_kinematics(SITE).unwrap();
    let rate = block.omega_inertial_body();
    let expected = DVec3::X + rate.cross(rate.cross(DVec3::Y));
    assert!((thrusting.specific_force_site_m_s2 - expected).length() < 1e-9);
}

#[test]
fn a_detached_site_has_no_kinematics() {
    let body = BodyEllipsoid::MOON_IAU;
    let mut tree = vehicle(0.0);
    let mut site = *tree.state(SITE).unwrap();
    site.attached = false;
    tree.set_state(SITE, site).unwrap();
    let block = vacuum_block(config(body, TEN_MS, rest_at(&body, 1_000.0)), tree);
    assert_eq!(
        block.site_kinematics(SITE),
        Err(MbdError::DetachedComponent { id: SITE })
    );
}
