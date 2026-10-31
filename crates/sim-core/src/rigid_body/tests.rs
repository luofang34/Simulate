use std::f64::consts::{PI, TAU};

use glam::{DMat3, DQuat};
use mbd_core::{ComponentSpec, Placement, VacuumAero};
use sim_atmosphere::VacuumAtmosphere;
use sim_geo::Geodetic;

use super::*;

mod aero;
mod mass;
mod replay;
mod staging;

const CORE: ComponentId = ComponentId(0);
const TANK: ComponentId = ComponentId(1);
const SITE: ComponentId = ComponentId(2);
const EPOCH: f64 = 1_700_000_000.0;
const TEN_MS: TickSpan = TickSpan::from_ticks(crate::TICKS_PER_SECOND / 100);

fn spec(name: &str, placement: Placement, mass: f64, inertia: DVec3) -> ComponentSpec {
    ComponentSpec {
        name: name.to_owned(),
        placement,
        dry_mass_kg: mass,
        dry_inertia_body: DMat3::from_diagonal(inertia),
        thermal_capacity_j_k: 1.0,
        nominal_power_w: 0.0,
    }
}

fn component(temperature_k: f64) -> ComponentState {
    ComponentState::at_temperature(temperature_k)
}

/// A 1,000 kg core with an inertia of 500 about each axis, a tank 2 m forward
/// holding `fluid_kg`, and a massless site 1 m to the right.
fn vehicle(fluid_kg: f64) -> ComponentTree {
    let tank = ComponentState {
        fluid_mass_kg: fluid_kg,
        ..component(290.0)
    };
    ComponentTree::new([
        (
            spec("core", Placement::ROOT, 1_000.0, DVec3::splat(500.0)),
            component(290.0),
        ),
        (
            spec(
                "tank",
                Placement::child_of(CORE, DVec3::new(2.0, 0.0, 0.0)),
                0.0,
                DVec3::ZERO,
            ),
            tank,
        ),
        (
            spec(
                "site",
                Placement::child_of(CORE, DVec3::new(0.0, 1.0, 0.0)),
                0.0,
                DVec3::ZERO,
            ),
            component(290.0),
        ),
    ])
    .unwrap()
}

fn rest_at(body: &BodyEllipsoid, height_m: f64) -> VehicleState {
    let position = body.geodetic_to_fixed(&Geodetic::from_degrees(0.0, 0.0, height_m));
    VehicleState {
        position_ecef: position,
        velocity_ecef: DVec3::ZERO,
        attitude: DQuat::IDENTITY,
        angular_rate_body: DVec3::ZERO,
    }
}

fn config(body: BodyEllipsoid, period: TickSpan, initial: VehicleState) -> RigidBodyConfig {
    RigidBodyConfig {
        body,
        period,
        epoch: UtcTime::from_unix_seconds(EPOCH),
        initial,
    }
}

fn vacuum_block(config: RigidBodyConfig, tree: ComponentTree) -> RigidBody6DofBlock {
    RigidBody6DofBlock::new(
        config,
        tree,
        Box::new(VacuumAero),
        Box::new(VacuumAtmosphere),
    )
    .unwrap()
}

fn run(block: &mut RigidBody6DofBlock, steps: usize, input: &RigidBodyInput) {
    for _ in 0..steps {
        block.step(input).unwrap();
    }
}

fn no_spin(body: BodyEllipsoid) -> BodyEllipsoid {
    BodyEllipsoid {
        omega_rad_s: 0.0,
        ..body
    }
}

#[test]
fn free_fall_follows_central_gravity_on_any_body() {
    for (body, height_m) in [
        (no_spin(BodyEllipsoid::WGS84), 10_000.0),
        (no_spin(BodyEllipsoid::MOON_IAU), 15_000.0),
    ] {
        let start = rest_at(&body, height_m);
        let mut block = vacuum_block(config(body, TEN_MS, start), vehicle(0.0));
        let radius = start.position_ecef.length();
        let gravity = body.mu_m3_s2 / (radius * radius);
        run(&mut block, 100, &RigidBodyInput::default());

        let end = block.output();
        let fallen = start.position_ecef - end.position_ecef;
        let expected = 0.5 * gravity;
        assert!(((fallen.length() - expected) / expected).abs() < 1e-5);
        assert!(fallen.normalize().dot(start.position_ecef.normalize()) > 1.0 - 1e-12);
        assert!(((end.velocity_ecef.length() - gravity) / gravity).abs() < 1e-5);
        assert_eq!(block.specific_force_cg_body(), DVec3::ZERO);
    }
}

#[test]
fn a_vehicle_at_rest_on_a_rotating_body_falls_with_the_apparent_gravity() {
    let body = BodyEllipsoid::WGS84;
    let start = rest_at(&body, 0.0);
    let period = TickSpan::period_of_hz(1_000).unwrap();
    let mut block = vacuum_block(config(body, period, start), vehicle(0.0));
    run(&mut block, 1, &RigidBodyInput::default());

    let spin = DVec3::Z * body.omega_rad_s;
    let apparent =
        body.gravity_fixed(start.position_ecef) - spin.cross(spin.cross(start.position_ecef));
    let measured = block.output().velocity_ecef / period.as_seconds();
    assert!((measured - apparent).length() < 1e-5, "{measured}");
    assert!(apparent.length() < body.gravity_fixed(start.position_ecef).length() - 0.03);
}

#[test]
fn a_circular_orbit_stays_circular_in_the_rotating_frame() {
    let body = BodyEllipsoid::WGS84;
    let radius = body.semi_major_m + 400_000.0;
    let speed = (body.mu_m3_s2 / radius).sqrt();
    let position = DVec3::new(radius, 0.0, 0.0);
    let inertial_velocity = DVec3::new(0.0, speed, 0.0);
    let start = VehicleState {
        position_ecef: position,
        velocity_ecef: inertial_velocity - (DVec3::Z * body.omega_rad_s).cross(position),
        attitude: DQuat::IDENTITY,
        angular_rate_body: DVec3::ZERO,
    };
    let period = TickSpan::SECOND;
    let mut block = vacuum_block(config(body, period, start), vehicle(0.0));
    let orbit_s = TAU * radius / speed;
    let steps = orbit_s.floor() as usize;
    run(&mut block, steps, &RigidBodyInput::default());

    let end = block.output();
    assert!((end.position_ecef.length() - radius).abs() < 0.01);
    let turned = DQuat::from_rotation_z(-body.omega_rad_s * steps as f64);
    let closure = (turned.inverse() * end.position_ecef)
        .normalize()
        .dot(DVec3::X);
    let missing_s = orbit_s - steps as f64;
    assert!(closure.acos() * radius < speed * missing_s + 1.0);
}

#[test]
fn a_torque_free_tumble_conserves_inertial_angular_momentum() {
    let body = BodyEllipsoid::MOON_IAU;
    let tree = ComponentTree::new([(
        spec(
            "core",
            Placement::ROOT,
            800.0,
            DVec3::new(100.0, 200.0, 250.0),
        ),
        component(290.0),
    )])
    .unwrap();
    let start = VehicleState {
        angular_rate_body: DVec3::new(0.02, 1.2, 0.03),
        ..rest_at(&body, 100_000.0)
    };
    let inertia = tree.mass_properties().unwrap().inertia_cg_body;
    let mut block = vacuum_block(config(body, TEN_MS, start), tree);

    let momentum = |block: &RigidBody6DofBlock, seconds: f64| {
        let unspun = DQuat::from_rotation_z(body.omega_rad_s * seconds);
        unspun * (block.output().attitude * (inertia * block.omega_inertial_body()))
    };
    let initial = momentum(&block, 0.0);
    run(&mut block, 3_000, &RigidBodyInput::default());
    let drift = (momentum(&block, 30.0) - initial).length() / initial.length();
    assert!(drift < 1e-8, "{drift}");
    let quaternion_norm = block.output().attitude.length();
    assert!((quaternion_norm - 1.0).abs() < 1e-12);
}

#[test]
fn angular_rate_output_is_relative_and_euler_runs_on_the_inertial_rate() {
    let body = BodyEllipsoid::WGS84;
    let start = rest_at(&body, 500_000.0);
    let block = vacuum_block(config(body, TEN_MS, start), vehicle(0.0));
    assert_eq!(block.output().angular_rate_body, DVec3::ZERO);
    let expected = DVec3::Z * body.omega_rad_s;
    assert!((block.omega_inertial_body() - expected).length() < 1e-15);
}

#[test]
fn quarter_turn_of_attitude_matches_the_commanded_rate() {
    let body = no_spin(BodyEllipsoid::MOON_IAU);
    let rate = PI / 2.0;
    let start = VehicleState {
        angular_rate_body: DVec3::new(0.0, 0.0, rate),
        ..rest_at(&body, 100_000.0)
    };
    let mut block = vacuum_block(config(body, TEN_MS, start), vehicle(0.0));
    run(&mut block, 100, &RigidBodyInput::default());
    let forward = block.output().attitude * DVec3::X;
    assert!((forward - DVec3::Y).length() < 1e-9, "{forward}");
}
