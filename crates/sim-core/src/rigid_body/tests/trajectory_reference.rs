//! Recorded end states of the rigid-body block's reference cases.

use super::aero::{capsule_aero, capsule_tree};
use super::*;
use crate::recorded::{Row, assert_rows, vec_rows};

fn state_rows(block: &RigidBody6DofBlock, recorded: &[[f64; 3]; 4], tol: [f64; 4]) -> Vec<Row> {
    let state = block.output();
    let mut rows = Vec::new();
    rows.extend(vec_rows(
        ["px", "py", "pz"],
        state.position_ecef,
        recorded[0],
        tol[0],
    ));
    rows.extend(vec_rows(
        ["vx", "vy", "vz"],
        state.velocity_ecef,
        recorded[1],
        tol[1],
    ));
    rows.extend(vec_rows(
        ["nose_x", "nose_y", "nose_z"],
        state.attitude * DVec3::X,
        recorded[2],
        tol[2],
    ));
    rows.extend(vec_rows(
        ["wx", "wy", "wz"],
        block.omega_inertial_body(),
        recorded[3],
        tol[3],
    ));
    rows
}

const FREE_FALL: [[f64; 3]; 4] = [
    [6387649.513774571, 0.23698282144937027, 0.0],
    [-97.49972498574905, 0.07109520800099045, 0.0],
    [1.0, 0.0, 0.0],
    [0.0, 0.0, 7.292115e-5],
];
const LOW_ORBIT: [[f64; 3]; 4] = [
    [4059658.03099577, -4904545.489292042, -2320356.044329903],
    [5554.990230666002, 2803.7437438553784, 3774.843794149492],
    [
        0.15814753346832322,
        -0.9196301490414122,
        -0.3595410221823875,
    ],
    [0.01, -0.02, 0.030072921149999998],
];
const TUMBLE: [[f64; 3]; 4] = [
    [1837333.3714959626, -2.3984040169952783, -6.928923265867475],
    [
        -14.203814327626864,
        -0.3779618231698393,
        -0.9495998060826988,
    ],
    [-0.5120496823582285, 0.24317637148407245, 0.8238145271532169],
    [0.17999999999999583, 1.0884492615175507, -1.5140655011885298],
];
const CAPSULE_TRIM: [[f64; 3]; 4] = [
    [6677473.644851327, 0.0, -1768.561297985985],
    [7124.0345535796005, 0.0, -117.50791838757505],
    [0.9407766039660764, 0.0, 0.3390271101697569],
    [0.0, -0.08585733716577587, 0.0],
];

#[test]
fn free_fall_on_the_rotating_earth_ends_where_it_was_recorded() {
    let body = BodyEllipsoid::WGS84;
    let start = rest_at(&body, 10_000.0);
    let mut block = vacuum_block(config(body, TEN_MS, start), vehicle(0.0));
    run(&mut block, 1_000, &RigidBodyInput::default());
    // Ten seconds of free fall beside the rotating Earth: the truncation of
    // a fourth-order step at 100 Hz is far below a micrometre, whichever
    // axes the fall is integrated in.
    assert_rows(&state_rows(&block, &FREE_FALL, [1e-6, 1e-7, 1e-10, 1e-12]));
}

#[test]
fn a_low_orbit_ends_where_it_was_recorded() {
    let body = BodyEllipsoid::WGS84;
    let radius = body.semi_major_m + 400_000.0;
    let speed = (body.mu_m3_s2 / radius).sqrt();
    let position = DVec3::new(radius, 0.0, 0.0);
    let start = VehicleState {
        position_ecef: position,
        velocity_ecef: DVec3::new(0.0, speed * 0.8, speed * 0.6)
            - (DVec3::Z * body.omega_rad_s).cross(position),
        attitude: DQuat::IDENTITY,
        angular_rate_body: DVec3::new(0.01, -0.02, 0.03),
    };
    let mut block = vacuum_block(config(body, TickSpan::SECOND, start), vehicle(0.0));
    run(&mut block, 5_000, &RigidBodyInput::default());
    // One-second steps over most of a revolution of a tumbling body: the
    // fourth-order truncation of the orbit is centimetres, and the tumble
    // must not leak into it.
    assert_rows(&state_rows(&block, &LOW_ORBIT, [0.05, 1e-4, 1e-6, 1e-9]));
}

#[test]
fn a_thrusting_venting_tumble_ends_where_it_was_recorded() {
    let body = BodyEllipsoid::MOON_IAU;
    let start = VehicleState {
        angular_rate_body: DVec3::new(0.02, 0.3, 0.03),
        ..rest_at(&body, 100_000.0)
    };
    let input = RigidBodyInput {
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
    };
    let mut block = vacuum_block(config(body, TEN_MS, start), vehicle(100.0));
    run(&mut block, 1_000, &input);
    let mass = block.mass_properties().total_mass_kg;
    let mut rows = state_rows(&block, &TUMBLE, [1e-6, 1e-7, 1e-9, 1e-10]);
    rows.push(("mass_kg", mass, 1084.9999999999995, 1e-9));
    // Thrust in body axes and mass both hold over a step, so only
    // integration truncation varies.
    assert_rows(&rows);
}

#[test]
fn a_capsule_trimming_in_uniform_air_ends_where_it_was_recorded() {
    let body = BodyEllipsoid {
        mu_m3_s2: 0.0,
        omega_rad_s: 0.0,
        ..BodyEllipsoid::WGS84
    };
    let start = VehicleState {
        velocity_ecef: DVec3::new(7_500.0, 0.0, 0.0),
        ..rest_at(&body, 80_000.0)
    };
    let mut block = RigidBody6DofBlock::new(
        config(body, TEN_MS, start),
        capsule_tree(DVec3::new(-1.15, 0.0, -0.112)),
        capsule_aero(DVec3::new(-1.15, 0.0, -0.112)),
        Box::new(super::aero::UniformAir {
            density_kg_m3: 2e-4,
            wind_ned_m_s: DVec3::ZERO,
        }),
    )
    .unwrap();
    run(&mut block, 3_000, &RigidBodyInput::default());
    let force = block.aero_wrench().force_body_n;
    let mut rows = state_rows(&block, &CAPSULE_TRIM, [1e-4, 1e-6, 1e-7, 1e-8]);
    rows.extend(vec_rows(
        ["fx", "fy", "fz"],
        force,
        [-69354.96500514561, 0.0, 2285.5277270168644],
        1e-3,
    ));
    // A lightly damped pitch oscillation thirty seconds in: 1e-7 rad of its
    // phase is what the truncation of a fourth-order step at 100 Hz leaves.
    assert_rows(&rows);
}
