//! Recorded trajectories of the multibody vehicle block: end states of fixed
//! scenarios, held to stated tolerances so a change of integrator or force
//! evaluation shows up as a number.

use super::*;
use crate::recorded::{Row, assert_rows, vec_rows};

fn state_rows(state: &VehicleState, recorded: &[[f64; 3]; 3], tol: [f64; 3]) -> Vec<Row> {
    let attitude = state.attitude * DVec3::X;
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
        attitude,
        recorded[2],
        tol[2],
    ));
    rows
}

#[test]
fn a_capsule_arc_through_the_upper_atmosphere_ends_where_it_was_recorded() {
    let (mut rigid, mut multibody) = capsule_pair();
    for _ in 0..100 {
        rigid.step(&RigidBodyInput::default()).unwrap();
        multibody.step(&MultibodyVehicleInput::default()).unwrap();
    }
    // The capsule's pitch damping reads the inertial rate of its link, the
    // planet's spin included: 4.5e-5 rad/s across the capsule's axis here,
    // 5e-4 N·m of damping moment, 2e-7 rad of attitude over this second
    // against damping on the rate relative to the air. A micrometre of
    // trajectory does not show it.
    let tolerance = [1e-6, 1e-6, 1e-6];
    let mut rows = state_rows(&multibody.output(), &CAPSULE_MULTIBODY, tolerance);
    rows.extend(state_rows(&rigid.output(), &CAPSULE_RIGID, tolerance));
    assert_rows(&rows);
}

const CAPSULE_MULTIBODY: [[f64; 3]; 3] = [
    [
        923_445.468_584_342_2,
        -5_577_366.675_032_665,
        3_049_415.243_432_878_5,
    ],
    [
        92.325_900_528_544_3,
        157.443_115_917_951_99,
        238.143_047_578_745_04,
    ],
    [
        0.313_797_877_867_905_03,
        0.539_400_445_925_005_5,
        0.781_394_939_055_472_3,
    ],
];
const CAPSULE_RIGID: [[f64; 3]; 3] = [
    [
        923_445.468_584_342_6,
        -5_577_366.675_032_642,
        3_049_415.243_432_878_5,
    ],
    [
        92.325_900_528_530_9,
        157.443_115_917_941_75,
        238.143_047_578_732_96,
    ],
    [
        0.313_797_877_867_677_54,
        0.539_400_445_924_998_6,
        0.781_394_939_055_567_9,
    ],
];

#[test]
fn a_driven_arm_on_a_floating_bus_ends_where_it_was_recorded() {
    let mut block = floating_arm_block();
    let input = MultibodyVehicleInput {
        joint_drives: vec![mbd_multibody::JointDriveInput::scalar(ComponentId(1), 15.0)],
        ..MultibodyVehicleInput::default()
    };
    for _ in 0..80 {
        block.step(&input).unwrap();
    }
    let joint = block.tree().state(ComponentId(1)).unwrap().joint;
    let base = block.floating_base_fixed();
    // A constant torque on a revolute joint with no field around it leaves
    // only the round-off of a fourth-order step to vary.
    assert_rows(&[
        (
            "joint_angle",
            joint.q_scalar,
            0.092_685_467_056_574_92,
            1e-10,
        ),
        (
            "joint_rate",
            joint.q_vel_scalar,
            0.231_603_384_336_504_6,
            1e-10,
        ),
        (
            "base_rate_z",
            base.angular_vel_root_rad_s.z,
            -0.059_085_897_913_275_72,
            1e-10,
        ),
        (
            "base_turn_w",
            base.rotation_world_from_root.w,
            0.999_930_049_826_009_3,
            1e-10,
        ),
        (
            "base_vel_y",
            base.linear_vel_root_m_s.y,
            -0.007_563_895_563_945_003,
            1e-10,
        ),
    ]);
}

#[test]
fn a_lunar_ascent_burn_from_rest_ends_where_it_was_recorded() {
    let pad = Geodetic::from_degrees(0.0, 0.0, 1.0);
    let mut block = lunar_lander_block(pad);
    block.step(&MultibodyVehicleInput::default()).unwrap();
    let burn = MultibodyVehicleInput {
        propulsion: PropulsionCommands {
            liquid_throttles: vec![1.0],
            ..PropulsionCommands::default()
        },
        ..MultibodyVehicleInput::default()
    };
    for _ in 0..25 {
        block.step(&burn).unwrap();
    }
    let state = block.output();
    let height = BodyEllipsoid::MOON_IAU
        .fixed_to_geodetic(state.position_ecef)
        .height_m;
    let up = state.position_ecef.normalize();
    // Thrust and mass hold over each step, so the climb is a chain of
    // exactly integrated arcs; only round-off varies.
    assert_rows(&[
        ("height_m", height, 1.964_820_946_566_760_5, 1e-6),
        (
            "climb_m_s",
            state.velocity_ecef.dot(up),
            4.024_083_027_696_806,
            1e-6,
        ),
        (
            "propellant_kg",
            block.telemetry().total_propellant_kg,
            198.032_701_839_913_73,
            1e-9,
        ),
    ]);
}

fn landed(
    body: BodyEllipsoid,
    pos: Geodetic,
    seconds: u64,
) -> (MultibodyVehicleBlock, VehicleState) {
    let mut block = footpad_lander(body, pos, footpad_set());
    let start = block.output();
    for _ in 0..seconds * 50 {
        block.step(&MultibodyVehicleInput::default()).unwrap();
    }
    (block, start)
}

#[test]
fn a_lander_dropped_on_its_footpads_settles_where_it_was_recorded() {
    let body = BodyEllipsoid::MOON_IAU;
    let (block, start) = landed(body, Geodetic::from_degrees(0.0, 0.0, 1.3), 6);
    let state = block.output();
    let height = body.fixed_to_geodetic(state.position_ecef).height_m;
    let slid = (state.position_ecef - start.position_ecef)
        .reject_from(start.position_ecef.normalize())
        .length();
    assert!(block.telemetry().in_ground_contact);
    // The settled height is the static deflection of the pads, which no
    // detail of integration moves; the bounce before it shifts by a fraction
    // of the 1 cm damping ramp with how finely a step resolves contact.
    assert_rows(&[
        ("rest_height_m", height, 1.073_584_486_031_904_8, 2e-3),
        ("slid_m", slid, 0.0, 1e-3),
        ("speed_m_s", state.velocity_ecef.length(), 0.0, 1e-6),
    ]);
}

#[test]
fn a_lander_standing_on_the_rotating_earth_stays_on_its_spot() {
    let body = BodyEllipsoid::WGS84;
    let (block, start) = landed(body, Geodetic::from_degrees(35.0, -117.0, 1.0), 6);
    let state = block.output();
    let moved = (state.position_ecef - start.position_ecef)
        .reject_from(start.position_ecef.normalize())
        .length();
    let height = body.fixed_to_geodetic(state.position_ecef).height_m;
    assert!(block.telemetry().in_ground_contact);
    // The ground moves at 380 m/s in inertial axes here: a pad that did not
    // ride with it would be dragged metres in the first second.
    assert_rows(&[
        ("rest_height_m", height, 1.024_548_886_171_391_8, 2e-3),
        ("moved_m", moved, 0.0, 1e-3),
        ("speed_m_s", state.velocity_ecef.length(), 0.0, 1e-3),
    ]);
}
