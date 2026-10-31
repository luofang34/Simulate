use aviate_xil_contract::FcState;

use super::*;
use crate::MotorCommand;
use crate::wire::{new_shared_state, read_motor_command};

const FULL: f64 = X500_MAX_SPEED_RAD_S;

fn motors(speeds: &[f64]) -> MotorCommand {
    let mut shared = new_shared_state(1);
    shared.command.num_motors = speeds.len() as u32;
    for (lane, speed) in shared.command.motor_vel_bits.iter_mut().zip(speeds) {
        *lane = speed.to_bits();
    }
    read_motor_command(&mut shared).unwrap()
}

fn ready(speeds: &[f64]) -> FlightControllerCommand {
    FlightControllerCommand {
        sim_step: 1,
        motors: motors(speeds),
        fc_state: FcState::Ready,
        fc_ready: true,
    }
}

fn total_force(thrusters: &[ThrusterWrench]) -> DVec3 {
    thrusters.iter().map(|t| t.force_body_n).sum()
}

/// Moment about the root origin, N·m.
fn total_moment(thrusters: &[ThrusterWrench]) -> DVec3 {
    thrusters
        .iter()
        .map(|t| t.application_point_body_m.cross(t.force_body_n) + t.moment_body_nm)
        .sum()
}

#[test]
fn thrust_follows_the_square_of_speed_upward_along_body_z() {
    let x500 = RotorLayout::x500();
    let thrusters = x500.thrusters(&ready(&[500.0; 4])).unwrap();
    let expected = 4.0 * X500_MOTOR_CONSTANT * 500.0 * 500.0;
    let force = total_force(&thrusters);
    assert!((force.z + expected).abs() < 1e-9, "{force}");
    assert_eq!((force.x, force.y), (0.0, 0.0));
}

#[test]
fn equal_speeds_are_balanced_about_every_axis() {
    let thrusters = RotorLayout::x500().thrusters(&ready(&[700.0; 4])).unwrap();
    assert!(total_moment(&thrusters).length() < 1e-12);
}

#[test]
fn more_left_thrust_rolls_right() {
    let moment = total_moment(
        &RotorLayout::x500()
            .thrusters(&ready(&[400.0, 800.0, 800.0, 400.0]))
            .unwrap(),
    );
    assert!(moment.x > 0.0, "{moment}");
    assert!(moment.y.abs() < 1e-9 && moment.z.abs() < 1e-9, "{moment}");
}

#[test]
fn more_front_thrust_pitches_nose_up() {
    let moment = total_moment(
        &RotorLayout::x500()
            .thrusters(&ready(&[800.0, 400.0, 800.0, 400.0]))
            .unwrap(),
    );
    assert!(moment.y > 0.0, "{moment}");
    assert!(moment.x.abs() < 1e-9 && moment.z.abs() < 1e-9, "{moment}");
}

#[test]
fn more_counter_clockwise_rotor_yaws_right() {
    let moment = total_moment(
        &RotorLayout::x500()
            .thrusters(&ready(&[800.0, 800.0, 400.0, 400.0]))
            .unwrap(),
    );
    assert!(moment.z > 0.0, "{moment}");
    assert!(moment.x.abs() < 1e-9 && moment.y.abs() < 1e-9, "{moment}");
}

#[test]
fn commands_saturate_at_the_rotor_limit_and_never_reverse() {
    let x500 = RotorLayout::x500();
    let limit = x500.thrusters(&ready(&[FULL; 4])).unwrap();
    let over = x500.thrusters(&ready(&[5.0 * FULL; 4])).unwrap();
    assert_eq!(limit, over);
    let reversed = x500.thrusters(&ready(&[-300.0; 4])).unwrap();
    assert_eq!(total_force(&reversed).length(), 0.0);
}

#[test]
fn a_controller_not_ready_leaves_the_rotors_stopped_whatever_its_lanes_hold() {
    let command = FlightControllerCommand {
        fc_state: FcState::Converging,
        fc_ready: false,
        ..ready(&[900.0; 4])
    };
    let x500 = RotorLayout::x500();
    let thrusters = x500.thrusters(&command).unwrap();
    assert_eq!(thrusters.len(), 4);
    assert_eq!(total_force(&thrusters).length(), 0.0);
    assert_eq!(total_moment(&thrusters).length(), 0.0);
    assert_eq!(
        x500.control_inputs(&command).unwrap(),
        ControlInputs::default()
    );
}

#[test]
fn a_ready_controller_on_the_wrong_airframe_is_refused() {
    assert_eq!(
        RotorLayout::x500().thrusters(&ready(&[700.0; 6])),
        Err(AviateBridgeError::RotorCountMismatch {
            commanded: 6,
            fitted: 4
        }),
    );
    assert_eq!(
        RotorLayout::x500().control_inputs(&ready(&[700.0; 2])),
        Err(AviateBridgeError::RotorCountMismatch {
            commanded: 2,
            fitted: 4
        }),
    );
}

#[test]
fn control_inputs_read_mean_thrust_as_throttle() {
    let x500 = RotorLayout::x500();
    let full = x500.control_inputs(&ready(&[FULL; 4])).unwrap();
    assert_eq!(
        full,
        ControlInputs {
            throttle: 1.0,
            ..ControlInputs::default()
        }
    );
    let half = x500
        .control_inputs(&ready(&[FULL * 0.5_f64.sqrt(); 4]))
        .unwrap();
    assert!((half.throttle - 0.5).abs() < 1e-12);
}

#[test]
fn control_inputs_reach_full_scale_at_the_physical_extremes() {
    let x500 = RotorLayout::x500();
    // Lane order: front right, rear left, front left, rear right.
    for (speeds, roll, pitch, yaw) in [
        ([0.0, FULL, FULL, 0.0], 1.0, 0.0, 0.0),
        ([FULL, 0.0, 0.0, FULL], -1.0, 0.0, 0.0),
        ([FULL, 0.0, FULL, 0.0], 0.0, 1.0, 0.0),
        ([0.0, FULL, 0.0, FULL], 0.0, -1.0, 0.0),
        ([FULL, FULL, 0.0, 0.0], 0.0, 0.0, 1.0),
        ([0.0, 0.0, FULL, FULL], 0.0, 0.0, -1.0),
    ] {
        let inputs = x500.control_inputs(&ready(&speeds)).unwrap();
        assert!((inputs.roll - roll).abs() < 1e-12, "{speeds:?} {inputs:?}");
        assert!(
            (inputs.pitch - pitch).abs() < 1e-12,
            "{speeds:?} {inputs:?}"
        );
        assert!((inputs.yaw - yaw).abs() < 1e-12, "{speeds:?} {inputs:?}");
        assert!(
            (inputs.throttle - 0.5).abs() < 1e-12,
            "{speeds:?} {inputs:?}"
        );
    }
}

#[test]
fn control_inputs_on_a_coaxial_stack_have_no_arm_to_divide_by() {
    let rotor = Rotor {
        position_body_m: DVec3::ZERO,
        spin: RotorSpin::Clockwise,
        motor_constant: 1e-5,
        moment_constant_m: 0.01,
        max_speed_rad_s: 100.0,
    };
    let inputs = RotorLayout::new(vec![rotor])
        .control_inputs(&ready(&[100.0]))
        .unwrap();
    assert_eq!((inputs.roll, inputs.pitch), (0.0, 0.0));
    assert_eq!(inputs.yaw, -1.0);
    assert_eq!(inputs.throttle, 1.0);
}
