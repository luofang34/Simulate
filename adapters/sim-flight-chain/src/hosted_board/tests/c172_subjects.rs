//! The C172 on Aviate's fixed-wing law through its own sensors. The bounds
//! are wide: they say the aeroplane is flown, not how well the law is tuned.

use std::f64::consts::FRAC_PI_2;

use sim_aviate::{AviateDirectConfig, AviateDirectRunner};

use super::*;

fn mean(values: impl Iterator<Item = f64>) -> f64 {
    let (sum, count) = values.fold((0.0, 0_u32), |(s, n), v| (s + v, n + 1));
    sum / f64::from(count.max(1))
}

/// The last third of a flight, where a step has had time to settle.
fn settled(probes: &[Probe]) -> &[Probe] {
    &probes[probes.len() * 2 / 3..]
}

fn indicated(probes: &[Probe]) -> f64 {
    mean(probes.iter().map(|p| f64::from(p.indicated_m_s)))
}

fn height(probes: &[Probe]) -> f64 {
    mean(probes.iter().map(|p| p.height_m))
}

fn course_deg(probes: &[Probe]) -> f64 {
    mean(probes.iter().map(|p| p.course_rad)).to_degrees()
}

#[test]
fn the_c172_holds_cruise_on_its_own_sensors() {
    let mut board = c172_board(3);
    let armed_after_s = boot_and_arm(&mut board);
    assert!(
        armed_after_s < 2.0,
        "an air start seeds within two seconds, took {armed_after_s}"
    );
    let mut sequence = 0;
    let cruise = fly(
        &mut board,
        &mut sequence,
        60.0,
        0.0,
        CRUISE_M_S,
        START_HEIGHT_M,
    );
    assert!(
        cruise
            .iter()
            .all(|p| p.mode == ControlMode::VelocityControl)
    );
    assert!(
        cruise
            .iter()
            .all(|p| (p.height_m - START_HEIGHT_M).abs() < 10.0),
        "height wandered: {:?}",
        cruise.iter().map(|p| p.height_m).fold(f64::NAN, f64::min)
    );
    assert!(cruise.iter().all(|p| p.roll_rad.abs() < 0.1));
    let tail = settled(&cruise);
    assert!(
        (indicated(tail) - CRUISE_M_S).abs() < 1.0,
        "{}",
        indicated(tail)
    );
    assert!(course_deg(tail).abs() < 5.0, "{}", course_deg(tail));
    // Level flight at this speed takes most of the engine: a throttle near
    // idle or pinned open means the lanes are not reaching the aeroplane.
    let throttle = mean(tail.iter().map(|p| f64::from(p.lanes[0])));
    assert!((0.6..0.98).contains(&throttle), "{throttle}");
}

#[test]
fn the_c172_follows_speed_height_and_course_commands() {
    let mut board = c172_board(5);
    boot_and_arm(&mut board);
    let mut sequence = 0;
    fly(
        &mut board,
        &mut sequence,
        20.0,
        0.0,
        CRUISE_M_S,
        START_HEIGHT_M,
    );

    let slow = fly(&mut board, &mut sequence, 40.0, 0.0, 45.0, START_HEIGHT_M);
    assert!((indicated(settled(&slow)) - 45.0).abs() < 1.5);
    assert!((height(settled(&slow)) - START_HEIGHT_M).abs() < 10.0);

    let high = START_HEIGHT_M + 100.0;
    let climb = fly(&mut board, &mut sequence, 90.0, 0.0, CRUISE_M_S, high);
    assert!((height(settled(&climb)) - high).abs() < 10.0);
    assert!((indicated(settled(&climb)) - CRUISE_M_S).abs() < 1.5);

    let turn = fly(&mut board, &mut sequence, 60.0, FRAC_PI_2, CRUISE_M_S, high);
    assert!((course_deg(settled(&turn)) - 90.0).abs() < 5.0);
    assert!(
        turn.iter().all(|p| (p.height_m - high).abs() < 20.0),
        "the turn lost height"
    );
    assert!(
        turn.iter().any(|p| p.roll_rad > 0.3),
        "a right turn banks right"
    );
    assert!(turn.iter().all(|p| p.sideslip_rad.abs() < 0.15));
    assert!(settled(&turn).iter().all(|p| p.roll_rad.abs() < 0.1));
}

#[test]
fn a_silent_receiver_leaves_the_c172_flying_wings_level() {
    let mut board = c172_board(7);
    boot_and_arm(&mut board);
    let mut sequence = 0;
    fly(
        &mut board,
        &mut sequence,
        20.0,
        0.0,
        CRUISE_M_S,
        START_HEIGHT_M,
    );
    board.suite_mut().receiver_mut().set_tracking(false);
    let early = fly(
        &mut board,
        &mut sequence,
        4.0,
        0.0,
        CRUISE_M_S,
        START_HEIGHT_M,
    );
    assert!(
        early.iter().all(|p| p.mode == ControlMode::VelocityControl),
        "the estimate outlives a short gap"
    );
    let denied = fly(
        &mut board,
        &mut sequence,
        30.0,
        0.0,
        CRUISE_M_S,
        START_HEIGHT_M,
    );
    let tail = settled(&denied);
    assert!(tail.iter().all(|p| p.mode == ControlMode::Attitude));
    assert!(tail.iter().all(|p| p.roll_rad.abs() < 0.15));
    // Airspeed is held on pitch from the pitot alone.
    assert!(
        (indicated(tail) - CRUISE_M_S).abs() < 3.0,
        "{}",
        indicated(tail)
    );
}

/// The command the flight controller receives on `step` of the replayed
/// flight: armed after a second, then a course change half way.
fn scripted_command(step: u64) -> Option<SystemCommand> {
    if step == RATE_HZ {
        return Some(SystemCommand::Arm);
    }
    if step < RATE_HZ || !step.is_multiple_of(RATE_HZ / 5) {
        return None;
    }
    let course = if step < 8 * RATE_HZ { 0.0 } else { 0.5 };
    Some(velocity(
        u32::try_from(step).unwrap(),
        course,
        CRUISE_M_S,
        START_HEIGHT_M,
    ))
}

#[test]
fn recorded_samples_alone_reproduce_the_actuator_commands() {
    let mut board = c172_board(11);
    let mut record = Vec::new();
    for step in 0..16 * RATE_HZ {
        if let Some(command) = scripted_command(step) {
            board.send_command(command);
        }
        let at = board.now();
        board.step().unwrap();
        record.push((at, *board.last_samples(), *board.last_command()));
    }
    assert!(
        record
            .iter()
            .any(|(_, _, command)| command.outputs[1] > 0.6),
        "the flight being replayed turned"
    );

    // A second flight controller that is given the recorded samples and
    // never an airframe: equal commands mean the samples are all the first
    // one saw of the world.
    let kernel = build_direct_fixed_wing_kernel(&c172_airframe()).unwrap();
    let config = AviateDirectConfig {
        decimal_year: 2026.5,
        ..AviateDirectConfig::ideal(ground())
    };
    let mut replay = Box::new(AviateDirectRunner::new(kernel, config).unwrap());
    for (step, (at, samples, flown)) in (0_u64..).zip(&record) {
        if let Some(command) = scripted_command(step) {
            replay.send_command(command);
        }
        let command = replay.step_sensors(*at, samples).unwrap();
        assert_eq!(command.armed, flown.armed, "step {step}");
        assert_eq!(command.outputs, flown.outputs, "step {step}");
    }
}
