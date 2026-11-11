//! A stress case for the C172 on its own sensors: an uncalibrated
//! accelerometer and a steep turn the moment the flight controller is armed.
//!
//! The case is one that flew the aeroplane into the ground on an earlier
//! fixed-wing law: the route's first waypoint lay under the arming point,
//! the mission turned about 200 degrees right at a 45-degree bank straight
//! after arming, and the filter's pitch ran away under this accelerometer
//! bias. The bias is set as the vector that unit had, and the bank is
//! commanded directly, since the route bank is shallower now.

use std::f64::consts::{PI, TAU};

use aviate_core::math::{Quaternion, Vector3};
use aviate_core::types::NormalizedThrust;

use super::*;
use crate::{AccelCalibration, DeviceOutages, c172_sensor_suite_with};

/// The accelerometer bias of the unit that crashed, m/s², body axes.
const CRASH_ACCEL_BIAS_M_S2: DVec3 = DVec3::new(-0.578, -0.224, -0.360);
/// The bank of the turn, degrees, right wing down.
const BANK_DEG: f64 = 45.0;
/// The course change the turn flies through, degrees.
const TURN_DEG: f64 = 200.0;
/// Throttle added to the trimmed setting for the turn's extra drag.
const TURN_THROTTLE_ADDED: f64 = 0.15;
/// How long after arming the case is watched, seconds.
const WATCH_S: f64 = 60.0;

/// How long the flight controller may take to become ready, seconds.
const BOOT_LIMIT_S: f64 = 10.0;

/// Boots and arms the board; false when it was not ready in `limit_s`.
fn boot_and_arm_within(board: &mut C172Board, limit_s: f64) -> bool {
    let mut steps = 0_u64;
    while !board.controller().is_ready() {
        if steps as f64 >= limit_s * RATE_HZ as f64 {
            return false;
        }
        board.step().unwrap();
        steps = steps.wrapping_add(1);
    }
    board.send_command(SystemCommand::Arm);
    board.step().unwrap();
    board.controller().is_armed()
}

/// What the case showed.
#[derive(Debug, Default)]
struct Watched {
    /// The largest angle between the estimated and the true attitude.
    worst_attitude_error_deg: f64,
    /// When that was, seconds after arming.
    worst_at_s: f64,
    /// Estimated and true pitch, degrees, 8 s after arming.
    pitch_at_8_s: (f64, f64),
    /// Estimated and true pitch, degrees, 20 s after arming.
    pitch_at_20_s: (f64, f64),
    /// The largest difference of estimated and true pitch, degrees.
    worst_pitch_error_deg: f64,
    /// The largest difference of estimated and true heading, degrees.
    worst_heading_error_deg: f64,
    /// The worst roll, pitch and heading error, degrees, while turning.
    turn_rph_deg: [f64; 3],
    /// The same after the roll-out.
    after_rph_deg: [f64; 3],
    /// The lowest and highest height above the board's origin, metres.
    lowest_m: f64,
    highest_m: f64,
    /// When the turn was rolled out of, seconds after arming.
    rolled_out_s: Option<f64>,
}

fn heading_deg(attitude: glam::DQuat, at: DVec3) -> f64 {
    let frame = LocalFrame::at(&ecef_to_geodetic(at));
    let forward = frame.ecef_to_ned_vector(attitude * DVec3::X);
    forward.y.atan2(forward.x).to_degrees()
}

fn roll_deg(attitude: glam::DQuat, at: DVec3) -> f64 {
    let frame = LocalFrame::at(&ecef_to_geodetic(at));
    let right = frame.ecef_to_ned_vector(attitude * DVec3::Y);
    let down = frame.ecef_to_ned_vector(attitude * DVec3::Z);
    right.z.atan2(down.z).to_degrees()
}

fn pitch_deg(attitude: glam::DQuat, at: DVec3) -> f64 {
    let frame = LocalFrame::at(&ecef_to_geodetic(at));
    let forward = frame.ecef_to_ned_vector(attitude * DVec3::X);
    (-forward.z).clamp(-1.0, 1.0).asin().to_degrees()
}

fn bank_command(sequence: u32, roll_rad: f64, pitch_rad: f64, throttle: f64) -> SystemCommand {
    let about =
        |x, y, z, angle: f64| Quaternion::from_axis_angle(Vector3::new(x, y, z), angle as f32);
    SystemCommand::FlightControl(Command {
        mode: ControlMode::Attitude,
        setpoint: Setpoint {
            attitude: Some(about(0.0, 1.0, 0.0, pitch_rad).mul(&about(1.0, 0.0, 0.0, roll_rad))),
            collective_thrust: NormalizedThrust(throttle as f32),
            ..Setpoint::default()
        },
        config_mode_request: None,
        sensor_overrides: None,
        sequence,
        source: CommandSource::Gcs,
    })
}

/// The command of the moment: the bank until the turn is round, then the
/// new course level at cruise.
fn command_now(
    sequence: u32,
    rolled_out: bool,
    (trim_pitch_rad, trim_throttle): (f64, f64),
    start_course_rad: f64,
) -> SystemCommand {
    if rolled_out {
        return velocity(
            sequence,
            start_course_rad + TURN_DEG.to_radians(),
            CRUISE_M_S,
            START_HEIGHT_M,
        );
    }
    bank_command(
        sequence,
        BANK_DEG.to_radians(),
        trim_pitch_rad,
        (trim_throttle + TURN_THROTTLE_ADDED).min(1.0),
    )
}

/// The angle between the estimated and the true attitude, degrees, and the
/// two pitches.
fn attitude_error(board: &C172Board) -> Option<(f64, (f64, f64), f64)> {
    let truth = board.true_state();
    let estimate = board.estimated_state()?;
    let angle = (estimate.attitude.inverse() * truth.attitude)
        .to_axis_angle()
        .1;
    let pitches = (
        pitch_deg(estimate.attitude, truth.position_ecef),
        pitch_deg(truth.attitude, truth.position_ecef),
    );
    let heading = heading_deg(estimate.attitude, truth.position_ecef)
        - heading_deg(truth.attitude, truth.position_ecef);
    let heading = (heading + 180.0).rem_euclid(360.0) - 180.0;
    Some((angle.min(TAU - angle).to_degrees(), pitches, heading.abs()))
}

/// Arms the board, banks it [`BANK_DEG`] right until the course has turned
/// [`TURN_DEG`], then flies the new course level, watching the estimate.
fn turn_on_arming(board: &mut C172Board) -> Watched {
    let start = board.true_state();
    let trim = (
        pitch_deg(start.attitude, start.position_ecef).to_radians(),
        board.airframe().trim().throttle,
    );
    if !boot_and_arm_within(board, BOOT_LIMIT_S) {
        return Watched::default();
    }
    let start_course = probe(board).course_rad;
    let mut watched = Watched {
        lowest_m: f64::INFINITY,
        ..Watched::default()
    };
    let (mut sequence, mut turned_rad, mut last_course) = (0_u32, 0.0, start_course);
    for step in 0..(WATCH_S * RATE_HZ as f64) as u64 {
        let t_s = step as f64 / RATE_HZ as f64;
        if step.is_multiple_of(RATE_HZ / 5) {
            sequence = sequence.wrapping_add(1);
            let rolled_out = watched.rolled_out_s.is_some();
            board.send_command(command_now(sequence, rolled_out, trim, start_course));
        }
        board.step().unwrap();
        let now = probe(board);
        turned_rad += (now.course_rad - last_course + PI).rem_euclid(TAU) - PI;
        last_course = now.course_rad;
        if watched.rolled_out_s.is_none() && turned_rad >= TURN_DEG.to_radians() {
            watched.rolled_out_s = Some(t_s);
        }
        watched.lowest_m = watched.lowest_m.min(now.height_m);
        watched.highest_m = watched.highest_m.max(now.height_m);
        let Some((error_deg, pitches, heading_error)) = attitude_error(board) else {
            continue;
        };
        watched.worst_pitch_error_deg = watched
            .worst_pitch_error_deg
            .max((pitches.0 - pitches.1).abs());
        watched.worst_heading_error_deg = watched.worst_heading_error_deg.max(heading_error);
        if let Some(estimate) = board.estimated_state() {
            let truth = board.true_state();
            let roll_error = (roll_deg(estimate.attitude, truth.position_ecef)
                - roll_deg(truth.attitude, truth.position_ecef))
            .abs();
            let errors = [roll_error, (pitches.0 - pitches.1).abs(), heading_error];
            let phase = match watched.rolled_out_s {
                None => &mut watched.turn_rph_deg,
                Some(_) => &mut watched.after_rph_deg,
            };
            for (worst, error) in phase.iter_mut().zip(errors) {
                *worst = worst.max(error);
            }
        }
        if error_deg > watched.worst_attitude_error_deg {
            watched.worst_attitude_error_deg = error_deg;
            watched.worst_at_s = t_s;
        }
        let at = |mark: f64| (t_s - mark).abs() < 0.5 / RATE_HZ as f64;
        if at(8.0) {
            watched.pitch_at_8_s = pitches;
        }
        if at(20.0) {
            watched.pitch_at_20_s = pitches;
        }
    }
    watched
}

/// The crash case's board: seed 21's suite with the crashed unit's
/// accelerometer bias, the receiver silent from the start when `no_gnss`.
fn crash_board(no_gnss: bool) -> C172Board {
    let seed = 21;
    let suite = c172_sensor_suite_with(2026.5, seed, AccelCalibration::Uncalibrated20Mg);
    let mut board = c172_board(
        &c172_setup(seed),
        &suite,
        SharedAir::new(StandardAtmosphere),
    )
    .unwrap();
    board
        .suite_mut()
        .imu_mut()
        .set_accel_bias_m_s2(CRASH_ACCEL_BIAS_M_S2);
    board.set_outages(DeviceOutages {
        gnss: no_gnss,
        air_data: false,
    });
    board
}

fn assert_survived(watched: &Watched) {
    assert!(watched.rolled_out_s.is_some(), "the turn never came round");
    assert!(
        watched.lowest_m > START_HEIGHT_M - 150.0,
        "down to {} m",
        watched.lowest_m
    );
    assert!(
        watched.worst_attitude_error_deg < 15.0,
        "attitude error {} deg at {} s",
        watched.worst_attitude_error_deg,
        watched.worst_at_s
    );
}

/// The crash case: the turn must stay a turn, and the estimate must keep
/// its attitude through it.
#[test]
#[ignore = "stress case; run it by name. Aviate d9f8d5b's filter loses its attitude in it"]
fn the_crash_bias_survives_a_steep_turn_on_arming() {
    tracing_subscriber::fmt().with_test_writer().try_init().ok();
    let watched = turn_on_arming(&mut crash_board(false));
    tracing::info!(?watched, "steep turn on arming");
    assert_survived(&watched);
}

/// The crash case with the receiver silent from power-up: the flight
/// controller must come up and fly the turn on its inertial and air data.
#[test]
#[ignore = "stress case; run it by name. Aviate d9f8d5b needs a fix to become ready"]
fn the_crash_bias_survives_a_steep_turn_on_arming_without_gnss() {
    tracing_subscriber::fmt().with_test_writer().try_init().ok();
    let watched = turn_on_arming(&mut crash_board(true));
    tracing::info!(?watched, "steep turn on arming without GNSS");
    assert_survived(&watched);
}
