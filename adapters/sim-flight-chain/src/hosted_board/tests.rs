use aviate_core::control::{Command, CommandSource, Setpoint};
use aviate_core::types::{Meters, MetersPerSecond};
use glam::DVec3;
use sim_atmosphere::StandardAtmosphere;
use sim_geo::{LocalFrame, UtcTime, ecef_to_geodetic};

use super::*;
use crate::{C172_RATE_HZ as RATE_HZ, C172Board, C172Setup, SharedAir, VehicleId};
use crate::{c172_board, c172_sensor_suite};

mod c172_stress;
mod c172_subjects;

/// Height of the start above the board's origin, metres.
const START_HEIGHT_M: f64 = 600.0;
const CRUISE_M_S: f64 = 55.0;

fn ground() -> Geodetic {
    Geodetic::from_degrees(39.1582, -75.5244, 15.0)
}

fn c172_setup(seed: u64) -> C172Setup {
    C172Setup {
        vehicle: VehicleId::new(1),
        origin: ground(),
        position: Geodetic {
            height_m: ground().height_m + START_HEIGHT_M,
            ..ground()
        },
        heading_rad: 0.0,
        speed_m_s: CRUISE_M_S,
        utc: UtcTime::from_unix_seconds(1_783_000_000.0),
        decimal_year: 2026.5,
        seed,
    }
}

fn c172(seed: u64) -> C172Board {
    let suite = c172_sensor_suite(2026.5, seed);
    c172_board(
        &c172_setup(seed),
        &suite,
        SharedAir::new(StandardAtmosphere),
    )
    .unwrap()
}

/// What the aeroplane is doing at one instant, from truth, and what the
/// flight controller commands.
#[derive(Debug, Clone, Copy)]
struct Probe {
    /// Height in the board's local frame: above the plane tangent at its origin.
    height_m: f64,
    roll_rad: f64,
    course_rad: f64,
    sideslip_rad: f64,
    lanes: [f32; 4],
    indicated_m_s: f32,
    mode: ControlMode,
}

fn probe(board: &C172Board) -> Probe {
    let state = board.true_state();
    let position = ecef_to_geodetic(state.position_ecef);
    let frame = LocalFrame::at(&position);
    let velocity_ned = frame.ecef_to_ned_vector(state.velocity_ecef);
    let velocity_body = state.attitude.inverse() * state.velocity_ecef;
    let right = frame.ecef_to_ned_vector(state.attitude * DVec3::Y);
    let down = frame.ecef_to_ned_vector(state.attitude * DVec3::Z);
    let origin_frame = LocalFrame::at(&ground());
    let from_origin =
        origin_frame.ecef_to_ned_vector(state.position_ecef - sim_geo::geodetic_to_ecef(&ground()));
    let lanes = board.last_command().outputs;
    Probe {
        height_m: -from_origin.z,
        roll_rad: right.z.atan2(down.z),
        course_rad: velocity_ned.y.atan2(velocity_ned.x),
        sideslip_rad: (velocity_body.y / velocity_body.length()).asin(),
        lanes: [lanes[0], lanes[1], lanes[2], lanes[3]],
        indicated_m_s: board
            .controller()
            .board()
            .kernel()
            .state
            .controller
            .air
            .indicated_mps,
        mode: board.flight_mode(),
    }
}

/// A velocity command: fly `course_rad` at `airspeed_m_s` indicated and hold
/// `height_m` above the board's origin.
fn velocity(sequence: u32, course_rad: f64, airspeed_m_s: f64, height_m: f64) -> SystemCommand {
    let (sin, cos) = course_rad.sin_cos();
    SystemCommand::FlightControl(Command {
        mode: ControlMode::VelocityControl,
        setpoint: Setpoint {
            velocity: Some([
                MetersPerSecond((airspeed_m_s * cos) as f32),
                MetersPerSecond((airspeed_m_s * sin) as f32),
                MetersPerSecond(0.0),
            ]),
            altitude: Some(Meters(height_m as f32)),
            ..Setpoint::default()
        },
        config_mode_request: None,
        sensor_overrides: None,
        sequence,
        source: CommandSource::Gcs,
    })
}

/// Steps the board until the flight controller is ready, arms it, and
/// returns the seconds that took.
fn boot_and_arm(board: &mut C172Board) -> f64 {
    let mut steps = 0_u64;
    while !board.controller().is_ready() {
        board.step().unwrap();
        steps += 1;
        assert!(
            steps < RATE_HZ * 10,
            "the flight controller never became ready"
        );
    }
    board.send_command(SystemCommand::Arm);
    board.step().unwrap();
    assert!(board.controller().is_armed());
    (steps + 1) as f64 / RATE_HZ as f64
}

/// Flies `seconds` on a velocity command refreshed five times a second and
/// returns a probe every tenth of a second.
fn fly(
    board: &mut C172Board,
    sequence: &mut u32,
    seconds: f64,
    course_rad: f64,
    airspeed_m_s: f64,
    height_m: f64,
) -> Vec<Probe> {
    let steps = (seconds * RATE_HZ as f64) as u64;
    let mut probes = Vec::new();
    for step in 0..steps {
        if step.is_multiple_of(RATE_HZ / 5) {
            *sequence = sequence.wrapping_add(1);
            board.send_command(velocity(*sequence, course_rad, airspeed_m_s, height_m));
        }
        board.step().unwrap();
        if step.is_multiple_of(RATE_HZ / 10) {
            probes.push(probe(board));
        }
    }
    probes
}
