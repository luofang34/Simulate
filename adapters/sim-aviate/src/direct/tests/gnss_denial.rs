use aviate_core::control::{ControlMode, SensorOverrides};
use aviate_core::sensor::GnssHealth;
use aviate_core::state::StateValidFlags;
use aviate_xil_direct::DirectSensorRates;

use super::*;

/// How the flight controller is denied GNSS.
#[derive(Clone, Copy, Debug)]
enum Denial {
    /// The typed command carries `gnss_force_state = Lost`.
    ForcedLost,
    /// The synthesised receiver stops producing fixes.
    SilentReceiver,
}

struct Observation {
    seconds_denied: f64,
    flags: StateValidFlags,
    effective: ControlMode,
    truth_up_m: f64,
    estimate_error_m: f64,
}

fn hold_with(sequence: u32, z_ned: f32, denial: Option<Denial>) -> SystemCommand {
    let SystemCommand::FlightControl(mut cmd) = hold_cmd(sequence, z_ned) else {
        panic!("hold_cmd is a flight-control command");
    };
    if matches!(denial, Some(Denial::ForcedLost)) {
        cmd.sensor_overrides = Some(SensorOverrides {
            gnss_force_state: Some(GnssHealth::Lost),
        });
    }
    SystemCommand::FlightControl(cmd)
}

/// Hovers on PositionHold with GNSS, denies it, and samples the estimator and
/// the mode gate every half second.
fn fly_then_deny(denial: Denial, noise: DirectNoiseTier) -> Vec<Observation> {
    let origin = Geodetic::from_degrees(39.1582, -75.5244, 15.0);
    let period = TickSpan::from_ticks(TICKS_PER_SECOND / 400);
    let mut plant = x500_rigid_body(&origin, period);
    let rotors = RotorLayout::x500();
    let mut runner = AviateDirectRunner::new(
        build_direct_x500_kernel().unwrap(),
        AviateDirectConfig {
            noise,
            ..AviateDirectConfig::ideal(origin)
        },
    )
    .unwrap();
    let g_rest = DVec3::new(0.0, 0.0, -9.801);
    let mut now = SimInstant::START;
    for _ in 0..800 {
        let _ = runner.step_cg(now, &plant.output(), g_rest).unwrap();
        now = now.advanced(period);
    }
    assert!(runner.is_ready());
    runner.send_command(hold_with(1, -4.0, None));
    runner.send_command(SystemCommand::Arm);

    let hover_steps = 3_200_u32;
    let denied_steps = 4_000_u32;
    let mut seq = 1_u32;
    let mut out = Vec::new();
    for k in 0..hover_steps + denied_steps {
        let denied = k >= hover_steps;
        if k == hover_steps && matches!(denial, Denial::SilentReceiver) {
            runner.board_mut().set_sensor_rates(DirectSensorRates {
                gnss_period_us: u64::MAX,
                ..DirectSensorRates::default()
            });
        }
        if k % 40 == 0 {
            seq = seq.wrapping_add(1);
            runner.send_command(hold_with(seq, -4.0, denied.then_some(denial)));
        }
        let stim = AviateStimulus {
            vehicle: plant.output(),
            site: plant.site_kinematics(ComponentId(0)).unwrap(),
        };
        let act = runner.step_site(now, &stim).unwrap();
        plant
            .step(&RigidBodyInput {
                thrusters: rotors.direct_thrusters(&act).unwrap(),
                ..RigidBodyInput::default()
            })
            .unwrap();
        now = now.advanced(period);
        if k % 200 == 0 {
            let board = runner.board();
            let estimate = board.kernel().state.estimator.get_estimate();
            let truth = plant.output();
            let est = runner.world.vehicle_from_estimate(&estimate);
            out.push(Observation {
                seconds_denied: (f64::from(k) - f64::from(hover_steps)) / 400.0,
                flags: estimate.valid_flags,
                effective: board.last_effective_command().mode,
                truth_up_m: sim_geo::ecef_to_geodetic(truth.position_ecef).height_m
                    - origin.height_m,
                estimate_error_m: (est.position_ecef - truth.position_ecef).length(),
            });
        }
    }
    out
}

fn assert_denial_drops_navigation(denial: Denial) {
    let timeline = fly_then_deny(denial, DirectNoiseTier::Industrial);
    let nav = StateValidFlags::POSITION | StateValidFlags::VELOCITY;
    let before = timeline
        .iter()
        .filter(|o| o.seconds_denied < 0.0 && o.seconds_denied > -4.0);
    for o in before {
        assert!(
            o.flags.contains(nav),
            "{denial:?} at {:+.1} s",
            o.seconds_denied
        );
        assert_eq!(o.effective, ControlMode::PositionHold);
        assert!(o.estimate_error_m < 2.0, "{} m", o.estimate_error_m);
    }
    let after = timeline.iter().filter(|o| o.seconds_denied > 6.0);
    for o in after {
        assert!(
            !o.flags.intersects(nav),
            "{denial:?} at {:+.1} s",
            o.seconds_denied
        );
        assert_eq!(o.effective, ControlMode::Attitude);
        assert!(
            o.truth_up_m < 4.2,
            "no closed-loop collective: {} m",
            o.truth_up_m
        );
    }
}

#[test]
fn forced_gnss_loss_drops_navigation_validity_and_falls_back_to_attitude() {
    assert_denial_drops_navigation(Denial::ForcedLost);
}

#[test]
fn silent_gnss_receiver_drops_navigation_validity_and_falls_back_to_attitude() {
    assert_denial_drops_navigation(Denial::SilentReceiver);
}
