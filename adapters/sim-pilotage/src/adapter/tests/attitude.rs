use glam::{DQuat, DVec3, EulerRot};
use pilotage_protocol::AttitudeThrustIntent;

use super::*;
use crate::adapter::control::ControlLaw;

/// The start heading, east, so a level attitude command holds course.
const HEADING_RAD: f64 = PI / 2.0;

fn attitude(roll: f64, pitch: f64, yaw: f64, thrust: f32) -> AttitudeThrustIntent {
    let q = DQuat::from_euler(EulerRot::ZYX, yaw, pitch, roll);
    AttitudeThrustIntent {
        frame: ReferenceFrame::LocalNed,
        qw: q.w as f32,
        qx: q.x as f32,
        qy: q.y as f32,
        qz: q.z as f32,
        thrust,
    }
}

fn attitude_command(intent: AttitudeThrustIntent) -> ScopedControlFrame {
    command(Some(ControlIntent::AttitudeThrust(intent)))
}

#[test]
fn the_motion_scope_advertises_ned_attitude_beside_body_rates() {
    let capabilities = kinematic().capabilities();
    let intents = &capabilities.vehicles[0].scopes[0].intents;
    let families: Vec<_> = intents.iter().map(|intent| intent.family).collect();
    assert_eq!(
        families,
        [IntentFamily::BodyRate, IntentFamily::AttitudeThrust]
    );
    let attitude = &intents[1];
    assert_eq!(attitude.frames, [ReferenceFrame::LocalNed]);
    assert!((attitude.max_angular - 1.05).abs() < 1e-6);
}

#[test]
fn an_attitude_inside_the_envelope_is_accepted() {
    let mut adapter = kinematic();
    let outcome = adapter.apply_control(&attitude_command(attitude(0.3, 0.1, HEADING_RAD, 0.6)));
    assert_eq!(outcome.disposition, Disposition::Accepted);
}

#[test]
fn an_attitude_past_the_envelope_is_clamped_and_says_so() {
    for intent in [
        attitude(1.4, 0.0, HEADING_RAD, 0.5),
        attitude(-1.4, 0.0, HEADING_RAD, 0.5),
        attitude(0.0, 0.9, HEADING_RAD, 0.5),
        attitude(0.0, 0.0, HEADING_RAD, 1.5),
    ] {
        let mut adapter = kinematic();
        assert_eq!(
            adapter.apply_control(&attitude_command(intent)).disposition,
            Disposition::Constrained
        );
        adapter.step(StepBudget { ticks: 240 });
        let state = adapter.vehicle_state();
        assert!(state.position_ecef.is_finite() && state.attitude.is_finite());
    }
    // Clamped to 60 degrees of bank, the vehicle settles there, not at 80.
    let mut adapter = kinematic();
    adapter.apply_control(&attitude_command(attitude(1.4, 0.0, HEADING_RAD, 0.5)));
    adapter.step(StepBudget { ticks: 600 });
    let bank = bank_rad(&adapter.vehicle_state());
    assert!((bank - 1.05).abs() < 0.1, "{bank}");
}

#[test]
fn an_attitude_that_cannot_be_flown_is_refused() {
    let nan = AttitudeThrustIntent {
        qx: f32::NAN,
        ..attitude(0.0, 0.0, HEADING_RAD, 0.5)
    };
    let body = AttitudeThrustIntent {
        frame: ReferenceFrame::BodyFrd,
        ..attitude(0.0, 0.0, HEADING_RAD, 0.5)
    };
    let empty = AttitudeThrustIntent {
        qw: 0.0,
        qx: 0.0,
        qy: 0.0,
        qz: 0.0,
        ..attitude(0.0, 0.0, HEADING_RAD, 0.5)
    };
    let mut adapter = kinematic();
    for intent in [nan, body, empty] {
        assert!(matches!(
            adapter.apply_control(&attitude_command(intent)).disposition,
            Disposition::Rejected(RejectReason::Other(_))
        ));
    }
}

#[test]
fn a_bank_command_demands_a_positive_roll_rate() {
    let mut law = ControlLaw::default();
    let mut armed = true;
    let intent = attitude(0.4, 0.0, HEADING_RAD, 0.5);
    let outcome = law.apply(&attitude_command(intent), &mut armed);
    assert_eq!(outcome.disposition, Disposition::Accepted);
    let level = DQuat::from_euler(EulerRot::ZYX, HEADING_RAD, 0.0, 0.0);
    let stick = law.controls(DVec3::ZERO, level, 1.0 / 120.0, armed);
    assert!(stick.roll > 0.0, "{stick:?}");
    // The f32 wire quaternion leaves a heading error of order 1e-8 rad.
    assert!(
        stick.pitch.abs() < 1e-6 && stick.yaw.abs() < 1e-6,
        "{stick:?}"
    );
    assert!((stick.throttle - 0.5).abs() < 1e-9);
}

/// The same attitude must be flown on both dynamics sources, though one reads
/// stick as rate and the other as surface deflection.
#[test]
fn a_bank_command_rolls_into_and_holds_the_bank() {
    let intent = attitude(0.4, 0.0, HEADING_RAD, 0.5);
    let mut kinematic = kinematic();
    kinematic.apply_control(&attitude_command(intent));
    kinematic.step(StepBudget { ticks: 6 });
    assert!(kinematic.vehicle_state().angular_rate_body.x > 0.0);
    kinematic.step(StepBudget { ticks: 474 });
    let bank = bank_rad(&kinematic.vehicle_state());
    assert!((bank - 0.4).abs() < 0.05, "{bank}");

    let mut trainer = trainer();
    trainer.apply_control(&attitude_command(intent));
    trainer.step(StepBudget { ticks: 480 });
    let bank = bank_rad(&trainer.vehicle_state());
    assert!((bank - 0.4).abs() < 0.15, "{bank}");
}
