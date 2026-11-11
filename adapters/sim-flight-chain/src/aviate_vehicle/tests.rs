use pilotage_protocol::{ControlPayload, Generation, SequenceNum, SessionId, VelocityIntent};
use pilotage_timing::MonoTimestamp;
use sim_geo::{LocalFrame, geodetic_to_ecef};

use super::*;

const VEHICLE: VehicleId = VehicleId::new(7);
const FRAME_TICKS: u32 = (FLIGHT_RATE_HZ / 20) as u32;

fn pad() -> Geodetic {
    Geodetic::from_degrees(39.1582, -75.5244, 15.0)
}

fn vehicle(noise: DirectNoiseTier) -> AviateX500Vehicle {
    AviateX500Vehicle::new(AviateVehicleConfig {
        vehicle: VEHICLE,
        pad: pad(),
        heading_rad: 0.0,
        noise,
        noise_seed: 11,
    })
    .unwrap()
}

fn frame(
    sequence: u32,
    intent: Option<ControlIntent>,
    actions: Vec<ControlAction>,
) -> ScopedControlFrame {
    ScopedControlFrame {
        timing: None,
        admitted_timing: None,
        session: SessionId::new(1),
        vehicle: VEHICLE,
        scope: ScopeId::new(MOTION_SCOPE),
        generation: Generation::new(1),
        sequence: SequenceNum::new(sequence),
        sampled_at: MonoTimestamp::from_nanos(0),
        profile_revision: 0,
        activation_revision: 0,
        payload: ControlPayload::default(),
        intent,
        action_ids: Vec::new(),
        actions,
    }
}

fn body_velocity(vx: f32, vz: f32) -> Option<ControlIntent> {
    Some(ControlIntent::Velocity(VelocityIntent {
        frame: ReferenceFrame::BodyFrd,
        vx,
        vy: 0.0,
        vz,
        yaw_rate: 0.0,
    }))
}

fn fly(vehicle: &mut AviateX500Vehicle, sequence: &mut u32, seconds: u32, vx: f32, vz: f32) {
    for _ in 0..seconds * 20 {
        *sequence = sequence.wrapping_add(1);
        let outcome = vehicle.apply_control(&frame(*sequence, body_velocity(vx, vz), Vec::new()));
        assert_eq!(outcome.disposition, Disposition::Accepted);
        assert_eq!(
            vehicle.step(StepBudget { ticks: FRAME_TICKS }).advanced,
            FRAME_TICKS
        );
    }
}

fn north_east_up(vehicle: &AviateX500Vehicle) -> (f64, f64, f64) {
    let state = vehicle.true_state();
    let frame = LocalFrame::at(&pad());
    let offset = frame.ecef_to_ned_vector(state.position_ecef - geodetic_to_ecef(&pad()));
    (offset.x, offset.y, -offset.z)
}

#[test]
fn mavlink_frames_arm_lift_and_fly_the_x500() {
    let mut vehicle = vehicle(DirectNoiseTier::Industrial);
    let arm = frame(1, None, vec![ControlAction::Arm]);
    assert_eq!(
        vehicle.apply_control(&arm).disposition,
        Disposition::Rejected(RejectReason::MeasurementUnavailable),
        "no command leaves before the flight controller reports a pose"
    );
    let mut waited = 0_u32;
    while vehicle.downlink.pose().is_none() {
        assert_eq!(vehicle.step(StepBudget { ticks: 48 }).advanced, 48);
        waited += 48;
        assert!(waited < 480 * 20, "flight controller never reported a pose");
    }
    let mut arm_attempts = 0_u32;
    while vehicle.reports_armed() != Some(true) {
        let outcome = vehicle.apply_control(&arm);
        assert_eq!(outcome.disposition, Disposition::Accepted);
        assert!(outcome.action_results.iter().all(|result| result.accepted));
        vehicle.step(StepBudget { ticks: 240 });
        arm_attempts += 1;
        assert!(arm_attempts < 40, "flight controller never armed");
    }
    assert!(
        vehicle.is_on_pad(),
        "an armed vehicle with no setpoint stays down"
    );

    let mut sequence = 1_u32;
    fly(&mut vehicle, &mut sequence, 10, 0.0, -2.0);
    fly(&mut vehicle, &mut sequence, 20, 10.0, 0.0);
    fly(&mut vehicle, &mut sequence, 8, 0.0, 0.0);
    let (north, east, up) = north_east_up(&vehicle);
    let speed = vehicle.true_state().velocity_ecef.length();
    assert_eq!(vehicle.reports_armed(), Some(true));
    assert!(up > 12.0 && up < 28.0, "climbed {up} m");
    assert!(north > 150.0 && north < 260.0, "flew {north} m north");
    assert!(east.abs() < 10.0, "drifted {east} m east");
    assert!(speed < 1.0, "still moving at {speed} m/s");
    assert_eq!(vehicle.refused_uplink_bytes(), 0);
}

fn direct_frame(sequence: u32, pitch_rad: f64, thrust: f32) -> ScopedControlFrame {
    let (sin, cos) = (pitch_rad / 2.0).sin_cos();
    let intent = ControlIntent::AttitudeThrust(pilotage_protocol::AttitudeThrustIntent {
        frame: ReferenceFrame::LocalNed,
        qw: cos as f32,
        qx: 0.0,
        qy: sin as f32,
        qz: 0.0,
        thrust,
    });
    ScopedControlFrame {
        scope: ScopeId::new(DIRECT_SCOPE),
        ..frame(sequence, Some(intent), Vec::new())
    }
}

/// Sends `direct` frames at 20 Hz for `seconds`.
fn fly_direct(vehicle: &mut AviateX500Vehicle, seconds: u32, pitch_rad: f64, thrust: f32) {
    for sequence in 0..seconds * 20 {
        let outcome = vehicle.apply_control(&direct_frame(sequence, pitch_rad, thrust));
        assert_eq!(outcome.disposition, Disposition::Accepted);
        vehicle.step(StepBudget { ticks: FRAME_TICKS });
    }
}

#[test]
fn a_controller_denied_gnss_still_flies_attitude_and_thrust() {
    let mut vehicle = vehicle(DirectNoiseTier::Industrial);
    while vehicle.downlink.pose().is_none() {
        vehicle.step(StepBudget { ticks: 48 });
    }
    let arm = frame(1, None, vec![ControlAction::Arm]);
    while vehicle.reports_armed() != Some(true) {
        vehicle.apply_control(&arm);
        vehicle.step(StepBudget { ticks: 240 });
    }
    let mut sequence = 1_u32;
    fly(&mut vehicle, &mut sequence, 10, 0.0, -2.0);
    assert_eq!(vehicle.flight_mode(), ControlMode::VelocityControl);
    let (_, _, hover_up) = north_east_up(&vehicle);

    vehicle.silence_gnss();
    let mut refused = 0_u32;
    for _ in 0..7 * 20 {
        sequence = sequence.wrapping_add(1);
        let outcome = vehicle.apply_control(&frame(sequence, body_velocity(0.0, 0.0), Vec::new()));
        if outcome.disposition == Disposition::Rejected(RejectReason::MeasurementUnavailable) {
            refused += 1;
        }
        vehicle.step(StepBudget { ticks: FRAME_TICKS });
    }
    // Its position no longer vouched for, the adapter sends no velocity; the
    // controller, hearing nothing, starts its link-loss descent.
    assert!(refused > 0, "velocity frames were still taken");
    assert_eq!(vehicle.flight_mode(), ControlMode::AltitudeHold);
    let (_, _, sinking_from) = north_east_up(&vehicle);

    fly_direct(&mut vehicle, 4, 0.0, 0.75);
    let (_, _, up) = north_east_up(&vehicle);
    assert!(
        sinking_from < hover_up,
        "descending: {sinking_from} m from {hover_up} m"
    );
    assert!(
        up > sinking_from + 3.0,
        "climbed from {sinking_from} m to {up} m on thrust"
    );

    fly_direct(&mut vehicle, 5, -10_f64.to_radians(), 0.62);
    let (north, east, _) = north_east_up(&vehicle);
    assert!(north > 10.0, "nose down flew {north} m north");
    assert!(east.abs() < 5.0, "and {east} m east");
    assert_eq!(vehicle.flight_mode(), ControlMode::Attitude);
}
