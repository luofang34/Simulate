use glam::{DQuat, DVec3};
use sim_geo::Geodetic;

use super::*;
use crate::{ControlInputs, KinematicInput, KinematicModel, TICKS_PER_SECOND};

const CAMERA: SensorChannelId = SensorChannelId(7);
const CAMERA_PERIOD: TickSpan = TickSpan::from_ticks(TICKS_PER_SECOND / 30);

fn session() -> SimSession<KinematicModel> {
    let model = KinematicModel::new(VehicleState::level_flight(
        &Geodetic::from_degrees(39.1582, -75.5244, 800.0),
        0.3,
        60.0,
    ));
    let mut session = SimSession::new(model, KinematicInput::default());
    session.register_sensor(CAMERA, CAMERA_PERIOD).unwrap();
    session
}

fn input(roll: f64, pitch: f64, yaw: f64, throttle: f64) -> KinematicInput {
    KinematicInput {
        controls: ControlInputs {
            roll,
            pitch,
            yaw,
            throttle,
        },
    }
}

/// Fifteen manoeuvres over ten seconds, deliberately stamped off the physics
/// and display grids.
fn script() -> Vec<(SimInstant, KinematicInput)> {
    (0_u64..15)
        .map(|k| {
            let at = SimInstant::from_nanos(k * 653_000_017 + 11);
            let s = k as f64;
            let cmd = input(
                (s * 0.7).sin(),
                0.4 * (s * 1.3).cos(),
                0.2 * (s * 0.5).sin(),
                0.3 + 0.05 * s,
            );
            (at, cmd)
        })
        .collect()
}

/// Drives a session the way a shell with frame span `frame` would: before
/// each frame it submits every scripted input stamped inside that frame.
fn run_shell(frame: TickSpan, target: SimInstant) -> (VehicleState, Vec<SensorExposureEvent>) {
    let mut session = session();
    let mut script = script().into_iter().peekable();
    let mut exposures = Vec::new();
    while session.now() < target {
        let span = frame.min(target.duration_since(session.now()));
        let frame_end = session.now().advanced(span);
        while let Some((at, cmd)) = script.next_if(|(at, _)| *at <= frame_end) {
            session.submit_ordered(at, cmd).unwrap();
        }
        session.advance(span).unwrap();
        exposures.extend(session.take_exposures());
    }
    assert_eq!(session.now(), target);
    (session.truth_state(), exposures)
}

#[test]
fn headless_and_30_60_144_fps_shells_reach_identical_truth_and_exposures() {
    let target = SimInstant::START.advanced(TickSpan::from_ticks(10 * TICKS_PER_SECOND));
    let headless = run_shell(TickSpan::from_ticks(10 * TICKS_PER_SECOND), target);
    let fps30 = run_shell(TickSpan::from_ticks(TICKS_PER_SECOND / 30), target);
    let fps60 = run_shell(TickSpan::from_ticks(TICKS_PER_SECOND / 60), target);
    let fps144_span = TickSpan::from_ticks(SimInstant::from_nanos(1_000_000_000 / 144).ticks());
    let fps144 = run_shell(fps144_span, target);

    assert_eq!(headless.1.len(), 300);
    for (k, event) in headless.1.iter().enumerate() {
        assert_eq!(event.channel, CAMERA);
        assert_eq!(event.sequence, k as u64);
        let expected_at = SimInstant::START.advanced(TickSpan::from_ticks_u128(
            CAMERA_PERIOD.ticks_u128() * (k as u128 + 1),
        ));
        assert_eq!(event.exposure_at, expected_at);
    }
    for shell in [&fps30, &fps60, &fps144] {
        assert_eq!(shell.0, headless.0);
        assert_eq!(shell.1.len(), headless.1.len());
        assert_eq!(shell.1, headless.1);
    }
    let start = session().truth_state();
    assert!(headless.0.position_ecef.distance(start.position_ecef) > 300.0);
}

#[test]
fn display_state_lags_one_step_and_interpolates_while_truth_stays_exact() {
    let mut session = session();
    let start = session.truth_state();
    let half_step = TickSpan::from_ticks(TICKS_PER_SECOND / 240);
    session.submit_at(SimInstant::START, input(0.0, 0.0, 0.0, 1.0));
    session
        .advance(TickSpan::from_ticks(TICKS_PER_SECOND / 120))
        .unwrap();
    let after_one = session.truth_state();
    assert_ne!(after_one, start);
    assert_eq!(
        session.display_state(),
        interpolate_vehicle_state(&start, &after_one, 0.0)
    );
    session.advance(half_step).unwrap();
    assert_eq!(session.truth_state(), after_one);
    assert_eq!(
        session.display_state(),
        interpolate_vehicle_state(&start, &after_one, 0.5)
    );
    session.advance(half_step).unwrap();
    let after_two = session.truth_state();
    assert_ne!(after_two, after_one);
    session.advance(half_step).unwrap();
    assert_eq!(
        session.display_state(),
        interpolate_vehicle_state(&after_one, &after_two, 0.5)
    );
    assert_eq!(session.truth_state(), after_two);
}

#[test]
fn interpolation_clamps_phase_and_slerps_attitude() {
    let prev = VehicleState {
        position_ecef: DVec3::ZERO,
        velocity_ecef: DVec3::X,
        attitude: DQuat::IDENTITY,
        angular_rate_body: DVec3::ZERO,
    };
    let curr = VehicleState {
        position_ecef: DVec3::new(10.0, 0.0, 0.0),
        velocity_ecef: DVec3::Y,
        attitude: DQuat::from_rotation_z(1.0),
        angular_rate_body: DVec3::ONE,
    };
    assert_eq!(interpolate_vehicle_state(&prev, &curr, -3.0), prev);
    assert_eq!(interpolate_vehicle_state(&prev, &curr, f64::NAN), prev);
    let past = interpolate_vehicle_state(&prev, &curr, 7.0);
    assert_eq!(past.position_ecef, curr.position_ecef);
    let mid = interpolate_vehicle_state(&prev, &curr, 0.5);
    assert_eq!(mid.position_ecef, DVec3::new(5.0, 0.0, 0.0));
    assert!(mid.attitude.angle_between(DQuat::from_rotation_z(0.5)) < 1e-12);
}

#[test]
fn input_queue_clamps_rejects_and_latches_newest_due_in_fifo_order() {
    let at = |s: u64| SimInstant::START.advanced(TickSpan::from_ticks(s * TICKS_PER_SECOND));
    let mut queue = StampedInputQueue::new();
    queue.submit_ordered(at(2), 'a').unwrap();
    assert_eq!(
        queue.submit_ordered(at(1), 'x'),
        Err(SessionError::InputOutOfOrder {
            at: at(1),
            last: at(2)
        })
    );
    assert_eq!(queue.submit_at(at(1), 'b'), at(2));
    queue.submit_ordered(at(3), 'c').unwrap();
    assert_eq!(queue.latch_due(at(1)), None);
    assert_eq!(queue.latch_due(at(2)), Some('b'));
    assert_eq!(queue.len(), 1);
    assert_eq!(queue.latch_due(at(9)), Some('c'));
    assert!(queue.is_empty());
}

#[test]
fn sensor_registration_rejects_zero_period_and_duplicates() {
    let mut session = session();
    assert_eq!(
        session.register_sensor(SensorChannelId(1), TickSpan::ZERO),
        Err(SessionError::ZeroSensorPeriod {
            channel: SensorChannelId(1)
        })
    );
    assert_eq!(
        session.register_sensor(CAMERA, CAMERA_PERIOD),
        Err(SessionError::DuplicateSensorChannel { channel: CAMERA })
    );
}

fn seconds(s: u64) -> SimInstant {
    SimInstant::START.advanced(TickSpan::from_ticks(s * TICKS_PER_SECOND))
}

fn third_of_a_step() -> TickSpan {
    TickSpan::from_ticks(TICKS_PER_SECOND / 360)
}

#[test]
fn restore_exact_continues_bit_identically_and_keeps_source_epoch() {
    let mut session = session();
    session
        .submit_ordered(seconds(1), input(0.5, 0.1, 0.0, 0.8))
        .unwrap();
    session.advance_to(seconds(2)).unwrap();
    session.advance(third_of_a_step()).unwrap();
    session
        .submit_ordered(seconds(3), input(-0.7, 0.2, 0.1, 0.4))
        .unwrap();
    session.take_exposures();
    let snap = session.snapshot_exact();

    session.advance_to(seconds(4)).unwrap();
    let first_truth = session.truth_state();
    let first_display = session.display_state();
    let first_exposures = session.take_exposures();

    session.restore_exact(&snap);
    assert_eq!(session.source_epoch(), snap.source_epoch());
    assert_eq!(session.pending_inputs().len(), 1);
    session.advance_to(seconds(4)).unwrap();
    assert_eq!(session.truth_state(), first_truth);
    assert_eq!(session.display_state(), first_display);
    assert_eq!(session.take_exposures(), first_exposures);
    assert_eq!(session.source_epoch(), 0);
}

#[test]
fn rewind_timeline_advances_epoch_and_drops_abandoned_inputs() {
    let mut session = session();
    session.advance_to(seconds(2)).unwrap();
    session.advance(third_of_a_step()).unwrap();
    let checkpoint = session.checkpoint();
    let hold = input(0.0, 0.0, 0.0, 0.5);
    session
        .submit_ordered(seconds(3), input(1.0, 1.0, 1.0, 1.0))
        .unwrap();
    session.advance_to(seconds(4)).unwrap();
    session
        .submit_ordered(seconds(5), input(-1.0, 0.0, 0.0, 0.0))
        .unwrap();

    session.rewind_timeline(&checkpoint, hold);
    assert_eq!(session.source_epoch(), 1);
    assert!(session.pending_inputs().is_empty());
    assert!(session.exposures().is_empty());
    assert_eq!(session.now(), checkpoint.at);
    assert_eq!(session.physics_time(), checkpoint.at);
    assert_eq!(session.display_state(), session.truth_state());
    session.advance_to(seconds(6)).unwrap();
    let rewound = session.truth_state();
    let exposures = session.take_exposures();
    assert_eq!(
        exposures.first().map(|e| e.exposure_at),
        Some(checkpoint.at.advanced(CAMERA_PERIOD))
    );

    let mut reference = self::session();
    reference.rewind_timeline(&checkpoint, hold);
    reference.advance_to(seconds(6)).unwrap();
    assert_eq!(rewound, reference.truth_state());

    let mut wrapping = self::session();
    wrapping.source_epoch = u32::MAX;
    wrapping.rewind_timeline(&checkpoint, hold);
    assert_eq!(wrapping.source_epoch(), 0);
}

#[test]
fn reset_episode_restarts_clock_sensors_and_identity() {
    let mut session = session();
    let initial = session.block().snapshot();
    let initial_truth = session.truth_state();
    session
        .submit_ordered(seconds(5), input(0.3, 0.0, 0.0, 0.9))
        .unwrap();
    session.advance_to(seconds(1)).unwrap();
    session.reset_episode(&initial, KinematicInput::default());

    assert_eq!(session.now(), SimInstant::START);
    assert_eq!(session.truth_state(), initial_truth);
    assert_eq!(session.episode_id(), 1);
    assert_eq!(session.source_epoch(), 1);
    assert!(session.pending_inputs().is_empty());
    session.advance_to(seconds(1)).unwrap();
    let exposures = session.take_exposures();
    assert_eq!(exposures.len(), 30);
    assert_eq!(exposures.first().map(|e| e.sequence), Some(0));
}

#[test]
fn restore_exact_brings_back_untaken_exposures_bit_for_bit() {
    let mut session = session();
    session
        .submit_ordered(seconds(1), input(0.4, -0.2, 0.1, 0.9))
        .unwrap();
    session.advance_to(seconds(2)).unwrap();
    session.advance(third_of_a_step()).unwrap();
    let pending = session.exposures().to_vec();
    assert_eq!(pending.len(), 60);
    let snap = session.snapshot_exact();
    assert_eq!(snap.pending_exposures(), pending.as_slice());

    session.advance_to(seconds(3)).unwrap();
    let first = session.take_exposures();
    assert_eq!(first.len(), 90);
    assert_eq!(first.get(..60), Some(pending.as_slice()));

    session.restore_exact(&snap);
    assert_eq!(session.exposures(), pending.as_slice());
    session.advance_to(seconds(3)).unwrap();
    assert_eq!(session.take_exposures(), first);
}

#[test]
fn rewind_and_reset_still_drop_untaken_exposures() {
    let mut session = session();
    let initial = session.block().snapshot();
    session.advance_to(seconds(1)).unwrap();
    let checkpoint = session.checkpoint();
    session.advance_to(seconds(2)).unwrap();
    assert!(!session.exposures().is_empty());
    session.rewind_timeline(&checkpoint, KinematicInput::default());
    assert!(session.exposures().is_empty());
    session.advance_to(seconds(2)).unwrap();
    assert!(!session.exposures().is_empty());
    session.reset_episode(&initial, KinematicInput::default());
    assert!(session.exposures().is_empty());
}
