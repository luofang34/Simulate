use aviate_xil_contract::{
    AttachError, MAGIC, SharedStateV2, pack_fc_status, seqlock_read, seqlock_write,
};
use core::sync::atomic::AtomicU32;
use glam::{DQuat, DVec3};
use mbd_core::compute_site_kinematics;
use sim_core::{BridgeError, HitlSyncMode, TickSpan};
use sim_geo::{BodyEllipsoid, Geodetic};

use super::*;
use crate::new_shared_state;

const STEP: TickSpan = TickSpan::from_ticks(TICKS_PER_SECOND / 250);

fn origin() -> Geodetic {
    Geodetic::from_degrees(39.1, -75.6, 100.0)
}

fn world() -> EnuWorld {
    EnuWorld::new(&BodyEllipsoid::WGS84, &origin())
}

fn stimulus() -> AviateStimulus {
    AviateStimulus {
        vehicle: VehicleState::level_flight(&origin(), 0.0, 10.0),
        site: compute_site_kinematics(
            DVec3::ZERO,
            DVec3::ZERO,
            DVec3::ZERO,
            (DVec3::ZERO, DQuat::IDENTITY),
            DVec3::ZERO,
        ),
    }
}

/// Behaves as the contract asks of a flight controller: one seqlocked command,
/// its acknowledgement, and `Ready` for the current epoch.
fn well_behaved(shared: &mut SharedStateV2) {
    let step = shared.state.sim_step;
    answer(shared, step, shared.header.reset_generation);
}

fn answer(shared: &mut SharedStateV2, acknowledged: u64, generation: u32) {
    let block = &mut shared.command;
    let (motors, bits, count) = (
        &mut block.seq,
        &mut block.motor_vel_bits,
        &mut block.num_motors,
    );
    seqlock_write(AtomicU32::from_mut(motors), || {
        *count = 4;
        *bits = [650.0_f64.to_bits(); 8];
    });
    block.fc_step_ack = acknowledged;
    shared.control.fc_status = pack_fc_status(generation, FcState::Ready);
}

fn attach<F: FnMut(&mut SharedStateV2)>(peer: F) -> AviateSharedMemoryEndpoint<F> {
    AviateSharedMemoryEndpoint::attach(new_shared_state(1), world(), peer).unwrap()
}

fn after_seconds(seconds: u64) -> SimInstant {
    SimInstant::START.advanced(TickSpan::from_seconds_and_subsec_ticks(seconds, 0))
}

#[test]
fn attaching_to_a_foreign_block_fails_before_the_peer_runs() {
    let mut shared = new_shared_state(1);
    shared.header.magic = MAGIC ^ 1;
    let refused =
        AviateSharedMemoryEndpoint::attach(shared, world(), |_: &mut SharedStateV2| panic!());
    assert!(matches!(
        refused,
        Err(AviateBridgeError::Attach(AttachError::BadMagic { .. }))
    ));
}

#[test]
fn steps_continue_from_the_blocks_own_count() {
    let mut shared = new_shared_state(1);
    shared.state.sim_step = 41;
    let mut endpoint = AviateSharedMemoryEndpoint::attach(shared, world(), well_behaved).unwrap();
    let answer = endpoint.round_trip(SimInstant::START, &stimulus()).unwrap();
    assert_eq!(answer.sim_step, 42);
    assert_eq!(endpoint.shared().state.sim_step, 42);
}

#[test]
fn a_round_trip_publishes_a_snapshot_the_controller_can_read_coherently() {
    let mut seen = Vec::new();
    let peer = |shared: &mut SharedStateV2| {
        let (seq, state) = (&shared.state.seq, shared.state);
        let snapshot = seqlock_read(&AtomicU32::new(*seq), || state);
        seen.push(snapshot.unwrap());
        well_behaved(shared);
    };
    let mut endpoint = attach(peer);
    let at = after_seconds(3).advanced(STEP);
    let answer = endpoint.round_trip(at, &stimulus()).unwrap();
    assert_eq!(answer.motors.speeds_rad_s(), &[650.0; 4]);
    assert!(answer.fc_ready);
    assert_eq!(answer.fc_state, FcState::Ready);
    drop(endpoint);
    let snapshot = seen[0];
    assert_eq!(snapshot.valid, 1);
    assert_eq!(snapshot.sim_step, 1);
    assert_eq!(snapshot.time_us, 3_004_000);
    assert_eq!(snapshot.seq & 1, 0);
}

#[test]
fn a_lockstep_step_the_controller_did_not_finish_stops_the_exchange() {
    let mut endpoint = attach(|shared: &mut SharedStateV2| {
        answer(shared, shared.state.sim_step.wrapping_sub(1), 1);
    });
    assert_eq!(
        endpoint.round_trip(SimInstant::START, &stimulus()),
        Err(AviateBridgeError::StepNotAcknowledged {
            expected: 1,
            acknowledged: 0
        }),
    );
}

#[test]
fn a_late_acknowledgement_is_tolerated_with_lockstep_off() {
    let mut shared = new_shared_state(1);
    shared.control.lockstep_enabled = 0;
    let peer = |shared: &mut SharedStateV2| answer(shared, 0, 1);
    let mut endpoint = AviateSharedMemoryEndpoint::attach(shared, world(), peer).unwrap();
    assert!(endpoint.round_trip(SimInstant::START, &stimulus()).is_ok());
}

#[test]
fn ready_from_an_earlier_epoch_does_not_count() {
    let mut endpoint = attach(|shared: &mut SharedStateV2| {
        let step = shared.state.sim_step;
        shared.header.reset_generation = 2;
        answer(shared, step, 1);
    });
    let answer = endpoint.round_trip(SimInstant::START, &stimulus()).unwrap();
    assert_eq!(answer.fc_state, FcState::Ready);
    assert!(!answer.fc_ready);
}

#[test]
fn a_peer_that_corrupts_the_header_fails_the_exchange() {
    let mut endpoint = attach(|shared: &mut SharedStateV2| {
        well_behaved(shared);
        shared.header.layout_version = 2;
    });
    assert_eq!(
        endpoint.round_trip(SimInstant::START, &stimulus()),
        Err(AviateBridgeError::Attach(AttachError::VersionMismatch {
            found: 2
        })),
    );
}

#[test]
fn a_peer_that_abandons_the_block_fails_the_next_publish() {
    let mut endpoint = attach(|shared: &mut SharedStateV2| {
        well_behaved(shared);
    });
    endpoint.round_trip(SimInstant::START, &stimulus()).unwrap();
    endpoint.shared.header.plugin_ready = 0;
    assert_eq!(
        endpoint.round_trip(SimInstant::START, &stimulus()),
        Err(AviateBridgeError::NotReady),
    );
}

#[test]
fn time_past_the_field_is_an_error_rather_than_a_wrap() {
    let far = SimInstant::START.advanced(TickSpan::from_seconds_and_subsec_ticks(u64::MAX / 2, 0));
    let mut endpoint = attach(well_behaved);
    assert_eq!(
        endpoint.round_trip(far, &stimulus()),
        Err(AviateBridgeError::TimeOutOfRange {
            seconds: u64::MAX / 2
        }),
    );
}

#[test]
fn microseconds_round_down_within_the_second() {
    let span = TickSpan::from_ticks(TICKS_PER_MICROSECOND * 1_500 + 1);
    assert_eq!(
        micros_since_start(SimInstant::START.advanced(span)),
        Ok(1_500)
    );
}

#[test]
fn the_lockstep_bridge_delivers_after_the_charged_compute_time() {
    let delay = TickSpan::from_ticks(STEP.ticks() / 2);
    let mode = HitlSyncMode::LockstepBarrier {
        processing_delay: delay,
    };
    let mut bridge: AviateXilBridge<_> = AviateXilBridge::new(attach(well_behaved), mode);
    bridge.stimulate(SimInstant::START, &stimulus()).unwrap();
    assert!(bridge.poll(SimInstant::START).unwrap().is_empty());
    let delivered = bridge.poll(SimInstant::START.advanced(STEP)).unwrap();
    assert_eq!(delivered.len(), 1);
    assert_eq!(delivered[0].deliver_at, SimInstant::START.advanced(delay));
    assert_eq!(delivered[0].response.sim_step, 1);
}

#[test]
fn the_real_time_path_answers_through_the_mailbox_stamped_for_its_instant() {
    let mut endpoint = attach(well_behaved);
    let at = SimInstant::START.advanced(STEP);
    endpoint.post(at, &stimulus()).unwrap();
    let stamped = endpoint.try_receive().unwrap().unwrap();
    assert_eq!(stamped.deliver_at, at);
    assert!(stamped.response.fc_ready);
    assert_eq!(endpoint.try_receive().unwrap(), None);

    let mut bridge = AviateXilBridge::new(attach(well_behaved), HitlSyncMode::RealTimeStamped);
    bridge.stimulate(at, &stimulus()).unwrap();
    bridge.stimulate(at.advanced(STEP), &stimulus()).unwrap();
    let delivered = bridge.poll(at.advanced(STEP)).unwrap();
    assert_eq!(delivered.len(), 1);
    assert_eq!(delivered[0].response.sim_step, 2, "the newest wins");
    assert_eq!(bridge.stats().dropped, 1);
}

#[test]
fn a_failed_exchange_reaches_the_bridge_with_its_reason() {
    let mode = HitlSyncMode::LockstepBarrier {
        processing_delay: STEP,
    };
    let mut bridge = AviateXilBridge::new(attach(|_: &mut SharedStateV2| {}), mode);
    let failure = bridge
        .stimulate(SimInstant::START, &stimulus())
        .unwrap_err();
    let BridgeError::Endpoint { at, source } = failure;
    assert_eq!(at, SimInstant::START);
    assert!(source.detail.contains("acknowledged step 0"), "{source}");
    let mut endpoint = attach(|_: &mut SharedStateV2| {});
    assert!(endpoint.post(SimInstant::START, &stimulus()).is_err());
    assert!(
        endpoint
            .exchange_blocking(SimInstant::START, &stimulus())
            .is_err()
    );
}

#[test]
fn the_shared_memory_endpoint_declares_an_in_process_synchronous_boundary() {
    let endpoint = attach(well_behaved);
    assert_eq!(
        endpoint.execution_boundary(),
        AviateExecutionBoundary::InProcessSynchronous
    );
    assert_eq!(endpoint.metrics(), Tier1StepMetrics::default());
}

#[test]
fn metrics_correlate_each_command_with_the_step_it_acknowledges() {
    let mut endpoint = attach(well_behaved);
    let first = endpoint.round_trip(after_seconds(0), &stimulus()).unwrap();
    let second = endpoint.round_trip(after_seconds(1), &stimulus()).unwrap();
    let metrics = endpoint.metrics();
    assert_eq!(metrics.step_calls, 2);
    assert_eq!(metrics.causal_steps, 2);
    assert_eq!(metrics.acausal_steps, 0);
    assert_eq!(metrics.last_published_step, second.sim_step);
    assert_eq!(metrics.last_acknowledged_step, second.sim_step);
    assert_eq!(second.sim_step, first.sim_step + 1);
    assert_eq!(metrics.budget_overruns, 0);
    assert_eq!(metrics.max_peer_wall, Duration::ZERO);
}

#[test]
fn a_lagging_acknowledgement_counts_as_acausal_with_lockstep_off_and_on() {
    let mut shared = new_shared_state(1);
    shared.control.lockstep_enabled = 0;
    let peer = |shared: &mut SharedStateV2| answer(shared, 0, 1);
    let mut endpoint = AviateSharedMemoryEndpoint::attach(shared, world(), peer).unwrap();
    endpoint.round_trip(SimInstant::START, &stimulus()).unwrap();
    assert_eq!(endpoint.metrics().acausal_steps, 1);
    assert_eq!(endpoint.metrics().last_acknowledged_step, 0);

    let mut strict = attach(|shared: &mut SharedStateV2| answer(shared, 0, 1));
    assert!(strict.round_trip(SimInstant::START, &stimulus()).is_err());
    assert_eq!(strict.metrics().step_calls, 1);
    assert_eq!(strict.metrics().acausal_steps, 1);
}

#[test]
fn a_peer_slower_than_its_budget_counts_overruns_without_changing_results() {
    let slow = |shared: &mut SharedStateV2| {
        std::thread::sleep(Duration::from_millis(3));
        well_behaved(shared);
    };
    let mut timed = attach(slow).with_step_budget(Duration::from_millis(1));
    let mut untimed = attach(slow);
    for k in 0..3 {
        assert_eq!(
            timed.round_trip(after_seconds(k), &stimulus()).unwrap(),
            untimed.round_trip(after_seconds(k), &stimulus()).unwrap()
        );
    }
    let metrics = timed.metrics();
    assert_eq!(metrics.budget_overruns, 3);
    assert!(metrics.max_peer_wall >= Duration::from_millis(3));
    assert_eq!(untimed.metrics().budget_overruns, 0);

    let mut generous = attach(well_behaved).with_step_budget(Duration::from_secs(5));
    generous.round_trip(after_seconds(0), &stimulus()).unwrap();
    assert_eq!(generous.metrics().budget_overruns, 0);
}
