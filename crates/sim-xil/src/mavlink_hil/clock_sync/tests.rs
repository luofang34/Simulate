use super::*;

const MS: u64 = 1_000;
const PX4_UPTIME_USEC: u64 = 86_400_000_000;

fn at_ms(ms: u64) -> SimInstant {
    SimInstant::START.advanced(TickSpan::from_ticks(ms * (TICKS_PER_SECOND / 1_000)))
}

fn tracker(max_reply_age: Option<TickSpan>) -> HilCorrelationTracker {
    HilCorrelationTracker::new(DEFAULT_CLOCK_RESYNC_THRESHOLD, max_reply_age)
}

#[test]
fn a_px4_boot_clock_a_day_in_maps_onto_the_simulation_timeline() {
    let mut clock = PeerClockSynchronizer::new(DEFAULT_CLOCK_RESYNC_THRESHOLD);
    let first = clock.map(PX4_UPTIME_USEC, at_ms(40));
    assert_eq!(first, at_ms(40));
    assert_eq!(clock.anchor(), Some((PX4_UPTIME_USEC, at_ms(40))));
    assert_eq!(clock.map(PX4_UPTIME_USEC + 4 * MS, at_ms(44)), at_ms(44));
    assert_eq!(clock.map(PX4_UPTIME_USEC + 250 * MS, at_ms(44)), at_ms(290));
    assert_eq!(clock.clock_resyncs(), 0);

    let raw_day = SimInstant::START.advanced(TickSpan::from_ticks_u128(
        u128::from(PX4_UPTIME_USEC) * u128::from(TICKS_PER_SECOND / 1_000_000),
    ));
    assert!(raw_day.duration_since(first) > TickSpan::from_ticks(TICKS_PER_SECOND * 86_000));
}

#[test]
fn a_peer_reboot_or_large_jump_reanchors_monotonically_and_counts_a_resync() {
    let mut clock = PeerClockSynchronizer::new(DEFAULT_CLOCK_RESYNC_THRESHOLD);
    clock.map(PX4_UPTIME_USEC, at_ms(0));
    assert_eq!(
        clock.map(PX4_UPTIME_USEC + 500 * MS, at_ms(100)),
        at_ms(500)
    );

    // Reboot: the peer clock restarts near zero while the simulation is at
    // 200 ms; the new anchor cannot fall before the 500 ms already mapped.
    assert_eq!(clock.map(1_500, at_ms(200)), at_ms(500));
    assert_eq!(clock.clock_resyncs(), 1);
    assert_eq!(clock.map(1_500 + 20 * MS, at_ms(520)), at_ms(520));

    // A forward jump past the threshold (a paused peer) re-anchors as well.
    assert_eq!(clock.map(1_500 + 5_000 * MS, at_ms(600)), at_ms(600));
    assert_eq!(clock.clock_resyncs(), 2);
}

#[test]
fn a_small_backward_step_keeps_the_anchor_and_maps_into_the_past() {
    let mut clock = PeerClockSynchronizer::new(DEFAULT_CLOCK_RESYNC_THRESHOLD);
    clock.map(10_000 * MS, at_ms(1_000));
    assert_eq!(clock.map(10_000 * MS + 30 * MS, at_ms(1_030)), at_ms(1_030));
    assert_eq!(clock.map(10_000 * MS - 10 * MS, at_ms(1_030)), at_ms(990));
    assert_eq!(clock.clock_resyncs(), 0);
}

#[test]
fn replies_correlate_with_the_sensor_frame_they_answer_and_measure_round_trip() {
    let mut link = tracker(None);
    assert_eq!(link.on_sensor_sent(at_ms(0)), 0);
    assert_eq!(
        link.on_actuator(PX4_UPTIME_USEC),
        ActuatorVerdict::Accepted {
            deliver_at: at_ms(0),
            answers_sensor: Some(0),
        }
    );
    link.on_sensor_sent(at_ms(4));
    link.on_sensor_sent(at_ms(8));
    link.on_sensor_sent(at_ms(12));
    // The peer answered the 4 ms frame; the simulation has since sent 12 ms.
    assert_eq!(
        link.on_actuator(PX4_UPTIME_USEC + 4 * MS),
        ActuatorVerdict::Accepted {
            deliver_at: at_ms(4),
            answers_sensor: Some(1),
        }
    );
    let stats = link.stats();
    assert_eq!(stats.sensor_frames_sent, 4);
    assert_eq!(stats.correlated_actuator_frames, 2);
    assert_eq!(
        stats.last_round_trip_ticks,
        at_ms(12).duration_since(at_ms(4)).ticks()
    );
    assert_eq!(stats.stale_actuator_drops, 0);
}

#[test]
fn stale_replies_are_rejected_without_replacing_fresh_ones() {
    let mut link = tracker(Some(DEFAULT_MAX_REPLY_AGE));
    link.on_sensor_sent(at_ms(0));
    link.on_actuator(PX4_UPTIME_USEC);
    for ms in (4..=200).step_by(4) {
        link.on_sensor_sent(at_ms(ms));
    }
    // 150 ms behind the newest sensor frame: past the 100 ms limit.
    assert_eq!(
        link.on_actuator(PX4_UPTIME_USEC + 50 * MS),
        ActuatorVerdict::Stale {
            mapped_at: at_ms(50),
            age: at_ms(200).duration_since(at_ms(50)),
        }
    );
    // Fresh: 8 ms behind.
    assert!(matches!(
        link.on_actuator(PX4_UPTIME_USEC + 192 * MS),
        ActuatorVerdict::Accepted { deliver_at, answers_sensor: Some(48) } if deliver_at == at_ms(192)
    ));
    // Within the age limit but older than the reply just accepted.
    assert!(matches!(
        link.on_actuator(PX4_UPTIME_USEC + 180 * MS),
        ActuatorVerdict::Stale { .. }
    ));
    let stats = link.stats();
    assert_eq!(stats.stale_actuator_drops, 2);
    assert_eq!(stats.correlated_actuator_frames, 2);
    assert_eq!(
        stats.last_round_trip_ticks,
        at_ms(200).duration_since(at_ms(192)).ticks()
    );
}

#[test]
fn without_an_age_limit_old_but_ordered_replies_still_deliver() {
    let mut link = tracker(None);
    link.on_sensor_sent(at_ms(0));
    link.on_actuator(0);
    link.on_sensor_sent(at_ms(900));
    assert!(matches!(
        link.on_actuator(0),
        ActuatorVerdict::Accepted { deliver_at, .. } if deliver_at == at_ms(0)
    ));
    assert_eq!(link.stats().stale_actuator_drops, 0);
}

#[test]
fn a_reply_before_any_sensor_frame_is_uncorrelated_and_history_is_bounded() {
    let mut link = tracker(None);
    assert!(matches!(
        link.on_actuator(7),
        ActuatorVerdict::Accepted {
            answers_sensor: None,
            ..
        }
    ));
    assert_eq!(link.stats().uncorrelated_actuator_frames, 1);
    for ms in 0..100 {
        link.on_sensor_sent(at_ms(ms));
    }
    assert_eq!(link.sent.len(), SENT_FRAME_HISTORY);
    assert_eq!(link.sim_now(), at_ms(99));
}
