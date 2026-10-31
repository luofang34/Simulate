use sim_core::{TICKS_PER_SECOND, TickSpan};

use super::*;

#[test]
fn a_30_hz_frame_stamps_on_the_right_nanosecond_and_domain() {
    let bridge = MasterClockBridge::default();
    let frame = SimInstant::START.advanced(TickSpan::from_ticks(TICKS_PER_SECOND / 30));
    let stamp = bridge
        .stamp(
            SourceId::new(7),
            SourceEpoch::new(2),
            WrappingSequence::new(9),
            frame,
        )
        .unwrap();
    assert_eq!(stamp.acquired_at.as_nanos(), 33_333_333);
    assert_eq!(stamp.clock, MASTER_CLOCK_DOMAIN);
    assert_eq!(stamp.source, SourceId::new(7));
    assert_eq!(stamp.epoch, SourceEpoch::new(2));
    assert_eq!(stamp.sequence, WrappingSequence::new(9));
}

#[test]
fn monotonic_time_survives_the_round_trip_through_the_master_clock() {
    let bridge = MasterClockBridge::default();
    for nanos in [0, 1, 33_333_333, 500_000_000, u64::MAX] {
        let time = MonotonicNanos::from_nanos(nanos);
        assert_eq!(bridge.to_monotonic(bridge.to_instant(time)).unwrap(), time);
    }
}

#[test]
fn a_stamp_never_reads_later_than_its_instant() {
    let bridge = MasterClockBridge::default();
    let instant = SimInstant::START.advanced(TickSpan::from_ticks(47));
    let time = bridge.to_monotonic(instant).unwrap();
    assert!(bridge.to_instant(time) <= instant);
}

#[test]
fn an_instant_past_the_nanosecond_range_is_refused_not_saturated() {
    let far = SimInstant::from_nanos(u64::MAX).advanced(TickSpan::SECOND);
    assert!(matches!(
        MasterClockBridge::default().to_monotonic(far),
        Err(NavigateBridgeError::InstantOutOfRange { .. })
    ));
}
