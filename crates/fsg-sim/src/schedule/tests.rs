use super::*;

#[test]
fn rate_adapter_carries_remainders_exactly() {
    let mut fdm = RateAdapter::new(TickSpan::period_of_hz(120).unwrap());
    let mut agc = RateAdapter::new(TickSpan::AGC_MCT);
    let frame_60hz = TickSpan::period_of_hz(60).unwrap();
    let (mut fdm_steps, mut agc_steps) = (0, 0);
    for _ in 0..60 {
        fdm_steps += fdm.steps_for(frame_60hz);
        agc_steps += agc.steps_for(frame_60hz);
    }
    assert_eq!(fdm_steps, 120);
    // 85,333⅓ cycles per second: the third is carried, not dropped.
    assert_eq!(agc_steps, 85_333);
    assert_eq!(agc.steps_for(TickSpan::from_ticks(120)), 1);
}

#[test]
fn uneven_frames_never_lose_steps() {
    let mut adapter = RateAdapter::new(TickSpan::from_ticks(7));
    let total: u64 = [3, 5, 11, 1, 2]
        .iter()
        .map(|&t| adapter.steps_for(TickSpan::from_ticks(t)))
        .sum();
    assert_eq!(total, 22 / 7);
}

#[test]
fn real_time_clock_keeps_fractions_and_caps_stalls() {
    let cap = TickSpan::period_of_hz(4).unwrap();
    let mut clock = RealTimeClock::new(cap);
    let frame_s = 1.0 / 144.0;
    let ticks: u64 = (0..1_000)
        .map(|_| clock.advance(frame_s, 1.0).ticks())
        .sum();
    let expected = 1_000.0 * frame_s * TICKS_PER_SECOND as f64;
    assert!(
        (ticks as f64 - expected).abs() <= 1.0,
        "{ticks} vs {expected}"
    );
    assert_eq!(clock.advance(10.0, 1.0), cap);
    assert_eq!(clock.advance(-1.0, 1.0), TickSpan::ZERO);
    assert_eq!(clock.advance(f64::NAN, 1.0), TickSpan::ZERO);
    assert_eq!(clock.advance(0.001, 0.0), TickSpan::ZERO);
}
