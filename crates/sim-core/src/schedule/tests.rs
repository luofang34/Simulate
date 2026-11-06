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
    assert_eq!(
        agc.steps_for(TickSpan::from_ticks(TickSpan::AGC_MCT.ticks() * 2 / 3)),
        1
    );
}

#[test]
fn rational_rate_adapter_handles_non_dividing_and_drifted_oscillators() {
    // GPS fundamental frequency 10.23 MHz does not divide TICKS_PER_SECOND,
    // yet 1 second of 60 Hz frames yields exactly 10,230,000 steps.
    let mut gps = RateAdapter::from_hz_ratio(10_230_000, 1);
    let frame_60hz = TickSpan::period_of_hz(60).unwrap();
    let steps: u64 = (0..60).map(|_| gps.steps_for(frame_60hz)).sum();
    assert_eq!(steps, 10_230_000);

    // NTSC 60,000 / 1,001 Hz over 1,001 seconds yields 60,000 steps.
    let mut ntsc = RateAdapter::from_hz_ratio(60_000, 1_001);
    let one_sec = TickSpan::SECOND;
    let ntsc_steps: u64 = (0..1_001).map(|_| ntsc.steps_for(one_sec)).sum();
    assert_eq!(ntsc_steps, 60_000);

    // +15 ppm drifted 16 MHz crystal (16,000,240 Hz).
    let mut drifted_mcu = RateAdapter::from_hz_ratio(16_000_240, 1);
    assert_eq!(drifted_mcu.steps_for(one_sec), 16_000_240);
}

#[test]
fn century_fast_warp_advances_without_overflow_or_precision_loss() {
    let cap = TickSpan::period_of_hz(4).unwrap();
    let mut clock = RealTimeClock::new(cap);
    // 100 Julian years in a single 0.1 s wall-clock frame at time_scale = 31_557_600_000.
    let one_century_s: u64 = 100 * 31_557_600;
    let warp_span = clock.advance(0.1, (one_century_s as f64) * 10.0);
    assert_eq!(warp_span.seconds_and_subsec_ticks(), (one_century_s, 0));

    // A 1 Hz orbital/thermal beacon over that 100-year jump steps exactly one_century_s times.
    let mut beacon_1hz = RateAdapter::new(TickSpan::SECOND);
    assert_eq!(beacon_1hz.steps_for(warp_span), one_century_s);
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
    let ticks: u128 = (0..1_000)
        .map(|_| clock.advance(frame_s, 1.0).ticks_u128())
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

#[test]
fn rate_adapter_phase_tracks_sub_step_progress() {
    // A 120 Hz block driven by a 1200 Hz master step advances a tenth of a
    // block period per tick and wraps to zero on every execution.
    let mut fdm = RateAdapter::new(TickSpan::period_of_hz(120).unwrap());
    let tick_1200hz = TickSpan::period_of_hz(1200).unwrap();
    assert_eq!(fdm.phase(), 0.0);
    let mut steps = 0;
    for tick in 1..=25_u32 {
        steps += fdm.steps_for(tick_1200hz);
        let expected = f64::from(tick % 10) / 10.0;
        assert!((fdm.phase() - expected).abs() < 1e-12, "tick {tick}");
        assert!((0.0..1.0).contains(&fdm.phase()));
    }
    assert_eq!(steps, 2);

    // Rational oscillators report the carried fraction of one step.
    let mut ntsc = RateAdapter::from_hz_ratio(60_000, 1_001);
    assert_eq!(ntsc.steps_for(TickSpan::SECOND), 59);
    let expected = 60_000.0 / 1_001.0 - 59.0;
    assert!((ntsc.phase() - expected).abs() < 1e-9);

    assert_eq!(RateAdapter::new(TickSpan::ZERO).phase(), 0.0);
    assert_eq!(RateAdapter::from_hz_ratio(0, 1).phase(), 0.0);
}
