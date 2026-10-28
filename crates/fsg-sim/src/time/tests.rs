use super::*;

#[test]
fn frame_and_agc_rates_divide_the_master_clock() {
    assert_eq!(
        TickSpan::period_of_hz(120),
        Some(TickSpan::from_ticks(128_000))
    );
    assert_eq!(
        TickSpan::period_of_hz(60).map(TickSpan::ticks),
        Some(256_000)
    );
    // AGC scaler (1600 Hz), gyro torquing and PIPA interrogation (3200 Hz).
    assert_eq!(
        TickSpan::period_of_hz(1_600).map(TickSpan::ticks),
        Some(9_600)
    );
    assert_eq!(
        TickSpan::period_of_hz(3_200).map(TickSpan::ticks),
        Some(4_800)
    );
    assert_eq!(TickSpan::AGC_MCT.ticks() * 1_024_000 / 12, TICKS_PER_SECOND);
}

#[test]
fn rates_that_would_drift_are_rejected() {
    assert_eq!(TickSpan::period_of_hz(0), None);
    assert_eq!(TickSpan::period_of_hz(7), None);
    assert_eq!(TickSpan::period_of_hz(2_048_000), None);
}

#[test]
fn instants_advance_by_whole_spans() {
    let t = SimInstant::START
        .advanced(TickSpan::AGC_MCT)
        .advanced(TickSpan::AGC_MCT);
    assert_eq!(t.ticks(), 360);
    assert!((TickSpan::from_ticks(TICKS_PER_SECOND).as_seconds() - 1.0).abs() < f64::EPSILON);
}
