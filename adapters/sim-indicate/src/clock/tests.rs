use sim_core::TickSpan;

use super::*;

#[test]
fn milliseconds_round_down_from_the_master_clock() {
    let span = TickSpan::from_seconds_and_subsec_ticks(3, TICKS_PER_MILLISECOND * 3 / 2);
    assert_eq!(instant_ms(SimInstant::START.advanced(span)), 3_001);
}

#[test]
fn a_reading_from_the_future_has_no_age() {
    let later = SimInstant::START.advanced(TickSpan::SECOND);
    assert_eq!(age_ms(later, SimInstant::START), 1_000.0);
    assert_eq!(age_ms(SimInstant::START, later), 0.0);
}
