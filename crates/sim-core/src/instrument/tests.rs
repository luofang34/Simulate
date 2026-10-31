use super::*;

const SECOND: TickSpan = TickSpan::SECOND;

#[test]
fn only_live_sources_are_usable() {
    assert!(SourceValidity::Valid.is_usable());
    assert!(SourceValidity::Degraded.is_usable());
    for validity in [
        SourceValidity::Stale,
        SourceValidity::Missing,
        SourceValidity::Failed,
    ] {
        assert!(!validity.is_usable(), "{validity:?}");
    }
}

#[test]
fn a_reading_goes_stale_only_past_its_age_limit() {
    let reading = StampedReading::new(1.0, SimInstant::START, SourceValidity::Valid);
    let at = |span| SimInstant::START.advanced(span);

    assert_eq!(
        reading.aged(at(SECOND), SECOND).validity,
        SourceValidity::Valid
    );
    let late = at(SECOND).advanced(TickSpan::from_ticks(1));
    assert_eq!(reading.aged(late, SECOND).validity, SourceValidity::Stale);
}

#[test]
fn ageing_never_upgrades_a_failed_or_missing_reading() {
    let late = SimInstant::START.advanced(SECOND).advanced(SECOND);
    for validity in [SourceValidity::Failed, SourceValidity::Missing] {
        let reading = StampedReading::new(0.0, SimInstant::START, validity);
        assert_eq!(reading.aged(late, SECOND).validity, validity);
    }
}
