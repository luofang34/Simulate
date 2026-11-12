use sim_navigate::trial::{
    EventKind, HoldingEntryName, HoldingFix, HoldingLegName, HoldingPatternRecord, HoldingTurnName,
};

use super::{HoldState, pattern_words};

fn pattern() -> HoldingPatternRecord {
    HoldingPatternRecord {
        fix_ident: "ZA5".to_owned(),
        fix: HoldingFix {
            latitude_rad: 41.74_f64.to_radians(),
            longitude_rad: (-1.05_f64).to_radians(),
            altitude_m: 2050.0,
        },
        inbound_course_rad: 90.0_f64.to_radians(),
        turn: HoldingTurnName::Right,
        leg_time_s: 60.0,
        turn_radius_m: 900.0,
        leg_length_m: 3300.0,
    }
}

#[test]
fn a_hold_follows_its_events_into_words() {
    assert_eq!(
        HoldState::after(
            None,
            &EventKind::GuidanceRefused {
                refusal: "x".into()
            }
        ),
        None
    );
    let started = HoldState::after(None, &EventKind::HoldingStarted { pattern: pattern() });
    assert_eq!(
        started.as_ref().map(HoldState::words).as_deref(),
        Some("HOLD at ZA5: to the fix, 0 circuits flown")
    );
    let entered = HoldState::after(
        started,
        &EventKind::HoldingEntered {
            entry: HoldingEntryName::Teardrop,
        },
    );
    let flying = HoldState::after(
        entered,
        &EventKind::HoldingLegChanged {
            leg: HoldingLegName::Inbound,
            circuits: 1,
        },
    );
    assert_eq!(
        flying.as_ref().map(HoldState::words).as_deref(),
        Some("HOLD at ZA5: inbound, 1 circuit flown")
    );
    assert_eq!(
        pattern_words(&pattern()),
        "holding at ZA5, inbound 090°, right turns, 60 s legs"
    );
}
