use navigate_contract::GeodeticPosition;
use sim_flight_chain::HoldingPatternStatus;

use super::*;

fn pattern() -> HoldingPattern {
    HoldingPattern {
        fix_ident: "WP4".to_owned(),
        fix: GeodeticPosition::new(0.7284, -0.0183, 2050.0),
        inbound_course_rad: 3.5,
        turn: HoldingTurn::Right,
        leg_time_s: 60.0,
        turn_radius_m: 930.0,
        leg_length_m: 3_640.0,
    }
}

/// Pilotage's display a few seconds into a flight: its phase and steering
/// source are its own to state.
fn flown() -> NavigationDisplay {
    let spec = super::super::tests::spec(None);
    let mut trial = crate::ChainTrial::new(&spec, &sim_navigate::trial::DirectoryStores).unwrap();
    trial.step_until(4.0).unwrap();
    trial.navigation().unwrap().clone()
}

fn display(flown: &NavigationDisplay, leg: HoldingLeg, circuits: u32) -> NavigationDisplay {
    NavigationDisplay {
        waypoints: Vec::new(),
        active_waypoint: None,
        plan_complete: true,
        leg: None,
        holding_pattern: Some(HoldingPatternStatus {
            pattern: pattern(),
            entry: Some(HoldingEntry::Teardrop),
            leg,
            circuits,
            outbound_time_s: 60.0,
        }),
        ..flown.clone()
    }
}

fn at(seconds: u64) -> SimInstant {
    SimInstant::from_ticks_u128(u128::from(seconds) * u128::from(sim_core::TICKS_PER_SECOND))
}

/// The hold reaches the progress stream as Pilotage states it: the pattern
/// with its own names and units, the entry, and one record for each change
/// of the part flown, none while it stays the same.
#[test]
fn the_hold_is_recorded_as_pilotage_flies_it() {
    let flown = flown();
    let mut watch = HoldingWatch::default();
    let started = [
        (
            at(940),
            HostEvent::Mission(MissionEvent::HoldingPatternStarted { pattern: pattern() }),
        ),
        (
            at(941),
            HostEvent::Mission(MissionEvent::HoldingPatternEntered {
                entry: HoldingEntry::Teardrop,
            }),
        ),
    ];
    let records = watch.records(
        941.0,
        &started,
        Some(&display(&flown, HoldingLeg::ToFix, 0)),
    );
    let kinds: Vec<&EventKind> = records
        .iter()
        .filter_map(|r| match r {
            ProgressRecord::Event(e) => Some(&e.kind),
            _ => None,
        })
        .collect();
    let EventKind::HoldingStarted { pattern: record } = kinds[0] else {
        panic!("{kinds:?}");
    };
    assert_eq!(record.fix_ident, "WP4");
    assert_eq!((record.inbound_course_rad, record.leg_time_s), (3.5, 60.0));
    assert_eq!(record.turn, HoldingTurnName::Right);
    assert_eq!(record.fix.altitude_m, 2050.0);
    assert_eq!(
        *kinds[1],
        EventKind::HoldingEntered {
            entry: HoldingEntryName::Teardrop
        }
    );
    assert_eq!(
        *kinds[2],
        EventKind::HoldingLegChanged {
            leg: HoldingLegName::ToFix,
            circuits: 0
        }
    );
    assert!(
        watch
            .records(942.0, &[], Some(&display(&flown, HoldingLeg::ToFix, 0)))
            .is_empty()
    );
    let turned = watch.records(990.0, &[], Some(&display(&flown, HoldingLeg::Inbound, 1)));
    assert_eq!(turned.len(), 1);
    assert!(watch.records(991.0, &[], None).is_empty());
}
