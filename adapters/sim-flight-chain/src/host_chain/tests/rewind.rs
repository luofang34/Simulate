//! A chain that returns to a snapshot flies the same flight again, byte for byte.

use super::*;

/// Long enough for the plan to load, the controller to arm and the mission to steer.
const BEFORE_S: f64 = 20.0;
/// Long enough for the host to tick and command many times after the snapshot.
const AFTER_S: f64 = 15.0;

/// Everything a consumer of the chain sees in one cycle.
#[derive(Debug, PartialEq)]
struct Cycle {
    state: VehicleState,
    uplink: Vec<Vec<u8>>,
    downlink: Vec<u8>,
    events: Vec<String>,
    counters: LinkCounters,
}

fn fly(
    chain: &mut HostChain<C172Airframe, FixedWingController, FixedWingMixer>,
    seconds: f64,
) -> Vec<Cycle> {
    let cycles = (seconds * C172_RATE_HZ as f64).round() as usize;
    (0..cycles)
        .map(|_| {
            chain.advance_tick().unwrap();
            Cycle {
                state: chain.true_state(),
                uplink: chain.last_uplink().to_vec(),
                downlink: chain.last_downlink().to_vec(),
                events: chain
                    .drain_events()
                    .into_iter()
                    .map(|(at, event)| format!("{:.4} {event:?}", seconds_of(at)))
                    .collect(),
                counters: chain.counters(),
            }
        })
        .collect()
}

fn seconds_of(at: SimInstant) -> f64 {
    at.duration_since(SimInstant::START).as_seconds()
}

#[test]
fn a_chain_returned_to_a_snapshot_flies_the_same_flight_again() {
    let mut chain = chain(7);
    fly(&mut chain, BEFORE_S);
    let at = chain.now();
    let snapshot = chain.snapshot().unwrap();

    let first = fly(&mut chain, AFTER_S);
    assert!(
        first.iter().any(|cycle| !cycle.uplink.is_empty()),
        "the host commanded the controller after the snapshot"
    );
    assert_ne!(
        first.first().map(|c| c.state),
        first.last().map(|c| c.state)
    );

    // Twice, so the snapshot is shown to survive a restore.
    for _ in 0..2 {
        chain.restore(&snapshot).unwrap();
        assert_eq!(chain.now(), at);
        let again = fly(&mut chain, AFTER_S);
        assert_eq!(again.len(), first.len());
        for (index, (a, b)) in first.iter().zip(&again).enumerate() {
            assert_eq!(a, b, "cycle {index} after the snapshot differs");
        }
    }
}

#[test]
fn a_snapshot_taken_before_the_plan_is_loaded_loads_it_again() {
    let mut chain = chain(7);
    fly(&mut chain, 0.5);
    assert!(chain.record().is_none());
    let snapshot = chain.snapshot().unwrap();
    fly(&mut chain, PLAN_LOADED_AT_S + 1.0);
    assert!(chain.record().is_some());

    chain.restore(&snapshot).unwrap();
    assert!(chain.record().is_none(), "the plan is not loaded yet");
    fly(&mut chain, PLAN_LOADED_AT_S + 1.0);
    assert!(chain.record().is_some());
}
