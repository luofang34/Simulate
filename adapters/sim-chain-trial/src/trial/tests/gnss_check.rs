use sim_navigate::trial::{DirectoryStores, GnssCheckState};

use sim_flight_chain::SourceIntegrityState;

use super::{advanced, samples_of, spec};
use crate::{ChainTrial, Steering};

/// Every set with the receiver has its navigator check it, and reports the
/// navigator's verdict beside its samples; a set without it reports none.
#[test]
fn the_gnss_sets_report_their_navigators_check_of_the_receiver() {
    let mut trial = ChainTrial::new(&spec(None), &DirectoryStores).unwrap();
    let outputs = advanced(trial.step_until(30.0).unwrap());
    let checked: Vec<_> = outputs
        .gnss_checks
        .iter()
        .filter(|(set, _)| set == "ins.baro.gnss")
        .map(|(_, check)| check)
        .collect();
    assert_eq!(
        checked.len(),
        samples_of(&outputs, "ins.baro.gnss").count(),
        "a check beside each sample"
    );
    assert!(
        outputs
            .gnss_checks
            .iter()
            .all(|(set, _)| set == "ins.baro.gnss"),
        "ins.baro has no receiver to check"
    );
    // Under a true sky the navigator finds nothing against its receiver.
    let last = checked.last().unwrap();
    tracing::info!(?last, "the check after 30 s of a true sky");
    assert_eq!(last.state, GnssCheckState::Consistent);
    assert!(last.last.is_some(), "a fix was compared with the reference");
}

/// The steering set's verdict on its receiver travels to Pilotage with each
/// solution, and Pilotage's display states it back.
#[test]
fn pilotage_is_told_what_the_steering_navigator_makes_of_its_receiver() {
    let mut on_set = spec(None);
    on_set.steering = Steering::Set("ins.baro.gnss".to_owned());
    let mut trial = ChainTrial::new(&on_set, &DirectoryStores).unwrap();
    advanced(trial.step_until(20.0).unwrap());
    let stated = trial
        .navigation()
        .and_then(|display| display.steering.source_integrity)
        .unwrap();
    assert_eq!(stated.class, navigate_contract::SensorClass::Gnss);
    assert_eq!(stated.state, SourceIntegrityState::Consistent);

    // A set without the receiver has nothing to state of it.
    on_set.steering = Steering::Set("ins.baro".to_owned());
    let mut trial = ChainTrial::new(&on_set, &DirectoryStores).unwrap();
    advanced(trial.step_until(20.0).unwrap());
    let display = trial.navigation().unwrap();
    assert_eq!(display.steering.source_integrity, None);
}
