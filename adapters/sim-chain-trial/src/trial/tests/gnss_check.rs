use sim_navigate::trial::{DirectoryStores, GnssCheckState};

use super::{advanced, samples_of, spec};
use crate::ChainTrial;

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
