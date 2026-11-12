use sim_navigate::trial::DirectoryStores;

use super::{advanced, samples_of, spec};
use crate::{ChainTrial, Steering};

#[test]
fn the_steered_solution_is_the_steering_sets_and_none_on_the_flight_controller() {
    let on_controller = ChainTrial::new(&spec(None), &DirectoryStores).unwrap();
    assert_eq!(on_controller.steered(), None);

    let mut on_set = spec(None);
    on_set.steering = Steering::Set("ins.baro".to_owned());
    let mut trial = ChainTrial::new(&on_set, &DirectoryStores).unwrap();
    let before = trial.steered().unwrap();
    assert_eq!((before.set, before.solution), ("ins.baro", None));
    let outputs = advanced(trial.step_until(10.0).unwrap());
    let steered = trial.steered().unwrap();
    let solution = steered.solution.unwrap();
    // The solution handed to Pilotage is the set's own: the progress stream's sample
    // of the same step states the same position.
    let last = samples_of(&outputs, "ins.baro").last().unwrap();
    assert_eq!(
        (last.estimate.lat_deg, last.estimate.lon_deg),
        (
            solution.position.latitude_rad.to_degrees(),
            solution.position.longitude_rad.to_degrees()
        )
    );
}
