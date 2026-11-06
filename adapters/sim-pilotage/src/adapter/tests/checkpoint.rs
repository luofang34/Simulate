use pilotage_adapter_api::AvionicsSample;

use super::*;

#[test]
fn restoring_a_checkpoint_returns_its_state_in_a_new_epoch() {
    let mut adapter = kinematic();
    adapter.set_surface_elevation_m(Some(850.0));
    adapter.apply_control(&command(body_rate(0.2, 0.0, 0.0, 0.6)));
    adapter.step(StepBudget { ticks: 60 });
    let checkpoint = adapter.checkpoint();
    let saved_bits = truth_bits(&adapter);
    let before = sample(&mut adapter).sim_truth.unwrap().stamp;

    adapter.apply_control(&frame(MOTION_SCOPE, None, vec![ControlAction::Disarm]));
    adapter.set_surface_elevation_m(None);
    adapter.step(StepBudget { ticks: 120 });
    assert_ne!(truth_bits(&adapter), saved_bits);

    let at = adapter.now();
    adapter.restore_checkpoint(&checkpoint, at);

    assert_eq!(truth_bits(&adapter), saved_bits);
    assert_eq!(adapter.now(), at);
    let telemetry = sample(&mut adapter);
    assert_eq!(telemetry.tick, SimTick::new(181));
    let fc = telemetry.fc_state.unwrap();
    assert_eq!(fc.arm_state, 2);
    assert!(fc.landing_approach.is_some());
    let after = telemetry.sim_truth.unwrap().stamp;
    assert_eq!(after.source_epoch, before.source_epoch.wrapping_add(1));
    // The restart zeroes the sequence and the restored state is its first
    // measurement.
    assert_eq!(after.sequence, 1);
    assert!(before.sequence > after.sequence);
}

#[test]
fn a_restored_checkpoint_flies_a_neutral_law_and_replays_identically() {
    let mut adapter = trainer();
    adapter.apply_control(&command(body_rate(0.1, 0.05, 0.0, 0.5)));
    adapter.step(StepBudget { ticks: 60 });
    let checkpoint = adapter.checkpoint();
    let at = adapter.now();

    let mut replay = || {
        adapter.restore_checkpoint(&checkpoint, at);
        adapter.apply_control(&command(body_rate(-0.1, 0.0, 0.0, 0.7)));
        adapter.step(StepBudget { ticks: 90 });
        let bits = truth_bits(&adapter);
        let telemetry = sample(&mut adapter);
        (bits, telemetry.avionics, telemetry.sim_truth.unwrap().stamp)
    };
    let (first_bits, first_avionics, first_stamp) = replay();
    let (second_bits, second_avionics, second_stamp) = replay();

    assert_eq!(first_bits, second_bits);
    assert_eq!(
        second_stamp.source_epoch,
        first_stamp.source_epoch.wrapping_add(1)
    );
    assert_eq!(second_stamp.sequence, first_stamp.sequence);
    // The noise stream restarts with the epoch, so the estimates differ only
    // in their stamps.
    let attitude = |avionics: Option<AvionicsSample>| match avionics.unwrap().attitude {
        Some(AvionicsAttitudeSample::Canonical { quat_wxyz, .. }) => quat_wxyz,
        other => panic!("{other:?}"),
    };
    assert_eq!(attitude(first_avionics), attitude(second_avionics));
}
