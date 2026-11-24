//! A fleet of detumbling magsats is the same bits however the orbital tier
//! is threaded and across a checkpoint taken between two exchanges.

use glam::DVec3;
use sim_orbital::{ContentHash, VesselId};
use sim_universe::Universe;

use super::orbit::{FLIGHT_MAGNETOMETER, World, at_millis, launch, magsat, millis, world};
use super::satellite::MagsatSpec;

const FLEET: [VesselId; 4] = [VesselId(1), VesselId(2), VesselId(3), VesselId(4)];
const END_MS: u64 = 240_000;
/// Half a sample period past an exchange.
const CHECKPOINT_MS: u64 = 100_500;

/// Four satellites spread along the orbit, each with its own tumble and its
/// own magnetometer errors.
#[cfg(test)]
fn fleet(workers: usize) -> World {
    let mut world = world(millis(1_000), workers);
    for (index, id) in (0_u32..).zip(FLEET) {
        let k = f64::from(index);
        let spec = MagsatSpec {
            sample_period: millis(1_000),
            magnetometer: FLIGHT_MAGNETOMETER,
            magnetometer_seed: 100 + u64::from(index),
            torquers_on: true,
            gravity_gradient: true,
        };
        let tumble = DVec3::new(0.05 - 0.01 * k, -0.06, 0.04 + 0.01 * k);
        launch(&mut world, id, spec, 0.7 * k, tumble);
    }
    world
}

/// The orbital tier's full replay hash, every checkpoint's hash and every
/// plant's internals.
#[cfg(test)]
fn fingerprint(universe: &Universe) -> (ContentHash, Vec<ContentHash>, Vec<String>) {
    let orbital = universe.orbital();
    (
        orbital.universe_hash(),
        FLEET
            .iter()
            .map(|&id| orbital.committed_checkpoint(id).unwrap().checkpoint_hash())
            .collect(),
        FLEET
            .iter()
            .map(|&id| magsat(universe, id).internals_fingerprint())
            .collect(),
    )
}

#[test]
fn worker_count_and_a_mid_run_restore_leave_every_bit_alone() {
    let mut single = fleet(1);
    single.universe.advance_to(at_millis(END_MS)).unwrap();
    let reference = fingerprint(&single.universe);
    for id in FLEET {
        let held = single
            .universe
            .orbital()
            .committed_checkpoint(id)
            .unwrap()
            .external
            .loads
            .dipole_body_a_m2();
        assert_ne!(held, DVec3::ZERO, "the torquers are working");
        assert_eq!(
            held.to_array().map(f64::to_bits),
            magsat(&single.universe, id)
                .dipole_a_m2()
                .to_array()
                .map(f64::to_bits),
            "the checkpoint holds the dipole the plant commanded"
        );
    }

    let mut threaded = fleet(4);
    threaded.universe.advance_to(at_millis(END_MS)).unwrap();
    assert_eq!(
        fingerprint(&threaded.universe),
        reference,
        "four workers commit the bits one does"
    );

    // Sliced differently, checkpointed between two exchanges, flown on,
    // taken back and flown again.
    let mut replayed = fleet(4);
    replayed.universe.advance_to(at_millis(37_250)).unwrap();
    replayed
        .universe
        .advance_to(at_millis(CHECKPOINT_MS))
        .unwrap();
    let saved = replayed.universe.checkpoint();
    let at_save = fingerprint(&replayed.universe);
    replayed.universe.advance_to(at_millis(END_MS)).unwrap();
    assert_eq!(fingerprint(&replayed.universe), reference);
    replayed.universe.restore(&saved).unwrap();
    assert_eq!(
        fingerprint(&replayed.universe),
        at_save,
        "the restore puts back the orbital tier and every plant"
    );
    replayed.universe.advance_to(at_millis(END_MS)).unwrap();
    assert_eq!(
        fingerprint(&replayed.universe),
        reference,
        "and the run from the checkpoint is the uninterrupted one"
    );
}
