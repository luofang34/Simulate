//! Checkpoints, and reaching a vessel's own plant type.

use sim_core::TickSpan;
use sim_orbital::{LatchedBurnLaw, PlayerCommandPayload, VesselId};

use super::policy::Held;
use super::{STEP, seconds, universe_with};
use crate::UniverseError;
use crate::local::CoastingRigidBody;

#[test]
fn a_restored_universe_replays_its_orbital_vessels_and_their_commands() {
    let burn = PlayerCommandPayload::SetBurnLaw(LatchedBurnLaw::InertialThrust {
        dir_icrf: [0.0, 0.0, 1.0],
        thrust_n: 5_000.0,
        exhaust_vel_m_s: 3_000.0,
        rw_torque_cmd_body_nm: [0.0; 3],
    });
    let mut universe = universe_with(&[1]);
    universe.promote_to_orbital(VesselId(1)).unwrap();
    universe.command(VesselId(1), seconds(8), burn).unwrap();
    let half_step = TickSpan::from_ticks(STEP.ticks() / 2);
    universe.advance_to(seconds(5).advanced(half_step)).unwrap();
    let saved = universe.checkpoint();
    assert_eq!(saved.at(), seconds(5).advanced(half_step));

    universe.advance_to(seconds(20)).unwrap();
    let first = universe.inertial_state(VesselId(1)).unwrap();
    universe.restore(&saved).unwrap();
    assert_eq!(universe.now(), saved.at());
    universe.advance_to(seconds(20)).unwrap();
    assert_eq!(universe.inertial_state(VesselId(1)).unwrap(), first);
}

#[test]
fn a_checkpoint_is_refused_when_tiers_or_local_plants_have_moved_on() {
    let mut universe = universe_with(&[1, 2]);
    universe.promote_to_orbital(VesselId(2)).unwrap();
    universe.advance_to(seconds(2)).unwrap();
    let saved = universe.checkpoint();

    universe.advance_to(seconds(4)).unwrap();
    assert!(matches!(
        universe.restore(&saved),
        Err(UniverseError::LocalAheadOfCheckpoint {
            vessel: VesselId(1)
        })
    ));
    universe.promote_to_orbital(VesselId(1)).unwrap();
    assert!(matches!(
        universe.restore(&saved),
        Err(UniverseError::TiersChangedSinceCheckpoint)
    ));
    assert_eq!(
        universe.now(),
        seconds(4),
        "a refused restore changed the clock"
    );
}

#[test]
fn a_local_plant_is_reachable_as_the_type_that_built_it() {
    let mut universe = universe_with(&[1]);
    assert!(universe.local::<CoastingRigidBody>(VesselId(1)).is_some());
    assert!(universe.local::<Held>(VesselId(1)).is_none());
    assert!(
        universe
            .local_mut::<CoastingRigidBody>(VesselId(9))
            .is_none()
    );
    universe.promote_to_orbital(VesselId(1)).unwrap();
    assert!(
        universe
            .local_mut::<CoastingRigidBody>(VesselId(1))
            .is_none()
    );
}
