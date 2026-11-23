//! A plant the orbital tier carries: reaching it, bringing it down between
//! exchanges, and what a checkpoint holds of it.

use glam::DVec3;
use mbd_core::{ComponentId, WrenchAtSite};
use sim_core::{MultibodyVehicleInput, SimInstant, TickSpan};
use sim_geo::BodyEllipsoid;
use sim_orbital::VesselId;

use super::{STEP, epoch, leo_state, multibody_coaster, seconds, solar_system_context};
use crate::local::{LocalFactory, LocalPlant, MultibodyLocalPlant};
use crate::tiering::{TierChange, TierPolicy};
use crate::{Tier, Universe, UniverseError};

fn multibody_universe() -> Universe {
    let epoch = epoch();
    let plant = multibody_coaster(
        BodyEllipsoid::WGS84,
        &leo_state(),
        SimInstant::START,
        &epoch,
    )
    .unwrap();
    let factory: LocalFactory = Box::new(|_, _, _, _| Err(UniverseError::NotCarryable));
    let mut universe = Universe::new(epoch, solar_system_context());
    universe.insert_local(VesselId(1), plant, factory).unwrap();
    universe
}

#[test]
fn a_carried_plant_comes_down_between_exchanges_at_its_own_step_instants() {
    let mut universe = multibody_universe();
    universe.promote_to_orbital(VesselId(1)).unwrap();
    assert!(universe.local::<MultibodyLocalPlant>(VesselId(1)).is_none());
    let off_grid = seconds(13).advanced(TickSpan::from_ticks(STEP.ticks() * 3 / 2));
    universe.advance_to(off_grid).unwrap();
    let carried = universe
        .carried::<MultibodyLocalPlant>(VesselId(1))
        .unwrap();
    assert_eq!(
        carried.now(),
        seconds(10),
        "the plant's internals describe its last exchange"
    );
    assert!(matches!(
        universe.demote_to_local(VesselId(1)),
        Err(UniverseError::BetweenSteps {
            vessel: VesselId(1)
        })
    ));
    assert_eq!(universe.tier(VesselId(1)).unwrap(), Tier::Orbital);

    let on_grid = seconds(13).advanced(TickSpan::from_ticks(STEP.ticks() * 2));
    universe.advance_to(on_grid).unwrap();
    // The factory refuses, so only the carried plant can have come back.
    universe.demote_to_local(VesselId(1)).unwrap();
    let plant = universe.local::<MultibodyLocalPlant>(VesselId(1)).unwrap();
    assert_eq!(plant.now(), on_grid);
    universe.advance_to(seconds(20)).unwrap();
    assert_eq!(universe.tier(VesselId(1)).unwrap(), Tier::Local);
}

#[test]
fn a_checkpoint_puts_a_local_multibody_plant_back_itself() {
    let mut universe = multibody_universe();
    universe.advance_to(seconds(2)).unwrap();
    let saved = universe.checkpoint();
    universe.advance_to(seconds(4)).unwrap();
    let first = universe.vehicle_state(VesselId(1)).unwrap();

    universe.restore(&saved).unwrap();
    let plant = universe.local::<MultibodyLocalPlant>(VesselId(1)).unwrap();
    assert_eq!(plant.now(), seconds(2));
    universe.advance_to(seconds(4)).unwrap();
    assert_eq!(universe.vehicle_state(VesselId(1)).unwrap(), first);

    universe.promote_to_orbital(VesselId(1)).unwrap();
    assert!(matches!(
        universe.restore(&saved),
        Err(UniverseError::TiersChangedSinceCheckpoint)
    ));
}

#[test]
fn a_policy_brings_down_a_carried_plant_that_asks_for_flight_dynamics() {
    let mut universe = multibody_universe().with_tier_policy(TierPolicy::standard());
    universe.advance_to(seconds(5)).unwrap();
    assert_eq!(universe.tier(VesselId(1)).unwrap(), Tier::Orbital);
    // An external wrench is a load the carried path does not report.
    let pushed = MultibodyVehicleInput {
        external_wrenches: vec![WrenchAtSite {
            site: ComponentId(0),
            point_local_m: DVec3::ZERO,
            force_local_n: DVec3::ZERO,
            torque_local_nm: DVec3::ZERO,
        }],
        ..MultibodyVehicleInput::default()
    };
    universe
        .carried_mut::<MultibodyLocalPlant>(VesselId(1))
        .unwrap()
        .set_input(pushed);
    universe.advance_to(seconds(30)).unwrap();
    let change = |at, to| TierChange {
        vessel: VesselId(1),
        at: seconds(at),
        to,
    };
    assert_eq!(
        universe.take_tier_changes(),
        vec![change(1, Tier::Orbital), change(11, Tier::Local)],
        "the plant comes down at its next exchange, where its internals describe the instant"
    );
}
