//! Tier policies: vessels that change tier by themselves, and those that
//! must not.

use glam::{DQuat, DVec3};
use sim_core::{SimInstant, TICKS_PER_SECOND, TickSpan, VehicleState};
use sim_geo::BodyEllipsoid;
use sim_orbital::VesselId;

use super::{coaster, epoch, factory, leo_state, seconds, solar_system_context};
use crate::UniverseError;
use crate::local::{LocalPlant, OrbitalBody};
use crate::tiering::{TierChange, TierPolicy};
use crate::universe::{Tier, Universe};

/// A 120 km by 400 km orbit, starting at apogee: it dips through the
/// standard policy's Earth band once a revolution.
fn dipping_orbit() -> VehicleState {
    let apogee = BodyEllipsoid::WGS84.semi_major_m + 400_000.0;
    let perigee = BodyEllipsoid::WGS84.semi_major_m + 120_000.0;
    let semi_major = 0.5 * (apogee + perigee);
    let speed = (BodyEllipsoid::WGS84.mu_m3_s2 * (2.0 / apogee - 1.0 / semi_major)).sqrt();
    let position = DVec3::new(apogee, 0.0, 0.0);
    let spin = DVec3::Z * BodyEllipsoid::WGS84.omega_rad_s;
    VehicleState {
        position_ecef: position,
        velocity_ecef: DVec3::new(0.0, speed, 0.0) - spin.cross(position),
        attitude: DQuat::IDENTITY,
        angular_rate_body: DVec3::ZERO,
    }
}

fn fly_dipping_orbit(slice: TickSpan) -> (Vec<TierChange>, VehicleState) {
    let epoch = epoch();
    let mut universe = Universe::new(epoch.clone(), solar_system_context())
        .with_tier_policy(TierPolicy::standard());
    let plant = coaster(
        BodyEllipsoid::WGS84,
        &dipping_orbit(),
        SimInstant::START,
        &epoch,
    )
    .unwrap();
    universe
        .insert_local(VesselId(1), plant, factory())
        .unwrap();
    let end = seconds(5_400);
    let mut t = SimInstant::START;
    while t < end {
        t = t.advanced(slice).min(end);
        universe.advance_to(t).unwrap();
    }
    let state = universe.vehicle_state(VesselId(1)).unwrap();
    (universe.take_tier_changes(), state)
}

#[test]
fn a_policy_moves_a_vessel_between_tiers_as_it_dips_into_the_band() {
    let (changes, state) = fly_dipping_orbit(TickSpan::from_seconds_and_subsec_ticks(600, 0));
    let tiers: Vec<Tier> = changes.iter().map(|change| change.to).collect();
    assert_eq!(tiers, [Tier::Orbital, Tier::Local, Tier::Orbital]);
    assert_eq!(changes[0].at, seconds(1), "promoted at the first check");
    assert!(changes[1].at > seconds(1_500) && changes[1].at < seconds(2_700));
    assert!(changes[2].at > seconds(2_700) && changes[2].at < seconds(3_900));

    let (changes_sliced, state_sliced) =
        fly_dipping_orbit(TickSpan::from_ticks(TICKS_PER_SECOND / 144 * 1_051));
    assert_eq!(changes, changes_sliced, "tier changes depend on slicing");
    assert_eq!(state, state_sliced, "and so does the vessel");
}

pub(super) struct Held(Box<dyn LocalPlant>);

impl LocalPlant for Held {
    fn body(&self) -> BodyEllipsoid {
        self.0.body()
    }
    fn period(&self) -> TickSpan {
        self.0.period()
    }
    fn now(&self) -> SimInstant {
        self.0.now()
    }
    fn state(&self) -> VehicleState {
        self.0.state()
    }
    fn orbital_body(&self) -> Option<OrbitalBody> {
        self.0.orbital_body()
    }
    fn holds_local_tier(&self) -> bool {
        true
    }
    fn as_any(&self) -> &dyn std::any::Any {
        self
    }
    fn as_any_mut(&mut self) -> &mut dyn std::any::Any {
        self
    }
    fn step(&mut self) -> Result<(), UniverseError> {
        self.0.step()
    }
}

#[test]
fn a_vessel_that_holds_its_local_tier_is_left_there() {
    let epoch = epoch();
    let mut universe = Universe::new(epoch.clone(), solar_system_context())
        .with_tier_policy(TierPolicy::standard());
    let free = coaster(
        BodyEllipsoid::WGS84,
        &leo_state(),
        SimInstant::START,
        &epoch,
    )
    .unwrap();
    let held = coaster(
        BodyEllipsoid::WGS84,
        &leo_state(),
        SimInstant::START,
        &epoch,
    )
    .unwrap();
    universe.insert_local(VesselId(1), free, factory()).unwrap();
    universe
        .insert_local(VesselId(2), Box::new(Held(held)), factory())
        .unwrap();
    universe.advance_to(seconds(5)).unwrap();
    assert_eq!(universe.tier(VesselId(1)).unwrap(), Tier::Orbital);
    assert_eq!(universe.tier(VesselId(2)).unwrap(), Tier::Local);
    assert_eq!(universe.take_tier_changes().len(), 1);
    assert!(universe.take_tier_changes().is_empty());
}

/// A plant only flight dynamics can fly, as an aircraft is.
struct Airborne(Box<dyn LocalPlant>);

impl LocalPlant for Airborne {
    fn body(&self) -> BodyEllipsoid {
        self.0.body()
    }
    fn period(&self) -> TickSpan {
        self.0.period()
    }
    fn now(&self) -> SimInstant {
        self.0.now()
    }
    fn state(&self) -> VehicleState {
        self.0.state()
    }
    fn orbital_body(&self) -> Option<OrbitalBody> {
        None
    }
    fn holds_local_tier(&self) -> bool {
        false
    }
    fn as_any(&self) -> &dyn std::any::Any {
        self
    }
    fn as_any_mut(&mut self) -> &mut dyn std::any::Any {
        self
    }
    fn step(&mut self) -> Result<(), UniverseError> {
        self.0.step()
    }
}

#[test]
fn a_plant_only_flight_dynamics_can_fly_never_goes_orbital() {
    let epoch = epoch();
    let mut universe = Universe::new(epoch.clone(), solar_system_context())
        .with_tier_policy(TierPolicy::standard());
    let plant = coaster(
        BodyEllipsoid::WGS84,
        &leo_state(),
        SimInstant::START,
        &epoch,
    )
    .unwrap();
    universe
        .insert_local(VesselId(1), Box::new(Airborne(plant)), factory())
        .unwrap();
    universe.advance_to(seconds(3)).unwrap();
    assert_eq!(universe.tier(VesselId(1)).unwrap(), Tier::Local);
    assert!(universe.take_tier_changes().is_empty());
    assert!(matches!(
        universe.promote_to_orbital(VesselId(1)),
        Err(UniverseError::LocalOnly(VesselId(1)))
    ));
}
