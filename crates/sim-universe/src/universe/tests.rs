use std::sync::Arc;

use glam::{DMat3, DQuat, DVec3};
use mbd_core::{ComponentSpec, ComponentState, ComponentTree, Placement, VacuumAero};
use sim_atmosphere::VacuumAtmosphere;
use sim_core::{
    RigidBody6DofBlock, RigidBodyConfig, SimInstant, TICKS_PER_SECOND, TickSpan, VehicleState,
};
use sim_geo::{BodyEllipsoid, UtcTime};
use sim_orbital::{
    BuiltinSolarSystem2026_2028, Dop853Tolerances, ForceModel, LatchedBurnLaw, NaifId,
    PlayerCommandPayload, VesselId,
};
use sim_spacetime::SimEpoch;

use super::{OrbitalContext, Tier, Universe};
use crate::UniverseError;
use crate::local::{CoastingRigidBody, LocalFactory, LocalPlant};
use crate::tiering::{TierChange, TierPolicy};

/// `2027-03-01T00:00:00Z`.
const START_UTC_UNIX_S: f64 = 1_803_859_200.0;
const STEP: TickSpan = TickSpan::from_ticks(TICKS_PER_SECOND / 10);
const QUANTUM: TickSpan = TickSpan::from_ticks(TICKS_PER_SECOND);

fn seconds(s: u64) -> SimInstant {
    SimInstant::START.advanced(TickSpan::from_seconds_and_subsec_ticks(s, 0))
}

fn epoch() -> SimEpoch {
    SimEpoch::builtin_from_utc(UtcTime::from_unix_seconds(START_UTC_UNIX_S)).unwrap()
}

/// The full built-in solar system. An Earth-centred propagation must carry
/// the Sun and Moon: the frame's own acceleration towards them is always
/// applied, and only their pull on the vessel cancels it. Against
/// `RigidBody6DofBlock`, which flies Earth's monopole and `J2` alone, the
/// remaining difference is their tide, about `1e-6` m/s² in low orbit.
fn solar_system_context() -> OrbitalContext {
    let solar = BuiltinSolarSystem2026_2028::load_verified().unwrap();
    let force_model = ForceModel::standard_solar_system(solar.constants.clone()).unwrap();
    OrbitalContext {
        ephemeris: Arc::new(solar.ephemeris),
        orientation: Arc::new(solar.orientation),
        force_model,
        quantum: QUANTUM,
        tolerances: Dop853Tolerances {
            rtol: 1e-12,
            atol_pos_m: 1e-6,
            atol_vel_m_s: 1e-9,
            ..Dop853Tolerances::default()
        },
    }
}

fn satellite_tree() -> ComponentTree {
    ComponentTree::new([(
        ComponentSpec {
            name: "bus".to_owned(),
            placement: Placement::ROOT,
            dry_mass_kg: 500.0,
            dry_inertia_body: DMat3::from_diagonal(DVec3::new(200.0, 300.0, 250.0)),
            thermal_capacity_j_k: 1.0,
            nominal_power_w: 0.0,
        },
        ComponentState::at_temperature(290.0),
    )])
    .unwrap()
}

fn coaster(
    body: BodyEllipsoid,
    state: &VehicleState,
    start: SimInstant,
    epoch: &SimEpoch,
) -> Result<Box<dyn LocalPlant>, UniverseError> {
    let block = RigidBody6DofBlock::new(
        RigidBodyConfig {
            body,
            period: STEP,
            epoch: epoch.utc_at(start)?,
            initial: *state,
        },
        satellite_tree(),
        Box::new(VacuumAero),
        Box::new(VacuumAtmosphere),
    )?;
    Ok(Box::new(CoastingRigidBody::new(block, body, start, 2.0)))
}

fn factory() -> LocalFactory {
    Box::new(coaster)
}

/// A 400 km circular orbit's state in the rotating frame, inclined 51.6°.
fn leo_state() -> VehicleState {
    let r = 6_778_137.0;
    let v_inertial = (BodyEllipsoid::WGS84.mu_m3_s2 / r).sqrt();
    let inc = 51.6_f64.to_radians();
    let position = DVec3::new(r, 0.0, 0.0);
    let velocity_inertial = DVec3::new(0.0, inc.cos(), inc.sin()) * v_inertial;
    let spin = DVec3::Z * BodyEllipsoid::WGS84.omega_rad_s;
    VehicleState {
        position_ecef: position,
        velocity_ecef: velocity_inertial - spin.cross(position),
        attitude: DQuat::IDENTITY,
        angular_rate_body: DVec3::new(0.0, 0.001, 0.0),
    }
}

fn universe_with(ids: &[u64]) -> Universe {
    let epoch = epoch();
    let mut universe = Universe::new(epoch.clone(), solar_system_context());
    for &id in ids {
        let plant = coaster(
            BodyEllipsoid::WGS84,
            &leo_state(),
            SimInstant::START,
            &epoch,
        )
        .unwrap();
        universe
            .insert_local(VesselId(id), plant, factory())
            .unwrap();
    }
    universe
}

#[test]
fn flight_and_orbital_dynamics_agree_across_a_handoff() {
    let mut flown_locally = universe_with(&[1]);
    let mut handed_off = universe_with(&[1]);
    handed_off.promote_to_orbital(VesselId(1)).unwrap();
    assert_eq!(handed_off.tier(VesselId(1)).unwrap(), Tier::Orbital);

    let end = seconds(300);
    flown_locally.advance_to(end).unwrap();
    handed_off.advance_to(end).unwrap();
    handed_off.demote_to_local(VesselId(1)).unwrap();

    let local = flown_locally.vehicle_state(VesselId(1)).unwrap();
    let orbital = handed_off.vehicle_state(VesselId(1)).unwrap();
    let miss = (local.position_ecef - orbital.position_ecef).length();
    let speed_miss = (local.velocity_ecef - orbital.velocity_ecef).length();
    // Sun and Moon tides over 300 s: 0.5 * ~1.5e-6 m/s² * 300² ≈ 0.07 m.
    assert!(miss < 0.2, "the tiers disagree by {miss} m after 300 s");
    assert!(speed_miss < 1e-3, "and by {speed_miss} m/s");
    let attitude_miss = local.attitude.angle_between(orbital.attitude);
    assert!(
        attitude_miss < 1e-6,
        "attitude differs by {attitude_miss} rad"
    );
}

#[test]
fn results_do_not_depend_on_how_time_is_sliced() {
    let mut one_call = universe_with(&[1, 2]);
    let mut many_calls = universe_with(&[1, 2]);
    one_call.promote_to_orbital(VesselId(2)).unwrap();
    many_calls.promote_to_orbital(VesselId(2)).unwrap();

    one_call.advance_to(seconds(30)).unwrap();
    let mut t = SimInstant::START;
    let frame = TickSpan::from_ticks(TICKS_PER_SECOND / 144);
    while t < seconds(30) {
        t = t.advanced(frame).min(seconds(30));
        many_calls.advance_to(t).unwrap();
    }
    for id in [VesselId(1), VesselId(2)] {
        assert_eq!(
            one_call.vehicle_state(id).unwrap(),
            many_calls.vehicle_state(id).unwrap(),
            "{id:?} depends on frame slicing"
        );
    }
}

#[test]
fn handoffs_between_local_steps_and_duplicates_are_refused() {
    let mut universe = universe_with(&[1]);
    assert!(matches!(
        universe.insert_local(
            VesselId(1),
            coaster(
                BodyEllipsoid::WGS84,
                &leo_state(),
                SimInstant::START,
                universe.epoch()
            )
            .unwrap(),
            factory(),
        ),
        Err(UniverseError::DuplicateVessel(_))
    ));
    let half_step = TickSpan::from_ticks(STEP.ticks() / 2);
    universe.advance_to(seconds(1).advanced(half_step)).unwrap();
    assert!(matches!(
        universe.promote_to_orbital(VesselId(1)),
        Err(UniverseError::BetweenSteps { .. })
    ));
    assert!(matches!(
        universe.demote_to_local(VesselId(1)),
        Err(UniverseError::AlreadyInTier(_))
    ));
    assert!(matches!(
        universe.advance_to(SimInstant::START),
        Err(UniverseError::Backwards)
    ));
}

#[test]
fn a_vessel_can_change_tier_at_any_instant_its_local_step_reaches() {
    let mut universe = universe_with(&[1]);
    let off_grid = seconds(1).advanced(STEP);
    universe.advance_to(off_grid).unwrap();
    let before = universe.vehicle_state(VesselId(1)).unwrap();
    universe.promote_to_orbital(VesselId(1)).unwrap();
    let promoted = universe.vehicle_state(VesselId(1)).unwrap();
    assert!((promoted.position_ecef - before.position_ecef).length() < 1e-6);
    assert!((promoted.velocity_ecef - before.velocity_ecef).length() < 1e-9);

    let later = off_grid.advanced(TickSpan::from_ticks(TICKS_PER_SECOND / 3));
    universe.advance_to(later).unwrap();
    let orbital = universe.vehicle_state(VesselId(1)).unwrap();
    universe.demote_to_local(VesselId(1)).unwrap();
    assert_eq!(universe.tier(VesselId(1)).unwrap(), Tier::Local);
    let demoted = universe.vehicle_state(VesselId(1)).unwrap();
    let jump = (demoted.position_ecef - orbital.position_ecef).length();
    assert!(jump < 1e-3, "the vessel jumped {jump} m at the handoff");
}

#[test]
fn a_commanded_burn_starts_at_its_instant_however_time_is_sliced() {
    let thrust_n = 5_000.0;
    let burn = PlayerCommandPayload::SetBurnLaw(LatchedBurnLaw::InertialThrust {
        dir_icrf: [0.0, 0.0, 1.0],
        thrust_n,
        exhaust_vel_m_s: 3_000.0,
        rw_torque_cmd_body_nm: [0.0; 3],
    });
    let ignition = seconds(2).advanced(TickSpan::from_ticks(TICKS_PER_SECOND / 7));
    let fly = |slice: TickSpan, burning: bool| {
        let mut universe = universe_with(&[1]);
        universe.promote_to_orbital(VesselId(1)).unwrap();
        if burning {
            universe.command(VesselId(1), ignition, burn).unwrap();
        }
        let mut t = SimInstant::START;
        while t < seconds(12) {
            t = t.advanced(slice).min(seconds(12));
            universe.advance_to(t).unwrap();
        }
        universe.inertial_state(VesselId(1)).unwrap()
    };
    let coasting = fly(TickSpan::from_ticks(TICKS_PER_SECOND), false);
    let burned = fly(TickSpan::from_ticks(TICKS_PER_SECOND), true);
    let burned_finely = fly(TickSpan::from_ticks(TICKS_PER_SECOND / 144), true);
    assert_eq!(burned, burned_finely, "the burn depends on frame slicing");

    let burn_s = seconds(12).duration_since(ignition).as_seconds();
    let gained = (burned.vel_rel_icrf_m_s - coasting.vel_rel_icrf_m_s).z;
    let expected = thrust_n / 500.0 * burn_s;
    assert!(
        (gained - expected).abs() < 1e-3 * expected,
        "gained {gained} m/s along the thrust, expected {expected}"
    );
}

#[test]
fn switching_central_body_moves_the_origin_and_not_the_vessel() {
    let mut universe = universe_with(&[1]);
    assert!(matches!(
        universe.command(
            VesselId(1),
            seconds(1),
            PlayerCommandPayload::SwitchPrimary(NaifId::MOON)
        ),
        Err(UniverseError::NotOrbital(_))
    ));
    universe.promote_to_orbital(VesselId(1)).unwrap();
    let switch = seconds(3);
    universe.advance_to(switch).unwrap();
    assert_eq!(universe.central_body(VesselId(1)).unwrap(), NaifId::EARTH);
    let before = universe
        .orbital()
        .sample_vessel_barycentric(VesselId(1), switch)
        .unwrap();

    universe
        .command(
            VesselId(1),
            switch,
            PlayerCommandPayload::SwitchPrimary(NaifId::MOON),
        )
        .unwrap();
    universe.advance_to(switch).unwrap();
    assert_eq!(universe.central_body(VesselId(1)).unwrap(), NaifId::MOON);
    let after = universe
        .orbital()
        .sample_vessel_barycentric(VesselId(1), switch)
        .unwrap();
    let jump = (after.pos_m - before.pos_m).length();
    let speed_jump = (after.vel_m_s - before.vel_m_s).length();
    assert!(
        jump < 1e-3,
        "the vessel moved {jump} m at the origin switch"
    );
    assert!(speed_jump < 1e-6, "and changed speed by {speed_jump} m/s");

    // Far outside the Moon's sphere of influence, the scheduler takes the
    // vessel back to Earth's origin on its own.
    universe.advance_to(seconds(10)).unwrap();
    assert_eq!(universe.central_body(VesselId(1)).unwrap(), NaifId::EARTH);
    universe.demote_to_local(VesselId(1)).unwrap();
    let altitude = universe
        .vehicle_state(VesselId(1))
        .unwrap()
        .position_ecef
        .length()
        - BodyEllipsoid::WGS84.semi_major_m;
    assert!(
        (altitude - 400_000.0).abs() < 5_000.0,
        "back in flight dynamics {altitude} m above Earth"
    );
}

#[test]
fn a_heliocentric_vessel_has_no_body_fixed_state() {
    let mut universe = universe_with(&[1]);
    universe.promote_to_orbital(VesselId(1)).unwrap();
    universe
        .command(
            VesselId(1),
            SimInstant::START,
            PlayerCommandPayload::SwitchPrimary(NaifId::SSB),
        )
        .unwrap();
    universe.advance_to(SimInstant::START).unwrap();
    assert_eq!(universe.central_body(VesselId(1)).unwrap(), NaifId::SSB);
    assert!(matches!(
        universe.vehicle_state(VesselId(1)),
        Err(UniverseError::UnmodelledBody)
    ));
    assert!(matches!(
        universe.demote_to_local(VesselId(1)),
        Err(UniverseError::UnmodelledBody)
    ));
    assert_eq!(universe.tier(VesselId(1)).unwrap(), Tier::Orbital);
}

#[test]
fn a_vessel_comes_down_about_the_body_it_orbits() {
    let moon = BodyEllipsoid::MOON_IAU;
    let r = moon.semi_major_m + 100_000.0;
    let low_lunar_orbit = VehicleState {
        position_ecef: DVec3::new(r, 0.0, 0.0),
        velocity_ecef: DVec3::new(0.0, 0.0, (moon.mu_m3_s2 / r).sqrt()),
        attitude: DQuat::IDENTITY,
        angular_rate_body: DVec3::ZERO,
    };
    let epoch = epoch();
    let mut universe = Universe::new(epoch.clone(), solar_system_context());
    let plant = coaster(moon, &low_lunar_orbit, SimInstant::START, &epoch).unwrap();
    universe
        .insert_local(VesselId(1), plant, factory())
        .unwrap();
    assert_eq!(universe.central_body(VesselId(1)).unwrap(), NaifId::MOON);

    universe.promote_to_orbital(VesselId(1)).unwrap();
    universe.advance_to(seconds(60)).unwrap();
    assert_eq!(universe.central_body(VesselId(1)).unwrap(), NaifId::MOON);
    universe.demote_to_local(VesselId(1)).unwrap();

    let state = universe.vehicle_state(VesselId(1)).unwrap();
    let altitude = state.position_ecef.length() - moon.semi_major_m;
    assert!(
        (altitude - 100_000.0).abs() < 2_000.0,
        "rebuilt {altitude} m above the Moon"
    );
    assert_eq!(universe.central_body(VesselId(1)).unwrap(), NaifId::MOON);
}

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

struct Held(Box<dyn LocalPlant>);

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
    fn mass_and_inertia(&self) -> (f64, DMat3) {
        self.0.mass_and_inertia()
    }
    fn bounding_radius_m(&self) -> f64 {
        self.0.bounding_radius_m()
    }
    fn holds_local_tier(&self) -> bool {
        true
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
