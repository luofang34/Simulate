use std::sync::Arc;

use glam::{DMat3, DQuat, DVec3};
use mbd_core::{ComponentSpec, ComponentState, ComponentTree, Placement, VacuumAero};
use sim_atmosphere::VacuumAtmosphere;
use sim_core::{
    RigidBody6DofBlock, RigidBodyConfig, SimInstant, TICKS_PER_SECOND, TickSpan, VehicleState,
};
use sim_geo::{BodyEllipsoid, UtcTime};
use sim_orbital::{BuiltinSolarSystem2026_2028, Dop853Tolerances, ForceModel, VesselId};
use sim_spacetime::SimEpoch;

use super::{OrbitalContext, Tier, Universe};
use crate::UniverseError;
use crate::local::{CoastingRigidBody, LocalFactory, LocalPlant};

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
    state: &VehicleState,
    start: SimInstant,
    epoch: &SimEpoch,
) -> Result<Box<dyn LocalPlant>, UniverseError> {
    let block = RigidBody6DofBlock::new(
        RigidBodyConfig {
            body: BodyEllipsoid::WGS84,
            period: STEP,
            epoch: epoch.utc_at(start)?,
            initial: *state,
        },
        satellite_tree(),
        Box::new(VacuumAero),
        Box::new(VacuumAtmosphere),
    )?;
    Ok(Box::new(CoastingRigidBody::new(
        block,
        BodyEllipsoid::WGS84,
        start,
    )))
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
        let plant = coaster(&leo_state(), SimInstant::START, &epoch).unwrap();
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
fn handoffs_off_the_quantum_grid_and_duplicates_are_refused() {
    let mut universe = universe_with(&[1]);
    assert!(matches!(
        universe.insert_local(
            VesselId(1),
            coaster(&leo_state(), SimInstant::START, universe.epoch()).unwrap(),
            factory(),
        ),
        Err(UniverseError::DuplicateVessel(_))
    ));
    universe.advance_to(seconds(1).advanced(STEP)).unwrap();
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
