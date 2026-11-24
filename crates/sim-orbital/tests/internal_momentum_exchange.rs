//! A vessel whose owner moves parts inside it: the angular momentum those parts hold
//! turns with the vessel, an angular impulse at an exchange hands the body what the parts
//! took, the two replay bit for bit across rollback and snapshots, and a vessel on a
//! closed-form path refuses loads that path cannot apply.

use std::sync::Arc;

use glam::{DMat3, DQuat, DVec3};
use sim_orbital::{
    BodyOrientationCatalog, BuiltinSolarSystem2026_2028, ContentHash, ContinuousPlantState,
    Dop853Tolerances, EnvironmentLoadSpec, EphemerisCatalog, ExternalLoads, ExternalMassProperties,
    ForceModel, GravityGradient, MassPropertiesModel, NaifId, OrbitalUniverse, PropagatorKind,
    PropellantMode, ReactionWheelMode, SchedulerError, SimClockAdapter, SimulationManifestDigest,
    TdbEpoch, VesselCheckpoint, VesselConfig, VesselId,
};
use sim_spacetime::{SimInstant, TickSpan};

const SHIP: VesselId = VesselId(7);
const MASS_KG: f64 = 500.0;
const INERTIA_KG_M2: [f64; 3] = [200.0, 300.0, 250.0];
const TUMBLE_RAD_S: DVec3 = DVec3::new(2.0e-3, -1.0e-3, 1.5e-3);

#[cfg(test)]
fn seconds(s: u64) -> SimInstant {
    SimInstant::START.advanced(TickSpan::from_seconds_and_subsec_ticks(s, 0))
}

#[cfg(test)]
fn inertia() -> DMat3 {
    DMat3::from_diagonal(DVec3::from(INERTIA_KG_M2))
}

/// The vessel's own mass with `momentum_body_nms` held by its internal parts.
#[cfg(test)]
fn holding(momentum_body_nms: DVec3) -> ExternalLoads {
    let mass = ExternalMassProperties::new(MASS_KG, inertia());
    ExternalLoads::new(DVec3::ZERO, DVec3::ZERO, mass).with_internal_momentum(momentum_body_nms)
}

#[cfg(test)]
fn low_orbit() -> VesselCheckpoint {
    let plant = ContinuousPlantState {
        primary_body: NaifId::EARTH,
        pos_rel_icrf_m: DVec3::new(6_778_137.0, 0.0, 150_000.0),
        vel_rel_icrf_m_s: DVec3::new(0.0, 7_600.0, 1_200.0),
        q_body_to_icrf: DQuat::IDENTITY,
        omega_body_rad_s: TUMBLE_RAD_S,
        propellant_mass_kg: 0.0,
        h_rw_body_nms: DVec3::ZERO,
    };
    VesselCheckpoint::new(SHIP, SimInstant::START, plant, 3)
}

/// An empty universe and the configuration of the one tumbling vessel the tests fly.
#[cfg(test)]
fn universe() -> (OrbitalUniverse, VesselConfig) {
    let solar = BuiltinSolarSystem2026_2028::load_verified().unwrap();
    let base_tdb = TdbEpoch::new(825_000_000, 0);
    let force_model = ForceModel::standard_solar_system(solar.constants.clone()).unwrap();
    let tolerances = Dop853Tolerances {
        rtol: 1.0e-12,
        atol_att_rad: 1.0e-13,
        atol_omega_rad_s: 1.0e-15,
        ..Dop853Tolerances::default()
    };
    let manifest = SimulationManifestDigest::compute(
        base_tdb,
        solar.ephemeris.content_hash(),
        solar.constants.content_hash(),
        solar.orientation.content_hash(),
        solar.time_scales.content_hash(),
        ContentHash::of_bytes(b"no-terrain"),
        force_model.content_hash(),
        tolerances.content_hash(),
    );
    let universe = OrbitalUniverse::new(
        manifest,
        SimClockAdapter::new(base_tdb),
        force_model,
        Arc::new(solar.ephemeris),
        Arc::new(solar.orientation),
    );
    let config = VesselConfig {
        mass_model: MassPropertiesModel::diagonal(MASS_KG, INERTIA_KG_M2, [0.0; 3]),
        propellant_mode: PropellantMode::InfiniteConstantMass,
        rw_mode: ReactionWheelMode::ConserveMomentum {
            h_max_nms: 0.0,
            tau_max_nm: 0.0,
        },
        tolerances,
        bounding_radius_m: 3.0,
        auto_soi_switch: true,
    };
    (universe, config)
}

#[cfg(test)]
fn tumbling() -> OrbitalUniverse {
    let (mut universe, config) = universe();
    universe.insert_vessel(config, low_orbit()).unwrap();
    universe
}

/// Angular momentum of the body and of what it holds inside, in inertial axes.
#[cfg(test)]
fn angular_momentum_icrf(universe: &OrbitalUniverse) -> DVec3 {
    let checkpoint = universe.committed_checkpoint(SHIP).unwrap();
    let held = checkpoint.external.loads.internal_momentum_body_nms();
    let plant = &checkpoint.plant;
    plant.q_body_to_icrf * (inertia() * plant.omega_body_rad_s + held)
}

#[cfg(test)]
fn state_bits(universe: &OrbitalUniverse, at: SimInstant) -> [u64; 17] {
    let state = universe.sample_vessel_state(SHIP, at).unwrap();
    state.to_ode_array().map(f64::to_bits)
}

/// What the internal parts hold at exchange `at_s`: a rotor spun up and down about an
/// axis that is not a principal one.
#[cfg(test)]
fn rotor(at_s: u64) -> DVec3 {
    DVec3::new(0.3, 0.1, 0.25) * (at_s as f64 / 150.0).sin()
}

/// Flies to `until_s`, at each ten-second exchange from `from_s` on holding what the
/// rotor has and handing the body the change since the exchange before.
#[cfg(test)]
fn exchange_until(universe: &mut OrbitalUniverse, from_s: u64, until_s: u64) {
    for at_s in (from_s..until_s).step_by(10) {
        universe.advance_committed_to(seconds(at_s)).unwrap();
        universe
            .set_external_loads(SHIP, holding(rotor(at_s)))
            .unwrap();
        let taken = rotor(at_s) - rotor(at_s - 10);
        universe.exchange_angular_momentum(SHIP, -taken).unwrap();
    }
    universe.advance_committed_to(seconds(until_s)).unwrap();
}

#[test]
fn held_internal_momentum_turns_with_the_vessel() {
    let mut with_rotor = tumbling();
    let held = DVec3::new(0.4, -0.2, 0.3);
    assert!(with_rotor.set_external_loads(SHIP, holding(held)).unwrap());
    let mut without = tumbling();
    let first = angular_momentum_icrf(&with_rotor);
    let mut drift = 0.0_f64;
    for at_s in (60..=1_800).step_by(60) {
        with_rotor.advance_committed_to(seconds(at_s)).unwrap();
        drift = drift.max((angular_momentum_icrf(&with_rotor) - first).length());
    }
    assert!(
        drift < 1e-11 * first.length(),
        "body and rotor together drift {drift} of {} N·m·s",
        first.length()
    );
    // The rotor is a third of the momentum, so the body does not turn as it would alone.
    without.advance_committed_to(seconds(1_800)).unwrap();
    let alone = without.committed_checkpoint(SHIP).unwrap().plant;
    let coupled = with_rotor.committed_checkpoint(SHIP).unwrap().plant;
    assert!(alone.q_body_to_icrf.angle_between(coupled.q_body_to_icrf) > 0.1);
}

#[test]
fn an_angular_impulse_hands_the_body_what_its_parts_took() {
    let mut universe = tumbling();
    universe.advance_committed_to(seconds(100)).unwrap();
    let before = angular_momentum_icrf(&universe);
    let rate_before = universe
        .sample_vessel_state(SHIP, seconds(100))
        .unwrap()
        .omega_body_rad_s;
    let taken = DVec3::new(0.2, 0.05, -0.1);
    assert!(universe.set_external_loads(SHIP, holding(taken)).unwrap());
    assert!(universe.exchange_angular_momentum(SHIP, -taken).unwrap());
    let rate_after = universe
        .committed_checkpoint(SHIP)
        .unwrap()
        .plant
        .omega_body_rad_s;
    let expected = rate_before + inertia().inverse() * (-taken);
    assert!(
        (rate_after - expected).length() < 1e-15,
        "the body turns at {rate_after}, not {expected}"
    );
    let after = angular_momentum_icrf(&universe);
    assert!(
        (after - before).length() < 1e-15 * before.length().max(1.0),
        "the exchange moved the total from {before} to {after}"
    );
    assert!(
        !universe
            .exchange_angular_momentum(SHIP, DVec3::ZERO)
            .unwrap(),
        "no impulse changes nothing"
    );
    assert_eq!(
        universe.exchange_angular_momentum(SHIP, DVec3::X * f64::NAN),
        Err(SchedulerError::InvalidExternalLoads {
            vessel_id: SHIP,
            instant: seconds(100)
        })
    );
    assert_eq!(universe.command_log().entries().len(), 0);
    // Through thirty minutes of exchanges the total stays what it was. The rotor's
    // account starts at the exchange before, so the first one hands over the difference.
    universe
        .exchange_angular_momentum(SHIP, taken - rotor(100))
        .unwrap();
    universe
        .set_external_loads(SHIP, holding(rotor(100)))
        .unwrap();
    exchange_until(&mut universe, 110, 1_900);
    let drift = (angular_momentum_icrf(&universe) - before).length();
    assert!(
        drift < 1e-11 * before.length(),
        "body and rotor together drift {drift} of {} N·m·s",
        before.length()
    );
}

#[test]
fn rollback_across_a_momentum_exchange_replays_the_same_bits() {
    let end = seconds(600);
    let mut reference = tumbling();
    exchange_until(&mut reference, 10, 600);
    let expected = (reference.universe_hash(), state_bits(&reference, end));

    // Speculation buffered well past each exchange is dropped there and re-flown.
    let mut speculating = tumbling();
    for at_s in (10..600).step_by(10) {
        let requests = speculating.build_worker_requests(seconds(at_s + 900));
        let proposals: Vec<_> = requests
            .iter()
            .map(|(cp, cfg, horizon, len)| {
                speculating
                    .evaluate_worker_request(cp, cfg, *horizon, *len)
                    .unwrap()
            })
            .collect();
        speculating.ingest_proposals(proposals);
        speculating.advance_committed_to(seconds(at_s)).unwrap();
        assert!(speculating.speculative_segment_count(SHIP).unwrap() > 0);
        speculating
            .set_external_loads(SHIP, holding(rotor(at_s)))
            .unwrap();
        let taken = rotor(at_s) - rotor(at_s - 10);
        assert!(speculating.exchange_angular_momentum(SHIP, -taken).unwrap());
        assert_eq!(speculating.speculative_segment_count(SHIP), Some(0));
    }
    speculating.advance_committed_to(end).unwrap();
    assert_eq!(
        (speculating.universe_hash(), state_bits(&speculating, end)),
        expected
    );

    // A snapshot taken mid-step between exchanges restores to the same continuation.
    let mut interrupted = tumbling();
    exchange_until(&mut interrupted, 10, 255);
    let saved = interrupted.capture_snapshot();
    let mut restored = tumbling();
    restored.restore_snapshot(&saved).unwrap();
    assert_eq!(restored.universe_hash(), interrupted.universe_hash());
    exchange_until(&mut restored, 260, 600);
    assert_eq!(
        (restored.universe_hash(), state_bits(&restored, end)),
        expected
    );
}

#[cfg(test)]
fn refused(propagator: PropagatorKind, instant: SimInstant) -> SchedulerError {
    SchedulerError::LoadsNotPropagated {
        vessel_id: SHIP,
        propagator,
        instant,
    }
}

#[test]
fn a_closed_form_path_refuses_loads_it_cannot_apply() {
    for propagator in [
        PropagatorKind::DormantKepler,
        PropagatorKind::PinnedSurfaceRest,
    ] {
        let on_path = || {
            let mut checkpoint = low_orbit();
            checkpoint.integrator_hint.propagator = propagator;
            checkpoint
        };
        // Loads the vessel arrives with.
        let (mut universe, config) = universe();
        let mut pushed = on_path();
        pushed.external.loads = holding(DVec3::ZERO).with_dipole(DVec3::Z);
        assert_eq!(
            universe.insert_vessel(config, pushed),
            Err(refused(propagator, SimInstant::START))
        );
        let in_gradient =
            on_path().with_environment_load(EnvironmentLoadSpec::GravityGradient(GravityGradient));
        assert_eq!(
            universe.insert_vessel(config, in_gradient),
            Err(refused(propagator, SimInstant::START))
        );
        if propagator == PropagatorKind::PinnedSurfaceRest {
            continue;
        }
        // Loads it is given on the way. Its own mass alone is no load.
        universe.insert_vessel(config, on_path()).unwrap();
        universe.advance_committed_to(seconds(120)).unwrap();
        assert!(
            universe
                .set_external_loads(SHIP, holding(DVec3::ZERO))
                .unwrap()
        );
        let mass = ExternalMassProperties::new(MASS_KG, inertia());
        for loads in [
            ExternalLoads::new(DVec3::X, DVec3::ZERO, mass),
            ExternalLoads::new(DVec3::ZERO, DVec3::Y, mass),
            holding(DVec3::ZERO).with_dipole(DVec3::Z),
            holding(DVec3::X),
        ] {
            assert_eq!(
                universe.set_external_loads(SHIP, loads),
                Err(refused(propagator, seconds(120)))
            );
        }
        assert_eq!(
            universe.exchange_angular_momentum(SHIP, DVec3::X),
            Err(refused(propagator, seconds(120)))
        );
        universe.advance_committed_to(seconds(240)).unwrap();
    }
}
