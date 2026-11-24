//! A vessel whose thrust and mass are decided outside the scheduler: externally held
//! loads act from the committed instant they are set at, add to a mission plan's own
//! burn instead of replacing it, never touch the command log, and replay bit for bit
//! across rollback and snapshots.

use std::sync::Arc;

use glam::{DMat3, DQuat, DVec3};
use sim_orbital::{
    BodyOrientationCatalog, BuiltinSolarSystem2026_2028, ContentHash, ContinuousPlantState,
    Dop853Tolerances, EphemerisCatalog, ExternalLoads, ExternalMassProperties, ForceModel,
    LatchedBurnLaw, MassPropertiesModel, NaifId, OrbitalUniverse, PlayerCommandPayload,
    PropellantMode, ReactionWheelMode, SchedulerError, SimClockAdapter, SimulationManifestDigest,
    TdbEpoch, VesselCheckpoint, VesselConfig, VesselId,
};
use sim_spacetime::{SimInstant, TickSpan};

const SHIP: VesselId = VesselId(7);
const MASS_KG: f64 = 12_000.0;
const INERTIA_KG_M2: [f64; 3] = [42_000.0, 42_000.0, 14_000.0];
const FORCE_N: f64 = 12.0;

#[cfg(test)]
fn seconds(s: u64) -> SimInstant {
    SimInstant::START.advanced(TickSpan::from_seconds_and_subsec_ticks(s, 0))
}

#[cfg(test)]
fn own_mass(mass_kg: f64) -> ExternalMassProperties {
    ExternalMassProperties::new(mass_kg, DMat3::from_diagonal(DVec3::from(INERTIA_KG_M2)))
}

/// A body-axis push along `force_body_n` from a vessel weighing `mass_kg`.
#[cfg(test)]
fn pushing(force_body_n: DVec3, mass_kg: f64) -> ExternalLoads {
    ExternalLoads::new(force_body_n, DVec3::ZERO, own_mass(mass_kg))
}

/// One non-rotating vessel in low Earth orbit, body axes along ICRF.
#[cfg(test)]
fn universe() -> OrbitalUniverse {
    let solar = BuiltinSolarSystem2026_2028::load_verified().unwrap();
    let base_tdb = TdbEpoch::new(825_000_000, 0);
    let force_model = ForceModel::standard_solar_system(solar.constants.clone()).unwrap();
    let tolerances = Dop853Tolerances::default();
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
    let mut universe = OrbitalUniverse::new(
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
        bounding_radius_m: 6.0,
        auto_soi_switch: true,
    };
    let plant = ContinuousPlantState {
        primary_body: NaifId::EARTH,
        pos_rel_icrf_m: DVec3::new(6_778_137.0, 0.0, 150_000.0),
        vel_rel_icrf_m_s: DVec3::new(0.0, 7_600.0, 1_200.0),
        q_body_to_icrf: DQuat::IDENTITY,
        omega_body_rad_s: DVec3::ZERO,
        propellant_mass_kg: 0.0,
        h_rw_body_nms: DVec3::ZERO,
    };
    universe
        .insert_vessel(
            config,
            VesselCheckpoint::new(SHIP, SimInstant::START, plant, 3),
        )
        .unwrap();
    universe
}

#[cfg(test)]
fn velocity(universe: &OrbitalUniverse, at: SimInstant) -> DVec3 {
    universe
        .sample_vessel_state(SHIP, at)
        .unwrap()
        .vel_rel_icrf_m_s
}

#[cfg(test)]
fn state_bits(universe: &OrbitalUniverse, at: SimInstant) -> [u64; 17] {
    let state = universe.sample_vessel_state(SHIP, at).unwrap();
    state.to_ode_array().map(f64::to_bits)
}

/// Flies `universe` to `until`, setting `loads(k)` at each ten-second exchange `k` from
/// `from_s` on.
#[cfg(test)]
fn exchange_until(
    universe: &mut OrbitalUniverse,
    from_s: u64,
    until_s: u64,
    loads: impl Fn(u64) -> ExternalLoads,
) {
    for at_s in (from_s..until_s).step_by(10) {
        universe.advance_committed_to(seconds(at_s)).unwrap();
        universe.set_external_loads(SHIP, loads(at_s)).unwrap();
    }
    universe.advance_committed_to(seconds(until_s)).unwrap();
}

#[test]
fn external_loads_and_mass_act_from_the_instant_they_are_set() {
    let mut coasting = universe();
    let mut pushed = universe();
    let mut pushed_and_lightened = universe();
    coasting.advance_committed_to(seconds(30)).unwrap();
    let push = DVec3::X * FORCE_N;
    exchange_until(&mut pushed, 10, 30, |_| pushing(push, MASS_KG));
    exchange_until(&mut pushed_and_lightened, 10, 30, |at_s| {
        pushing(push, if at_s < 20 { MASS_KG } else { MASS_KG / 2.0 })
    });

    let gained =
        |u: &OrbitalUniverse| (velocity(u, seconds(30)) - velocity(&coasting, seconds(30))).x;
    let per_second = FORCE_N / MASS_KG;
    assert!(
        (gained(&pushed) / (20.0 * per_second) - 1.0).abs() < 1e-3,
        "the push acts from the tenth second: {} m/s",
        gained(&pushed)
    );
    assert!(
        (gained(&pushed_and_lightened) / (30.0 * per_second) - 1.0).abs() < 1e-3,
        "half the mass doubles the acceleration from the twentieth: {} m/s",
        gained(&pushed_and_lightened)
    );

    let checkpoint = pushed_and_lightened.committed_checkpoint(SHIP).unwrap();
    assert!(
        !checkpoint.audit.mass_conservation_exempt && !checkpoint.audit.angular_momentum_exempt,
        "an external push is not thrust without mass flow: its mass is accounted for elsewhere"
    );
    assert_eq!(checkpoint.external.loads, pushing(push, MASS_KG / 2.0));
    assert_ne!(
        pushed.universe_hash(),
        pushed_and_lightened.universe_hash(),
        "the external mass is part of the replay identity"
    );
    assert_ne!(
        pushed
            .committed_checkpoint(SHIP)
            .unwrap()
            .physics_state_hash(),
        checkpoint.physics_state_hash(),
        "and of the physical state"
    );
}

#[test]
fn a_plan_burn_and_external_loads_superpose() {
    let burn = PlayerCommandPayload::SetBurnLaw(LatchedBurnLaw::ConstantBody {
        main_thrust_body_n: [0.0, 0.0, 30.0],
        main_exhaust_vel_m_s: 3_000.0,
        rcs_force_body_n: [0.0; 3],
        rcs_exhaust_vel_m_s: 3_000.0,
        rw_torque_cmd_body_nm: [0.0; 3],
    });
    let push = DVec3::new(FORCE_N, -0.5 * FORCE_N, 0.0);
    let fly = |planned: bool, external: bool| {
        let mut u = universe();
        if planned {
            u.stamp_player_command(seconds(10), SHIP, burn).unwrap();
        }
        let loads = if external {
            pushing(push, MASS_KG)
        } else {
            ExternalLoads::default()
        };
        exchange_until(&mut u, 10, 70, |_| loads);
        if planned {
            assert_eq!(
                u.committed_checkpoint(SHIP).unwrap().gnc.latched_law,
                LatchedBurnLaw::ConstantBody {
                    main_thrust_body_n: [0.0, 0.0, 30.0],
                    main_exhaust_vel_m_s: 3_000.0,
                    rcs_force_body_n: [0.0; 3],
                    rcs_exhaust_vel_m_s: 3_000.0,
                    rw_torque_cmd_body_nm: [0.0; 3],
                },
                "external loads leave the plan's burn law in place"
            );
        }
        velocity(&u, seconds(70))
    };
    let coast = fly(false, false);
    let plan_dv = fly(true, false) - coast;
    let external_dv = fly(false, true) - coast;
    let both_dv = fly(true, true) - coast;
    assert!(
        (plan_dv.z / (60.0 * 30.0 / MASS_KG) - 1.0).abs() < 5e-3
            && (external_dv.x / (60.0 * FORCE_N / MASS_KG) - 1.0).abs() < 5e-3,
        "each alone delivers its own impulse, less what the gravity gradient takes over a \
         minute: {plan_dv} and {external_dv} m/s"
    );
    let residual = both_dv - (plan_dv + external_dv);
    assert!(
        residual.length() < 1e-7 * both_dv.length(),
        "together they deliver the sum: {both_dv} against {} m/s",
        plan_dv + external_dv
    );
}

#[test]
fn external_loads_never_grow_the_command_log() {
    let mut universe = universe();
    universe
        .stamp_player_command(seconds(5), SHIP, PlayerCommandPayload::SetGncMode(2))
        .unwrap();
    let mut changed = 0_u32;
    for at_s in (10..1_010).step_by(10) {
        universe.advance_committed_to(seconds(at_s)).unwrap();
        let thrust = if (at_s / 100) % 2 == 0 { FORCE_N } else { 0.0 };
        let loads = pushing(DVec3::X * thrust, MASS_KG - at_s as f64 * 1e-3);
        if universe.set_external_loads(SHIP, loads).unwrap() {
            changed = changed.wrapping_add(1);
        }
        assert!(
            !universe.set_external_loads(SHIP, loads).unwrap(),
            "the same bits again change nothing"
        );
        assert_eq!(universe.command_log().entries().len(), 1);
    }
    assert_eq!(changed, 100);
    let checkpoint = universe.committed_checkpoint(SHIP).unwrap();
    assert_eq!(checkpoint.external.generation, 100);
    let lost = ExternalLoads::new(DVec3::X * f64::NAN, DVec3::ZERO, own_mass(MASS_KG));
    assert_eq!(
        universe.set_external_loads(SHIP, lost),
        Err(SchedulerError::InvalidExternalLoads {
            vessel_id: SHIP,
            instant: seconds(1_000)
        })
    );
    let weightless = pushing(DVec3::ZERO, 0.0);
    assert!(universe.set_external_loads(SHIP, weightless).is_err());
}

/// Thrust that varies between exchanges so every exchange changes the trajectory.
#[cfg(test)]
fn varying(at_s: u64) -> ExternalLoads {
    let phase = at_s as f64 / 100.0;
    let push = DVec3::new(phase.cos(), phase.sin(), 0.25) * FORCE_N;
    ExternalLoads::new(
        push,
        DVec3::new(0.0, 0.0, 0.02 * phase.sin()),
        own_mass(MASS_KG - phase),
    )
}

#[test]
fn rollback_across_an_exchange_replays_the_same_bits() {
    let end = seconds(600);
    let mut reference = universe();
    exchange_until(&mut reference, 10, 600, varying);
    let expected = (reference.universe_hash(), state_bits(&reference, end));

    // Speculation buffered well past each exchange is dropped there and re-flown.
    let mut speculating = universe();
    for at_s in (10..600).step_by(10) {
        let far = seconds(at_s + 900);
        let requests = speculating.build_worker_requests(far);
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
        let buffered = speculating.speculative_segment_count(SHIP).unwrap();
        assert!(
            buffered > 0,
            "speculation reaches past the exchange at {at_s} s"
        );
        assert!(speculating.set_external_loads(SHIP, varying(at_s)).unwrap());
        assert_eq!(speculating.speculative_segment_count(SHIP), Some(0));
    }
    speculating.advance_committed_to(end).unwrap();
    assert_eq!(
        (speculating.universe_hash(), state_bits(&speculating, end)),
        expected
    );

    // A snapshot taken mid-step between exchanges restores to the same continuation.
    let mut interrupted = universe();
    exchange_until(&mut interrupted, 10, 255, varying);
    let saved = interrupted.capture_snapshot();
    let mut restored = universe();
    restored.restore_snapshot(&saved).unwrap();
    assert_eq!(restored.universe_hash(), interrupted.universe_hash());
    exchange_until(&mut restored, 260, 600, varying);
    assert_eq!(
        (restored.universe_hash(), state_bits(&restored, end)),
        expected
    );
}
