//! A vessel whose thrust and mass are decided outside the scheduler: a latched
//! body wrench applies force without flowing propellant, and a stamped mass model
//! takes effect at its instant, is part of the replay hash, and survives a snapshot.

use std::sync::Arc;

use glam::{DQuat, DVec3};
use sim_orbital::{
    BodyOrientationCatalog, BuiltinSolarSystem2026_2028, ContentHash, ContinuousPlantState,
    Dop853Tolerances, EphemerisCatalog, ForceModel, LatchedBurnLaw, MassPropertiesModel, NaifId,
    OrbitalUniverse, PlayerCommandPayload, PropellantMode, ReactionWheelMode, SimClockAdapter,
    SimulationManifestDigest, TdbEpoch, VesselCheckpoint, VesselConfig, VesselId,
};
use sim_spacetime::{SimInstant, TickSpan};

const SHIP: VesselId = VesselId(7);
const MASS_KG: f64 = 12_000.0;
const FORCE_N: f64 = 12.0;

#[cfg(test)]
fn seconds(s: u64) -> SimInstant {
    SimInstant::START.advanced(TickSpan::from_seconds_and_subsec_ticks(s, 0))
}

#[cfg(test)]
fn mass_model(mass_kg: f64) -> MassPropertiesModel {
    MassPropertiesModel::diagonal(mass_kg, [42_000.0, 42_000.0, 14_000.0], [0.0; 3])
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
        mass_model: mass_model(MASS_KG),
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

#[test]
fn a_body_wrench_and_a_stamped_mass_model_act_from_their_instants() {
    let wrench = PlayerCommandPayload::SetBurnLaw(LatchedBurnLaw::BodyWrench {
        force_body_n: [FORCE_N, 0.0, 0.0],
        torque_body_nm: [0.0; 3],
    });
    let halved = PlayerCommandPayload::SetMassModel(mass_model(MASS_KG / 2.0));
    let mut coasting = universe();
    let mut pushed = universe();
    let mut pushed_and_lightened = universe();
    for universe in [&mut pushed, &mut pushed_and_lightened] {
        universe
            .stamp_player_command(seconds(10), SHIP, wrench)
            .unwrap();
    }
    pushed_and_lightened
        .stamp_player_command(seconds(20), SHIP, halved)
        .unwrap();
    for universe in [&mut coasting, &mut pushed, &mut pushed_and_lightened] {
        universe.advance_committed_to(seconds(30)).unwrap();
    }

    let gained = |universe: &OrbitalUniverse| {
        let velocity = |u: &OrbitalUniverse| {
            u.sample_vessel_state(SHIP, seconds(30))
                .unwrap()
                .vel_rel_icrf_m_s
        };
        (velocity(universe) - velocity(&coasting)).x
    };
    let per_second = FORCE_N / MASS_KG;
    assert!(
        (gained(&pushed) / (20.0 * per_second) - 1.0).abs() < 1e-3,
        "the wrench pushes from the tenth second: {} m/s",
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
        "a body wrench is not thrust without mass flow: its mass is accounted for elsewhere"
    );
    assert_ne!(
        pushed.universe_hash(),
        pushed_and_lightened.universe_hash(),
        "the stamped mass model is part of the replay identity"
    );
    let saved = pushed_and_lightened.capture_snapshot();
    let mut restored = universe();
    restored.restore_snapshot(&saved).unwrap();
    assert_eq!(
        restored.vessel_config(SHIP).unwrap().mass_model,
        mass_model(MASS_KG / 2.0),
        "a snapshot carries the mass model the vessel has now"
    );
    assert_eq!(
        restored.universe_hash(),
        pushed_and_lightened.universe_hash(),
        "and restores to the same replay identity"
    );
}
