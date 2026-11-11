//! Speculation horizons that follow a vessel's own step: a quiescent vessel takes
//! natural steps, commands inside a long segment still act at their instants,
//! results stay independent of how time is sliced, and a vessel beside another
//! or switching origin keeps the quantum cadence.

use std::sync::Arc;

use glam::{DQuat, DVec3};
use sim_orbital::{
    BodyOrientationCatalog, BuiltinSolarSystem2026_2028, ContentHash, ContinuousPlantState,
    Dop853Tolerances, EphemerisCatalog, ForceModel, LatchedBurnLaw, MassPropertiesModel, NaifId,
    OrbitalUniverse, PlayerCommandPayload, PropellantMode, ReactionWheelMode, SimClockAdapter,
    SimulationManifestDigest, TdbEpoch, VesselCheckpoint, VesselConfig, VesselId, phase_ranks,
};
use sim_spacetime::{SimInstant, TICKS_PER_SECOND, TickSpan};

const TARGET: VesselId = VesselId(1);
const ESCORT: VesselId = VesselId(2);
/// The quantum the game's orbital tier speculates on.
const QUANTUM_S: u64 = 60;

#[cfg(test)]
fn universe(solar: &BuiltinSolarSystem2026_2028) -> OrbitalUniverse {
    let base_tdb = TdbEpoch::new(825_000_000, 0);
    let fm = ForceModel::standard_solar_system(solar.constants.clone()).expect("force model");
    let manifest = SimulationManifestDigest::compute(
        base_tdb,
        solar.ephemeris.content_hash(),
        solar.constants.content_hash(),
        solar.orientation.content_hash(),
        solar.time_scales.content_hash(),
        ContentHash::of_bytes(b"terrain-none"),
        fm.content_hash(),
        Dop853Tolerances::default().content_hash(),
    );
    OrbitalUniverse::new(
        manifest,
        SimClockAdapter::new(base_tdb),
        fm,
        Arc::new(solar.ephemeris.clone()),
        Arc::new(solar.orientation.clone()),
    )
    .with_horizon_quantum_ticks(u128::from(QUANTUM_S) * TICKS_PER_SECOND as u128)
}

#[cfg(test)]
fn config(auto_soi_switch: bool) -> VesselConfig {
    VesselConfig {
        mass_model: MassPropertiesModel::diagonal(
            12_000.0,
            [42_000.0, 42_000.0, 14_000.0],
            [0.0; 3],
        ),
        propellant_mode: PropellantMode::InfiniteConstantMass,
        rw_mode: ReactionWheelMode::ConserveMomentum {
            h_max_nms: 0.0,
            tau_max_nm: 0.0,
        },
        tolerances: Dop853Tolerances::default(),
        bounding_radius_m: 6.0,
        auto_soi_switch,
    }
}

#[cfg(test)]
fn plant(pos: DVec3, vel: DVec3) -> ContinuousPlantState {
    ContinuousPlantState {
        primary_body: NaifId::EARTH,
        pos_rel_icrf_m: pos,
        vel_rel_icrf_m_s: vel,
        q_body_to_icrf: DQuat::IDENTITY,
        omega_body_rad_s: DVec3::ZERO,
        propellant_mass_kg: 0.0,
        h_rw_body_nms: DVec3::ZERO,
    }
}

#[cfg(test)]
fn at(seconds: f64) -> SimInstant {
    SimInstant::START.advanced(TickSpan::from_ticks_u128(
        (seconds * TICKS_PER_SECOND as f64).round() as u128,
    ))
}

/// Flies `TARGET` (and, with `escorted`, a second vessel a kilometre beside it) from
/// `start`, advancing in slices of `slice_s`, to `end_s`, and returns `TARGET`'s state
/// with the universe it ended in.
#[cfg(test)]
fn fly(
    solar: &BuiltinSolarSystem2026_2028,
    start: ContinuousPlantState,
    auto_soi_switch: bool,
    escorted: bool,
    burn: Option<(f64, f64)>,
    slice_s: f64,
    end_s: f64,
) -> (ContinuousPlantState, OrbitalUniverse) {
    let mut uni = universe(solar);
    uni.insert_vessel(
        config(auto_soi_switch),
        VesselCheckpoint::new(TARGET, SimInstant::START, start, 7),
    )
    .expect("insert");
    if escorted {
        let beside = ContinuousPlantState {
            pos_rel_icrf_m: start.pos_rel_icrf_m + DVec3::new(0.0, 0.0, 1_000.0),
            ..start
        };
        uni.insert_vessel(
            config(auto_soi_switch),
            VesselCheckpoint::new(ESCORT, SimInstant::START, beside, 9),
        )
        .expect("insert escort");
    }
    if let Some((on_s, off_s)) = burn {
        uni.stamp_player_command(
            at(on_s),
            TARGET,
            PlayerCommandPayload::SetBurnLaw(LatchedBurnLaw::InertialThrust {
                dir_icrf: [0.0, 1.0, 0.0],
                thrust_n: 120.0,
                exhaust_vel_m_s: 3_000.0,
                rw_torque_cmd_body_nm: [0.0; 3],
            }),
        )
        .expect("burn on");
        uni.stamp_player_command(
            at(off_s),
            TARGET,
            PlayerCommandPayload::SetBurnLaw(LatchedBurnLaw::Coast),
        )
        .expect("burn off");
    }
    let mut t = 0.0;
    while t < end_s {
        t = (t + slice_s).min(end_s);
        uni.advance_committed_to(at(t)).expect("advance");
    }
    let state = uni.committed_checkpoint(TARGET).expect("target").plant;
    (state, uni)
}

#[cfg(test)]
fn high_orbit() -> ContinuousPlantState {
    let r = 2.0e8;
    let v = (3.986_004_418e14_f64 / r).sqrt();
    plant(DVec3::new(r, 0.0, 0.0), DVec3::new(0.0, v * 0.9, v * 0.3))
}

#[cfg(test)]
fn bits(state: &ContinuousPlantState) -> [u64; 6] {
    let p = state.pos_rel_icrf_m.to_array();
    let v = state.vel_rel_icrf_m_s.to_array();
    [p[0], p[1], p[2], v[0], v[1], v[2]].map(f64::to_bits)
}

#[test]
fn a_burn_inside_a_long_segment_acts_at_its_instants() {
    let solar = BuiltinSolarSystem2026_2028::load_verified().expect("solar");
    let burn = Some((5_000.25, 5_300.75));
    let two_days = 2.0 * 86_400.0;
    let (long, _) = fly(&solar, high_orbit(), false, false, burn, 86_400.0, two_days);
    let (again, _) = fly(&solar, high_orbit(), false, false, burn, 86_400.0, two_days);
    let (sliced, _) = fly(&solar, high_orbit(), false, false, burn, 1_234.5, two_days);
    let (quantum, _) = fly(&solar, high_orbit(), false, true, burn, 86_400.0, two_days);
    let (unburnt, _) = fly(&solar, high_orbit(), false, false, None, 86_400.0, two_days);

    assert_eq!(bits(&long), bits(&again), "a repeat run differs");
    assert_eq!(bits(&long), bits(&sliced), "results depend on slicing");
    assert_ne!(
        bits(&long),
        bits(&quantum),
        "the escort did not shorten segments"
    );
    let apart_m = (long.pos_rel_icrf_m - quantum.pos_rel_icrf_m).length();
    let burn_moved_m = (long.pos_rel_icrf_m - unburnt.pos_rel_icrf_m).length();
    assert!(
        burn_moved_m > 1.0e4,
        "the burn moved the vessel {burn_moved_m} m"
    );
    assert!(
        apart_m < 1.0e-5,
        "natural steps and quantum steps end {apart_m} m apart"
    );
}

#[test]
fn an_origin_switch_fires_within_a_quantum_of_quantum_cadence() {
    let solar = BuiltinSolarSystem2026_2028::load_verified().expect("solar");
    // Leaving Earth's sphere of influence radially, about 13 hours from its edge.
    let start = plant(DVec3::new(9.0e8, 0.0, 0.0), DVec3::new(1_500.0, 0.0, 0.0));
    let switch_at = |uni: &OrbitalUniverse| {
        uni.fired_events()
            .iter()
            .find(|key| {
                key.phase_rank == phase_ranks::PRIMARY_FRAME_SWITCH
                    && key.participants.contains(&TARGET)
            })
            .map(|key| key.instant)
            .expect("an origin switch")
    };
    let (_, long) = fly(&solar, start, true, false, None, 3_600.0, 86_400.0);
    let (_, quantum) = fly(&solar, start, true, true, None, 3_600.0, 86_400.0);
    let (a, b) = (switch_at(&long), switch_at(&quantum));
    let gap = if a > b {
        a.duration_since(b)
    } else {
        b.duration_since(a)
    };
    assert!(
        gap <= TickSpan::from_seconds_and_subsec_ticks(QUANTUM_S, 0),
        "switched at {a:?} and {b:?}"
    );
    assert_eq!(
        long.committed_checkpoint(TARGET)
            .expect("target")
            .plant
            .primary_body,
        NaifId::SSB
    );
}
