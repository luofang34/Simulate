//! A snapshot holds everything that decides how a vessel evolves next: restoring
//! it and advancing reproduces the uninterrupted run bit for bit, wherever the
//! snapshot falls among the integration steps the scheduler has buffered.

use std::sync::Arc;

use glam::{DQuat, DVec3};
use sim_orbital::{
    BodyOrientationCatalog, BuiltinSolarSystem2026_2028, ContentHash, ContinuousPlantState,
    Dop853Tolerances, EphemerisCatalog, ForceModel, MassPropertiesModel, NaifId, OrbitalUniverse,
    PropellantMode, ReactionWheelMode, SimClockAdapter, SimulationManifestDigest, TdbEpoch,
    UniverseSnapshot, VesselCheckpoint, VesselConfig, VesselId,
};
use sim_spacetime::{SimInstant, TICKS_PER_SECOND, TickSpan};

const SHIP: VesselId = VesselId(4);
const END_S: u64 = 900;

#[cfg(test)]
fn at_ms(ms: u64) -> SimInstant {
    SimInstant::START.advanced(TickSpan::from_ticks(TICKS_PER_SECOND / 1_000 * ms))
}

/// One vessel coasting in low Earth orbit, tumbling slowly.
#[cfg(test)]
fn coasting_universe() -> OrbitalUniverse {
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
        tolerances,
        bounding_radius_m: 6.0,
        auto_soi_switch: true,
    };
    let plant = ContinuousPlantState {
        primary_body: NaifId::EARTH,
        pos_rel_icrf_m: DVec3::new(6_778_137.0, 0.0, 150_000.0),
        vel_rel_icrf_m_s: DVec3::new(0.0, 7_600.0, 1_200.0),
        q_body_to_icrf: DQuat::IDENTITY,
        omega_body_rad_s: DVec3::new(0.01, -0.02, 0.005),
        propellant_mass_kg: 0.0,
        h_rw_body_nms: DVec3::ZERO,
    };
    universe
        .insert_vessel(
            config,
            VesselCheckpoint::new(SHIP, SimInstant::START, plant, 5),
        )
        .unwrap();
    universe
}

/// What must match between two runs: the replay identity and the sampled state.
#[cfg(test)]
fn fingerprint(universe: &OrbitalUniverse, at: SimInstant) -> (ContentHash, [u64; 17]) {
    let state = universe.sample_vessel_state(SHIP, at).unwrap();
    (
        universe.universe_hash(),
        state.to_ode_array().map(f64::to_bits),
    )
}

#[test]
fn restoring_a_snapshot_continues_bit_for_bit_from_any_buffered_instant() {
    let end = at_ms(END_S * 1_000);
    let mut uninterrupted = coasting_universe();
    uninterrupted.advance_committed_to(end).unwrap();
    let expected = fingerprint(&uninterrupted, end);

    // Mid-step, just after a quantum boundary, and deep into a long buffered proposal.
    for snapshot_ms in [30_000, 30_500, 75_000, 137_250, 437_125] {
        let mut interrupted = coasting_universe();
        let at = at_ms(snapshot_ms);
        interrupted.advance_committed_to(at).unwrap();
        let buffered = interrupted.speculative_segment_count(SHIP).unwrap();
        assert!(
            buffered > 1,
            "at {snapshot_ms} ms the vessel has {buffered} segments buffered; the check \
             needs more than the one spanning the snapshot"
        );
        let json = serde_json::to_string(&interrupted.capture_snapshot()).unwrap();
        let snapshot: UniverseSnapshot = serde_json::from_str(&json).unwrap();

        let mut restored = coasting_universe();
        restored.restore_snapshot(&snapshot).unwrap();
        assert_eq!(
            restored.universe_hash(),
            interrupted.universe_hash(),
            "restoring at {snapshot_ms} ms rebuilds the same replay identity"
        );
        assert_eq!(
            restored.speculative_segment_count(SHIP),
            Some(buffered),
            "and the same buffered lookahead"
        );
        restored.advance_committed_to(end).unwrap();
        interrupted.advance_committed_to(end).unwrap();
        assert_eq!(
            fingerprint(&interrupted, end),
            expected,
            "taking a snapshot at {snapshot_ms} ms does not disturb the run"
        );
        assert_eq!(
            fingerprint(&restored, end),
            expected,
            "a run restored at {snapshot_ms} ms is the uninterrupted run"
        );
    }
}

#[test]
fn the_snapshot_hash_covers_the_buffered_lookahead() {
    let mut universe = coasting_universe();
    universe.advance_committed_to(at_ms(30_000)).unwrap();
    let full = universe.capture_snapshot();
    let record = full.vessels.first().unwrap();
    assert!(record.speculative_segments.len() > 1);

    let mut shortened = full.clone();
    if let Some(record) = shortened.vessels.first_mut() {
        record.speculative_segments.truncate(1);
        record.frontier_checkpoint = record
            .speculative_segments
            .first()
            .unwrap()
            .end_checkpoint
            .clone();
    }
    assert_eq!(full.physics_state_hash(), shortened.physics_state_hash());
    assert_ne!(
        full.snapshot_hash(),
        shortened.snapshot_hash(),
        "a snapshot that would continue on other step boundaries is another snapshot"
    );
}
