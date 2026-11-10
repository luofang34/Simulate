//! Transactional multi-vessel scheduler determinism verification: unique `EventOrderKey` tie breaking,
//! stale-generation suppression, `MAX_SAME_TICK_TRANSITIONS` zero-time loop cap, `24 FPS` vs `144 FPS`,
//! camera-switch and worker-permutation bitwise identity, manifest-verified mid-segment
//! save/restore, and bounded `O(log S)` rollback metrics.

use std::sync::Arc;

use glam::{DQuat, DVec3};
use sim_orbital::{
    BodyOrientationCatalog, BuiltinSolarSystem2026_2028, ContentHash, ContinuousPlantState,
    Dop853Tolerances, EphemerisCatalog, EventOrderKey, ForceModel, LatchedBurnLaw,
    MAX_SAME_TICK_TRANSITIONS, MAX_SPECULATIVE_SEGMENTS, ManifestMismatchField,
    MassPropertiesModel, NaifId, OrbitalUniverse, PlayerCommandPayload, PropellantMode,
    ReactionWheelMode, ScheduledEvent, SchedulerError, SimClockAdapter, SimulationManifestDigest,
    TdbEpoch, UniverseEventPayload, UniverseSnapshot, VesselCheckpoint, VesselConfig, VesselId,
    phase_ranks,
};
use sim_spacetime::{SimInstant, TickSpan};

#[cfg(test)]
fn build_test_universe(solar: &BuiltinSolarSystem2026_2028) -> OrbitalUniverse {
    let base_tdb = TdbEpoch::new(825_000_000, 0);
    let fm = ForceModel::standard_solar_system(solar.constants.clone()).expect("force model");
    let tol = Dop853Tolerances::default();
    let manifest = SimulationManifestDigest::compute(
        base_tdb,
        solar.ephemeris.content_hash(),
        solar.constants.content_hash(),
        solar.orientation.content_hash(),
        solar.time_scales.content_hash(),
        ContentHash::of_bytes(b"gate3-terrain-none"),
        fm.content_hash(),
        tol.content_hash(),
    );
    OrbitalUniverse::new(
        manifest,
        SimClockAdapter::new(base_tdb),
        fm,
        Arc::new(solar.ephemeris.clone()),
        Arc::new(solar.orientation.clone()),
    )
}

#[cfg(test)]
fn default_vessel_config() -> VesselConfig {
    VesselConfig {
        mass_model: MassPropertiesModel::diagonal(
            12_000.0,
            [42_000.0, 42_000.0, 14_000.0],
            [0.0, 0.0, 0.0],
        ),
        propellant_mode: PropellantMode::InfiniteConstantMass,
        rw_mode: ReactionWheelMode::ConserveMomentum {
            h_max_nms: 5.0e6,
            tau_max_nm: 8.0e4,
        },
        tolerances: Dop853Tolerances::default(),
        bounding_radius_m: 6.0,
        auto_soi_switch: true,
    }
}

#[cfg(test)]
fn leo_plant(offset_x_m: f64, vy_m_s: f64) -> ContinuousPlantState {
    ContinuousPlantState {
        primary_body: NaifId::EARTH,
        pos_rel_icrf_m: DVec3::new(6_778_137.0 + offset_x_m, 0.0, 150_000.0),
        vel_rel_icrf_m_s: DVec3::new(0.0, vy_m_s, 1_200.0),
        q_body_to_icrf: DQuat::IDENTITY,
        omega_body_rad_s: DVec3::new(0.01, -0.02, 0.005),
        propellant_mass_kg: 0.0,
        h_rw_body_nms: DVec3::ZERO,
    }
}

#[test]
fn simultaneous_same_vessel_and_multi_vessel_event_order_key_ties_verified() {
    let solar = BuiltinSolarSystem2026_2028::load_verified().expect("solar");
    let mut uni = build_test_universe(&solar);
    let (v1, v2, v3) = (VesselId(1), VesselId(2), VesselId(3));

    assert_eq!(
        EventOrderKey::pair(SimInstant::START, 40, v3, v1, 5, 0),
        EventOrderKey::pair(SimInstant::START, 40, v1, v3, 5, 0)
    );

    for vid in [v1, v2] {
        uni.insert_vessel(
            default_vessel_config(),
            VesselCheckpoint::new(vid, SimInstant::START, leo_plant(0.0, 7_660.0), 42),
        )
        .expect("insert");
    }

    let t_ev = SimInstant::START.advanced(TickSpan::from_seconds_and_subsec_ticks(5, 0));
    let k_marker_b = EventOrderKey::single(t_ev, phase_ranks::CUSTOM_DETECTOR, v1, 7, 1);
    let k_marker_a = EventOrderKey::single(t_ev, phase_ranks::CUSTOM_DETECTOR, v1, 7, 0);
    let k_v2_marker = EventOrderKey::single(t_ev, phase_ranks::CUSTOM_DETECTOR, v2, 1, 0);
    let k_stale_after_cmd = EventOrderKey::single(t_ev, phase_ranks::CUSTOM_DETECTOR, v1, 9, 2);

    uni.enqueue_event(ScheduledEvent::single(
        k_marker_b,
        v1,
        None,
        UniverseEventPayload::Marker { tag: 102 },
    ))
    .expect("enq b");
    uni.enqueue_event(ScheduledEvent::single(
        k_marker_a,
        v1,
        None,
        UniverseEventPayload::Marker { tag: 101 },
    ))
    .expect("enq a");
    uni.enqueue_event(ScheduledEvent::single(
        k_v2_marker,
        v2,
        None,
        UniverseEventPayload::Marker { tag: 201 },
    ))
    .expect("enq v2");
    uni.enqueue_event(ScheduledEvent::single(
        k_stale_after_cmd,
        v1,
        Some(0),
        UniverseEventPayload::Marker { tag: 999 },
    ))
    .expect("enq stale");

    let cmd = uni
        .stamp_player_command(t_ev, v1, PlayerCommandPayload::SetGncMode(77))
        .expect("stamp cmd");
    let k_cmd = EventOrderKey::single(t_ev, phase_ranks::PLAYER_COMMAND, v1, 0, cmd.command_seq);

    uni.advance_committed_to(t_ev).expect("advance to t_ev");

    assert_eq!(
        uni.fired_events(),
        &[k_cmd, k_marker_a, k_marker_b, k_v2_marker]
    );
    assert_eq!(uni.committed_checkpoint(v1).expect("v1").gnc.mode_code, 77);
}

#[test]
fn zero_time_event_loop_cap_and_legitimate_chained_transitions_verified() {
    let solar = BuiltinSolarSystem2026_2028::load_verified().expect("solar");
    let mut uni = build_test_universe(&solar);
    let v1 = VesselId(1);
    uni.insert_vessel(
        default_vessel_config(),
        VesselCheckpoint::new(v1, SimInstant::START, leo_plant(0.0, 7_660.0), 7),
    )
    .expect("insert");

    let t1 = SimInstant::START.advanced(TickSpan::SECOND);
    let k_ok = EventOrderKey::single(t1, phase_ranks::CUSTOM_DETECTOR, v1, 99, 0);
    uni.enqueue_event(ScheduledEvent::single(
        k_ok,
        v1,
        Some(0),
        UniverseEventPayload::ModeTransition {
            next_mode_code: 10,
            remaining_chain: 5,
        },
    ))
    .expect("enq finite chain");
    uni.advance_committed_to(t1).expect("6 transitions <= 32");
    assert_eq!(uni.committed_checkpoint(v1).expect("cp").gnc.mode_code, 15);

    let t2 = t1.advanced(TickSpan::SECOND);
    let cur_gen = uni
        .committed_checkpoint(v1)
        .expect("cp")
        .generations
        .state_gen;
    let k_inf = EventOrderKey::single(t2, phase_ranks::CUSTOM_DETECTOR, v1, 99, 100);
    uni.enqueue_event(ScheduledEvent::single(
        k_inf,
        v1,
        Some(cur_gen),
        UniverseEventPayload::ModeTransition {
            next_mode_code: 100,
            remaining_chain: u32::MAX,
        },
    ))
    .expect("enq infinite loop");

    let err = uni
        .advance_committed_to(t2)
        .expect_err("must cap zero-time loop");
    assert_eq!(
        err,
        SchedulerError::ZeroTimeEventLoop {
            vessel_id: v1,
            instant: t2,
            limit: MAX_SAME_TICK_TRANSITIONS,
        }
    );
}

#[cfg(test)]
fn populate_three_vessel_fleet(uni: &mut OrbitalUniverse) {
    let cfg = default_vessel_config();
    let mut cp1 =
        VesselCheckpoint::new(VesselId(1), SimInstant::START, leo_plant(0.0, 7_660.0), 11);
    cp1.gnc.gnc_period_ticks = TickSpan::SECOND.ticks_u128();
    cp1.gnc.latched_law = LatchedBurnLaw::ConstantBody {
        main_thrust_body_n: [0.0, 24_000.0, 0.0],
        main_exhaust_vel_m_s: 3_500.0,
        rcs_force_body_n: [0.0; 3],
        rcs_exhaust_vel_m_s: 2_800.0,
        rw_torque_cmd_body_nm: [120.0, -80.0, 40.0],
    };
    let cp2 = VesselCheckpoint::new(
        VesselId(2),
        SimInstant::START,
        leo_plant(250_000.0, 7_520.0),
        22,
    );
    let cp3 = VesselCheckpoint::new(
        VesselId(3),
        SimInstant::START,
        ContinuousPlantState {
            primary_body: NaifId::MOON,
            pos_rel_icrf_m: DVec3::new(1_937_400.0, 0.0, 200_000.0),
            vel_rel_icrf_m_s: DVec3::new(0.0, 1_580.0, 120.0),
            q_body_to_icrf: DQuat::IDENTITY,
            omega_body_rad_s: DVec3::ZERO,
            propellant_mass_kg: 0.0,
            h_rw_body_nms: DVec3::ZERO,
        },
        33,
    );
    for cp in [cp1, cp2, cp3] {
        uni.insert_vessel(cfg, cp).expect("insert fleet vessel");
    }
    uni.stamp_player_command(
        SimInstant::START.advanced(TickSpan::from_seconds_and_subsec_ticks(4, 0)),
        VesselId(2),
        PlayerCommandPayload::SetBurnLaw(LatchedBurnLaw::InertialThrust {
            dir_icrf: [0.0, 1.0, 0.0],
            thrust_n: 36_000.0,
            exhaust_vel_m_s: 3_500.0,
            rw_torque_cmd_body_nm: [0.0; 3],
        }),
    )
    .expect("stamp v2 burn");
}

#[test]
fn fps_24_vs_144_camera_switch_and_worker_permutation_bitwise_identity() {
    let solar = BuiltinSolarSystem2026_2028::load_verified().expect("solar");
    let mut uni_24 = build_test_universe(&solar);
    let mut uni_144 = build_test_universe(&solar);
    let mut uni_perm = build_test_universe(&solar);
    populate_three_vessel_fleet(&mut uni_24);
    populate_three_vessel_fleet(&mut uni_144);
    populate_three_vessel_fleet(&mut uni_perm);

    let dt_24 = TickSpan::period_of_hz(24).expect("24 Hz exact");
    let cams = [
        Some(VesselId(1)),
        Some(VesselId(2)),
        Some(VesselId(3)),
        None,
    ];
    let mut t = SimInstant::START;
    for frame in 0..240_usize {
        t = t.advanced(dt_24);
        uni_24.set_active_camera_vessel(cams[frame % cams.len()]);
        uni_24.advance_committed_to(t).expect("24 fps step");
        for vid in [VesselId(1), VesselId(2), VesselId(3)] {
            let _ = uni_24.sample_vessel_state(vid, t).expect("sample 24");
        }
    }

    let dt_144 = TickSpan::period_of_hz(144).expect("144 Hz exact");
    let mut t_fast = SimInstant::START;
    for _ in 0..1_440_usize {
        t_fast = t_fast.advanced(dt_144);
        uni_144.advance_committed_to(t_fast).expect("144 fps step");
        assert!(uni_144.sample_vessel_state(VesselId(1), t_fast).is_some());
    }

    let end_10s = SimInstant::START.advanced(TickSpan::from_seconds_and_subsec_ticks(10, 0));
    assert_eq!(t, end_10s);
    assert_eq!(t_fast, end_10s);

    while uni_perm.committed_instant() < end_10s {
        let next_sec = uni_perm
            .committed_instant()
            .advanced(TickSpan::SECOND)
            .min(end_10s);
        let reqs = uni_perm.build_worker_requests(next_sec);
        let mut rev_props = Vec::new();
        for (cp, cfg, h, len) in reqs.iter().rev() {
            rev_props.push(
                uni_perm
                    .evaluate_worker_request(cp, cfg, *h, *len)
                    .expect("worker"),
            );
        }
        uni_perm.ingest_proposals(rev_props);
        uni_perm.advance_committed_to(next_sec).expect("perm step");
    }

    assert_eq!(uni_24.universe_hash(), uni_144.universe_hash());
    assert_eq!(uni_24.universe_hash(), uni_perm.universe_hash());
    for vid in [VesselId(1), VesselId(2), VesselId(3)] {
        assert_eq!(
            uni_24.sample_vessel_state(vid, end_10s),
            uni_144.sample_vessel_state(vid, end_10s)
        );
        assert_eq!(
            uni_24.sample_vessel_state(vid, end_10s),
            uni_perm.sample_vessel_state(vid, end_10s)
        );
    }
}

#[test]
fn checkpoint_mid_segment_save_restore_manifest_guard_and_rollback_metrics_verified() {
    let solar = BuiltinSolarSystem2026_2028::load_verified().expect("solar");
    let mut uni_full = build_test_universe(&solar);
    let mut uni_split = build_test_universe(&solar);
    populate_three_vessel_fleet(&mut uni_full);
    populate_three_vessel_fleet(&mut uni_split);

    let t_end = SimInstant::START.advanced(TickSpan::from_seconds_and_subsec_ticks(25, 0));
    uni_full.advance_committed_to(t_end).expect("full 25s");

    let t_mid =
        SimInstant::START.advanced(TickSpan::from_seconds_and_subsec_ticks(7, 13_824_000_000));
    uni_split
        .advance_committed_to(t_mid)
        .expect("advance mid-segment 7.3s");
    let json = serde_json::to_string(&uni_split.capture_snapshot()).expect("serialize snap");
    let snap: UniverseSnapshot = serde_json::from_str(&json).expect("deserialize snap");

    let mut uni_restored = build_test_universe(&solar);
    uni_restored.restore_snapshot(&snap).expect("restore");
    uni_restored
        .advance_committed_to(t_end)
        .expect("finish 25s");
    assert_eq!(uni_full.universe_hash(), uni_restored.universe_hash());

    let mut bad_snap = snap;
    bad_snap.manifest.force_model_hash = ContentHash::of_bytes(b"tampered-force-model");
    assert_eq!(
        uni_restored.restore_snapshot(&bad_snap),
        Err(SchedulerError::ManifestMismatch(
            ManifestMismatchField::ForceModel
        ))
    );

    verify_bounded_rollback_cost(&solar);
}

#[cfg(test)]
fn verify_bounded_rollback_cost(solar: &BuiltinSolarSystem2026_2028) {
    let mut uni = build_test_universe(solar)
        .with_horizon_quantum_ticks(TickSpan::from_seconds_and_subsec_ticks(3_600, 0).ticks_u128());
    let mut cfg = default_vessel_config();
    cfg.tolerances.h_max_s = 20.0;
    cfg.tolerances.h_init_s = 10.0;
    uni.insert_vessel(
        cfg,
        VesselCheckpoint::new(VesselId(1), SimInstant::START, leo_plant(0.0, 7_660.0), 99),
    )
    .expect("insert v1");

    let horizon_goal =
        SimInstant::START.advanced(TickSpan::from_seconds_and_subsec_ticks(2_000, 0));
    let reqs = uni.build_worker_requests(horizon_goal);
    let props: Vec<_> = reqs
        .iter()
        .map(|(cp, c, h, len)| uni.evaluate_worker_request(cp, c, *h, *len).expect("prop"))
        .collect();
    uni.ingest_proposals(props);
    let buffered = uni.speculative_segment_count(VesselId(1)).expect("count");
    assert!(buffered > 10 && buffered <= MAX_SPECULATIVE_SEGMENTS);

    let t_interrupt = SimInstant::START.advanced(TickSpan::from_seconds_and_subsec_ticks(35, 0));
    uni.stamp_player_command(
        t_interrupt,
        VesselId(1),
        PlayerCommandPayload::SetGncMode(9),
    )
    .expect("stamp interrupt");
    let metrics = uni
        .advance_committed_to(t_interrupt)
        .expect("rollback at 35s");
    assert_eq!(metrics.vessels_rolled_back, 1);
    assert!(metrics.segments_dropped > 5);
    assert!(metrics.binary_search_probes <= 7);
    assert!(metrics.replayed_steps <= 2);
}

#[test]
fn one_advance_longer_than_the_speculative_buffer_matches_many_short_ones() {
    let solar = BuiltinSolarSystem2026_2028::load_verified().expect("solar");
    let one_second = TickSpan::from_seconds_and_subsec_ticks(1, 0);
    let seconds = (MAX_SPECULATIVE_SEGMENTS as u64) * 3;
    let build = || {
        let mut uni =
            build_test_universe(&solar).with_horizon_quantum_ticks(one_second.ticks_u128());
        populate_three_vessel_fleet(&mut uni);
        uni
    };
    let mut one_call = build();
    let mut many_calls = build();

    let end = SimInstant::START.advanced(TickSpan::from_seconds_and_subsec_ticks(seconds, 0));
    one_call
        .advance_committed_to(end)
        .expect("one long advance");
    let mut t = SimInstant::START;
    while t < end {
        t = t.advanced(one_second);
        many_calls.advance_committed_to(t).expect("short advance");
    }
    assert_eq!(one_call.committed_instant(), end);
    assert_eq!(one_call.universe_hash(), many_calls.universe_hash());
}

#[test]
fn a_removed_vessel_leaves_at_the_committed_instant_with_its_events() {
    let solar = BuiltinSolarSystem2026_2028::load_verified().expect("solar");
    let mut uni = build_test_universe(&solar);
    populate_three_vessel_fleet(&mut uni);
    let ids: Vec<VesselId> = uni.vessel_ids().collect();
    let gone = ids[0];
    let mid =
        SimInstant::START.advanced(TickSpan::from_seconds_and_subsec_ticks(7, 13_824_000_000));
    uni.advance_committed_to(mid).expect("advance");
    uni.stamp_player_command(
        mid.advanced(TickSpan::from_seconds_and_subsec_ticks(5, 0)),
        gone,
        PlayerCommandPayload::SetNosecone {
            carrier_rad: 0.1,
            wrist_rad: 0.0,
        },
    )
    .expect("stamp");
    uni.set_active_camera_vessel(Some(gone));
    let sampled = uni.sample_vessel_state(gone, mid).expect("sample");

    let (_config, checkpoint) = uni.remove_vessel(gone).expect("remove");
    assert_eq!(checkpoint.instant, mid);
    let miss = (checkpoint.plant.pos_rel_icrf_m - sampled.pos_rel_icrf_m).length();
    assert!(
        miss < 1.0e-3,
        "integrated and dense states differ by {miss} m"
    );
    assert_eq!(uni.active_camera_vessel(), None);
    assert!(uni.vessel_ids().all(|id| id != gone));
    assert_eq!(
        uni.remove_vessel(gone),
        Err(SchedulerError::UnknownVessel(gone))
    );

    let end = mid.advanced(TickSpan::from_seconds_and_subsec_ticks(10, 0));
    uni.advance_committed_to(end)
        .expect("advance past the dropped command");
    assert!(
        uni.fired_events()
            .iter()
            .all(|key| key.instant <= mid || !key.participants.contains(&gone)),
        "an event of the removed vessel fired after it left"
    );
}
