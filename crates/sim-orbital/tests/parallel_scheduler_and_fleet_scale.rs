//! Parallel speculative worker pool bitwise determinism
//! (`1`, `2`, `4`, `8` threads + permuted completion order) and hardware-tagged
//! `10 / 100 / 1,000 / 10,000` fleet-scale benchmark.

use std::sync::Arc;

use glam::{DQuat, DVec3};
use sim_orbital::{
    BodyOrientationCatalog, BuiltinSolarSystem2026_2028, ContentHash, ContinuousPlantState,
    Dop853Tolerances, EphemerisCatalog, ForceModel, LatchedBurnLaw, MassPropertiesModel, NaifId,
    OrbitalUniverse, PlayerCommandPayload, PropagatorKind, PropellantMode, ReactionWheelMode,
    SimClockAdapter, SimulationManifestDigest, TdbEpoch, VesselCheckpoint, VesselConfig, VesselId,
    run_fleet_scale_benchmark,
};
use sim_spacetime::{SimInstant, TICKS_PER_SECOND, TickSpan};

#[cfg(test)]
fn insert_test_vessel(univ: &mut OrbitalUniverse, idx: u64) {
    let mu = 3.986_004_418e14_f64;
    let vid = VesselId(idx.wrapping_add(1));
    let r = 6_778_137.0 + (idx as f64) * 25_000.0;
    let v = (mu / r).sqrt();
    let angle = (idx as f64) * 0.35;
    let prop = if idx < 4 {
        PropagatorKind::FullCoupled6Dof
    } else if idx < 12 {
        PropagatorKind::DormantKepler
    } else {
        PropagatorKind::PinnedSurfaceRest
    };
    let cfg = VesselConfig {
        mass_model: MassPropertiesModel::diagonal(
            12_000.0,
            [85_000.0, 18_000.0, 85_000.0],
            [0.0; 3],
        ),
        propellant_mode: PropellantMode::InfiniteConstantMass,
        rw_mode: ReactionWheelMode::ConserveMomentum {
            h_max_nms: 5.0e6,
            tau_max_nm: 5.0e4,
        },
        tolerances: Dop853Tolerances::default(),
        bounding_radius_m: 6.0,
        auto_soi_switch: true,
    };
    let plant = ContinuousPlantState {
        primary_body: NaifId::EARTH,
        pos_rel_icrf_m: DVec3::new(r * angle.cos(), r * angle.sin(), 0.0),
        vel_rel_icrf_m_s: DVec3::new(-v * angle.sin(), v * angle.cos(), 0.0),
        q_body_to_icrf: DQuat::IDENTITY,
        omega_body_rad_s: DVec3::ZERO,
        propellant_mass_kg: 0.0,
        h_rw_body_nms: DVec3::ZERO,
    };
    let mut cp = VesselCheckpoint::new(vid, SimInstant::START, plant, vid.0);
    cp.integrator_hint.propagator = prop;
    cp.integrator_hint.h_trial_s = Some(5.0);
    univ.insert_vessel(cfg, cp).expect("insert vessel");
}

#[cfg(test)]
fn build_mixed_test_universe(solar: &BuiltinSolarSystem2026_2028) -> OrbitalUniverse {
    let fm = ForceModel::standard_solar_system(solar.constants.clone()).expect("fm");
    let t0 = TdbEpoch::new(835_000_000, 0);
    let clock = SimClockAdapter::new(t0);
    let manifest = SimulationManifestDigest::compute(
        t0,
        solar.ephemeris.content_hash(),
        solar.constants.content_hash(),
        solar.orientation.content_hash(),
        solar.time_scales.content_hash(),
        ContentHash::of_bytes(b"sim-scenery:terrain-empty:v1"),
        fm.content_hash(),
        Dop853Tolerances::default().content_hash(),
    );
    let mut univ = OrbitalUniverse::new(
        manifest,
        clock,
        fm,
        Arc::new(solar.ephemeris.clone()),
        Arc::new(solar.orientation.clone()),
    )
    .with_horizon_quantum_ticks(15 * (TICKS_PER_SECOND as u128));

    for idx in 0..16_u64 {
        insert_test_vessel(&mut univ, idx);
    }

    let cmd_time = SimInstant::START.advanced(TickSpan::from_seconds_and_subsec_ticks(45, 0));
    univ.stamp_player_command(
        cmd_time,
        VesselId(1),
        PlayerCommandPayload::SetBurnLaw(LatchedBurnLaw::ConstantBody {
            main_thrust_body_n: [0.0, 120_000.0, 0.0],
            main_exhaust_vel_m_s: 4_500.0,
            rcs_force_body_n: [0.0; 3],
            rcs_exhaust_vel_m_s: 3_000.0,
            rw_torque_cmd_body_nm: [0.0, 0.0, 250.0],
        }),
    )
    .expect("stamp command");
    univ
}

#[test]
fn parallel_worker_pool_1_2_4_8_threads_and_permuted_completion_bitwise_identity() {
    let solar = BuiltinSolarSystem2026_2028::load_verified().expect("solar");
    let target = SimInstant::START.advanced(TickSpan::from_seconds_and_subsec_ticks(120, 0));

    let mut u_serial = build_mixed_test_universe(&solar);
    u_serial
        .advance_committed_to(target)
        .expect("serial advance");
    let ref_snap = u_serial.capture_snapshot();
    let ref_hash = ref_snap.snapshot_hash();

    for threads in [1_usize, 2, 4, 8] {
        let mut u_par = build_mixed_test_universe(&solar);
        u_par
            .advance_committed_to_parallel(target, threads)
            .expect("parallel advance");
        let par_snap = u_par.capture_snapshot();
        assert_eq!(
            par_snap.snapshot_hash(),
            ref_hash,
            "state hash mismatch at {threads} worker threads"
        );
        assert_eq!(par_snap.vessels, ref_snap.vessels);
    }

    // Also verify reversed manual proposal completion order before advancing:
    let mut u_rev = build_mixed_test_universe(&solar);
    let step1 = SimInstant::START.advanced(TickSpan::from_seconds_and_subsec_ticks(15, 0));
    let reqs = u_rev.build_worker_requests(step1);
    let mut props: Vec<_> = reqs
        .iter()
        .rev()
        .map(|(cp, cfg, h, len)| {
            u_rev
                .evaluate_worker_request(cp, cfg, *h, *len)
                .expect("req")
        })
        .collect();
    props.reverse();
    u_rev.ingest_proposals(props);
    u_rev
        .advance_committed_to(target)
        .expect("reversed advance");
    assert_eq!(u_rev.capture_snapshot().snapshot_hash(), ref_hash);
}

#[test]
fn hardware_tagged_fleet_scale_10_100_1000_10000_vessels_benchmark_verified() {
    let solar = BuiltinSolarSystem2026_2028::load_verified().expect("solar");
    let report = run_fleet_scale_benchmark(&solar, &[10, 100, 1_000, 10_000], 5, 4)
        .expect("fleet scale benchmark");

    assert!(report.hardware.available_parallelism >= 1);
    assert_eq!(report.hardware.worker_threads_used, 4);
    assert_eq!(report.tiers.len(), 4);

    for tier in &report.tiers {
        assert!(tier.p50_ms >= 0.0 && tier.p95_ms >= tier.p50_ms && tier.p99_ms >= tier.p95_ms);
        assert_ne!(tier.state_hash.as_bytes(), &[0_u8; 32]);
    }

    let tier_1000 = report
        .tiers
        .iter()
        .find(|t| t.vessel_count == 1_000)
        .expect("1,000-vessel tier");
    assert!(
        tier_1000.p99_ms <= 4.0,
        "1,000-vessel fleet p99 frame time {} ms exceeded 4.0 ms budget",
        tier_1000.p99_ms
    );
}
