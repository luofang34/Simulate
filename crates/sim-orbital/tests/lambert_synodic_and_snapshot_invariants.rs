//! Invariant verification suite for Lambert long/short-way geometry, `SynodicFrame` velocity transport,
//! `UniverseSnapshot` replay hash completeness, `PinnedSurfaceRest` planetary co-rotation,
//! multi-worker panic/error reporting, and L2 single-shooting fail-closed semantics.

use std::sync::Arc;

use glam::{DQuat, DVec3};
use sim_orbital::{
    BodyOrientationCatalog, BuiltinSolarSystem2026_2028, ContentHash, ContinuousPlantState,
    Dop853Tolerances, EphemerisCatalog, EventOrderKey, ForceModel, L2BoundedArcTargetSpec,
    L2SingleShootingFailureReason, LagrangeError, LambertError, MassPropertiesModel, NaifId,
    OrbitalUniverse, PropagatorKind, PropellantMode, ReactionWheelMode, ScheduledEvent,
    SchedulerError, SimClockAdapter, SimulationManifestDigest, SynodicFrame, TdbEpoch,
    UniverseEventPayload, VesselCheckpoint, VesselConfig, VesselId, phase_ranks,
    propagate_kepler_universal, solve_lambert_universal, target_l2_terminal_x_arc_dop853,
};
use sim_spacetime::{SimInstant, TICKS_PER_SECOND, TickSpan};

#[test]
fn lambert_short_vs_long_way_and_degenerate_inputs() {
    let mu = 3.986_004_418e14_f64;
    let r1 = DVec3::new(7_000_000.0, 0.0, 0.0);
    let r2 = DVec3::new(-3_500_000.0, 6_062_177.826_491_07, 0.0);
    let tof_s = 3_600.0;

    let short = solve_lambert_universal(r1, r2, tof_s, mu, false).expect("short way");
    let long = solve_lambert_universal(r1, r2, tof_s, mu, true).expect("long way");

    let (r2_short, v2_short) = propagate_kepler_universal(r1, short.v1_m_s, tof_s, mu);
    let (r2_long, v2_long) = propagate_kepler_universal(r1, long.v1_m_s, tof_s, mu);
    assert!((r2_short - r2).length() < 0.05);
    assert!((r2_long - r2).length() < 0.05);
    assert!((v2_short - short.v2_m_s).length() < 1.0e-4);
    assert!((v2_long - long.v2_m_s).length() < 1.0e-4);

    assert!((short.v1_m_s - long.v1_m_s).length() > 1_000.0);
    assert!(short.transfer_angle_rad < std::f64::consts::PI);
    assert!(long.transfer_angle_rad > std::f64::consts::PI);

    let h_short = r1.cross(short.v1_m_s);
    let h_long = r1.cross(long.v1_m_s);
    assert!(h_short.z > 0.0 && h_long.z < 0.0);
    let (r_mid_short, _) = propagate_kepler_universal(r1, short.v1_m_s, tof_s * 0.5, mu);
    let (r_mid_long, _) = propagate_kepler_universal(r1, long.v1_m_s, tof_s * 0.5, mu);
    assert!(r_mid_short.y > 0.0 && r_mid_long.y < 0.0);

    assert!(matches!(
        solve_lambert_universal(DVec3::ZERO, r2, tof_s, mu, false),
        Err(LambertError::DegenerateRadius { .. })
    ));
    assert!(matches!(
        solve_lambert_universal(r1, -r1, tof_s, mu, false),
        Err(LambertError::CollinearTransferGeometry { .. })
    ));
    assert!(matches!(
        solve_lambert_universal(r1, r2, -10.0, mu, false),
        Err(LambertError::InvalidInput { .. })
    ));
}

#[test]
fn synodic_frame_velocity_finite_difference_and_convergence() {
    let solar = BuiltinSolarSystem2026_2028::load_verified().expect("solar");
    let t0 = TdbEpoch::new(835_000_000, 0);
    let p0 = DVec3::new(1.5e9, 2.0e8, 4.0e8);
    let v0 = DVec3::new(-120.0, 450.0, 180.0);

    let f0 =
        SynodicFrame::from_ephemeris(NaifId::SUN, NaifId::EARTH, t0, &solar.ephemeris).expect("f0");
    let (p_syn, v_syn) = f0.icrf_rel_secondary_to_synodic(p0, v0);
    let (p_back, v_back) = f0.synodic_to_icrf_rel_secondary(p_syn, v_syn);
    assert!((p_back - p0).length() < 1.0e-6);
    assert!((v_back - v0).length() < 1.0e-9);

    let eval_fd_err = |dt_s: i64| -> f64 {
        let tm = TdbEpoch::new(835_000_000 - dt_s, 0);
        let tp = TdbEpoch::new(835_000_000 + dt_s, 0);
        let fm = SynodicFrame::from_ephemeris(NaifId::SUN, NaifId::EARTH, tm, &solar.ephemeris)
            .expect("fm");
        let fp = SynodicFrame::from_ephemeris(NaifId::SUN, NaifId::EARTH, tp, &solar.ephemeris)
            .expect("fp");
        let dt = dt_s as f64;
        let (pm, _) = fm.icrf_rel_secondary_to_synodic(p0 - v0 * dt, v0);
        let (pp, _) = fp.icrf_rel_secondary_to_synodic(p0 + v0 * dt, v0);
        let v_fd = (pp - pm) / (2.0 * dt);
        (v_syn - v_fd).length()
    };
    let err_10000 = eval_fd_err(10_000);
    let err_1000 = eval_fd_err(1_000);
    let err_100 = eval_fd_err(100);
    let err_10 = eval_fd_err(10);
    assert!(
        err_100 < 1.0e-6 && err_10 < 1.0e-6,
        "Synodic velocity FD error at dt=100s ({err_100}) / dt=10s ({err_10}) must be < 1e-6 m/s"
    );
    assert!(
        err_10000 > err_1000 * 50.0,
        "Expected O(dt^2) convergence: err_10000={err_10000}, err_1000={err_1000}"
    );

    let rel = solar
        .ephemeris
        .geometric_state_icrf(
            NaifId::EARTH,
            NaifId::SUN,
            t0,
            sim_orbital::DerivativeOrder::Velocity,
        )
        .expect("rel");
    let (p_pri_syn, v_pri_syn) = f0.icrf_rel_secondary_to_synodic(-rel.pos_m, -rel.vel_m_s);
    assert!((p_pri_syn - DVec3::new(-f0.separation_m, 0.0, 0.0)).length() < 1.0e-3);
    assert!((v_pri_syn - DVec3::new(-f0.separation_rate_m_s, 0.0, 0.0)).length() < 1.0e-7);

    assert!(matches!(
        SynodicFrame::from_ephemeris(NaifId::EARTH, NaifId::EARTH, t0, &solar.ephemeris),
        Err(sim_spacetime::SynodicError::DegenerateSynodicFrame { .. })
    ));
}

#[cfg(test)]
fn default_test_vessel_config() -> VesselConfig {
    VesselConfig {
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
    }
}

#[cfg(test)]
fn build_single_vessel_universe(
    solar: &BuiltinSolarSystem2026_2028,
    plant: ContinuousPlantState,
    prop: PropagatorKind,
) -> OrbitalUniverse {
    let fm = ForceModel::standard_solar_system(solar.constants.clone()).expect("fm");
    let t0 = TdbEpoch::new(835_000_000, 0);
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
        SimClockAdapter::new(t0),
        fm,
        Arc::new(solar.ephemeris.clone()),
        Arc::new(solar.orientation.clone()),
    )
    .with_horizon_quantum_ticks(30 * (TICKS_PER_SECOND as u128));
    let mut cp = VesselCheckpoint::new(VesselId(1), SimInstant::START, plant, 1);
    cp.integrator_hint.propagator = prop;
    univ.insert_vessel(default_test_vessel_config(), cp)
        .expect("insert");
    univ
}

#[test]
fn snapshot_hash_completeness_and_replay_boundaries() {
    let solar = BuiltinSolarSystem2026_2028::load_verified().expect("solar");
    let plant = ContinuousPlantState {
        primary_body: NaifId::EARTH,
        pos_rel_icrf_m: DVec3::new(6_778_137.0, 0.0, 0.0),
        vel_rel_icrf_m_s: DVec3::new(0.0, 7_668.6, 0.0),
        q_body_to_icrf: DQuat::IDENTITY,
        omega_body_rad_s: DVec3::ZERO,
        propellant_mass_kg: 0.0,
        h_rw_body_nms: DVec3::ZERO,
    };
    let mut u1 = build_single_vessel_universe(&solar, plant, PropagatorKind::DormantKepler);
    let mut u2 = build_single_vessel_universe(&solar, plant, PropagatorKind::DormantKepler);

    let ev_time = SimInstant::START.advanced(TickSpan::from_seconds_and_subsec_ticks(10, 0));
    let key = EventOrderKey::single(ev_time, phase_ranks::CUSTOM_DETECTOR, VesselId(1), 1, 1);
    u1.enqueue_event(ScheduledEvent::single(
        key,
        VesselId(1),
        Some(0),
        UniverseEventPayload::ModeTransition {
            next_mode_code: 1,
            remaining_chain: 0,
        },
    ))
    .expect("enqueue u1");
    u2.enqueue_event(ScheduledEvent::single(
        key,
        VesselId(1),
        Some(0),
        UniverseEventPayload::ModeTransition {
            next_mode_code: 2,
            remaining_chain: 0,
        },
    ))
    .expect("enqueue u2");

    let s1 = u1.capture_snapshot();
    let s2 = u2.capture_snapshot();
    assert_eq!(s1.physics_state_hash(), s2.physics_state_hash());
    assert_ne!(s1.snapshot_hash(), s2.snapshot_hash());

    let target = SimInstant::START.advanced(TickSpan::from_seconds_and_subsec_ticks(20, 0));
    u1.advance_committed_to(target).expect("advance u1");
    u2.advance_committed_to(target).expect("advance u2");
    assert_ne!(
        u1.committed_checkpoint(VesselId(1))
            .expect("cp1")
            .gnc
            .mode_code,
        u2.committed_checkpoint(VesselId(1))
            .expect("cp2")
            .gnc
            .mode_code
    );
    assert_ne!(
        u1.capture_snapshot().snapshot_hash(),
        u2.capture_snapshot().snapshot_hash()
    );

    let before_cam = u1.capture_snapshot();
    u1.set_active_camera_vessel(Some(VesselId(1)));
    let after_cam = u1.capture_snapshot();
    assert_eq!(before_cam.snapshot_hash(), after_cam.snapshot_hash());
    assert_ne!(
        before_cam.display_state_hash(),
        after_cam.display_state_hash()
    );
}

#[cfg(test)]
fn make_pinned_surface_plant(
    solar: &BuiltinSolarSystem2026_2028,
    body: NaifId,
    fixed_pos_m: DVec3,
    t0: TdbEpoch,
) -> ContinuousPlantState {
    let rot0 = solar.orientation.body_rotation(body, t0).expect("rot0");
    let (pos_i, vel_i) = rot0.fixed_to_icrf_pv(fixed_pos_m, DVec3::ZERO);
    ContinuousPlantState {
        primary_body: body,
        pos_rel_icrf_m: pos_i,
        vel_rel_icrf_m_s: vel_i,
        q_body_to_icrf: DQuat::IDENTITY,
        omega_body_rad_s: DVec3::ZERO,
        propellant_mass_kg: 0.0,
        h_rw_body_nms: DVec3::ZERO,
    }
}

#[test]
fn pinned_surface_rest_corotation_earth_and_moon_serial_and_parallel() {
    let solar = BuiltinSolarSystem2026_2028::load_verified().expect("solar");
    let t0 = TdbEpoch::new(835_000_000, 0);
    let fixed_e = DVec3::new(6_378_137.0, 0.0, 0.0);
    let fixed_m = DVec3::new(1_737_400.0, 0.0, 0.0);
    let plant_e = make_pinned_surface_plant(&solar, NaifId::EARTH, fixed_e, t0);
    let plant_m = make_pinned_surface_plant(&solar, NaifId::MOON, fixed_m, t0);

    let mut u_serial =
        build_single_vessel_universe(&solar, plant_e, PropagatorKind::PinnedSurfaceRest);
    let mut cp_m = VesselCheckpoint::new(VesselId(2), SimInstant::START, plant_m, 2);
    cp_m.integrator_hint.propagator = PropagatorKind::PinnedSurfaceRest;
    u_serial
        .insert_vessel(default_test_vessel_config(), cp_m)
        .expect("insert moon");

    let mut u_par =
        build_single_vessel_universe(&solar, plant_e, PropagatorKind::PinnedSurfaceRest);
    u_par
        .restore_snapshot(&u_serial.capture_snapshot())
        .expect("restore");

    let target = SimInstant::START.advanced(TickSpan::from_seconds_and_subsec_ticks(60, 0));
    u_serial.advance_committed_to(target).expect("serial");
    u_par
        .advance_committed_to_parallel(target, 2)
        .expect("parallel");
    assert_eq!(
        u_serial.capture_snapshot().snapshot_hash(),
        u_par.capture_snapshot().snapshot_hash()
    );

    let t60 = t0.add_span(TickSpan::from_seconds_and_subsec_ticks(60, 0));
    let (exp_pos_e, exp_vel_e) = solar
        .orientation
        .body_rotation(NaifId::EARTH, t60)
        .expect("rot_e60")
        .fixed_to_icrf_pv(fixed_e, DVec3::ZERO);
    let st_e = u_serial
        .committed_checkpoint(VesselId(1))
        .expect("cp_e")
        .plant;
    assert!((st_e.pos_rel_icrf_m - plant_e.pos_rel_icrf_m).length() > 25_000.0);
    assert!((st_e.pos_rel_icrf_m - exp_pos_e).length() < 1.0e-4);
    assert!((st_e.vel_rel_icrf_m_s - exp_vel_e).length() < 1.0e-6);

    let (exp_pos_m, exp_vel_m) = solar
        .orientation
        .body_rotation(NaifId::MOON, t60)
        .expect("rot_m60")
        .fixed_to_icrf_pv(fixed_m, DVec3::ZERO);
    let st_m = u_serial
        .committed_checkpoint(VesselId(2))
        .expect("cp_m")
        .plant;
    assert!((st_m.pos_rel_icrf_m - exp_pos_m).length() < 1.0e-4);
    assert!((st_m.vel_rel_icrf_m_s - exp_vel_m).length() < 1.0e-6);

    let t45_inst = SimInstant::START.advanced(TickSpan::from_seconds_and_subsec_ticks(45, 0));
    let t45_epoch = t0.add_span(TickSpan::from_seconds_and_subsec_ticks(45, 0));
    let (exp_pos_45, _) = solar
        .orientation
        .body_rotation(NaifId::EARTH, t45_epoch)
        .expect("rot_e45")
        .fixed_to_icrf_pv(fixed_e, DVec3::ZERO);
    let sampled_45 = u_serial
        .sample_vessel_state(VesselId(1), t45_inst)
        .expect("sample 45");
    assert!((sampled_45.pos_rel_icrf_m - exp_pos_45).length() < 1.0e-4);
}

#[cfg(test)]
struct PanickingEphemeris {
    content_hash: ContentHash,
}

#[cfg(test)]
impl sim_orbital::EphemerisCatalog for PanickingEphemeris {
    fn content_hash(&self) -> ContentHash {
        self.content_hash
    }

    fn coverage(
        &self,
        _target: NaifId,
        _center: NaifId,
        _order: sim_orbital::DerivativeOrder,
    ) -> Vec<sim_orbital::TdbSpanNs> {
        Vec::new()
    }

    fn geometric_state_icrf(
        &self,
        _target: NaifId,
        _center: NaifId,
        _epoch: TdbEpoch,
        _order: sim_orbital::DerivativeOrder,
    ) -> Result<sim_orbital::KinematicStateIcrf, sim_orbital::EphemerisError> {
        std::panic::resume_unwind(Box::new("simulated worker panic in multi-worker scope"));
    }

    fn segment_knots_in(
        &self,
        _target: NaifId,
        _center: NaifId,
        _start: TdbEpoch,
        _end: TdbEpoch,
    ) -> Vec<TdbEpoch> {
        Vec::new()
    }
}

#[test]
fn parallel_worker_error_reporting_and_l2_nonconvergence_fail_closed() {
    let solar = BuiltinSolarSystem2026_2028::load_verified().expect("solar");
    let nominal_plant = ContinuousPlantState {
        primary_body: NaifId::EARTH,
        pos_rel_icrf_m: DVec3::new(6_778_137.0, 0.0, 0.0),
        vel_rel_icrf_m_s: DVec3::new(0.0, 7_668.6, 0.0),
        q_body_to_icrf: DQuat::IDENTITY,
        omega_body_rad_s: DVec3::ZERO,
        propellant_mass_kg: 0.0,
        h_rw_body_nms: DVec3::ZERO,
    };
    let singular_plant = ContinuousPlantState {
        primary_body: NaifId::EARTH,
        pos_rel_icrf_m: DVec3::ZERO,
        vel_rel_icrf_m_s: DVec3::new(0.0, 7_500.0, 0.0),
        q_body_to_icrf: DQuat::IDENTITY,
        omega_body_rad_s: DVec3::ZERO,
        propellant_mass_kg: 0.0,
        h_rw_body_nms: DVec3::ZERO,
    };
    // Multi-vessel (2 vessels, 2 threads) entering std::thread::scope for structured propagation error:
    let mut u_bad =
        build_single_vessel_universe(&solar, nominal_plant, PropagatorKind::FullCoupled6Dof);
    let cp2 = VesselCheckpoint::new(VesselId(2), SimInstant::START, singular_plant, 2);
    u_bad
        .insert_vessel(default_test_vessel_config(), cp2)
        .expect("insert v2");
    let target = SimInstant::START.advanced(TickSpan::from_seconds_and_subsec_ticks(30, 0));
    let res = u_bad.advance_committed_to_parallel(target, 2);
    assert!(matches!(
        res,
        Err(SchedulerError::Propagation {
            vessel_id: VesselId(2),
            ..
        })
    ));

    let fm = ForceModel::standard_solar_system(solar.constants.clone()).expect("fm");
    let t0 = TdbEpoch::new(835_000_000, 0);
    let mut tight_spec = L2BoundedArcTargetSpec::sun_earth_southern(t0, 60.0 * 86_400.0, 4.0e8);
    tight_spec.tolerance_x_m = 1.0;
    tight_spec.max_iterations = 1;
    let l2_res =
        target_l2_terminal_x_arc_dop853(&tight_spec, &fm, &solar.ephemeris, &solar.orientation);
    assert!(matches!(
        l2_res,
        Err(LagrangeError::SingleShootingFailed {
            reason: L2SingleShootingFailureReason::MaxIterationsReached { max_iterations: 1 },
            iterations: 1,
            ..
        })
    ));
}

#[test]
fn multi_worker_panic_returns_worker_batch_failed() {
    let solar = BuiltinSolarSystem2026_2028::load_verified().expect("solar");
    let nominal_plant = ContinuousPlantState {
        primary_body: NaifId::EARTH,
        pos_rel_icrf_m: DVec3::new(6_778_137.0, 0.0, 0.0),
        vel_rel_icrf_m_s: DVec3::new(0.0, 7_668.6, 0.0),
        q_body_to_icrf: DQuat::IDENTITY,
        omega_body_rad_s: DVec3::ZERO,
        propellant_mass_kg: 0.0,
        h_rw_body_nms: DVec3::ZERO,
    };
    let cfg = default_test_vessel_config();
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
    let mut u_panic = OrbitalUniverse::new(
        manifest,
        clock,
        fm,
        Arc::new(PanickingEphemeris {
            content_hash: solar.ephemeris.content_hash(),
        }),
        Arc::new(solar.orientation.clone()),
    );
    u_panic
        .insert_vessel(
            cfg,
            VesselCheckpoint::new(VesselId(1), SimInstant::START, nominal_plant, 1),
        )
        .expect("v1");
    u_panic
        .insert_vessel(
            cfg,
            VesselCheckpoint::new(VesselId(2), SimInstant::START, nominal_plant, 2),
        )
        .expect("v2");
    let target = SimInstant::START.advanced(TickSpan::from_seconds_and_subsec_ticks(30, 0));
    let panic_res = u_panic.advance_committed_to_parallel(target, 2);
    assert!(matches!(
        panic_res,
        Err(SchedulerError::WorkerBatchFailed {
            expected_proposals: 2,
            received_proposals: 0,
            ..
        })
    ));
}
