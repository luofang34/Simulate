//! Swept collision, bristle surface contact, and docking verification: Bernstein convex-hull counterexample
//! $x(t) = 1 - 16t^2(1-t)^2$, multiplicity-2 grazing tangency, initial overlap,
//! rotating 3D OBB boom collision, discontinuous vertical cliff `ElevationSlab`,
//! sloped-pad anchor bristle stiction zero creep + `PinnedSurfaceRest` wakeup,
//! and `APAS-95` docking/undocking momentum conservation (`< 1e-12` relative).

use glam::{DQuat, DVec3};
use sim_geo::{BodyEllipsoid, Geodetic, LocalFrame};
use sim_orbital::{
    Apas95PortSpec, BodyOrientationCatalog, BuiltinSolarSystem2026_2028, CoastControlLaw,
    ConstantWrenchLaw, ContactFootSpec, ContactRootKind, ContinuousPlantState, DockingError,
    Dop853Tolerances, ForceModel, MassPropertiesModel, NaifId, OrientedCollisionShape,
    PlantStageWrench, PropellantMode, ReactionWheelMode, SweptObbKinematics, SweptPoseTrajectory,
    SweptReachabilityTube, TdbEpoch, VesselContactState, VesselId, check_apas95_capture_gate,
    detect_swept_obb_contacts, detect_terrain_contacts, dock_vessels_conserving_momentum,
    form_rendezvous_groups, integrate_contact_interval, isolate_scalar_bernstein_roots,
    monomial_to_bernstein, propagate_plant_interval, undock_assembly_conserving_momentum,
    vessel_momenta_icrf,
};
use sim_spacetime::{DiscontinuousStepPatch, PatchedEllipsoidTerrain, SimInstant, TickSpan, sqrt};

#[test]
fn bernstein_counterexample_double_crossing_grazing_and_initial_overlap_verified() {
    // User review counterexample:
    //   x(t) = 1 - 16 t^2 (1 - t)^2 = 1 - 16 t^2 + 32 t^3 - 16 t^4
    //   g(t) = x(t)^2 - 0.1^2 on t in [0, 1]
    // Endpoint positions x(0)=x(1)=1 and endpoint velocities x'(0)=x'(1)=0, so cubic
    // Hermite interpolation stays constant at +1, yet true trajectory crosses +/-0.1 four times!
    let x_mono = [1.0, 0.0, -16.0, 32.0, -16.0];
    let [_, _, xb2, _, _] = monomial_to_bernstein(&x_mono);
    assert!((xb2 - (-5.0 / 3.0)).abs() < 1.0e-14);

    // g(t) = (1 - 16 t^2 + 32 t^3 - 16 t^4)^2 - 0.01:
    //   = 0.99 - 32 t^2 + 64 t^3 + 224 t^4 - 1024 t^5 + 1536 t^6 - 1024 t^7 + 256 t^8
    // Since x(t) = 1 - 16 t^2(1-t)^2 dips from +1 to minimum x(0.5) = 0 and back to +1,
    // it crosses x = +0.1 twice (Entry at t1, Exit at t2), whereas cubic Hermite stays at +1!
    let g_mono: [f64; 9] = [
        0.99, 0.0, -32.0, 64.0, 224.0, -1024.0, 1536.0, -1024.0, 256.0,
    ];
    let g_bern = monomial_to_bernstein(&g_mono);
    let roots = isolate_scalar_bernstein_roots(&g_bern, 0.0, 1.0, 1.0e-11, 1.0e-13, &|_| 0.0);
    assert_eq!(roots.len(), 2);

    let u_outer = sqrt(0.9 / 16.0);
    let t1 = 0.5 * (1.0 - sqrt(1.0 - 4.0 * u_outer));
    let t2 = 0.5 * (1.0 + sqrt(1.0 - 4.0 * u_outer));
    let expected = [(t1, ContactRootKind::Entry), (t2, ContactRootKind::Exit)];
    for (got, (exp_t, exp_kind)) in roots.iter().zip(expected.iter()) {
        assert_eq!(got.kind, *exp_kind);
        assert!((got.tau_s - *exp_t).abs() < 1.0e-10);
    }

    // Deeper excursion x2(t) = 1 - 20 t^2(1-t)^2 dips to -0.25 at t=0.5, crossing +/-0.1 4 times!
    let g2_mono: [f64; 9] = [
        0.99, 0.0, -40.0, 80.0, 360.0, -1600.0, 2400.0, -1600.0, 400.0,
    ];
    let g2_bern = monomial_to_bernstein(&g2_mono);
    let roots4 = isolate_scalar_bernstein_roots(&g2_bern, 0.0, 1.0, 1.0e-11, 1.0e-13, &|_| 0.0);
    assert_eq!(roots4.len(), 4);

    // Grazing tangency: g(t) = (t - t*)^2 = t*^2 - 2 t* t + t^2 (non-negative everywhere, g=g'=0 at t*).
    let t_graze = 0.371_234_567_f64;
    let graze_bern = monomial_to_bernstein(&[t_graze * t_graze, -2.0 * t_graze, 1.0]);
    let graze_roots =
        isolate_scalar_bernstein_roots(&graze_bern, 0.0, 1.0, 1.0e-11, 1.0e-13, &|_| 0.0);
    assert_eq!(graze_roots.len(), 1);
    let gr = graze_roots.first().copied().expect("graze root");
    assert_eq!(gr.kind, ContactRootKind::Grazing);
    assert!((gr.tau_s - t_graze).abs() < 1.0e-10);

    // Initial overlap: g(t) = -0.25 + t^2 (starts inside contact at t=0, exits at t=0.5).
    let overlap_bern = monomial_to_bernstein(&[-0.25, 0.0, 1.0]);
    let ov_roots =
        isolate_scalar_bernstein_roots(&overlap_bern, 0.0, 1.0, 1.0e-11, 1.0e-13, &|_| 0.0);
    assert_eq!(ov_roots.len(), 2);
    assert_eq!(
        ov_roots.first().map(|r| (r.kind, r.tau_s)),
        Some((ContactRootKind::InitialOverlap, 0.0))
    );
    let exit_r = ov_roots.get(1).copied().expect("exit");
    assert_eq!(exit_r.kind, ContactRootKind::Exit);
    assert!((exit_r.tau_s - 0.5).abs() < 1.0e-10);
}

#[cfg(test)]
struct SpinningStationaryRig {
    pos: DVec3,
    omega_z_rad_s: f64,
    dur_s: f64,
}

impl SweptPoseTrajectory for SpinningStationaryRig {
    fn duration_s(&self) -> f64 {
        self.dur_s
    }

    fn cg_bernstein_8(&self) -> [DVec3; 8] {
        [self.pos; 8]
    }

    fn sample_pose(&self, tau_s: f64) -> (DVec3, DQuat) {
        let half_ang = 0.5 * self.omega_z_rad_s * tau_s;
        let q = DQuat::from_xyzw(0.0, 0.0, half_ang.sin(), half_ang.cos());
        (self.pos, q)
    }
}

#[test]
fn rotating_obb_sweep_and_deterministic_rendezvous_group_verified() {
    // CGs remain fixed 4.0 m apart along +X while A rotates a 5.0 m half-length boom from +Y toward +X!
    // At t = 0, boom A points along Y (half_extents = (0.2, 5.0, 0.2)), target B is a 0.5 m cube at X = 4.0 m.
    // Pure CG translation sees 0 motion, yet rotational sweep catches the boom sweeping across B around t ~ 0.5 s!
    let rig_a = SpinningStationaryRig {
        pos: DVec3::ZERO,
        omega_z_rad_s: -std::f64::consts::PI,
        dur_s: 1.0,
    };
    let rig_b = SpinningStationaryRig {
        pos: DVec3::new(3.5, 0.0, 0.0),
        omega_z_rad_s: 0.0,
        dur_s: 1.0,
    };
    let kin_a = SweptObbKinematics {
        shape: OrientedCollisionShape::centered_box(0.2, 5.0, 0.2),
        omega_max_rad_s: std::f64::consts::PI,
        alpha_max_rad_s2: 0.0,
        interp_rem_m: 0.0,
    };
    let kin_b = SweptObbKinematics {
        shape: OrientedCollisionShape::centered_box(0.4, 0.4, 0.4),
        omega_max_rad_s: 0.0,
        alpha_max_rad_s2: 0.0,
        interp_rem_m: 0.0,
    };

    let hits = detect_swept_obb_contacts(&rig_a, &kin_a, &rig_b, &kin_b, 1.0e-10);
    assert_eq!(hits.len(), 2);
    let h_in = hits.first().copied().expect("boom entry");
    let h_out = hits.get(1).copied().expect("boom exit");
    assert_eq!(h_in.kind, ContactRootKind::Entry);
    assert_eq!(h_out.kind, ContactRootKind::Exit);
    assert!(h_in.tau_s > 0.35 && h_in.tau_s < 0.50);
    assert!(h_out.tau_s > 0.50 && h_out.tau_s < 0.65);
    assert!(h_in.value.abs() < 1.0e-8);
    assert!(h_out.value.abs() < 1.0e-8);

    verify_rendezvous_group_permutation_invariance();
}

#[cfg(test)]
fn verify_rendezvous_group_permutation_invariance() {
    let t0 = SimInstant::START;
    let t1 = t0.advanced(TickSpan::SECOND);
    let mk_tube = |vid: u64, x0: f64, x1: f64| SweptReachabilityTube {
        vessel_id: VesselId(vid),
        primary_body: NaifId::EARTH,
        start_instant: t0,
        end_instant: t1,
        aabb_min_m: DVec3::new(x0, -10.0, -10.0),
        aabb_max_m: DVec3::new(x1, 10.0, 10.0),
        margin_m: 5.0,
    };
    let fwd = [
        mk_tube(10, 0.0, 100.0),
        mk_tube(4, 80.0, 180.0),
        mk_tube(7, 160.0, 220.0),
        mk_tube(99, 5_000.0, 5_100.0),
    ];
    let [f0, f1, f2, f3] = fwd;
    let rev = [f3, f2, f0, f1];
    let g_fwd = form_rendezvous_groups(&fwd);
    let g_rev = form_rendezvous_groups(&rev);
    assert_eq!(g_fwd, g_rev);
    assert_eq!(g_fwd.len(), 1);
    let grp = g_fwd.first().expect("group");
    assert_eq!(grp.leader, VesselId(4));
    assert_eq!(grp.members.len(), 3);
}

#[cfg(test)]
fn moon_fixed_to_icrf_state(
    solar: &BuiltinSolarSystem2026_2028,
    t0: TdbEpoch,
    p_fixed: DVec3,
    v_fixed: DVec3,
    q_b2f: DQuat,
) -> ContinuousPlantState {
    let rot = solar
        .orientation
        .body_rotation(NaifId::MOON, t0)
        .expect("moon rot");
    let r_f2i = rot.r_icrf_to_fixed.transpose();
    let (pos_icrf, vel_icrf) = rot.fixed_to_icrf_pv(p_fixed, v_fixed);
    let q_f2i = DQuat::from_mat3(&r_f2i);
    ContinuousPlantState {
        primary_body: NaifId::MOON,
        pos_rel_icrf_m: pos_icrf,
        vel_rel_icrf_m_s: vel_icrf,
        q_body_to_icrf: (q_f2i * q_b2f).normalize(),
        omega_body_rad_s: DVec3::ZERO,
        propellant_mass_kg: 0.0,
        h_rw_body_nms: DVec3::ZERO,
    }
}

#[test]
fn discontinuous_cliff_slab_and_physical_impact_envelope_verified() {
    let solar = BuiltinSolarSystem2026_2028::load_verified().expect("solar");
    let fm = ForceModel::standard_solar_system(solar.constants.clone()).expect("fm");
    let t0 = TdbEpoch::new(830_000_000, 0);

    let cliff = DiscontinuousStepPatch {
        anchor_lat_rad: 0.0,
        anchor_lon_rad: 0.0,
        east_min_m: 20.0,
        east_max_m: 500.0,
        north_min_m: -500.0,
        north_max_m: 500.0,
        base_height_m: 80.0,
        slope_east: 0.0,
        slope_north: 0.0,
        mu_static: 0.85,
        mu_kinetic: 0.65,
    };
    let terrain = PatchedEllipsoidTerrain::moon_iau(7, vec![cliff]);

    let ell = BodyEllipsoid::MOON_IAU;
    let start_geo = Geodetic {
        lat_rad: 0.0,
        lon_rad: -30.0 / ell.semi_major_m,
        height_m: 35.0,
    };
    let lf = LocalFrame::at(&start_geo);
    let init = moon_fixed_to_icrf_state(
        &solar,
        t0,
        ell.geodetic_to_fixed(&start_geo),
        lf.enu_to_ecef() * DVec3::new(100.0, 0.0, 0.0),
        DQuat::IDENTITY,
    );
    let mass = MassPropertiesModel::diagonal(12_000.0, [40_000.0, 40_000.0, 8_000.0], [0.0; 3]);
    let tol = Dop853Tolerances {
        h_init_s: 1.0,
        h_max_s: 1.0,
        ..Dop853Tolerances::default()
    };
    let out = propagate_plant_interval(
        &init,
        t0,
        t0.add_span(TickSpan::SECOND),
        Some(1.0),
        &tol,
        &mass,
        PropellantMode::InfiniteConstantMass,
        ReactionWheelMode::ConserveMomentum {
            h_max_nms: 5.0e6,
            tau_max_nm: 5.0e4,
        },
        &fm,
        &solar.ephemeris,
        &solar.orientation,
        &CoastControlLaw,
    )
    .expect("skimmer prop");

    let mut all_hits = Vec::new();
    for seg in &out.segments {
        let base_off = seg.start_epoch.signed_offset_seconds_from(t0);
        for mut h in detect_terrain_contacts(seg, 1.5, 1.0e-4, &terrain, &solar.orientation, 1.0e-9)
        {
            h.tau_s += base_off;
            all_hits.push(h);
        }
    }
    let first_hit = all_hits.first().copied().expect("cliff hit");
    assert_eq!(first_hit.kind, ContactRootKind::Entry);
    assert!((first_hit.tau_s - 0.50).abs() < 0.02);
}

#[cfg(test)]
fn setup_sloped_lunar_pad_descent(
    solar: &BuiltinSolarSystem2026_2028,
    t0: TdbEpoch,
    y_cg_m: f64,
) -> (PatchedEllipsoidTerrain, ContinuousPlantState) {
    let slope_tan = 8.5_f64.to_radians().tan();
    let pad = DiscontinuousStepPatch {
        anchor_lat_rad: 0.0,
        anchor_lon_rad: 0.0,
        east_min_m: -200.0,
        east_max_m: 200.0,
        north_min_m: -200.0,
        north_max_m: 200.0,
        base_height_m: 10.0,
        slope_east: slope_tan,
        slope_north: 0.0,
        mu_static: 0.85,
        mu_kinetic: 0.65,
    };
    let terrain = PatchedEllipsoidTerrain::moon_iau(11, vec![pad]);
    let ell = BodyEllipsoid::MOON_IAU;
    let pad_geo = Geodetic {
        lat_rad: 0.0,
        lon_rad: 0.0,
        height_m: 10.0 + (y_cg_m - 0.0369) + 0.05,
    };
    let lf = LocalFrame::at(&pad_geo);
    let n_fixed = (lf.enu_to_ecef() * DVec3::new(-slope_tan, 0.0, 1.0).normalize()).normalize();
    let init = moon_fixed_to_icrf_state(
        solar,
        t0,
        ell.geodetic_to_fixed(&pad_geo),
        lf.enu_to_ecef() * DVec3::new(-0.35, 0.0, -0.75),
        DQuat::from_rotation_arc(DVec3::Y, n_fixed),
    );
    (terrain, init)
}

#[test]
fn sloped_pad_bristle_stiction_zero_creep_and_pinned_rest_liftoff_verified() {
    let solar = BuiltinSolarSystem2026_2028::load_verified().expect("solar");
    let fm = ForceModel::standard_solar_system(solar.constants.clone()).expect("fm");
    let t0 = TdbEpoch::new(835_000_000, 0);
    let feet = ContactFootSpec::raketa_four_fin_pads(4.2);
    let (terrain, init) = setup_sloped_lunar_pad_descent(&solar, t0, 4.2);
    let mass = MassPropertiesModel::diagonal(12_000.0, [85_000.0, 18_000.0, 85_000.0], [0.0; 3]);
    let rw = ReactionWheelMode::ConserveMomentum {
        h_max_nms: 5.0e6,
        tau_max_nm: 5.0e4,
    };

    let t_settle = t0.add_span(TickSpan::from_seconds_and_subsec_ticks(6, 0));
    let settled = integrate_contact_interval(
        &init,
        VesselContactState::default(),
        &feet,
        t0,
        t_settle,
        &mass,
        PropellantMode::InfiniteConstantMass,
        rw,
        &fm,
        &terrain,
        &solar.ephemeris,
        &solar.orientation,
        &CoastControlLaw,
    )
    .expect("settle on slope");
    let pinned_0 = settled.contact.pinned.expect("entered PinnedSurfaceRest");

    let t_hold = t_settle.add_span(TickSpan::from_seconds_and_subsec_ticks(600, 0));
    let held = integrate_contact_interval(
        &settled.state,
        settled.contact,
        &feet,
        t_settle,
        t_hold,
        &mass,
        PropellantMode::InfiniteConstantMass,
        rw,
        &fm,
        &terrain,
        &solar.ephemeris,
        &solar.orientation,
        &CoastControlLaw,
    )
    .expect("hold on slope");
    let pinned_1 = held.contact.pinned.expect("still pinned after 600 s");
    assert_eq!(pinned_0.pos_fixed_m, pinned_1.pos_fixed_m);

    let liftoff_law = ConstantWrenchLaw {
        wrench: PlantStageWrench {
            main_thrust_body_n: DVec3::new(0.0, 12_000.0 * 5.0, 0.0),
            ..PlantStageWrench::default()
        },
    };
    let wake = integrate_contact_interval(
        &held.state,
        held.contact,
        &feet,
        t_hold,
        t_hold.add_span(TickSpan::from_seconds_and_subsec_ticks(0, 11_520_000_000)),
        &mass,
        PropellantMode::InfiniteConstantMass,
        rw,
        &fm,
        &terrain,
        &solar.ephemeris,
        &solar.orientation,
        &liftoff_law,
    )
    .expect("liftoff wakeup");
    assert!(wake.contact.pinned.is_none());
    assert!(wake.min_foot_clearance_m > 0.0);
}

#[test]
fn apas95_docking_and_undocking_conserve_linear_and_angular_momentum_verified() {
    let mass_a = MassPropertiesModel::diagonal(12_000.0, [82_000.0, 16_000.0, 82_000.0], [0.0; 3]);
    let mass_b = MassPropertiesModel::diagonal(8_500.0, [48_000.0, 12_000.0, 51_000.0], [0.0; 3]);
    let port_a = Apas95PortSpec::raketa_nose_port(4.5);
    let port_b = Apas95PortSpec::standard(DVec3::new(0.15, 5.2, -0.10), DVec3::Y);

    let q_a = DQuat::from_euler(glam::EulerRot::XYZ, 0.25, -0.4, 0.7).normalize();
    // Align B so its +Y port faces antiparallel to A's +Y port (rotate PI about X in A-body, plus 1 deg roll/tilt).
    let q_flip =
        DQuat::from_euler(glam::EulerRot::XYZ, std::f64::consts::PI - 0.02, 0.35, 0.01).normalize();
    let q_b = (q_a * q_flip).normalize();

    let pos_a = DVec3::new(6_778_000.0, 210_000.0, -95_000.0);
    let pa_world = pos_a + q_a * port_a.pos_body_m;
    let na_world = q_a * port_a.outward_axis_body;
    // Place B's port 0.05 m axially ahead of A's port with 0.03 m lateral offset.
    let pb_target = pa_world + na_world * 0.05 + (q_a * DVec3::X) * 0.03;
    let pos_b = pb_target - q_b * port_b.pos_body_m;

    let st_a = ContinuousPlantState {
        primary_body: NaifId::EARTH,
        pos_rel_icrf_m: pos_a,
        vel_rel_icrf_m_s: DVec3::new(-180.0, 7_650.0, 410.0),
        q_body_to_icrf: q_a,
        omega_body_rad_s: DVec3::new(0.004, -0.006, 0.003),
        propellant_mass_kg: 0.0,
        h_rw_body_nms: DVec3::new(450.0, -320.0, 180.0),
    };
    let st_b = ContinuousPlantState {
        primary_body: NaifId::EARTH,
        pos_rel_icrf_m: pos_b,
        vel_rel_icrf_m_s: DVec3::new(-180.08, 7_649.94, 410.05),
        q_body_to_icrf: q_b,
        omega_body_rad_s: DVec3::new(-0.002, 0.005, -0.003),
        propellant_mass_kg: 0.0,
        h_rw_body_nms: DVec3::new(-210.0, 510.0, -90.0),
    };

    // Out-of-envelope speed check fails closed.
    let mut too_fast_b = st_b;
    too_fast_b.vel_rel_icrf_m_s += DVec3::new(2.5, 0.0, 0.0);
    assert!(matches!(
        check_apas95_capture_gate(&st_a, &port_a, &too_fast_b, &port_b),
        Err(DockingError::ExcessiveRelativeSpeed { .. })
    ));

    let (pa0, ha0) = vessel_momenta_icrf(&st_a, &mass_a);
    let (pb0, hb0) = vessel_momenta_icrf(&st_b, &mass_b);
    let p_init = pa0 + pb0;
    let h_init = ha0 + hb0;

    let (merged_st, merged_mass, rec) = dock_vessels_conserving_momentum(
        VesselId(1),
        &st_a,
        &mass_a,
        &port_a,
        VesselId(2),
        &st_b,
        &mass_b,
        &port_b,
    )
    .expect("APAS-95 soft capture");

    let (p_merged, h_merged) = vessel_momenta_icrf(&merged_st, &merged_mass);
    assert!((p_merged - p_init).length() / p_init.length() < 1.0e-14);
    assert!((h_merged - h_init).length() / h_init.length() < 1.0e-14);

    // Undock with 450 N*s push-off spring impulse and verify exact conservation of P and H_ICRF!
    let (st_a_sep, st_b_sep) = undock_assembly_conserving_momentum(&merged_st, &rec, 450.0);
    let (pa1, ha1) = vessel_momenta_icrf(&st_a_sep, &mass_a);
    let (pb1, hb1) = vessel_momenta_icrf(&st_b_sep, &mass_b);
    let p_after = pa1 + pb1;
    let h_after = ha1 + hb1;

    assert!((p_after - p_init).length() / p_init.length() < 1.0e-14);
    assert!((h_after - h_init).length() / h_init.length() < 1.0e-14);
    assert!((st_b_sep.vel_rel_icrf_m_s - st_a_sep.vel_rel_icrf_m_s).dot(na_world) > 0.05);
}
