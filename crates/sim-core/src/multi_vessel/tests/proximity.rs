//! Proximity-group shared-integrator invariants: momentum conservation to
//! round-off, step-rate convergence, multi-vessel chains, pinned endpoints,
//! and adaptive/stiff integrators.

use mbd_multibody::{
    DockingPhase, DockingSpec, DockingState, MultiBodyDynamics, PinnedSurfaceRest, TetherCommand,
    Tolerances, WinchTetherSpec, WinchTetherState,
};

use super::*;
use crate::{Block, VehicleIntegrator};

const ORBIT_X_M: f64 = 7_000_000.0;

fn unit_body() -> BodyEllipsoid {
    BodyEllipsoid {
        mu_m3_s2: 0.0,
        omega_rad_s: 0.0,
        semi_major_m: 1.0,
        flattening: 0.0,
        ..BodyEllipsoid::WGS84
    }
}

fn inertial_momentum(block: &MultibodyVehicleBlock, cm_ref_inertial: DVec3) -> (DVec3, DVec3, f64) {
    let dyn_masses = block.dynamic_mass_contributions().unwrap();
    let fb = block.floating_base_fixed();
    let mom = MultiBodyDynamics::system_momentum(block.tree(), &dyn_masses, &fb).unwrap();
    let ang_about_ref = mom.angular_about_com_world_nms
        + (mom.com_world_m - cm_ref_inertial).cross(mom.linear_world_kg_m_s);
    (mom.linear_world_kg_m_s, ang_about_ref, mom.total_mass_kg)
}

fn off_cg_vessel(
    pos_ecef: DVec3,
    vel_ecef: DVec3,
    omega_body: DVec3,
    body: BodyEllipsoid,
    steps_per_second: u64,
    integrator: VehicleIntegrator,
) -> MultibodyVehicleBlock {
    let tree = ComponentTree::new([
        part("core", Placement::ROOT, JointSpec::FIXED, 900.0, 350.0),
        part(
            "adapter",
            Placement::child_of(ComponentId(0), DVec3::new(1.2, 0.3, -0.2)),
            JointSpec::FIXED,
            150.0,
            60.0,
        ),
    ])
    .unwrap();
    MultibodyVehicleBlock::new(
        crate::multibody_vehicle::MultibodyVehicleConfig {
            body,
            period: TickSpan::from_ticks(TICKS_PER_SECOND / steps_per_second),
            epoch: UtcTime::from_unix_seconds(0.0),
            initial: VehicleState {
                position_ecef: pos_ecef,
                velocity_ecef: vel_ecef,
                attitude: DQuat::IDENTITY,
                angular_rate_body: omega_body,
            },
            integrator,
            time_scale: MotionTimeScale::Seconds(1.0),
        },
        tree,
        Vec::new(),
        MultibodySubsystems::default(),
        Box::new(VacuumAtmosphere),
        std::sync::Arc::new(body),
    )
    .unwrap()
}

fn off_cg_docking_spec() -> DockingSpec {
    DockingSpec {
        capture_radius_m: 1.5,
        capture_angle_rad: 0.6,
        max_closing_speed_m_s: 2.0,
        damping_vel_threshold_m_s: 0.2,
        damping_rate_threshold_rad_s: 0.1,
        retraction_speed_m_s: 0.5,
        linear_stiffness_n_m: 3_500.0,
        linear_damping_n_s_m: 1_400.0,
        angular_stiffness_nm_rad: 600.0,
        angular_damping_nm_s_rad: 180.0,
        undock_impulse_ns: 40.0,
        min_capture_standoff_m: DockingSpec::DEFAULT_MIN_CAPTURE_STANDOFF_M,
        damped_dwell_s: DockingSpec::DEFAULT_DAMPED_DWELL_S,
        latch_standoff_tolerance_m: DockingSpec::DEFAULT_LATCH_STANDOFF_TOLERANCE_M,
        latch_distance_m: DockingSpec::DEFAULT_LATCH_DISTANCE_M,
    }
}

fn tether_spec() -> WinchTetherSpec {
    WinchTetherSpec {
        stiffness_n_m: 2_000.0,
        damping_n_s_m: 400.0,
        max_length_m: 20.0,
        breaking_tension_n: 50_000.0,
        burn_through_energy_j: None,
        auto_cut_low_tension_n: None,
        auto_cut_dwell_s: 0.0,
    }
}

fn run_docking_momentum(
    origin_x: f64,
    v_bulk: DVec3,
    body: BodyEllipsoid,
    integrator: VehicleIntegrator,
) -> (f64, f64, bool) {
    let chaser = off_cg_vessel(
        DVec3::new(origin_x, 0.0, 0.0),
        v_bulk + DVec3::new(0.2, 0.0, 0.0),
        DVec3::new(0.02, -0.04, 0.05),
        body,
        50,
        integrator,
    );
    let target = off_cg_vessel(
        DVec3::new(origin_x + 1.4, 0.25, -0.15),
        v_bulk - DVec3::new(0.2, 0.0, 0.0),
        DVec3::new(-0.03, 0.02, -0.04),
        body,
        50,
        integrator,
    );
    let mut coord = MultiVesselCoordinator::new(chaser);
    let t_id = coord.spawn_independent(target);
    let ref_cm = DVec3::new(origin_x + 0.7, 0.0, 0.0);
    let (lin0_a, ang0_a, _) = inertial_momentum(coord.vessel(SimVesselId(1)).unwrap(), ref_cm);
    let (lin0_b, ang0_b, _) = inertial_momentum(coord.vessel(t_id).unwrap(), ref_cm);
    let (p0, l0) = (lin0_a + lin0_b, ang0_a + ang0_b);
    coord.add_link(InterVesselLink::Docking {
        id: 7,
        chaser: SimVesselId(1),
        port_a: ComponentId(1),
        offset_a_local_m: DVec3::new(0.3, -0.1, 0.05),
        axis_a_local: DVec3::X,
        target: t_id,
        port_b: ComponentId(1),
        offset_b_local_m: DVec3::new(-1.3, -0.2, 0.1),
        mate_placement: Placement::child_of(ComponentId(1), DVec3::new(0.5, 0.0, 0.0)),
        spec: off_cg_docking_spec(),
        state: DockingState {
            phase: DockingPhase::Open,
            standoff_m: 0.3,
            dwell_s: 0.0,
        },
    });
    let mut max_dp = 0.0_f64;
    let mut max_dl = 0.0_f64;
    let mut merged = false;
    for _ in 0..120 {
        let rep = coord.step_all(&BTreeMap::new()).unwrap();
        if coord.vessel(t_id).is_some() {
            let (la, aa, _) = inertial_momentum(coord.vessel(SimVesselId(1)).unwrap(), ref_cm);
            let (lb, ab, _) = inertial_momentum(coord.vessel(t_id).unwrap(), ref_cm);
            max_dp = max_dp.max(((la + lb) - p0).length() / p0.length().max(1.0));
            max_dl = max_dl.max(((aa + ab) - l0).length() / l0.length().max(1.0));
        }
        if !rep.merged_vessels.is_empty() {
            merged = true;
            break;
        }
    }
    (max_dp, max_dl, merged)
}

#[test]
fn free_space_docking_off_cg_with_spin_conserves_momentum_to_roundoff() {
    let v_bulk = DVec3::new(11.8, -3.5, 1.8);
    let (dp_adapt, dl_adapt, merged) = run_docking_momentum(
        10.0,
        v_bulk,
        unit_body(),
        VehicleIntegrator::adaptive(1e-12, 1e-12),
    );
    assert!(
        merged,
        "expected off-CG spinning capture to latch and merge"
    );
    assert!(dp_adapt < 1e-12, "linear momentum rel err {dp_adapt}");
    assert!(dl_adapt < 1e-11, "angular momentum rel err {dl_adapt}");

    let (dp_orbit, dl_orbit, merged_orbit) = run_docking_momentum(
        ORBIT_X_M,
        v_bulk,
        zero_grav_body(),
        VehicleIntegrator::REAL_TIME,
    );
    assert!(merged_orbit);
    assert!(
        dp_orbit < 1e-12,
        "orbital linear momentum rel err {dp_orbit}"
    );
    assert!(dl_orbit < 1e-7, "orbital RK4 angular rel err {dl_orbit}");
}

#[test]
fn free_space_tether_off_cg_with_spin_conserves_momentum_to_roundoff() {
    let carrier = off_cg_vessel(
        DVec3::new(10.0, 0.0, 0.0),
        DVec3::new(4.0, -1.5, 0.8),
        DVec3::new(0.03, -0.02, 0.04),
        unit_body(),
        50,
        VehicleIntegrator::adaptive(1e-12, 1e-12),
    );
    let payload = off_cg_vessel(
        DVec3::new(16.0, 0.5, -0.3),
        DVec3::new(4.8, -1.3, 0.6),
        DVec3::new(-0.04, 0.03, -0.02),
        unit_body(),
        50,
        VehicleIntegrator::adaptive(1e-12, 1e-12),
    );
    let mut coord = MultiVesselCoordinator::new(carrier);
    let p_id = coord.spawn_independent(payload);
    let ref_cm = DVec3::new(13.0, 0.0, 0.0);
    let (lin0_a, ang0_a, _) = inertial_momentum(coord.vessel(SimVesselId(1)).unwrap(), ref_cm);
    let (lin0_b, ang0_b, _) = inertial_momentum(coord.vessel(p_id).unwrap(), ref_cm);
    let (p0, l0) = (lin0_a + lin0_b, ang0_a + ang0_b);
    coord.add_link(InterVesselLink::WinchTether {
        id: 2,
        carrier: SimVesselId(1),
        anchor_carrier: ComponentId(1),
        offset_carrier_m: DVec3::new(0.2, 0.1, -0.1),
        payload: p_id,
        anchor_payload: ComponentId(1),
        offset_payload_m: DVec3::new(-0.8, -0.2, 0.15),
        spec: tether_spec(),
        state: WinchTetherState::intact_at(5.0),
        command: TetherCommand {
            payout_rate_m_s: -0.2,
            heat_rate_w: 0.0,
            cut: false,
        },
    });
    for _ in 0..100 {
        coord.step_all(&BTreeMap::new()).unwrap();
        let (la, aa, _) = inertial_momentum(coord.vessel(SimVesselId(1)).unwrap(), ref_cm);
        let (lb, ab, _) = inertial_momentum(coord.vessel(p_id).unwrap(), ref_cm);
        let dp_rel = ((la + lb) - p0).length() / p0.length().max(1.0);
        let dl_rel = ((aa + ab) - l0).length() / l0.length().max(1.0);
        assert!(dp_rel < 1e-12, "tether linear rel err {dp_rel}");
        assert!(dl_rel < 1e-11, "tether angular rel err {dl_rel}");
    }
}

fn taut_tether_with(steps_per_second: u64, substeps: u32) -> f64 {
    let integ = VehicleIntegrator::FixedRk4 { substeps };
    let a = off_cg_vessel(
        DVec3::new(10.0, 0.0, 0.0),
        DVec3::ZERO,
        DVec3::ZERO,
        unit_body(),
        steps_per_second,
        integ,
    );
    let b = off_cg_vessel(
        DVec3::new(20.0, 0.0, 0.0),
        DVec3::ZERO,
        DVec3::ZERO,
        unit_body(),
        steps_per_second,
        integ,
    );
    let mut coord = MultiVesselCoordinator::new(a);
    let b_id = coord.spawn_independent(b);
    let mut spec = tether_spec();
    spec.stiffness_n_m = 40_000.0;
    spec.damping_n_s_m = 800.0;
    spec.breaking_tension_n = 1_000_000.0;
    coord.add_link(InterVesselLink::WinchTether {
        id: 5,
        carrier: SimVesselId(1),
        anchor_carrier: ComponentId(0),
        offset_carrier_m: DVec3::new(0.5, 0.0, 0.0),
        payload: b_id,
        anchor_payload: ComponentId(0),
        offset_payload_m: DVec3::new(-0.5, 0.1, 0.0),
        spec,
        state: WinchTetherState::intact_at(6.0),
        command: TetherCommand {
            payout_rate_m_s: 0.0,
            heat_rate_w: 0.0,
            cut: false,
        },
    });
    for _ in 0..(steps_per_second / 10) {
        coord.step_all(&BTreeMap::new()).unwrap();
    }
    let ao = coord.vessel(SimVesselId(1)).unwrap().output();
    let bo = coord.vessel(b_id).unwrap().output();
    (bo.position_ecef - ao.position_ecef).length()
}

#[test]
fn smooth_tether_exhibits_step_halving_and_400_500hz_agreement() {
    let g50 = taut_tether_with(50, 1);
    let g100 = taut_tether_with(100, 1);
    let g200 = taut_tether_with(200, 1);
    let g400 = taut_tether_with(400, 1);
    let g500 = taut_tether_with(500, 1);
    let g800 = taut_tether_with(800, 1);
    let g50_x16 = taut_tether_with(50, 16);
    assert!((g800 - g50_x16).abs() < 1e-13);
    let e50 = (g50 - g800).abs();
    let e100 = (g100 - g800).abs();
    let e200 = (g200 - g800).abs();
    let e400 = (g400 - g800).abs();
    assert!(e50 > e100 * 10.0, "expected ~16x decay: {e50} -> {e100}");
    assert!(e100 > e200 * 10.0, "expected ~16x decay: {e100} -> {e200}");
    assert!(e200 > e400 * 10.0, "expected ~16x decay: {e200} -> {e400}");
    assert!(
        (g400 - g500).abs() < 1e-8,
        "400 vs 500 Hz diff: {}",
        (g400 - g500).abs()
    );
}

#[test]
fn three_vessel_chain_steps_with_stiff_rosenbrock_and_pinned_endpoint() {
    let body = BodyEllipsoid {
        omega_rad_s: 0.0,
        ..BodyEllipsoid::WGS84
    };
    let surf_x = body.semi_major_m;
    let a = off_cg_vessel(
        DVec3::new(surf_x + 8.0, 0.0, 0.0),
        DVec3::ZERO,
        DVec3::ZERO,
        body,
        50,
        VehicleIntegrator::Stiff(Tolerances::uniform(1e-7, 1e-7)),
    );
    let b = off_cg_vessel(
        DVec3::new(surf_x + 6.6, 0.0, 0.0),
        DVec3::ZERO,
        DVec3::ZERO,
        body,
        50,
        VehicleIntegrator::adaptive(1e-8, 1e-8),
    );
    let c_pos = DVec3::new(surf_x, 0.0, 0.0);
    let c = off_cg_vessel(
        c_pos,
        DVec3::ZERO,
        DVec3::ZERO,
        body,
        50,
        VehicleIntegrator::REAL_TIME,
    )
    .with_pinned_rest(PinnedSurfaceRest::Pinned {
        world_pos_m: c_pos,
        world_rot: DQuat::IDENTITY,
    });
    let mut coord = MultiVesselCoordinator::new(a);
    let b_id = coord.spawn_independent(b);
    let c_id = coord.spawn_independent(c);
    coord.add_link(InterVesselLink::Docking {
        id: 1,
        chaser: SimVesselId(1),
        port_a: ComponentId(0),
        offset_a_local_m: DVec3::ZERO,
        axis_a_local: DVec3::NEG_X,
        target: b_id,
        port_b: ComponentId(0),
        offset_b_local_m: DVec3::ZERO,
        mate_placement: Placement::child_of(ComponentId(0), DVec3::ZERO),
        spec: off_cg_docking_spec(),
        state: DockingState {
            phase: DockingPhase::Open,
            standoff_m: 1.4,
            dwell_s: 0.0,
        },
    });
    coord.add_link(InterVesselLink::WinchTether {
        id: 2,
        carrier: b_id,
        anchor_carrier: ComponentId(0),
        offset_carrier_m: DVec3::new(-0.5, 0.0, 0.0),
        payload: c_id,
        anchor_payload: ComponentId(0),
        offset_payload_m: DVec3::ZERO,
        spec: tether_spec(),
        state: WinchTetherState::intact_at(5.5),
        command: TetherCommand {
            payout_rate_m_s: -0.3,
            heat_rate_w: 0.0,
            cut: false,
        },
    });
    for _ in 0..20 {
        coord.step_all(&BTreeMap::new()).unwrap();
    }
    assert!(matches!(
        coord.vessel(c_id).unwrap().pinned_rest(),
        PinnedSurfaceRest::Pinned { .. }
    ));
    assert!(coord.vessel(b_id).unwrap().output().position_ecef.x < surf_x + 6.6);
}
