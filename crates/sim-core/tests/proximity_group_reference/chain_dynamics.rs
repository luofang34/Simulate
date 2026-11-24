//! Three-vessel articulated chain (`A --tether-- B --docking-- C`) conservation,
//! fourth-order step-halving convergence, adaptive reference match, and
//! singleton vessel bit-identity.

use std::collections::BTreeMap;

use glam::{DQuat, DVec3};
use mbd_core::ComponentId;
use mbd_multibody::{JointDriveInput, WinchTetherSpec};
use sim_core::{
    Block, MultiVesselCoordinator, MultibodyVehicleBlock, MultibodyVehicleInput, SimVesselId,
    VehicleIntegrator, VehicleState,
};

use super::models::{
    articulated_vessel, default_docking_spec, docking_link, free_space_body, report_line,
    system_totals, tether_link,
};

fn make_singleton(steps_per_second: u64, integrator: VehicleIntegrator) -> MultibodyVehicleBlock {
    articulated_vessel(
        "D",
        free_space_body(),
        VehicleState {
            position_ecef: DVec3::new(-50.0, 25.0, 10.0),
            velocity_ecef: DVec3::new(-1.0, 2.5, -0.7),
            attitude: DQuat::from_euler(glam::EulerRot::ZYX, 0.3, -0.2, 0.1),
            angular_rate_body: DVec3::new(0.11, -0.07, 0.09),
        },
        steps_per_second,
        integrator,
        DVec3::new(1.0, 0.0, 0.0),
        DVec3::Z,
        0.25,
        -0.2,
    )
}

fn spawn_chain_vessels(
    steps_per_second: u64,
    integrator: VehicleIntegrator,
) -> (MultiVesselCoordinator, SimVesselId, SimVesselId) {
    let body = free_space_body();
    let v_bulk = DVec3::new(3.2, -1.4, 0.9);
    let a = articulated_vessel(
        "A",
        body,
        VehicleState {
            position_ecef: DVec3::new(0.0, 0.0, 0.0),
            velocity_ecef: v_bulk + DVec3::new(-0.15, 0.08, -0.04),
            attitude: DQuat::from_euler(glam::EulerRot::ZYX, 0.12, -0.05, 0.08),
            angular_rate_body: DVec3::new(0.05, -0.04, 0.07),
        },
        steps_per_second,
        integrator,
        DVec3::new(1.1, 0.25, -0.1),
        DVec3::Z,
        0.2,
        0.15,
    );
    let b = articulated_vessel(
        "B",
        body,
        VehicleState {
            position_ecef: DVec3::new(5.8, 0.4, -0.2),
            velocity_ecef: v_bulk + DVec3::new(0.1, -0.05, 0.06),
            attitude: DQuat::from_euler(glam::EulerRot::ZYX, -0.09, 0.07, -0.04),
            angular_rate_body: DVec3::new(-0.04, 0.06, -0.03),
        },
        steps_per_second,
        integrator,
        DVec3::new(-0.9, -0.2, 0.15),
        DVec3::Y,
        -0.15,
        -0.12,
    );
    let c = articulated_vessel(
        "C",
        body,
        VehicleState {
            position_ecef: DVec3::new(7.4, 0.65, -0.1),
            velocity_ecef: v_bulk + DVec3::new(-0.08, -0.03, -0.02),
            attitude: DQuat::from_euler(glam::EulerRot::ZYX, 0.06, 0.04, -0.08),
            angular_rate_body: DVec3::new(0.03, 0.02, -0.06),
        },
        steps_per_second,
        integrator,
        DVec3::new(0.85, 0.15, 0.2),
        DVec3::X,
        0.1,
        0.18,
    );
    let mut coord = MultiVesselCoordinator::new(a);
    let b_id = coord.spawn_independent(b);
    let c_id = coord.spawn_independent(c);
    (coord, b_id, c_id)
}

fn build_chain_scenario(
    steps_per_second: u64,
    integrator: VehicleIntegrator,
    include_singleton: bool,
    initial_dock_phase: mbd_multibody::DockingPhase,
    initial_tether_len_m: f64,
) -> (
    MultiVesselCoordinator,
    Vec<SimVesselId>,
    Option<MultibodyVehicleBlock>,
) {
    let (mut coord, b_id, c_id) = spawn_chain_vessels(steps_per_second, integrator);
    let mut chain_ids = vec![SimVesselId(1), b_id, c_id];
    let singleton = if include_singleton {
        let d_id = coord.spawn_independent(make_singleton(steps_per_second, integrator));
        chain_ids.push(d_id);
        Some(make_singleton(steps_per_second, integrator))
    } else {
        None
    };

    coord.add_link(tether_link(
        1,
        SimVesselId(1),
        ComponentId(1),
        DVec3::new(0.6, 0.12, -0.08),
        b_id,
        ComponentId(1),
        DVec3::new(-0.55, -0.14, 0.09),
        WinchTetherSpec {
            stiffness_n_m: 2_500.0,
            damping_n_s_m: 350.0,
            max_length_m: 12.0,
            breaking_tension_n: 100_000.0,
            burn_through_energy_j: None,
            auto_cut_low_tension_n: None,
            auto_cut_dwell_s: 0.0,
        },
        initial_tether_len_m,
        -0.15,
    ));
    let mut dock_spec = default_docking_spec();
    dock_spec.damped_dwell_s = 2.0; // Stay in compliant contact over the 0.8s test window.
    let mut link = docking_link(
        2,
        b_id,
        ComponentId(0),
        DVec3::new(0.75, 0.15, 0.05),
        DVec3::X,
        c_id,
        ComponentId(1),
        DVec3::new(-0.7, -0.1, -0.05),
        dock_spec,
        0.4,
    );
    if let sim_core::InterVesselLink::Docking { state, .. } = &mut link {
        state.phase = initial_dock_phase;
    }
    coord.add_link(link);
    (coord, chain_ids, singleton)
}

fn chain_inputs(with_drive: bool) -> BTreeMap<SimVesselId, MultibodyVehicleInput> {
    let mut map = BTreeMap::new();
    if with_drive {
        let mut inp = MultibodyVehicleInput::default();
        inp.joint_drives.push(JointDriveInput {
            component: ComponentId(1),
            effort_scalar: 4.5,
            torque_3d_nm: DVec3::ZERO,
            force_3d_n: DVec3::ZERO,
        });
        map.insert(SimVesselId(1), inp);
    }
    map
}

#[test]
fn three_vessel_articulated_chain_conserves_momentum_and_preserves_singleton() {
    let (mut coord, ids, singleton_opt) = build_chain_scenario(
        50,
        VehicleIntegrator::adaptive(1e-12, 1e-12),
        true,
        mbd_multibody::DockingPhase::Open,
        2.8,
    );
    let mut standalone = singleton_opt.unwrap();
    let chain_only = [ids[0], ids[1], ids[2]];
    let singleton_id = ids[3];
    let (p0, l_cm0, l_o0) = system_totals(&coord, &chain_only);
    let inputs = chain_inputs(true);

    let mut max_dp = 0.0_f64;
    let mut max_dl_cm = 0.0_f64;
    let mut max_dl_o = 0.0_f64;
    for _ in 0..40 {
        let rep = coord.step_all(&inputs).unwrap();
        assert!(rep.merged_vessels.is_empty());
        assert!(rep.cut_tethers.is_empty());
        standalone.step(&MultibodyVehicleInput::default()).unwrap();

        assert_eq!(
            coord.vessel(singleton_id).unwrap().output(),
            standalone.output(),
            "uncoupled singleton must remain bit-identical"
        );
        let (p, l_cm, l_o) = system_totals(&coord, &chain_only);
        max_dp = max_dp.max((p - p0).length() / p0.length().max(1.0));
        max_dl_cm = max_dl_cm.max((l_cm - l_cm0).length() / l_cm0.length().max(1.0));
        max_dl_o = max_dl_o.max((l_o - l_o0).length() / l_o0.length().max(1.0));
    }
    report_line(&format!(
        "3-vessel articulated chain (DP54 1e-12): max_dp_rel={max_dp:.3e}, max_dL_cm_rel={max_dl_cm:.3e}, max_dL_origin_rel={max_dl_o:.3e}"
    ));
    assert!(max_dp < 1e-12, "linear momentum rel drift {max_dp}");
    assert!(
        max_dl_cm < 1e-11,
        "CM angular momentum rel drift {max_dl_cm}"
    );
    assert!(
        max_dl_o < 1e-11,
        "origin angular momentum rel drift {max_dl_o}"
    );
}

fn run_chain_end_states(
    steps_per_second: u64,
    integrator: VehicleIntegrator,
    duration_num: u64,
    duration_den: u64,
) -> [VehicleState; 3] {
    let (mut coord, _, _) = build_chain_scenario(
        steps_per_second,
        integrator,
        false,
        mbd_multibody::DockingPhase::SoftContact,
        2.2,
    );
    let n_steps = steps_per_second * duration_num / duration_den;
    let inputs = chain_inputs(false);
    for _ in 0..n_steps {
        let rep = coord.step_all(&inputs).unwrap();
        assert!(rep.merged_vessels.is_empty());
    }
    [
        coord.vessel(SimVesselId(1)).unwrap().output(),
        coord.vessel(SimVesselId(2)).unwrap().output(),
        coord.vessel(SimVesselId(3)).unwrap().output(),
    ]
}

fn state_diff_max(a: &[VehicleState; 3], b: &[VehicleState; 3]) -> (f64, f64) {
    let mut pos_err = 0.0_f64;
    let mut vel_err = 0.0_f64;
    for (sa, sb) in a.iter().zip(b.iter()) {
        pos_err = pos_err.max((sa.position_ecef - sb.position_ecef).length());
        vel_err = vel_err.max((sa.velocity_ecef - sb.velocity_ecef).length());
    }
    (pos_err, vel_err)
}

#[test]
fn three_vessel_articulated_chain_matches_fine_reference_at_fourth_order() {
    let fine = run_chain_end_states(50, VehicleIntegrator::FixedRk4 { substeps: 32 }, 3, 5);
    let adapt = run_chain_end_states(50, VehicleIntegrator::adaptive(1e-12, 1e-12), 3, 5);
    let (ad_pos, ad_vel) = state_diff_max(&adapt, &fine);

    let r50 = run_chain_end_states(50, VehicleIntegrator::REAL_TIME, 3, 5);
    let r100 = run_chain_end_states(100, VehicleIntegrator::REAL_TIME, 3, 5);
    let r200 = run_chain_end_states(200, VehicleIntegrator::REAL_TIME, 3, 5);
    let r400 = run_chain_end_states(400, VehicleIntegrator::REAL_TIME, 3, 5);
    let (e50, _) = state_diff_max(&r50, &fine);
    let (e100, _) = state_diff_max(&r100, &fine);
    let (e200, _) = state_diff_max(&r200, &fine);
    let (e400, _) = state_diff_max(&r400, &fine);

    report_line(&format!(
        "3-vessel chain convergence vs 1600Hz RK4: e50={e50:.3e}, e100={e100:.3e}, e200={e200:.3e}, e400={e400:.3e}; DP54 vs 1600Hz pos={ad_pos:.3e} m, vel={ad_vel:.3e} m/s"
    ));
    assert!(
        e50 > e100 * 10.0,
        "expected ~16x 4th-order decay: {e50} -> {e100}"
    );
    assert!(
        e100 > e200 * 10.0,
        "expected ~16x 4th-order decay: {e100} -> {e200}"
    );
    assert!(
        e200 > e400 * 10.0,
        "expected ~16x 4th-order decay: {e200} -> {e400}"
    );
    assert!(
        ad_pos < 1e-11,
        "adaptive DP54 vs 1600Hz ref pos diff {ad_pos}"
    );
    assert!(
        ad_vel < 1e-11,
        "adaptive DP54 vs 1600Hz ref vel diff {ad_vel}"
    );
}

#[test]
fn rotating_planet_articulated_tether_conserves_inertial_momentum() {
    let body = super::models::orbital_zero_grav_body();
    let integ = VehicleIntegrator::adaptive(1e-11, 1e-11);
    let r = DVec3::new(6_800_000.0, 0.0, 0.0);
    // Zero ECEF linear velocity with non-zero WGS84 planet spin => active Coriolis/inertial velocity.
    let rot_a = articulated_vessel(
        "rot_a",
        body,
        VehicleState {
            position_ecef: r,
            velocity_ecef: DVec3::new(0.0, 0.0, 0.0),
            attitude: DQuat::IDENTITY,
            angular_rate_body: DVec3::new(0.02, -0.03, 0.04),
        },
        50,
        integ,
        DVec3::new(0.9, 0.2, 0.0),
        DVec3::Z,
        0.1,
        0.08,
    );
    let rot_b = articulated_vessel(
        "rot_b",
        body,
        VehicleState {
            position_ecef: r + DVec3::new(6.0, 0.3, -0.2),
            velocity_ecef: DVec3::new(-0.15, 0.25, -0.1),
            attitude: DQuat::IDENTITY,
            angular_rate_body: DVec3::new(-0.03, 0.02, -0.05),
        },
        50,
        integ,
        DVec3::new(-0.8, -0.15, 0.1),
        DVec3::Y,
        -0.1,
        -0.06,
    );
    let mut coord = MultiVesselCoordinator::new(rot_a);
    let b_id = coord.spawn_independent(rot_b);
    coord.add_link(tether_link(
        77,
        SimVesselId(1),
        ComponentId(1),
        DVec3::new(0.4, 0.1, 0.0),
        b_id,
        ComponentId(1),
        DVec3::new(-0.4, -0.1, 0.0),
        WinchTetherSpec {
            stiffness_n_m: 3_500.0,
            damping_n_s_m: 450.0,
            max_length_m: 15.0,
            breaking_tension_n: 80_000.0,
            burn_through_energy_j: None,
            auto_cut_low_tension_n: None,
            auto_cut_dwell_s: 0.0,
        },
        3.2,
        -0.25,
    ));

    let ids = [SimVesselId(1), b_id];
    let (p0, _, l_o0) = system_totals(&coord, &ids);
    let mut max_dp = 0.0_f64;
    let mut max_dl_o = 0.0_f64;
    for _ in 0..30 {
        coord.step_all(&BTreeMap::new()).unwrap();
        let (p, _, l_o) = system_totals(&coord, &ids);
        max_dp = max_dp.max((p - p0).length() / p0.length().max(1.0));
        max_dl_o = max_dl_o.max((l_o - l_o0).length() / l_o0.length().max(1.0));
    }
    report_line(&format!(
        "rotating-planet articulated tether (WGS84 omega): max_dp_rel={max_dp:.3e}, max_dL_origin_rel={max_dl_o:.3e}"
    ));
    assert!(max_dp < 1e-12, "max_dp_rel={max_dp}");
    assert!(max_dl_o < 1e-11, "max_dL_origin_rel={max_dl_o}");
}
