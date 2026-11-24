//! Stiff tether reel-in / slack-to-taut rebound under Rosenbrock23,
//! DormandPrince54, and fixed RK4 step-halving, plus impulse-free tether cut.

use std::collections::BTreeMap;

use glam::{DQuat, DVec3};
use mbd_core::ComponentId;
use mbd_multibody::{TetherCommand, TetherCutReason, Tolerances, WinchTetherSpec};
use sim_core::{
    Block, InterVesselLink, MultiVesselCoordinator, SimVesselId, VehicleIntegrator, VehicleState,
};

use super::models::{
    articulated_vessel, free_space_body, report_line, rigid_off_cg_vessel, system_totals,
    tether_link,
};

fn stiff_tether_spec(stiffness_n_m: f64, damping_n_s_m: f64) -> WinchTetherSpec {
    WinchTetherSpec {
        stiffness_n_m,
        damping_n_s_m,
        max_length_m: 25.0,
        breaking_tension_n: 500_000.0,
        burn_through_energy_j: None,
        auto_cut_low_tension_n: None,
        auto_cut_dwell_s: 0.0,
    }
}

fn stiff_rebound_pair(
    steps_per_second: u64,
    integrator: VehicleIntegrator,
    initial_length_m: f64,
    payout_rate_m_s: f64,
    damping_n_s_m: f64,
) -> (MultiVesselCoordinator, SimVesselId) {
    let body = free_space_body();
    let a = rigid_off_cg_vessel(
        "carrier",
        body,
        VehicleState {
            position_ecef: DVec3::new(0.0, 0.0, 0.0),
            velocity_ecef: DVec3::new(1.2, -0.5, 0.3),
            attitude: DQuat::IDENTITY,
            angular_rate_body: DVec3::new(0.02, -0.03, 0.04),
        },
        steps_per_second,
        integrator,
        900.0,
        150.0,
        DVec3::new(0.8, 0.1, -0.05),
    );
    let b = articulated_vessel(
        "payload",
        body,
        VehicleState {
            position_ecef: DVec3::new(10.0, 0.3, -0.2),
            velocity_ecef: DVec3::new(2.0, -0.4, 0.25),
            attitude: DQuat::IDENTITY,
            angular_rate_body: DVec3::new(-0.03, 0.02, -0.05),
        },
        steps_per_second,
        integrator,
        DVec3::new(-0.6, -0.15, 0.1),
        DVec3::Z,
        0.1,
        0.05,
    );
    let mut coord = MultiVesselCoordinator::new(a);
    let b_id = coord.spawn_independent(b);
    coord.add_link(tether_link(
        10,
        SimVesselId(1),
        ComponentId(1),
        DVec3::new(0.2, 0.05, 0.0),
        b_id,
        ComponentId(1),
        DVec3::new(-0.3, -0.1, 0.05),
        stiff_tether_spec(60_000.0, damping_n_s_m),
        initial_length_m,
        payout_rate_m_s,
    ));
    (coord, b_id)
}

fn run_taut_case(sps: u64, integ: VehicleIntegrator) -> (f64, f64, f64, f64) {
    let (mut coord, b_id) = stiff_rebound_pair(sps, integ, 8.2, -0.4, 1_500.0);
    let ids = [SimVesselId(1), b_id];
    let (p0, l0, _) = system_totals(&coord, &ids);
    let mut max_dp = 0.0_f64;
    let mut max_dl = 0.0_f64;
    for _ in 0..(sps / 5) {
        coord.step_all(&BTreeMap::new()).unwrap();
        let (p, l, _) = system_totals(&coord, &ids);
        max_dp = max_dp.max((p - p0).length() / p0.length().max(1.0));
        max_dl = max_dl.max((l - l0).length() / l0.length().max(1.0));
    }
    let a_out = coord.vessel(SimVesselId(1)).unwrap().output();
    let b_out = coord.vessel(b_id).unwrap().output();
    let gap = (b_out.position_ecef - a_out.position_ecef).length();
    let rel_v = (b_out.velocity_ecef - a_out.velocity_ecef).length();
    (gap, rel_v, max_dp, max_dl)
}

#[test]
fn stiff_tether_reel_in_converges_under_rosenbrock_dp54_and_rk4() {
    let (g_fine, v_fine, dp_fine, dl_fine) =
        run_taut_case(50, VehicleIntegrator::FixedRk4 { substeps: 32 });
    let (g_50, _, _, _) = run_taut_case(50, VehicleIntegrator::REAL_TIME);
    let (g_100, _, _, _) = run_taut_case(100, VehicleIntegrator::REAL_TIME);
    let (g_200, _, _, _) = run_taut_case(200, VehicleIntegrator::REAL_TIME);
    let (g_dp, v_dp, dp_dp, dl_dp) = run_taut_case(50, VehicleIntegrator::adaptive(1e-11, 1e-11));
    let (g_ros, v_ros, dp_ros, dl_ros) = run_taut_case(
        50,
        VehicleIntegrator::Stiff(Tolerances::uniform(1e-9, 1e-9)),
    );
    let (g_ros11, _, dp_ros11, dl_ros11) = run_taut_case(
        50,
        VehicleIntegrator::Stiff(Tolerances::uniform(1e-11, 1e-11)),
    );

    let e50 = (g_50 - g_fine).abs();
    let e100 = (g_100 - g_fine).abs();
    let e200 = (g_200 - g_fine).abs();
    report_line(&format!(
        "stiff taut tether: fine={g_fine:.12} m; rk4 err(50,100,200)=({e50:.3e},{e100:.3e},{e200:.3e}); dp54 diff={:.3e} m (dp={dp_dp:.3e}, dL={dl_dp:.3e}); ros23(1e-9) diff={:.3e} m (dp={dp_ros:.3e}, dL={dl_ros:.3e}); ros23(1e-11) diff={:.3e} m (dp={dp_ros11:.3e}, dL={dl_ros11:.3e})",
        (g_dp - g_fine).abs(),
        (g_ros - g_fine).abs(),
        (g_ros11 - g_fine).abs(),
    ));
    assert!(dp_fine < 1e-12 && dl_fine < 1e-11);
    assert!(dp_dp < 1e-12 && dl_dp < 1e-11);
    assert!(dp_ros < 1e-12 && dl_ros < 1e-7);
    assert!(dp_ros11 < 1e-12 && dl_ros11 < 1e-9);
    assert!(e50 > e100 * 10.0 && e100 > e200 * 10.0);
    assert!((g_dp - g_fine).abs() < 1e-10);
    assert!((v_dp - v_fine).abs() < 1e-9);
    assert!((g_ros - g_fine).abs() < 1e-6);
    assert!((g_ros11 - g_fine).abs() < 1e-8);
    assert!((v_ros - v_fine).abs() < 1e-5);
}

#[test]
fn stiff_tether_rebound_crosses_slack_taut_and_conserves_momentum() {
    for (label, integ, dl_tol) in [
        ("DP54(1e-9)", VehicleIntegrator::adaptive(1e-9, 1e-9), 1e-10),
        (
            "Rosenbrock23(1e-8)",
            VehicleIntegrator::Stiff(Tolerances::uniform(1e-8, 1e-8)),
            1e-6,
        ),
    ] {
        // Initial anchor separation ~ 8.10 m, deployed = 8.20 m -> starts slack, separates to taut rebound.
        let (mut coord, b_id) = stiff_rebound_pair(50, integ, 8.20, 0.0, 200.0);
        let ids = [SimVesselId(1), b_id];
        let (p0, l0, _) = system_totals(&coord, &ids);
        let mut max_dp = 0.0_f64;
        let mut max_dl = 0.0_f64;
        let mut saw_inward = false;
        for _ in 0..40 {
            coord.step_all(&BTreeMap::new()).unwrap();
            let (p, l, _) = system_totals(&coord, &ids);
            max_dp = max_dp.max((p - p0).length() / p0.length().max(1.0));
            max_dl = max_dl.max((l - l0).length() / l0.length().max(1.0));
            let rel_vx = (coord.vessel(b_id).unwrap().output().velocity_ecef
                - coord.vessel(SimVesselId(1)).unwrap().output().velocity_ecef)
                .x;
            if rel_vx < 0.0 {
                saw_inward = true;
            }
        }
        let closing = (coord.vessel(b_id).unwrap().output().velocity_ecef
            - coord.vessel(SimVesselId(1)).unwrap().output().velocity_ecef)
            .x;
        report_line(&format!(
            "slack-to-taut rebound [{label}]: saw_inward={saw_inward}, final_rel_vx={closing:.6} m/s, max_dp={max_dp:.3e}, max_dL={max_dl:.3e}"
        ));
        assert!(
            saw_inward && closing < 0.0,
            "{label} should rebound inward: vx={closing}"
        );
        assert!(max_dp < 1e-12, "{label} dp={max_dp}");
        assert!(max_dl < dl_tol, "{label} dL={max_dl}");
    }
}

#[test]
fn group_step_error_rolls_back_atomically_without_dropping_vessels() {
    let (mut coord, b_id) = stiff_rebound_pair(50, VehicleIntegrator::REAL_TIME, 8.0, 0.0, 0.0);
    // Extreme stiffness causes NaN divergence within a few explicit RK4 steps.
    let InterVesselLink::WinchTether { spec, .. } = &mut coord.links_mut()[0] else {
        panic!("expected tether");
    };
    spec.stiffness_n_m = 1e14;
    spec.breaking_tension_n = 1e30;

    let mut before_a = coord.vessel(SimVesselId(1)).unwrap().output();
    let mut before_b = coord.vessel(b_id).unwrap().output();
    let mut captured_err = None;
    for _ in 0..10 {
        match coord.step_all(&BTreeMap::new()) {
            Ok(_) => {
                before_a = coord.vessel(SimVesselId(1)).unwrap().output();
                before_b = coord.vessel(b_id).unwrap().output();
            }
            Err(err) => {
                captured_err = Some(err);
                break;
            }
        }
    }
    let err = captured_err.expect("extreme stiffness must produce a step error");
    report_line(&format!("group rollback atomicity error: {err}"));

    assert_eq!(coord.links().len(), 1);
    assert_eq!(coord.vessel(SimVesselId(1)).unwrap().output(), before_a);
    assert_eq!(coord.vessel(b_id).unwrap().output(), before_b);
}

#[test]
fn tether_cut_under_high_tension_is_impulse_free() {
    let (mut coord, b_id) = stiff_rebound_pair(
        50,
        VehicleIntegrator::adaptive(1e-11, 1e-11),
        8.0,
        -0.3,
        900.0,
    );
    let ids = [SimVesselId(1), b_id];
    let (p0, l0, _) = system_totals(&coord, &ids);

    // Step 10 ticks under high tension (~60 kN range), then cut cleanly.
    for _ in 0..10 {
        coord.step_all(&BTreeMap::new()).unwrap();
    }
    let pre_a = coord.vessel(SimVesselId(1)).unwrap().output();
    let pre_b = coord.vessel(b_id).unwrap().output();

    let InterVesselLink::WinchTether { command, .. } = &mut coord.links_mut()[0] else {
        panic!("expected tether link");
    };
    *command = TetherCommand {
        payout_rate_m_s: -0.3,
        heat_rate_w: 0.0,
        cut: true,
    };

    let rep = coord.step_all(&BTreeMap::new()).unwrap();
    assert_eq!(rep.cut_tethers, vec![(10, TetherCutReason::Commanded)]);
    assert!(coord.links().is_empty());

    let post_a = coord.vessel(SimVesselId(1)).unwrap().output();
    let post_b = coord.vessel(b_id).unwrap().output();
    let dt = 0.02;
    assert!((post_a.velocity_ecef - pre_a.velocity_ecef).length() < 2.0);
    assert!((post_b.velocity_ecef - pre_b.velocity_ecef).length() < 2.0);
    assert!(
        (post_a.position_ecef - (pre_a.position_ecef + pre_a.velocity_ecef * dt)).length() < 0.05
    );
    assert!(
        (post_b.position_ecef - (pre_b.position_ecef + pre_b.velocity_ecef * dt)).length() < 0.05
    );

    // Coast 20 more steps with no link and verify momentum stays conserved to round-off.
    let mut coast_a = post_a.velocity_ecef;
    for _ in 0..20 {
        coord.step_all(&BTreeMap::new()).unwrap();
    }
    let after_a = coord.vessel(SimVesselId(1)).unwrap().output();
    // Free-coasting rigid body CG velocity stays constant to round-off.
    assert!((after_a.velocity_ecef - coast_a).length() < 1e-12);
    coast_a = after_a.velocity_ecef;
    let _ = coast_a;

    let (pf, lf, _) = system_totals(&coord, &ids);
    let dp_rel = (pf - p0).length() / p0.length().max(1.0);
    let dl_rel = (lf - l0).length() / l0.length().max(1.0);
    report_line(&format!(
        "tether high-tension cut: dp_rel={dp_rel:.3e}, dl_rel={dl_rel:.3e}"
    ));
    assert!(dp_rel < 1e-12, "dp_rel={dp_rel}");
    assert!(dl_rel < 1e-11, "dl_rel={dl_rel}");
}
