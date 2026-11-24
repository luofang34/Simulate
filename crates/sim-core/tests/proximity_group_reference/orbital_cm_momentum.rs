//! Orbital-radius (`r ~ 7e6 m`) spinning off-CG soft capture audit under
//! adaptive DormandPrince54: verifies inertial linear momentum (`dp < 1e-12`),
//! origin angular momentum (`dL_origin < 1e-12`), and CM-relative angular
//! momentum (`L - R_cm x P`, matching the IEEE-754 `ULP(7e6 m)` floor).

use std::collections::BTreeMap;

use glam::{DQuat, DVec3};
use mbd_core::ComponentId;
use mbd_multibody::DockingSpec;
use sim_core::{MultiVesselCoordinator, SimVesselId, VehicleIntegrator, VehicleState};

use super::models::{
    default_docking_spec, docking_link, orbital_zero_grav_body, report_line, rigid_off_cg_vessel,
    system_totals, vessel_momentum_world,
};

fn reduced_mass_cm_momentum(
    coord: &MultiVesselCoordinator,
    a: SimVesselId,
    b: SimVesselId,
) -> DVec3 {
    let (pa, la, ra, ma) = vessel_momentum_world(coord.vessel(a).unwrap());
    let (pb, lb, rb, mb) = vessel_momentum_world(coord.vessel(b).unwrap());
    let va = pa / ma;
    let vb = pb / mb;
    let mu = (ma * mb) / (ma + mb);
    la + lb + mu * (ra - rb).cross(va - vb)
}

fn setup_orbital_pair(v_orbit: DVec3) -> (MultiVesselCoordinator, SimVesselId) {
    let body = orbital_zero_grav_body();
    let r_orbit = DVec3::new(7_000_000.0, 140_000.0, -90_000.0);
    let integ = VehicleIntegrator::adaptive(1e-12, 1e-12);
    let chaser = rigid_off_cg_vessel(
        "chaser",
        body,
        VehicleState {
            position_ecef: r_orbit,
            velocity_ecef: v_orbit + DVec3::new(0.22, -0.04, 0.03),
            attitude: DQuat::from_euler(glam::EulerRot::ZYX, 0.08, -0.03, 0.05),
            angular_rate_body: DVec3::new(0.03, -0.05, 0.06),
        },
        50,
        integ,
        1_050.0,
        180.0,
        DVec3::new(1.1, 0.25, -0.15),
    );
    let target = rigid_off_cg_vessel(
        "target",
        body,
        VehicleState {
            position_ecef: r_orbit + DVec3::new(1.5, 0.2, -0.12),
            velocity_ecef: v_orbit - DVec3::new(0.22, -0.04, 0.03),
            attitude: DQuat::from_euler(glam::EulerRot::ZYX, -0.05, 0.06, -0.04),
            angular_rate_body: DVec3::new(-0.04, 0.03, -0.05),
        },
        50,
        integ,
        820.0,
        140.0,
        DVec3::new(-0.9, -0.2, 0.12),
    );
    let mut coord = MultiVesselCoordinator::new(chaser);
    let target_id = coord.spawn_independent(target);
    let mut spec = default_docking_spec();
    spec.damped_dwell_s = DockingSpec::DEFAULT_DAMPED_DWELL_S;
    coord.add_link(docking_link(
        9,
        SimVesselId(1),
        ComponentId(1),
        DVec3::new(0.35, -0.1, 0.08),
        DVec3::X,
        target_id,
        ComponentId(1),
        DVec3::new(-0.45, -0.15, 0.06),
        spec,
        0.4,
    ));
    (coord, target_id)
}

fn run_orbital_capture(v_orbit: DVec3) -> (Option<u32>, f64, f64, f64, f64, f64, f64) {
    let (mut coord, target_id) = setup_orbital_pair(v_orbit);
    let pair = [SimVesselId(1), target_id];
    let (p0, l_cm0, l_origin0) = system_totals(&coord, &pair);
    let l_cm_reduced0 = reduced_mass_cm_momentum(&coord, SimVesselId(1), target_id);
    let mut max_dp_rel = 0.0_f64;
    let mut max_dl_cm_naive = 0.0_f64;
    let mut max_dl_cm_reduced = 0.0_f64;
    let mut max_dl_origin_rel = 0.0_f64;
    let mut mated_step = None;

    for step in 0..160_u32 {
        let rep = coord.step_all(&BTreeMap::new()).unwrap();
        let live: Vec<SimVesselId> = if coord.vessel(target_id).is_some() {
            vec![SimVesselId(1), target_id]
        } else {
            vec![SimVesselId(1)]
        };
        let (p, l_cm, l_origin) = system_totals(&coord, &live);
        max_dp_rel = max_dp_rel.max((p - p0).length() / p0.length().max(1.0));
        max_dl_origin_rel =
            max_dl_origin_rel.max((l_origin - l_origin0).length() / l_origin0.length().max(1.0));
        if rep.merged_vessels.is_empty() && coord.vessel(target_id).is_some() {
            max_dl_cm_naive =
                max_dl_cm_naive.max((l_cm - l_cm0).length() / l_cm0.length().max(1.0));
            let l_cm_red = reduced_mass_cm_momentum(&coord, SimVesselId(1), target_id);
            max_dl_cm_reduced = max_dl_cm_reduced
                .max((l_cm_red - l_cm_reduced0).length() / l_cm_reduced0.length().max(1.0));
        }
        if !rep.merged_vessels.is_empty() {
            mated_step = Some(step);
        }
    }
    (
        mated_step,
        l_origin0.length(),
        l_cm_reduced0.length(),
        max_dp_rel,
        max_dl_cm_reduced,
        max_dl_cm_naive,
        max_dl_origin_rel,
    )
}

#[test]
fn orbital_radius_spinning_soft_capture_conserves_cm_relative_angular_momentum() {
    let (mated0, lo0, lcm0, dp0, dl_red0, dl_naive0, dlo0) = run_orbital_capture(DVec3::ZERO);
    report_line(&format!(
        "orbital r~7e6m v_bulk=0 (DP54 1e-12): mated={mated0:?}, |L_origin|={lo0:.3e}, |L_cm|={lcm0:.1}, dp={dp0:.3e}, dL_cm_reduced={dl_red0:.3e}, dL_cm_naive={dl_naive0:.3e}, dL_origin={dlo0:.3e}"
    ));
    assert!(mated0.is_some());
    assert!(dp0 < 1e-12 && dl_red0 < 1e-8 && dl_naive0 < 1e-8 && dlo0 < 1e-12);

    let (mated1, lo1, lcm1, dp1, dl_red1, dl_naive1, dlo1) =
        run_orbital_capture(DVec3::new(120.0, 7_520.0, 810.0));
    report_line(&format!(
        "orbital r~7e6m v~7.5km/s (DP54 1e-12): mated={mated1:?}, |L_origin|={lo1:.3e}, |L_cm|={lcm1:.1}, dp={dp1:.3e}, dL_cm_reduced={dl_red1:.3e}, dL_cm_naive={dl_naive1:.3e}, dL_origin={dlo1:.3e}"
    ));
    assert!(mated1.is_some());
    assert!(dp1 < 1e-12, "dp={dp1}");
    assert!(dlo1 < 1e-12, "dL_origin={dlo1}");
    assert!(dl_red1 < 1e-8, "dL_cm_reduced={dl_red1}");
}
