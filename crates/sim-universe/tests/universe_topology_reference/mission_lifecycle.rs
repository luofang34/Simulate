//! Full end-to-end mission topology lifecycle on `Universe`:
//! Orbital carried coast -> auto-demotion -> impulse stage separation ->
//! WinchTether deploy/retract with TierPolicy blocking -> Docking soft-capture
//! -> hard_mate -> Orbital carried coast -> demote -> undock_subtree, plus
//! single-vessel RailSlide + TierPolicy gating and free-space momentum check.

use glam::DVec3;
use mbd_core::{ComponentId, ComponentTree, JointSpec, Placement};
use mbd_multibody::{RailSlideSpec, RailSlideState, TetherCommand};
use sim_core::{TICKS_PER_SECOND, TickSpan, VehicleIntegrator, VehicleState};
use sim_geo::BodyEllipsoid;
use sim_orbital::VesselId;
use sim_universe::{
    MultibodyLocalPlant, Tier, TierPolicy, Universe, UniverseError, UniverseLinkSpec,
};

use super::models::{
    FAST_STEP, STAGE1_CORE_IN_CG_M, at_ms, build_dock_spec, build_leo_mission_universe,
    build_leo_pair_at, build_tether_link, make_mb_plant, make_mb_plant_with_integrator,
    orbital_ctx, pair_mom_cm, part, report_line, test_epoch,
};

#[test]
fn staging_and_tether_lifecycle_demotes_cleanly_and_blocks_policy_promotion() {
    let mut u = build_leo_mission_universe(true);
    u.promote_to_orbital(VesselId(1)).unwrap();
    u.promote_to_orbital(VesselId(2)).unwrap();
    u.advance_to(at_ms(2_000)).unwrap();
    assert_eq!(u.tier(VesselId(1)).unwrap(), Tier::Orbital);
    assert_eq!(u.tier(VesselId(2)).unwrap(), Tier::Orbital);

    let booster = u
        .separate_stage_impulse(VesselId(1), ComponentId(2), 180.0, DVec3::NEG_X)
        .unwrap();
    assert_eq!(u.tier(VesselId(1)).unwrap(), Tier::Local);
    assert_eq!(u.tier(booster).unwrap(), Tier::Local);
    assert_eq!(
        u.take_proximity_report().spawned_vessels,
        vec![(VesselId(1), booster)]
    );

    let tether_id = u
        .add_inter_vessel_link(build_tether_link(VesselId(1), VesselId(2), 3.4, -0.8))
        .unwrap();
    assert_eq!(u.tier(VesselId(2)).unwrap(), Tier::Local);
    assert!(matches!(
        u.promote_to_orbital(VesselId(1)),
        Err(UniverseError::VesselCoupled(VesselId(1)))
    ));

    let _ = u.take_tier_changes();
    u.advance_to(at_ms(5_000)).unwrap();
    assert_eq!(u.tier(VesselId(1)).unwrap(), Tier::Local);
    assert_eq!(u.tier(VesselId(2)).unwrap(), Tier::Local);
    assert_eq!(u.tier(booster).unwrap(), Tier::Orbital);
    assert!(
        u.take_tier_changes()
            .iter()
            .all(|c| c.vessel != VesselId(1) && c.vessel != VesselId(2))
    );

    u.set_tether_command(
        tether_id,
        TetherCommand {
            payout_rate_m_s: -1.2,
            ..Default::default()
        },
    )
    .unwrap();
    u.advance_to(at_ms(5_500)).unwrap();
    u.cut_tether(tether_id).unwrap();
    let p1 = u.vehicle_state(VesselId(1)).unwrap().position_ecef;
    let p2 = u.vehicle_state(VesselId(2)).unwrap().position_ecef;
    assert!((p2 - p1).length() < 4.5);
    u.promote_to_orbital(VesselId(1)).unwrap();
    assert_eq!(u.tier(VesselId(1)).unwrap(), Tier::Orbital);
}

#[test]
fn docking_hard_mate_orbital_coast_and_undock_lifecycle() {
    let mut u = build_leo_pair_at(true, STAGE1_CORE_IN_CG_M + DVec3::new(1.25, 0.0, 0.0));
    let lid = u
        .dock_soft_capture(build_dock_spec(VesselId(1), VesselId(2), 0.25))
        .unwrap();
    u.advance_to(at_ms(200)).unwrap();
    if u.inter_vessel_link(lid).is_some() {
        assert_eq!(u.tier(VesselId(1)).unwrap(), Tier::Local);
        assert_eq!(u.tier(VesselId(2)).unwrap(), Tier::Local);
    }
    u.advance_to(at_ms(3_000)).unwrap();
    let rep = u.take_proximity_report();
    assert_eq!(rep.merged_vessels, vec![(VesselId(1), VesselId(2))]);
    assert!(u.inter_vessel_link(lid).is_none());
    let mated_omega = u
        .vehicle_state(VesselId(1))
        .unwrap()
        .angular_rate_body
        .length();
    report_line(&format!(
        "docked LEO hard-mate |omega_body|={mated_omega:.4} rad/s"
    ));
    assert_eq!(u.tier(VesselId(1)).unwrap(), Tier::Orbital);

    let bus_before = u.bus_telemetry(VesselId(1)).unwrap();
    u.advance_to(at_ms(6_000)).unwrap();
    let bus_after = u.bus_telemetry(VesselId(1)).unwrap();
    assert!(bus_after.oscillator_tau_minus_tdb_s < bus_before.oscillator_tau_minus_tdb_s);
    assert!(u.carried::<MultibodyLocalPlant>(VesselId(1)).is_some());

    u.demote_to_local(VesselId(1)).unwrap();
    assert_eq!(u.tier(VesselId(1)).unwrap(), Tier::Local);
    let undocked = u
        .undock_subtree(VesselId(1), ComponentId(3), 45.0, DVec3::X)
        .unwrap();
    assert_eq!(u.tier(undocked).unwrap(), Tier::Local);
}

#[test]
fn free_space_full_lifecycle_conserves_linear_and_cm_angular_momentum() {
    let ep = test_epoch();
    let zg = BodyEllipsoid {
        mu_m3_s2: 0.0,
        omega_rad_s: 0.0,
        semi_major_m: 1.0,
        flattening: 0.0,
        ..BodyEllipsoid::WGS84
    };
    let s1 = VehicleState {
        position_ecef: DVec3::new(0.0, 0.0, 0.0),
        velocity_ecef: DVec3::new(0.0, 0.0, 0.0),
        attitude: glam::DQuat::IDENTITY,
        angular_rate_body: DVec3::new(0.002, -0.003, 0.004),
    };
    // Match `target` root relative to `core` root (`(+0.70, -0.028, 0)`) + `1.25 m`,
    // with `p_tot == 0` so soft-capture latch distance does not shift `R_cm x P_tot`.
    let s2 = VehicleState {
        position_ecef: DVec3::new(0.70 + 1.25, -0.028, 0.0),
        velocity_ecef: DVec3::new(0.0, 0.0, 0.0),
        attitude: glam::DQuat::IDENTITY,
        angular_rate_body: DVec3::new(-0.003, 0.002, -0.002),
    };
    let tree1 = ComponentTree::new([
        part("core", Placement::ROOT, JointSpec::FIXED, 900.0, 350.0),
        part(
            "booster",
            Placement::child_of(ComponentId(0), DVec3::new(-2.5, 0.1, 0.0)),
            JointSpec::FIXED,
            350.0,
            120.0,
        ),
    ])
    .unwrap();
    let tree2 = ComponentTree::new([part(
        "target",
        Placement::ROOT,
        JointSpec::FIXED,
        750.0,
        260.0,
    )])
    .unwrap();
    let integ = VehicleIntegrator::adaptive(1e-11, 1e-11);
    let (p1, f1) = make_mb_plant_with_integrator(zg, tree1, s1, FAST_STEP, &ep, integ);
    let (p2, f2) = make_mb_plant_with_integrator(zg, tree2, s2, FAST_STEP, &ep, integ);
    let mut u = Universe::new(ep, orbital_ctx());
    u.insert_local(VesselId(1), p1, f1).unwrap();
    u.insert_local(VesselId(2), p2, f2).unwrap();

    let (p0, l0, m0) = pair_mom_cm(&u, &[VesselId(1), VesselId(2)]);
    let booster = u
        .separate_stage_impulse(VesselId(1), ComponentId(1), 220.0, DVec3::NEG_X)
        .unwrap();
    let tether = u
        .add_inter_vessel_link(build_tether_link(VesselId(1), VesselId(2), 1.05, -0.4))
        .unwrap();
    u.advance_to(at_ms(800)).unwrap();
    u.cut_tether(tether).unwrap();
    u.dock_soft_capture(build_dock_spec(VesselId(1), VesselId(2), 0.35))
        .unwrap();
    u.advance_to(at_ms(3_000)).unwrap();
    assert!(
        u.tier(VesselId(2)).is_err(),
        "target should have hard-mated"
    );
    let undocked = u
        .undock_subtree(VesselId(1), ComponentId(1), 80.0, DVec3::X)
        .unwrap();

    let (pf, lf, mf) = pair_mom_cm(&u, &[VesselId(1), booster, undocked]);
    let dp_rel = (pf - p0).length() / p0.length().max(1.0);
    let dl_rel = (lf - l0).length() / l0.length().max(1.0);
    report_line(&format!(
        "free-space universe lifecycle (stage+tether+dock+undock): dM={:.3e}, dp_rel={dp_rel:.3e}, dL_cm_rel={dl_rel:.3e}",
        (mf - m0).abs()
    ));
    assert!((mf - m0).abs() < 1e-11);
    assert!(dp_rel < 1e-12, "dp_rel={dp_rel}");
    assert!(dl_rel < 1e-11, "dL_cm_rel={dl_rel}");
}

#[test]
fn active_single_vessel_rail_slide_keeps_vessel_in_local_until_release() {
    let ep = test_epoch();
    let body = BodyEllipsoid::WGS84;
    let tree = ComponentTree::new([
        part(
            "airframe",
            Placement::ROOT,
            JointSpec::FIXED,
            2_000.0,
            1_000.0,
        ),
        part(
            "seat",
            Placement::child_of(ComponentId(0), DVec3::ZERO),
            JointSpec::prismatic(DVec3::Z, -0.1, 2.0),
            100.0,
            10.0,
        ),
    ])
    .unwrap();
    let (p, f) = make_mb_plant(
        body,
        tree,
        super::models::leo_state(DVec3::ZERO),
        FAST_STEP,
        &ep,
    );
    let mut u = Universe::new(ep, orbital_ctx());
    let mut policy = TierPolicy::for_bodies(u.bodies());
    policy.check_period = TickSpan::from_ticks(TICKS_PER_SECOND / 10);
    u = u.with_tier_policy(policy);
    u.insert_local(VesselId(1), p, f).unwrap();

    u.add_inter_vessel_link(UniverseLinkSpec::RailSlide {
        vessel: VesselId(1),
        spec: RailSlideSpec {
            component: ComponentId(1),
            rail_length_m: 1.5,
            catapult_force_n: 1_000.0,
            guide_friction_n: 0.0,
        },
        state: RailSlideState {
            fired: true,
            ready_to_release: false,
        },
    })
    .unwrap();

    u.advance_to(at_ms(200)).unwrap();
    assert_eq!(u.tier(VesselId(1)).unwrap(), Tier::Local);
    assert!(matches!(
        u.promote_to_orbital(VesselId(1)),
        Err(UniverseError::VesselCoupled(VesselId(1)))
    ));

    u.advance_to(at_ms(1_000)).unwrap();
    let rep = u.take_proximity_report();
    assert_eq!(rep.spawned_vessels.len(), 1);
    let child = rep.spawned_vessels[0].1;
    assert_eq!(u.tier(VesselId(1)).unwrap(), Tier::Orbital);
    assert_eq!(u.tier(child).unwrap(), Tier::Orbital);
}
