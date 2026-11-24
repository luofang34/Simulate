//! Tether command/cut, time-slicing, and mid-operation checkpoint replay tests
//! on [`Universe`].

use glam::{DQuat, DVec3};
use mbd_core::{ComponentId, ComponentTree, JointSpec, Placement};
use mbd_multibody::{DockingPhase, DockingState, TetherCommand, TetherCutReason, WinchTetherState};
use sim_core::{MultibodySubsystems, SimInstant, VehicleState};
use sim_geo::BodyEllipsoid;
use sim_orbital::VesselId;

use super::proximity::{
    FAST_STEP, docking_pair_universe, docking_spec_default, mb_plant_with_body, part, steps,
    system_mom, tether_pair_universe, tether_spec_default, two_stage_universe,
};
use super::{epoch, solar_system_context};
use crate::proximity::{DockingLinkSpec, UniverseLinkSpec};
use crate::{Tier, Universe, UniverseError};

#[test]
fn tether_slicing_and_command_cut_checkpoint_replay_are_bit_identical() {
    let tether_spec = || UniverseLinkSpec::WinchTether {
        carrier: VesselId(1),
        anchor_carrier: ComponentId(0),
        offset_carrier_m: DVec3::new(0.5, 0.0, 0.0),
        payload: VesselId(2),
        anchor_payload: ComponentId(0),
        offset_payload_m: DVec3::new(-0.5, 0.1, 0.0),
        spec: tether_spec_default(),
        state: WinchTetherState::intact_at(9.5),
        command: TetherCommand {
            payout_rate_m_s: -0.5,
            heat_rate_w: 0.0,
            cut: false,
        },
    };
    let mut u_mono = tether_pair_universe();
    let lid = u_mono.add_inter_vessel_link(tether_spec()).unwrap();
    let mut u_sliced = tether_pair_universe();
    u_sliced.add_inter_vessel_link(tether_spec()).unwrap();

    let t_end = SimInstant::START.advanced(steps(20));
    u_mono.advance_to(t_end).unwrap();
    for k in 1..=20 {
        u_sliced
            .advance_to(SimInstant::START.advanced(steps(k)))
            .unwrap();
    }
    assert_eq!(
        u_mono.vehicle_state(VesselId(1)).unwrap(),
        u_sliced.vehicle_state(VesselId(1)).unwrap()
    );
    assert_eq!(
        u_mono.vehicle_state(VesselId(2)).unwrap(),
        u_sliced.vehicle_state(VesselId(2)).unwrap()
    );

    let cp_mid = u_mono.checkpoint();
    let reel_cmd = TetherCommand {
        payout_rate_m_s: -0.2,
        ..Default::default()
    };
    u_mono.set_tether_command(lid, reel_cmd).unwrap();
    u_mono.advance_to(t_end.advanced(steps(5))).unwrap();
    let after_cp = u_mono.vehicle_state(VesselId(2)).unwrap();

    u_mono.cut_tether(lid).unwrap();
    assert!(!u_mono.is_vessel_coupled(VesselId(1)));
    assert_eq!(
        u_mono.take_proximity_report().cut_tethers,
        vec![(lid, TetherCutReason::Commanded)]
    );

    u_mono.restore(&cp_mid).unwrap();
    assert!(u_mono.inter_vessel_link(lid).is_some());
    u_mono.set_tether_command(lid, reel_cmd).unwrap();
    u_mono.advance_to(t_end.advanced(steps(5))).unwrap();
    assert_eq!(u_mono.vehicle_state(VesselId(2)).unwrap(), after_cp);
}

#[test]
fn mid_docking_and_staging_checkpoints_restore_and_replay_bit_identically() {
    let mut u = docking_pair_universe(false);
    let lid = u
        .dock_soft_capture(DockingLinkSpec {
            chaser: VesselId(1),
            port_a: ComponentId(0),
            offset_a_local_m: DVec3::ZERO,
            axis_a_local: DVec3::X,
            target: VesselId(2),
            port_b: ComponentId(0),
            offset_b_local_m: DVec3::ZERO,
            mate_placement: Placement::child_of(ComponentId(0), DVec3::new(0.2, 0.0, 0.0)),
            spec: docking_spec_default(),
            state: DockingState {
                phase: DockingPhase::Open,
                standoff_m: 0.25,
                dwell_s: 0.0,
            },
        })
        .unwrap();
    u.advance_to(SimInstant::START.advanced(steps(3))).unwrap();
    let cp_mid = u.checkpoint();
    u.advance_to(SimInstant::START.advanced(steps(20))).unwrap();
    let mated_state = u.vehicle_state(VesselId(1)).unwrap();
    assert!(u.inter_vessel_link(lid).is_none());

    u.restore(&cp_mid).unwrap();
    assert!(u.inter_vessel_link(lid).is_some());
    assert!(u.vehicle_state(VesselId(2)).is_ok());
    u.advance_to(SimInstant::START.advanced(steps(20))).unwrap();
    assert_eq!(u.vehicle_state(VesselId(1)).unwrap(), mated_state);

    let mut us = two_stage_universe();
    let cp_pre_stage = us.checkpoint();
    let child = us
        .separate_stage_impulse(VesselId(1), ComponentId(1), 500.0, DVec3::NEG_X)
        .unwrap();
    us.advance_to(SimInstant::START.advanced(steps(2))).unwrap();
    let cp_post_stage = us.checkpoint();

    us.restore(&cp_pre_stage).unwrap();
    assert!(matches!(
        us.tier(child),
        Err(UniverseError::UnknownVessel(_))
    ));
    us.restore(&cp_post_stage).unwrap();
    assert_eq!(us.tier(child).unwrap(), Tier::Local);
}

fn free_space_pair_universe() -> Universe {
    let ep = epoch();
    let zg = BodyEllipsoid {
        mu_m3_s2: 0.0,
        omega_rad_s: 0.0,
        ..BodyEllipsoid::WGS84
    };
    let s1 = VehicleState {
        position_ecef: DVec3::new(10.0, 0.0, 0.0),
        velocity_ecef: DVec3::new(0.0, 12.0, 0.0),
        attitude: DQuat::IDENTITY,
        angular_rate_body: DVec3::new(0.0, 0.0, 0.08),
    };
    let s2 = VehicleState {
        position_ecef: DVec3::new(20.0, 0.0, 0.0),
        velocity_ecef: DVec3::new(0.0, 12.0, 0.0),
        attitude: DQuat::IDENTITY,
        angular_rate_body: DVec3::new(0.04, 0.0, -0.05),
    };
    let t1 = ComponentTree::new([
        part("core", Placement::ROOT, JointSpec::FIXED, 900.0, 350.0),
        part(
            "stage",
            Placement::child_of(ComponentId(0), DVec3::new(-2.0, 0.2, 0.0)),
            JointSpec::FIXED,
            300.0,
            100.0,
        ),
    ])
    .unwrap();
    let t2 = ComponentTree::new([part(
        "payload",
        Placement::ROOT,
        JointSpec::FIXED,
        700.0,
        250.0,
    )])
    .unwrap();
    let (p1, f1) = mb_plant_with_body(zg, t1, MultibodySubsystems::default(), s1, FAST_STEP, &ep);
    let (p2, f2) = mb_plant_with_body(zg, t2, MultibodySubsystems::default(), s2, FAST_STEP, &ep);
    let mut u = Universe::new(ep, solar_system_context());
    u.insert_local(VesselId(1), p1, f1).unwrap();
    u.insert_local(VesselId(2), p2, f2).unwrap();
    u
}

#[test]
fn free_space_staging_tether_and_hard_mate_conserve_momentum_to_round_off() {
    let mut u = free_space_pair_universe();

    let cm_ref = DVec3::new(14.0, 0.0, 0.0);
    let (p1_0, l1_0, _) = system_mom(&u, VesselId(1), cm_ref);
    let (p2_0, l2_0, _) = system_mom(&u, VesselId(2), cm_ref);
    let (p_tot0, l_tot0) = (p1_0 + p2_0, l1_0 + l2_0);

    let child = u
        .separate_stage_impulse(VesselId(1), ComponentId(1), 400.0, DVec3::NEG_X)
        .unwrap();
    u.add_inter_vessel_link(UniverseLinkSpec::WinchTether {
        carrier: VesselId(1),
        anchor_carrier: ComponentId(0),
        offset_carrier_m: DVec3::new(0.6, 0.15, -0.1),
        payload: VesselId(2),
        anchor_payload: ComponentId(0),
        offset_payload_m: DVec3::new(-0.5, -0.1, 0.2),
        spec: tether_spec_default(),
        state: WinchTetherState::intact_at(9.0),
        command: TetherCommand {
            payout_rate_m_s: -0.4,
            ..Default::default()
        },
    })
    .unwrap();
    u.advance_to(SimInstant::START.advanced(steps(10))).unwrap();
    u.hard_mate(
        VesselId(1),
        ComponentId(0),
        VesselId(2),
        ComponentId(0),
        Placement::child_of(ComponentId(0), DVec3::new(2.0, 0.0, 0.0)),
    )
    .unwrap();

    let (p1_f, l1_f, _) = system_mom(&u, VesselId(1), cm_ref);
    let (pc_f, lc_f, _) = system_mom(&u, child, cm_ref);
    assert!(((p1_f + pc_f) - p_tot0).length() / p_tot0.length().max(1.0) < 1e-11);
    assert!(((l1_f + lc_f) - l_tot0).length() / l_tot0.length().max(1.0) < 1e-11);
}
