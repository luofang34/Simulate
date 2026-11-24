//! Error-contract tests for invalid coupling, non-multibody vessels,
//! incompatible body/period, premature orbital promotion, and invalid
//! cut/mate/undock/grapple requests.

use glam::DVec3;
use mbd_core::{ComponentId, ComponentTree, JointSpec, Placement};
use mbd_multibody::{GrappleLockSpec, ManipulatorChain, TetherCommand};
use sim_core::{MultibodySubsystems, SimInstant, TickSpan, VehicleIntegrator};
use sim_geo::BodyEllipsoid;
use sim_orbital::VesselId;
use sim_universe::{LinkId, Tier, Universe, UniverseError};

use super::models::{
    FAST_STEP, at_ms, build_dock_spec, build_tether_link, leo_state, make_mb_plant,
    make_rigid_plant, multi_stage_articulated_tree, orbital_ctx, part, test_epoch,
};

#[test]
fn rejects_non_multibody_and_self_couplings_cleanly() {
    let ep = test_epoch();
    let (p_mb, f_mb) = make_mb_plant(
        BodyEllipsoid::WGS84,
        multi_stage_articulated_tree(),
        leo_state(DVec3::ZERO),
        FAST_STEP,
        &ep,
    );
    let (p_rig, f_rig) = make_rigid_plant(
        BodyEllipsoid::WGS84,
        leo_state(DVec3::new(1.0, 0.0, 0.0)),
        FAST_STEP,
        &ep,
    );
    let mut u = Universe::new(ep, orbital_ctx());
    u.insert_local(VesselId(1), p_mb, f_mb).unwrap();
    u.insert_local(VesselId(2), p_rig, f_rig).unwrap();
    u.promote_to_orbital(VesselId(1)).unwrap();

    assert!(matches!(
        u.dock_soft_capture(build_dock_spec(VesselId(1), VesselId(2), 0.5)),
        Err(UniverseError::NotMultibodyPlant(VesselId(2)))
    ));
    assert_eq!(u.tier(VesselId(1)).unwrap(), Tier::Orbital);
    assert!(matches!(
        u.hard_mate(
            VesselId(1),
            ComponentId(0),
            VesselId(2),
            ComponentId(0),
            Placement::ROOT
        ),
        Err(UniverseError::NotMultibodyPlant(VesselId(2)))
    ));
    assert!(matches!(
        u.separate_stage_impulse(VesselId(2), ComponentId(0), 10.0, DVec3::X),
        Err(UniverseError::NotMultibodyPlant(VesselId(2)))
    ));

    u.demote_to_local(VesselId(1)).unwrap();
    assert!(matches!(
        u.add_inter_vessel_link(build_tether_link(VesselId(1), VesselId(1), 5.0, 0.0)),
        Err(UniverseError::SameVesselLink(VesselId(1)))
    ));
    assert!(matches!(
        u.hard_mate(
            VesselId(1),
            ComponentId(0),
            VesselId(1),
            ComponentId(0),
            Placement::ROOT
        ),
        Err(UniverseError::SameVesselLink(VesselId(1)))
    ));
}

#[test]
fn rejects_incompatible_body_and_period_couplings() {
    let ep = test_epoch();
    let (p1, f1) = make_mb_plant(
        BodyEllipsoid::WGS84,
        multi_stage_articulated_tree(),
        leo_state(DVec3::ZERO),
        FAST_STEP,
        &ep,
    );
    let slow_period = TickSpan::from_ticks(FAST_STEP.ticks() * 2);
    let (p_slow, f_slow) = make_mb_plant(
        BodyEllipsoid::WGS84,
        ComponentTree::new([part(
            "slow",
            Placement::ROOT,
            JointSpec::FIXED,
            500.0,
            150.0,
        )])
        .unwrap(),
        leo_state(DVec3::new(2.0, 0.0, 0.0)),
        slow_period,
        &ep,
    );
    let mut u = Universe::new(ep.clone(), orbital_ctx());
    u.insert_local(VesselId(1), p1, f1).unwrap();
    u.insert_local(VesselId(3), p_slow, f_slow).unwrap();
    assert!(matches!(
        u.dock_soft_capture(build_dock_spec(VesselId(1), VesselId(3), 0.5)),
        Err(UniverseError::IncompatibleCoupling {
            vessel: VesselId(3),
            ..
        })
    ));

    let moon_body = u.bodies().get(sim_orbital::NaifId::MOON).unwrap().ellipsoid;
    let (p_moon, f_moon) = make_mb_plant(
        moon_body,
        ComponentTree::new([part(
            "lunar",
            Placement::ROOT,
            JointSpec::FIXED,
            500.0,
            150.0,
        )])
        .unwrap(),
        leo_state(DVec3::ZERO),
        FAST_STEP,
        &ep,
    );
    u.insert_local(VesselId(4), p_moon, f_moon).unwrap();
    assert!(matches!(
        u.add_inter_vessel_link(build_tether_link(VesselId(1), VesselId(4), 5.0, 0.0)),
        Err(UniverseError::IncompatibleCoupling {
            vessel: VesselId(4),
            ..
        })
    ));
}

fn arm_universe() -> Universe {
    let ep = test_epoch();
    let body = BodyEllipsoid::WGS84;
    let arm_tree = ComponentTree::new([
        part(
            "servicer",
            Placement::ROOT,
            JointSpec::FIXED,
            1_000.0,
            400.0,
        ),
        part(
            "ee",
            Placement::child_of(ComponentId(0), DVec3::new(1.0, 0.0, 0.0)),
            JointSpec::revolute(DVec3::Z, -core::f64::consts::PI, core::f64::consts::PI),
            50.0,
            10.0,
        ),
    ])
    .unwrap();
    let chain = ManipulatorChain::new(&arm_tree, vec![ComponentId(1)], ComponentId(1), DVec3::ZERO)
        .unwrap();
    let arm_block = sim_core::MultibodyVehicleBlock::new(
        sim_core::MultibodyVehicleConfig {
            body,
            period: FAST_STEP,
            epoch: ep.utc_at(SimInstant::START).unwrap(),
            initial: leo_state(DVec3::ZERO),
            integrator: VehicleIntegrator::REAL_TIME,
            time_scale: sim_core::MotionTimeScale::Seconds(60.0),
        },
        arm_tree,
        Vec::new(),
        MultibodySubsystems {
            manipulators: vec![("arm".to_owned(), chain)],
            ..MultibodySubsystems::default()
        },
        Box::new(sim_atmosphere::VacuumAtmosphere),
        std::sync::Arc::new(body),
    )
    .unwrap();
    let arm_plant = sim_universe::MultibodyLocalPlant::new(arm_block, body, SimInstant::START, 5.0);
    let (_, arm_fac) = make_mb_plant(
        body,
        multi_stage_articulated_tree(),
        leo_state(DVec3::ZERO),
        FAST_STEP,
        &ep,
    );
    let (p2, f2) = make_mb_plant(
        body,
        ComponentTree::new([part("tgt", Placement::ROOT, JointSpec::FIXED, 600.0, 200.0)]).unwrap(),
        leo_state(DVec3::new(8.0, 0.0, 0.0)),
        FAST_STEP,
        &ep,
    );
    let mut u = Universe::new(ep, orbital_ctx());
    u.insert_local(VesselId(1), Box::new(arm_plant), arm_fac)
        .unwrap();
    u.insert_local(VesselId(2), p2, f2).unwrap();
    u
}

#[test]
fn rejects_invalid_link_ops_premature_promotion_and_bad_topology_args() {
    let mut u = arm_universe();
    let g_spec = GrappleLockSpec {
        capture_radius_m: 0.5,
        capture_angle_rad: 0.5,
        max_relative_speed_m_s: 1.0,
    };
    assert!(matches!(
        u.grapple_lock(
            &g_spec,
            VesselId(1),
            "arm",
            VesselId(2),
            ComponentId(0),
            Placement::ROOT
        ),
        Err(UniverseError::Block(_))
    ));
    assert!(matches!(
        u.separate_stage_impulse(VesselId(1), ComponentId(0), 50.0, DVec3::X),
        Err(UniverseError::Block(_))
    ));
    assert!(matches!(
        u.undock_subtree(VesselId(99), ComponentId(1), 50.0, DVec3::X),
        Err(UniverseError::UnknownVessel(VesselId(99)))
    ));

    let dock_id = u
        .dock_soft_capture(build_dock_spec(VesselId(1), VesselId(2), 1.0))
        .unwrap();
    assert!(matches!(
        u.promote_to_orbital(VesselId(1)),
        Err(UniverseError::VesselCoupled(VesselId(1)))
    ));
    assert!(matches!(
        u.cut_tether(LinkId(999)),
        Err(UniverseError::UnknownLink(999))
    ));
    assert!(matches!(
        u.cut_tether(dock_id),
        Err(UniverseError::NotATetherLink(id)) if id == dock_id.0
    ));
    assert!(matches!(
        u.set_tether_command(dock_id, TetherCommand::default()),
        Err(UniverseError::NotATetherLink(id)) if id == dock_id.0
    ));

    u.remove_inter_vessel_link(dock_id);
    u.advance_to(at_ms(10)).unwrap();
    assert!(matches!(
        u.separate_stage_impulse(VesselId(1), ComponentId(1), 10.0, DVec3::X),
        Err(UniverseError::BetweenSteps {
            vessel: VesselId(1)
        })
    ));
}
