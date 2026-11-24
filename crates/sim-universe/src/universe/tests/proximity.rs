//! First-class staging, undocking, soft-capture docking, hard-mate, grapple,
//! winch tethers, tiering discipline, momentum conservation, and deterministic
//! checkpoint/slicing replay on [`Universe`].

use glam::{DMat3, DVec3};
use mbd_core::{ComponentId, ComponentSpec, ComponentState, ComponentTree, JointSpec, Placement};
use mbd_multibody::{
    DockingPhase, DockingSpec, DockingState, GrappleLockSpec, ManipulatorChain, MultiBodyDynamics,
    WinchTetherSpec,
};
use sim_atmosphere::VacuumAtmosphere;
use sim_core::{
    MotionTimeScale, MultibodySubsystems, MultibodyVehicleBlock, MultibodyVehicleConfig,
    SimInstant, TICKS_PER_SECOND, TickSpan, VehicleIntegrator, VehicleState,
};
use sim_geo::BodyEllipsoid;
use sim_orbital::VesselId;
use sim_spacetime::SimEpoch;

use super::{STEP, epoch, leo_state, solar_system_context, universe_with};
use crate::local::{LocalFactory, LocalPlant, MultibodyLocalPlant};
use crate::proximity::DockingLinkSpec;
use crate::{Tier, Universe, UniverseError};

pub(super) const FAST_STEP: TickSpan = TickSpan::from_ticks(TICKS_PER_SECOND / 50);

pub(super) fn steps(n: u64) -> TickSpan {
    TickSpan::from_ticks(STEP.ticks() * n)
}

pub(super) fn part(
    name: &str,
    placement: Placement,
    joint: JointSpec,
    mass_kg: f64,
    inertia: f64,
) -> (ComponentSpec, ComponentState) {
    (
        ComponentSpec {
            name: name.to_owned(),
            placement,
            dry_mass_kg: mass_kg,
            dry_inertia_body: DMat3::from_diagonal(DVec3::splat(inertia)),
            thermal_capacity_j_k: 500.0,
            nominal_power_w: 0.0,
            joint,
        },
        ComponentState::at_temperature(290.0),
    )
}

pub(super) fn mb_plant(
    tree: ComponentTree,
    subsystems: MultibodySubsystems,
    state: VehicleState,
    period: TickSpan,
    ep: &SimEpoch,
) -> (Box<dyn LocalPlant>, LocalFactory) {
    mb_plant_with_body(BodyEllipsoid::WGS84, tree, subsystems, state, period, ep)
}

pub(super) fn mb_plant_with_body(
    body: BodyEllipsoid,
    tree: ComponentTree,
    subsystems: MultibodySubsystems,
    state: VehicleState,
    period: TickSpan,
    ep: &SimEpoch,
) -> (Box<dyn LocalPlant>, LocalFactory) {
    let block = MultibodyVehicleBlock::new(
        MultibodyVehicleConfig {
            body,
            period,
            epoch: ep.utc_at(SimInstant::START).unwrap(),
            initial: state,
            integrator: VehicleIntegrator::REAL_TIME,
            time_scale: MotionTimeScale::Seconds(60.0),
        },
        tree,
        Vec::new(),
        subsystems,
        Box::new(VacuumAtmosphere),
        std::sync::Arc::new(body),
    )
    .unwrap();
    let plant = MultibodyLocalPlant::new(block, body, SimInstant::START, 5.0);
    let factory = plant.rebuild_factory();
    (Box::new(plant), factory)
}

pub(super) fn leo_vehicle_state(offset_ecef: DVec3) -> VehicleState {
    let mut s = leo_state();
    s.position_ecef += offset_ecef;
    s
}

pub(super) fn system_mom(u: &Universe, id: VesselId, cm_ref: DVec3) -> (DVec3, DVec3, f64) {
    let p = u.local::<MultibodyLocalPlant>(id).unwrap();
    let b = p.block();
    let m = b.dynamic_mass_contributions().unwrap();
    let sm = MultiBodyDynamics::system_momentum(b.tree(), &m, &b.floating_base_fixed()).unwrap();
    let ang =
        sm.angular_about_com_world_nms + (sm.com_world_m - cm_ref).cross(sm.linear_world_kg_m_s);
    (sm.linear_world_kg_m_s, ang, sm.total_mass_kg)
}

pub(super) fn docking_spec_default() -> DockingSpec {
    DockingSpec {
        capture_radius_m: 0.8,
        capture_angle_rad: 0.3,
        max_closing_speed_m_s: 2.0,
        damping_vel_threshold_m_s: 0.25,
        damping_rate_threshold_rad_s: 0.15,
        retraction_speed_m_s: 1.0,
        linear_stiffness_n_m: 4_000.0,
        linear_damping_n_s_m: 2_000.0,
        angular_stiffness_nm_rad: 800.0,
        angular_damping_nm_s_rad: 200.0,
        undock_impulse_ns: 50.0,
        min_capture_standoff_m: DockingSpec::DEFAULT_MIN_CAPTURE_STANDOFF_M,
        damped_dwell_s: DockingSpec::DEFAULT_DAMPED_DWELL_S,
        latch_standoff_tolerance_m: DockingSpec::DEFAULT_LATCH_STANDOFF_TOLERANCE_M,
        latch_distance_m: DockingSpec::DEFAULT_LATCH_DISTANCE_M,
    }
}

pub(super) fn tether_spec_default() -> WinchTetherSpec {
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

pub(super) fn two_stage_universe() -> Universe {
    let ep = epoch();
    let tree = ComponentTree::new([
        part("core", Placement::ROOT, JointSpec::FIXED, 800.0, 300.0),
        part(
            "booster",
            Placement::child_of(ComponentId(0), DVec3::new(-3.0, 0.0, 0.0)),
            JointSpec::FIXED,
            400.0,
            150.0,
        ),
    ])
    .unwrap();
    let (plant, fac) = mb_plant(
        tree,
        MultibodySubsystems::default(),
        leo_vehicle_state(DVec3::ZERO),
        FAST_STEP,
        &ep,
    );
    let mut u = Universe::new(ep, solar_system_context());
    u.insert_local(VesselId(1), plant, fac).unwrap();
    u
}

pub(super) fn docking_pair_universe(carried_target: bool) -> Universe {
    let ep = epoch();
    let s1 = leo_vehicle_state(DVec3::ZERO);
    let s2 = leo_vehicle_state(DVec3::new(0.25, 0.0, 0.0));
    let t1 = ComponentTree::new([part(
        "chaser",
        Placement::ROOT,
        JointSpec::FIXED,
        1_000.0,
        400.0,
    )])
    .unwrap();
    let t2 = ComponentTree::new([part(
        "target",
        Placement::ROOT,
        JointSpec::FIXED,
        800.0,
        300.0,
    )])
    .unwrap();
    let (p1, f1) = mb_plant(t1, MultibodySubsystems::default(), s1, FAST_STEP, &ep);
    let (p2, f2) = mb_plant(t2, MultibodySubsystems::default(), s2, FAST_STEP, &ep);
    let mut u = Universe::new(ep, solar_system_context());
    u.insert_local(VesselId(1), p1, f1).unwrap();
    u.insert_local(VesselId(2), p2, f2).unwrap();
    if carried_target {
        u.promote_to_orbital(VesselId(2)).unwrap();
    }
    u
}

pub(super) fn tether_pair_universe() -> Universe {
    let ep = epoch();
    let t1 = ComponentTree::new([part(
        "carrier",
        Placement::ROOT,
        JointSpec::FIXED,
        1_000.0,
        400.0,
    )])
    .unwrap();
    let t2 = ComponentTree::new([part(
        "payload",
        Placement::ROOT,
        JointSpec::FIXED,
        800.0,
        300.0,
    )])
    .unwrap();
    let (p1, f1) = mb_plant(
        t1,
        MultibodySubsystems::default(),
        leo_vehicle_state(DVec3::ZERO),
        FAST_STEP,
        &ep,
    );
    let (p2, f2) = mb_plant(
        t2,
        MultibodySubsystems::default(),
        leo_vehicle_state(DVec3::new(10.0, 0.0, 0.0)),
        FAST_STEP,
        &ep,
    );
    let mut u = Universe::new(ep, solar_system_context());
    u.insert_local(VesselId(1), p1, f1).unwrap();
    u.insert_local(VesselId(2), p2, f2).unwrap();
    u
}

#[test]
fn staging_and_undocking_preserve_momentum_bus_and_factory() {
    let mut u = two_stage_universe();
    let cm_ref = DVec3::ZERO;
    let (p0, l0, m0) = system_mom(&u, VesselId(1), cm_ref);
    let child = u
        .separate_stage_impulse(VesselId(1), ComponentId(1), 500.0, DVec3::NEG_X)
        .unwrap();
    assert_eq!(u.active_vessel(), Some(VesselId(1)));
    assert_eq!(u.cycle_active_vessel(), Some(child));
    assert_eq!(u.cycle_active_vessel(), Some(VesselId(1)));
    let rep = u.take_proximity_report();
    assert_eq!(rep.spawned_vessels, vec![(VesselId(1), child)]);

    let (pa, la, ma) = system_mom(&u, VesselId(1), cm_ref);
    let (pb, lb, mb) = system_mom(&u, child, cm_ref);
    assert!((ma + mb - m0).abs() < 1e-11);
    assert!(((pa + pb) - p0).length() / p0.length().max(1.0) < 1e-11);
    assert!(((la + lb) - l0).length() / l0.length().max(1.0) < 1e-11);
    assert!(u.bus_telemetry(child).is_ok());

    u.promote_to_orbital(child).unwrap();
    u.advance_to(SimInstant::START.advanced(FAST_STEP)).unwrap();
    u.demote_to_local(child).unwrap();
    assert_eq!(u.tier(child).unwrap(), Tier::Local);
}

#[test]
fn soft_capture_auto_demotes_orbital_target_holds_policy_then_merges_and_undocks() {
    let mut u = docking_pair_universe(true).with_regime_tier_policy();
    let link_id = u
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
    assert_eq!(u.tier(VesselId(2)).unwrap(), Tier::Local);
    assert!(u.is_vessel_coupled(VesselId(1)));
    assert!(matches!(
        u.promote_to_orbital(VesselId(1)),
        Err(UniverseError::VesselCoupled(VesselId(1)))
    ));

    // Mid-capture: policy checks must NOT promote while the docking link is active.
    u.advance_to(SimInstant::START.advanced(steps(5))).unwrap();
    if u.inter_vessel_link(link_id).is_some() {
        assert_eq!(u.tier(VesselId(1)).unwrap(), Tier::Local);
        assert_eq!(u.tier(VesselId(2)).unwrap(), Tier::Local);
    }

    // Complete capture + latch -> target retires -> policy promotes merged vessel.
    u.advance_to(SimInstant::START.advanced(steps(20))).unwrap();
    let rep = u.take_proximity_report();
    assert_eq!(rep.merged_vessels, vec![(VesselId(1), VesselId(2))]);
    assert!(u.inter_vessel_link(link_id).is_none());
    assert!(matches!(
        u.tier(VesselId(2)),
        Err(UniverseError::UnknownVessel(VesselId(2)))
    ));
    assert_eq!(u.tier(VesselId(1)).unwrap(), Tier::Orbital);

    let undocked = u
        .undock_subtree(VesselId(1), ComponentId(1), 50.0, DVec3::X)
        .unwrap();
    assert_eq!(u.tier(VesselId(1)).unwrap(), Tier::Local);
    assert_eq!(u.tier(undocked).unwrap(), Tier::Local);
}

#[test]
fn hard_mate_and_errors_preserve_momentum_and_active_focus() {
    let mut u_rig = universe_with(&[1]);
    let rigid_only = u_rig.separate_stage_impulse(VesselId(1), ComponentId(0), 10.0, DVec3::X);
    assert!(matches!(
        rigid_only,
        Err(UniverseError::NotMultibodyPlant(VesselId(1)))
    ));

    let mut u = docking_pair_universe(false);
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
    let cm_ref = DVec3::ZERO;
    let (pa0, la0, _) = system_mom(&u, VesselId(1), cm_ref);
    let (pb0, lb0, _) = system_mom(&u, VesselId(2), cm_ref);
    u.set_active_vessel(VesselId(2)).unwrap();
    u.hard_mate(
        VesselId(1),
        ComponentId(0),
        VesselId(2),
        ComponentId(0),
        Placement::child_of(ComponentId(0), DVec3::new(0.25, 0.0, 0.0)),
    )
    .unwrap();
    assert_eq!(u.active_vessel(), Some(VesselId(1)));
    let (pm, lm, mm) = system_mom(&u, VesselId(1), cm_ref);
    assert!((mm - 1_800.0).abs() < 1e-9);
    assert!((pm - (pa0 + pb0)).length() / pm.length().max(1.0) < 1e-11);
    assert!((lm - (la0 + lb0)).length() / lm.length().max(1.0) < 1e-11);
}

#[test]
fn grapple_lock_captures_and_retires_target() {
    let ep = epoch();
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
            JointSpec::revolute(DVec3::Z, -std::f64::consts::PI, std::f64::consts::PI),
            50.0,
            10.0,
        ),
    ])
    .unwrap();
    let chain = ManipulatorChain::new(&arm_tree, vec![ComponentId(1)], ComponentId(1), DVec3::ZERO)
        .unwrap();
    let arm_subs = MultibodySubsystems {
        manipulators: vec![("arm".to_owned(), chain)],
        ..MultibodySubsystems::default()
    };
    let (p_arm, f_arm) = mb_plant(
        arm_tree,
        arm_subs,
        leo_vehicle_state(DVec3::new(-1.0, 0.0, 0.0)),
        FAST_STEP,
        &ep,
    );
    let (p_tgt, f_tgt) = mb_plant(
        ComponentTree::new([part(
            "free",
            Placement::ROOT,
            JointSpec::FIXED,
            600.0,
            200.0,
        )])
        .unwrap(),
        MultibodySubsystems::default(),
        leo_vehicle_state(DVec3::ZERO),
        FAST_STEP,
        &ep,
    );
    let mut ug = Universe::new(ep, solar_system_context());
    ug.insert_local(VesselId(10), p_arm, f_arm).unwrap();
    ug.insert_local(VesselId(11), p_tgt, f_tgt).unwrap();
    let g_spec = GrappleLockSpec {
        capture_radius_m: 0.5,
        capture_angle_rad: 0.5,
        max_relative_speed_m_s: 1.0,
    };
    ug.grapple_lock(
        &g_spec,
        VesselId(10),
        "arm",
        VesselId(11),
        ComponentId(0),
        Placement::ROOT,
    )
    .unwrap();
    assert!(matches!(
        ug.tier(VesselId(11)),
        Err(UniverseError::UnknownVessel(VesselId(11)))
    ));
}
