//! Shared builders and momentum/orbital diagnostics using only `sim-universe`
//! public APIs.

use std::io::Write as _;
use std::sync::Arc;

use glam::{DMat3, DQuat, DVec3};
use mbd_core::{ComponentId, ComponentSpec, ComponentState, ComponentTree, JointSpec, Placement};
use mbd_multibody::{
    DockingPhase, DockingSpec, DockingState, MultiBodyDynamics, TetherCommand, WinchTetherSpec,
    WinchTetherState,
};
use sim_atmosphere::VacuumAtmosphere;
use sim_core::{
    MotionTimeScale, MultibodySubsystems, MultibodyVehicleBlock, MultibodyVehicleConfig,
    RigidBody6DofBlock, RigidBodyConfig, SimInstant, TICKS_PER_SECOND, TickSpan, VehicleIntegrator,
    VehicleState,
};
use sim_geo::{BodyEllipsoid, UtcTime};
use sim_orbital::{BuiltinSolarSystem2026_2028, Dop853Tolerances, ForceModel, VesselId};
use sim_spacetime::SimEpoch;
use sim_universe::{
    CoastingRigidBody, DockingLinkSpec, LocalFactory, LocalPlant, MultibodyLocalPlant,
    OrbitalContext, TierPolicy, Universe, UniverseError, UniverseLinkSpec,
};

pub const FAST_STEP: TickSpan = TickSpan::from_ticks(TICKS_PER_SECOND / 50);
pub const START_UTC_UNIX_S: f64 = 1_803_859_200.0;

pub fn report_line(msg: &str) {
    writeln!(std::io::stdout(), "{msg}").unwrap();
}

pub fn at_ms(ms: u64) -> SimInstant {
    SimInstant::START.advanced(TickSpan::from_ticks(ms * (TICKS_PER_SECOND / 1_000)))
}

pub fn test_epoch() -> SimEpoch {
    SimEpoch::builtin_from_utc(UtcTime::from_unix_seconds(START_UTC_UNIX_S)).unwrap()
}

pub fn orbital_ctx() -> OrbitalContext {
    let solar = BuiltinSolarSystem2026_2028::load_verified().unwrap();
    let force_model = ForceModel::standard_solar_system(solar.constants.clone()).unwrap();
    OrbitalContext {
        ephemeris: Arc::new(solar.ephemeris),
        orientation: Arc::new(solar.orientation),
        force_model,
        quantum: TickSpan::SECOND,
        tolerances: Dop853Tolerances {
            rtol: 1e-12,
            atol_pos_m: 1e-6,
            atol_vel_m_s: 1e-9,
            ..Dop853Tolerances::default()
        },
    }
}

pub fn leo_state(offset_ecef: DVec3) -> VehicleState {
    let r = 6_778_137.0;
    let v_i = (BodyEllipsoid::WGS84.mu_m3_s2 / r).sqrt();
    let inc = 51.6_f64.to_radians();
    let pos = DVec3::new(r, 0.0, 0.0);
    let vel_i = DVec3::new(0.0, inc.cos(), inc.sin()) * v_i;
    let spin = DVec3::Z * BodyEllipsoid::WGS84.omega_rad_s;
    VehicleState {
        position_ecef: pos + offset_ecef,
        velocity_ecef: vel_i - spin.cross(pos),
        attitude: DQuat::IDENTITY,
        angular_rate_body: DVec3::new(0.002, -0.001, 0.003),
    }
}

pub fn part(
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

/// Core (0) -> moving revolute gimbal (1) + detachable booster stage (2).
pub fn multi_stage_articulated_tree() -> ComponentTree {
    let mut tree = ComponentTree::new([
        part("core", Placement::ROOT, JointSpec::FIXED, 900.0, 350.0),
        part(
            "gimbal",
            Placement::child_of(ComponentId(0), DVec3::new(0.0, 1.2, 0.0)),
            JointSpec::revolute(DVec3::Z, -core::f64::consts::PI, core::f64::consts::PI),
            80.0,
            25.0,
        ),
        part(
            "booster",
            Placement::child_of(ComponentId(0), DVec3::new(-3.2, 0.0, 0.0)),
            JointSpec::FIXED,
            420.0,
            160.0,
        ),
    ])
    .unwrap();
    let mut st = tree.states().to_vec();
    st[1].joint.q_scalar = 0.15;
    st[1].joint.q_vel_scalar = 0.04;
    tree.restore_states(&st).unwrap();
    tree
}

pub fn make_mb_plant(
    body: BodyEllipsoid,
    tree: ComponentTree,
    state: VehicleState,
    period: TickSpan,
    ep: &SimEpoch,
) -> (Box<dyn LocalPlant>, LocalFactory) {
    make_mb_plant_with_integrator(body, tree, state, period, ep, VehicleIntegrator::REAL_TIME)
}

pub fn make_mb_plant_with_integrator(
    body: BodyEllipsoid,
    tree: ComponentTree,
    state: VehicleState,
    period: TickSpan,
    ep: &SimEpoch,
    integrator: VehicleIntegrator,
) -> (Box<dyn LocalPlant>, LocalFactory) {
    let block = MultibodyVehicleBlock::new(
        MultibodyVehicleConfig {
            body,
            period,
            epoch: ep.utc_at(SimInstant::START).unwrap(),
            initial: state,
            integrator,
            time_scale: MotionTimeScale::Seconds(60.0),
        },
        tree,
        Vec::new(),
        MultibodySubsystems::default(),
        Box::new(VacuumAtmosphere),
        Arc::new(body),
    )
    .unwrap();
    let plant = MultibodyLocalPlant::new(block, body, SimInstant::START, 5.0)
        .with_carry_period(TickSpan::SECOND);
    let tree_clone = plant.block().tree().clone();
    let factory: LocalFactory = Box::new(move |described, st, at, epoch| {
        let b = MultibodyVehicleBlock::new(
            MultibodyVehicleConfig {
                body: described.ellipsoid,
                period,
                epoch: epoch.utc_at(at)?,
                initial: *st,
                integrator,
                time_scale: MotionTimeScale::Seconds(60.0),
            },
            tree_clone.clone(),
            Vec::new(),
            MultibodySubsystems::default(),
            Box::new(VacuumAtmosphere),
            Arc::new(described.ellipsoid),
        )?;
        Ok(Box::new(
            MultibodyLocalPlant::new(b, described.ellipsoid, at, 5.0)
                .with_carry_period(TickSpan::SECOND),
        ))
    });
    (Box::new(plant), factory)
}

pub fn make_rigid_plant(
    body: BodyEllipsoid,
    state: VehicleState,
    period: TickSpan,
    ep: &SimEpoch,
) -> (Box<dyn LocalPlant>, LocalFactory) {
    let tree = ComponentTree::new([part(
        "rigid",
        Placement::ROOT,
        JointSpec::FIXED,
        500.0,
        200.0,
    )])
    .unwrap();
    let tree_clone = tree.clone();
    let block = RigidBody6DofBlock::new(
        RigidBodyConfig {
            body,
            period,
            epoch: ep.utc_at(SimInstant::START).unwrap(),
            initial: state,
        },
        tree,
        Vec::new(),
        Box::new(VacuumAtmosphere),
        Arc::new(body),
    )
    .unwrap();
    let factory: LocalFactory = Box::new(move |described, st, at, epoch| {
        let b = RigidBody6DofBlock::new(
            RigidBodyConfig {
                body: described.ellipsoid,
                period,
                epoch: epoch.utc_at(at)?,
                initial: *st,
            },
            tree_clone.clone(),
            Vec::new(),
            Box::new(VacuumAtmosphere),
            Arc::new(described.ellipsoid),
        )?;
        Ok(Box::new(CoastingRigidBody::new(
            b,
            described.ellipsoid,
            at,
            2.0,
        )))
    });
    (
        Box::new(CoastingRigidBody::new(block, body, SimInstant::START, 2.0)),
        factory,
    )
}

pub fn vessel_mom(u: &Universe, id: VesselId) -> (DVec3, DVec3, DVec3, f64) {
    let p = u.local::<MultibodyLocalPlant>(id).unwrap();
    let b = p.block();
    let m = b.dynamic_mass_contributions().unwrap();
    let sm = MultiBodyDynamics::system_momentum(b.tree(), &m, &b.floating_base_fixed()).unwrap();
    (
        sm.linear_world_kg_m_s,
        sm.angular_about_com_world_nms,
        sm.com_world_m,
        sm.total_mass_kg,
    )
}

pub fn pair_mom_cm(u: &Universe, ids: &[VesselId]) -> (DVec3, DVec3, f64) {
    let mut p_tot = DVec3::ZERO;
    let mut m_tot = 0.0;
    let mut mr = DVec3::ZERO;
    let mut items = Vec::with_capacity(ids.len());
    for &id in ids {
        let (p, l, r, m) = vessel_mom(u, id);
        p_tot += p;
        m_tot += m;
        mr += r * m;
        items.push((p, l, r));
    }
    let r_cm = mr / m_tot;
    let mut l_tot = DVec3::ZERO;
    for (p, l, r) in items {
        l_tot += l + (r - r_cm).cross(p);
    }
    (p_tot, l_tot, m_tot)
}

pub fn default_tether_spec() -> WinchTetherSpec {
    WinchTetherSpec {
        stiffness_n_m: 3_000.0,
        damping_n_s_m: 600.0,
        max_length_m: 25.0,
        breaking_tension_n: 80_000.0,
        burn_through_energy_j: None,
        auto_cut_low_tension_n: None,
        auto_cut_dwell_s: 0.0,
    }
}

pub fn default_docking_spec() -> DockingSpec {
    DockingSpec {
        capture_radius_m: 1.5,
        capture_angle_rad: 0.6,
        max_closing_speed_m_s: 2.5,
        damping_vel_threshold_m_s: 0.3,
        damping_rate_threshold_rad_s: 0.2,
        retraction_speed_m_s: 0.8,
        linear_stiffness_n_m: 4_000.0,
        linear_damping_n_s_m: 1_800.0,
        angular_stiffness_nm_rad: 800.0,
        angular_damping_nm_s_rad: 220.0,
        undock_impulse_ns: 60.0,
        min_capture_standoff_m: DockingSpec::DEFAULT_MIN_CAPTURE_STANDOFF_M,
        damped_dwell_s: DockingSpec::DEFAULT_DAMPED_DWELL_S,
        latch_standoff_tolerance_m: DockingSpec::DEFAULT_LATCH_STANDOFF_TOLERANCE_M,
        latch_distance_m: DockingSpec::DEFAULT_LATCH_DISTANCE_M,
    }
}

/// Offset of `multi_stage_articulated_tree()` root `core` from system CG (`-com_rel_root`).
pub const STAGE1_CORE_IN_CG_M: DVec3 = DVec3::new(0.96, -48.0 / 700.0, 0.0);

pub fn build_leo_pair_at(with_policy: bool, offset_b: DVec3) -> Universe {
    let ep = test_epoch();
    let body = BodyEllipsoid::WGS84;
    let (p1, f1) = make_mb_plant(
        body,
        multi_stage_articulated_tree(),
        leo_state(DVec3::ZERO),
        FAST_STEP,
        &ep,
    );
    let target_tree = ComponentTree::new([part(
        "target",
        Placement::ROOT,
        JointSpec::FIXED,
        750.0,
        280.0,
    )])
    .unwrap();
    let (p2, f2) = make_mb_plant(body, target_tree, leo_state(offset_b), FAST_STEP, &ep);
    let mut u = Universe::new(ep, orbital_ctx());
    if with_policy {
        let mut policy = TierPolicy::for_bodies(u.bodies());
        policy.check_period = TickSpan::SECOND;
        u = u.with_tier_policy(policy);
    }
    u.insert_local(VesselId(1), p1, f1).unwrap();
    u.insert_local(VesselId(2), p2, f2).unwrap();
    u
}

pub fn build_leo_mission_universe(with_policy: bool) -> Universe {
    build_leo_pair_at(with_policy, DVec3::new(4.2, 0.0, 0.0))
}

pub fn build_dock_spec(chaser: VesselId, target: VesselId, standoff_m: f64) -> DockingLinkSpec {
    DockingLinkSpec {
        chaser,
        port_a: ComponentId(0),
        offset_a_local_m: DVec3::new(0.5, 0.0, 0.0),
        axis_a_local: DVec3::X,
        target,
        port_b: ComponentId(0),
        offset_b_local_m: DVec3::new(-0.5, 0.0, 0.0),
        mate_placement: Placement::child_of(ComponentId(0), DVec3::new(1.0, 0.0, 0.0)),
        spec: default_docking_spec(),
        state: DockingState {
            phase: DockingPhase::Open,
            standoff_m,
            dwell_s: 0.0,
        },
    }
}

pub fn build_tether_link(
    carrier: VesselId,
    payload: VesselId,
    length_m: f64,
    payout_rate_m_s: f64,
) -> UniverseLinkSpec {
    UniverseLinkSpec::WinchTether {
        carrier,
        anchor_carrier: ComponentId(0),
        offset_carrier_m: DVec3::new(0.4, 0.05, 0.0),
        payload,
        anchor_payload: ComponentId(0),
        offset_payload_m: DVec3::new(-0.4, -0.05, 0.0),
        spec: default_tether_spec(),
        state: WinchTetherState::intact_at(length_m),
        command: TetherCommand {
            payout_rate_m_s,
            heat_rate_w: 0.0,
            cut: false,
        },
    }
}

#[allow(dead_code)]
pub fn assert_err_not_coupling(_e: UniverseError) {}
