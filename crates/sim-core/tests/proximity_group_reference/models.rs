//! Shared test builders and momentum diagnostics using only `sim-core`'s
//! public API.

use std::io::Write as _;

use glam::{DMat3, DVec3};
use mbd_core::{ComponentId, ComponentSpec, ComponentState, ComponentTree, JointSpec, Placement};
use mbd_multibody::{
    DockingPhase, DockingSpec, DockingState, MultiBodyDynamics, TetherCommand, WinchTetherSpec,
    WinchTetherState,
};
use sim_atmosphere::VacuumAtmosphere;
use sim_core::{
    InterVesselLink, MotionTimeScale, MultiVesselCoordinator, MultibodySubsystems,
    MultibodyVehicleBlock, MultibodyVehicleConfig, SimVesselId, TICKS_PER_SECOND, TickSpan,
    VehicleIntegrator, VehicleState,
};
use sim_geo::{BodyEllipsoid, UtcTime};

pub fn report_line(msg: &str) {
    writeln!(std::io::stdout(), "{msg}").unwrap();
}

pub fn free_space_body() -> BodyEllipsoid {
    BodyEllipsoid {
        mu_m3_s2: 0.0,
        omega_rad_s: 0.0,
        semi_major_m: 1.0,
        flattening: 0.0,
        ..BodyEllipsoid::WGS84
    }
}

pub fn orbital_zero_grav_body() -> BodyEllipsoid {
    BodyEllipsoid {
        mu_m3_s2: 0.0,
        omega_rad_s: 0.0,
        ..BodyEllipsoid::WGS84
    }
}

pub fn part(
    name: &str,
    placement: Placement,
    joint: JointSpec,
    mass_kg: f64,
    diag_inertia: DVec3,
) -> (ComponentSpec, ComponentState) {
    (
        ComponentSpec {
            name: name.to_owned(),
            placement,
            dry_mass_kg: mass_kg,
            dry_inertia_body: DMat3::from_diagonal(diag_inertia),
            thermal_capacity_j_k: 500.0,
            nominal_power_w: 0.0,
            joint,
        },
        ComponentState::at_temperature(290.0),
    )
}

#[allow(clippy::too_many_arguments)]
pub fn articulated_vessel(
    name: &str,
    body: BodyEllipsoid,
    state: VehicleState,
    steps_per_second: u64,
    integrator: VehicleIntegrator,
    arm_offset: DVec3,
    arm_axis: DVec3,
    arm_q_rad: f64,
    arm_qd_rad_s: f64,
) -> MultibodyVehicleBlock {
    let mut tree = ComponentTree::new([
        part(
            name,
            Placement::ROOT,
            JointSpec::FIXED,
            850.0,
            DVec3::new(320.0, 410.0, 290.0),
        ),
        part(
            "arm",
            Placement::child_of(ComponentId(0), arm_offset),
            JointSpec::revolute(arm_axis, -core::f64::consts::PI, core::f64::consts::PI),
            120.0,
            DVec3::new(25.0, 30.0, 18.0),
        ),
    ])
    .unwrap();
    let mut st = tree.states().to_vec();
    st[1].joint.q_scalar = arm_q_rad;
    st[1].joint.q_vel_scalar = arm_qd_rad_s;
    tree.restore_states(&st).unwrap();
    MultibodyVehicleBlock::new(
        MultibodyVehicleConfig {
            body,
            period: TickSpan::from_ticks(TICKS_PER_SECOND / steps_per_second),
            epoch: UtcTime::from_unix_seconds(0.0),
            initial: state,
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

#[allow(clippy::too_many_arguments)]
pub fn rigid_off_cg_vessel(
    name: &str,
    body: BodyEllipsoid,
    state: VehicleState,
    steps_per_second: u64,
    integrator: VehicleIntegrator,
    mass_a: f64,
    mass_b: f64,
    offset_b: DVec3,
) -> MultibodyVehicleBlock {
    let tree = ComponentTree::new([
        part(
            name,
            Placement::ROOT,
            JointSpec::FIXED,
            mass_a,
            DVec3::new(340.0, 420.0, 290.0),
        ),
        part(
            "adapter",
            Placement::child_of(ComponentId(0), offset_b),
            JointSpec::FIXED,
            mass_b,
            DVec3::new(55.0, 70.0, 62.0),
        ),
    ])
    .unwrap();
    MultibodyVehicleBlock::new(
        MultibodyVehicleConfig {
            body,
            period: TickSpan::from_ticks(TICKS_PER_SECOND / steps_per_second),
            epoch: UtcTime::from_unix_seconds(0.0),
            initial: state,
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

pub fn vessel_momentum_world(block: &MultibodyVehicleBlock) -> (DVec3, DVec3, DVec3, f64) {
    let masses = block.dynamic_mass_contributions().unwrap();
    let fb = block.floating_base_fixed();
    let mom = MultiBodyDynamics::system_momentum(block.tree(), &masses, &fb).unwrap();
    (
        mom.linear_world_kg_m_s,
        mom.angular_about_com_world_nms,
        mom.com_world_m,
        mom.total_mass_kg,
    )
}

/// Computes total mass, total linear momentum, system CM position, angular
/// momentum about system CM (`L_cm`), and angular momentum about world origin
/// (`L_O`) across `ids`.
pub fn system_totals(coord: &MultiVesselCoordinator, ids: &[SimVesselId]) -> (DVec3, DVec3, DVec3) {
    let mut p_tot = DVec3::ZERO;
    let mut m_tot = 0.0;
    let mut mr_tot = DVec3::ZERO;
    let mut per_vessel = Vec::with_capacity(ids.len());
    for &id in ids {
        let b = coord.vessel(id).unwrap();
        let (p, l_cm, r_cm, m) = vessel_momentum_world(b);
        p_tot += p;
        m_tot += m;
        mr_tot += r_cm * m;
        per_vessel.push((p, l_cm, r_cm));
    }
    let r_cm_sys = mr_tot / m_tot;
    let mut l_cm_sys = DVec3::ZERO;
    let mut l_origin = DVec3::ZERO;
    for (p, l_cm, r_cm) in per_vessel {
        l_cm_sys += l_cm + (r_cm - r_cm_sys).cross(p);
        l_origin += l_cm + r_cm.cross(p);
    }
    (p_tot, l_cm_sys, l_origin)
}

#[allow(clippy::too_many_arguments)]
pub fn tether_link(
    id: u64,
    carrier: SimVesselId,
    anchor_carrier: ComponentId,
    offset_carrier_m: DVec3,
    payload: SimVesselId,
    anchor_payload: ComponentId,
    offset_payload_m: DVec3,
    spec: WinchTetherSpec,
    deployed_length_m: f64,
    payout_rate_m_s: f64,
) -> InterVesselLink {
    InterVesselLink::WinchTether {
        id,
        carrier,
        anchor_carrier,
        offset_carrier_m,
        payload,
        anchor_payload,
        offset_payload_m,
        spec,
        state: WinchTetherState::intact_at(deployed_length_m),
        command: TetherCommand {
            payout_rate_m_s,
            heat_rate_w: 0.0,
            cut: false,
        },
    }
}

#[allow(clippy::too_many_arguments)]
pub fn docking_link(
    id: u64,
    chaser: SimVesselId,
    port_a: ComponentId,
    offset_a_local_m: DVec3,
    axis_a_local: DVec3,
    target: SimVesselId,
    port_b: ComponentId,
    offset_b_local_m: DVec3,
    spec: DockingSpec,
    standoff_m: f64,
) -> InterVesselLink {
    InterVesselLink::Docking {
        id,
        chaser,
        port_a,
        offset_a_local_m,
        axis_a_local,
        target,
        port_b,
        offset_b_local_m,
        mate_placement: Placement::child_of(port_a, DVec3::new(0.2, 0.0, 0.0)),
        spec,
        state: DockingState {
            phase: DockingPhase::Open,
            standoff_m,
            dwell_s: 0.0,
        },
    }
}

pub fn default_docking_spec() -> DockingSpec {
    DockingSpec {
        capture_radius_m: 1.8,
        capture_angle_rad: 0.8,
        max_closing_speed_m_s: 3.0,
        damping_vel_threshold_m_s: 0.25,
        damping_rate_threshold_rad_s: 0.15,
        retraction_speed_m_s: 0.4,
        linear_stiffness_n_m: 2_800.0,
        linear_damping_n_s_m: 1_100.0,
        angular_stiffness_nm_rad: 500.0,
        angular_damping_nm_s_rad: 150.0,
        undock_impulse_ns: 40.0,
        min_capture_standoff_m: DockingSpec::DEFAULT_MIN_CAPTURE_STANDOFF_M,
        damped_dwell_s: 0.2,
        latch_standoff_tolerance_m: DockingSpec::DEFAULT_LATCH_STANDOFF_TOLERANCE_M,
        latch_distance_m: DockingSpec::DEFAULT_LATCH_DISTANCE_M,
    }
}
