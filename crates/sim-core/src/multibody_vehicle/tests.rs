use glam::{DMat3, DQuat, DVec3};
use mbd_aero::{AeroElement, AeroElementKind, BluntCapsuleAero};
use mbd_core::{
    ComponentId, ComponentSpec, ComponentState, ComponentTree, JointSpec, MbdError, Placement,
    SUTTON_GRAVES_K_EARTH,
};
use mbd_multibody::{ContactPointSpec, ContactPointState, MultiBodyDynamics, PinnedSurfaceRest};
use mbd_systems::{
    FeedRegime, FluidNetwork, FluidSpecies, LiquidRocketSpec, LiquidRocketState, TankGeometry,
    TankId, TankSpec, TankState, TpsAblationSpec, TpsAblationState,
};
use sim_atmosphere::{ReentryAtmosphere, VacuumAtmosphere};
use sim_geo::{BodyEllipsoid, Geodetic, LocalFrame, UtcTime};

use super::*;

mod carried;
mod integrator_choice;
mod rotating_frame;
mod trajectory_reference;
use crate::{
    Block, MotionTimeScale, RigidBody6DofBlock, RigidBodyConfig, RigidBodyInput, TICKS_PER_SECOND,
    TickSpan, VehicleIntegrator, VehicleState,
};

fn rigid_part(
    name: &str,
    placement: Placement,
    mass: f64,
    diag: DVec3,
) -> (ComponentSpec, ComponentState) {
    (
        ComponentSpec {
            name: name.to_owned(),
            placement,
            dry_mass_kg: mass,
            dry_inertia_body: DMat3::from_diagonal(diag),
            thermal_capacity_j_k: 500.0,
            nominal_power_w: 0.0,
            joint: JointSpec::FIXED,
        },
        ComponentState::at_temperature(290.0),
    )
}

fn revolute_arm_tree() -> ComponentTree {
    let base = ComponentId(0);
    let joint = ComponentId(1);
    ComponentTree::new([
        rigid_part(
            "bus",
            Placement::ROOT,
            600.0,
            DVec3::new(300.0, 320.0, 280.0),
        ),
        (
            ComponentSpec {
                name: "arm_shoulder".to_owned(),
                placement: Placement::child_of(base, DVec3::new(1.0, 0.0, 0.0)),
                dry_mass_kg: 15.0,
                dry_inertia_body: DMat3::from_diagonal(DVec3::splat(2.0)),
                thermal_capacity_j_k: 50.0,
                nominal_power_w: 0.0,
                joint: JointSpec::revolute(DVec3::Z, -3.0, 3.0),
            },
            ComponentState::at_temperature(290.0),
        ),
        rigid_part(
            "arm_boom",
            Placement::child_of(joint, DVec3::new(2.0, 0.0, 0.0)),
            20.0,
            DVec3::splat(3.0),
        ),
    ])
    .unwrap()
}

/// The capsule's tree, air load and state at the top of its arc.
fn capsule_parts() -> (ComponentTree, Vec<AeroElement>, VehicleState) {
    let pos = Geodetic::from_degrees(28.5, -80.6, 50_000.0);
    let initial = VehicleState {
        angular_rate_body: DVec3::new(0.02, -0.05, 0.01),
        ..VehicleState::level_flight(&pos, 0.4, 300.0)
    };
    let tree = ComponentTree::new([rigid_part(
        "capsule",
        Placement::ROOT,
        2_400.0,
        DVec3::new(1_200.0, 1_500.0, 1_400.0),
    )])
    .unwrap();
    let capsule = BluntCapsuleAero::hypersonic_newtonian(
        1.2,
        2.5,
        DVec3::new(-2.0, 0.0, 0.0),
        0.1,
        1.3,
        -6.0,
        SUTTON_GRAVES_K_EARTH,
    )
    .unwrap();
    let aero = vec![AeroElement::new(
        ComponentId(0),
        AeroElementKind::BluntCapsule(capsule),
    )];
    (tree, aero, initial)
}

const CAPSULE_PERIOD: TickSpan = TickSpan::from_ticks(TICKS_PER_SECOND / 100);
const CAPSULE_EPOCH_UNIX_S: f64 = 1_700_000_000.0;

/// The capsule as a multibody vehicle integrated by `integrator`.
fn capsule_block(integrator: VehicleIntegrator) -> MultibodyVehicleBlock {
    let (tree, aero, initial) = capsule_parts();
    MultibodyVehicleBlock::new(
        MultibodyVehicleConfig {
            body: BodyEllipsoid::WGS84,
            period: CAPSULE_PERIOD,
            epoch: UtcTime::from_unix_seconds(CAPSULE_EPOCH_UNIX_S),
            initial,
            integrator,
            time_scale: MotionTimeScale::SurfaceGravity,
        },
        tree,
        aero,
        MultibodySubsystems::default(),
        Box::new(ReentryAtmosphere),
        std::sync::Arc::new(BodyEllipsoid::WGS84),
    )
    .unwrap()
}

/// The same capsule on the same arc as a rigid-body block and as a multibody
/// vehicle, both at rest in the same state.
fn capsule_pair() -> (RigidBody6DofBlock, MultibodyVehicleBlock) {
    let (tree, aero, initial) = capsule_parts();
    let rigid = RigidBody6DofBlock::new(
        RigidBodyConfig {
            body: BodyEllipsoid::WGS84,
            period: CAPSULE_PERIOD,
            epoch: UtcTime::from_unix_seconds(CAPSULE_EPOCH_UNIX_S),
            initial,
        },
        tree,
        aero,
        Box::new(ReentryAtmosphere),
        std::sync::Arc::new(BodyEllipsoid::WGS84),
    )
    .unwrap();
    (rigid, capsule_block(VehicleIntegrator::REAL_TIME))
}

/// The bus and its arm far from any field, at rest.
fn floating_arm_block() -> MultibodyVehicleBlock {
    let body = BodyEllipsoid {
        mu_m3_s2: 0.0,
        omega_rad_s: 0.0,
        ..BodyEllipsoid::WGS84
    };
    let period = TickSpan::from_ticks(TICKS_PER_SECOND / 100);
    let tree = revolute_arm_tree();
    let initial = VehicleState {
        position_ecef: DVec3::new(7_000_000.0, 0.0, 0.0),
        velocity_ecef: DVec3::ZERO,
        attitude: DQuat::IDENTITY,
        angular_rate_body: DVec3::ZERO,
    };
    MultibodyVehicleBlock::new(
        MultibodyVehicleConfig {
            body,
            period,
            epoch: UtcTime::from_unix_seconds(0.0),
            initial,
            integrator: VehicleIntegrator::REAL_TIME,
            // No field sets a time here; the arm slews in about a second.
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

#[test]
fn floating_arm_articulation_conserves_inertial_angular_momentum() {
    let mut block = floating_arm_block();
    assert!(!block.telemetry().used_rigid_fast_path);

    let mom0 = MultiBodyDynamics::system_momentum(block.tree(), &[], &block.floating_base_fixed())
        .unwrap();
    let input = MultibodyVehicleInput {
        joint_drives: vec![mbd_multibody::JointDriveInput::scalar(ComponentId(1), 15.0)],
        ..MultibodyVehicleInput::default()
    };
    for _ in 0..80 {
        block.step(&input).unwrap();
    }
    let mom1 = MultiBodyDynamics::system_momentum(block.tree(), &[], &block.floating_base_fixed())
        .unwrap();
    assert!(
        (mom1.angular_about_com_world_nms - mom0.angular_about_com_world_nms).length() < 1e-9,
        "L0={:?} L1={:?}",
        mom0.angular_about_com_world_nms,
        mom1.angular_about_com_world_nms
    );
    assert!(block.floating_base_fixed().angular_vel_root_rad_s.z < -0.01);
}

fn lunar_lander_subsystems() -> MultibodySubsystems {
    let mut fluids = FluidNetwork::new();
    fluids
        .add_tank(
            TankSpec {
                id: TankId(0),
                component: ComponentId(0),
                center_local_m: DVec3::new(0.0, 0.0, -0.5),
                geometry: TankGeometry::Sphere { radius_m: 0.7 },
                species: FluidSpecies::Mmh,
                fluid_density_kg_m3: 900.0,
                capacity_mass_kg: 500.0,
                minimum_unusable_kg: 2.0,
                feed_regime: FeedRegime::BladderOrDiaphragm,
                slosh: None,
                passive_heat_leak_w: 0.0,
            },
            TankState::nominal(200.0, 200_000.0, 290.0),
        )
        .unwrap();
    let engine = LiquidRocketSpec {
        site: ComponentId(0),
        mount_offset_local_m: DVec3::ZERO,
        thrust_axis_local: DVec3::NEG_Z,
        gimbal_max_rad: 0.0,
        max_vacuum_thrust_n: 12_000.0,
        isp_vac_s: 311.0,
        isp_sl_s: 300.0,
        fuel_tank: Some(TankId(0)),
        oxidizer_tank: None,
        mixture_ratio_o_f: 0.0,
        min_throttle: 0.0,
        startup_tau_s: 0.0,
        shutdown_tau_s: 0.0,
        nominal_heat_to_site_w: 0.0,
    };
    let tps = TpsAblationSpec {
        component: ComponentId(0),
        nose_radius_m: 1.0,
        shield_area_m2: 3.0,
        char_surface_temp_k: 1800.0,
        emissivity: 0.85,
        heat_of_ablation_j_kg: 1.0e7,
        backface_soak_fraction: 0.05,
        initial_ablator_mass_kg: 50.0,
        radiative_heating_coeff: 0.0,
    };
    let mut subs = MultibodySubsystems {
        fluids,
        tps_shields: vec![(tps, TpsAblationState::fresh(&tps))],
        pinned_dwell_s: 0.2,
        ..MultibodySubsystems::default()
    };
    subs.propulsion
        .liquid_rockets
        .push((engine, LiquidRocketState::default()));
    subs
}

/// A footpad for the 1,000 kg lander: four of them sink 8 mm on the Moon and
/// 49 mm on the Earth, at 0.7 of critical damping.
fn footpad_spec(offset_local_m: DVec3) -> ContactPointSpec {
    ContactPointSpec {
        component: ComponentId(0),
        offset_local_m,
        stiffness_n_m: 5.0e4,
        damping_n_s_m: 5.0e3,
        mu_static: 0.8,
        mu_dynamic: 0.6,
        bristle_stiffness_n_m: 5.0e4,
        bristle_damping_n_s_m: 2.0e3,
        damping_ramp_depth_m: ContactPointSpec::DEFAULT_DAMPING_RAMP_DEPTH_M,
    }
}

/// Four footpads 1.5 m out and 1 m below the root of the lander.
fn footpad_set() -> Vec<(ContactPointSpec, ContactPointState)> {
    [(1.5, 0.0), (-1.5, 0.0), (0.0, 1.5), (0.0, -1.5)]
        .into_iter()
        .map(|(x, y)| {
            (
                footpad_spec(DVec3::new(x, y, 1.0)),
                ContactPointState::default(),
            )
        })
        .collect()
}

/// The lander level over `pos` on `body`, body `+z` down, free to move.
fn footpad_lander(
    body: BodyEllipsoid,
    pos: Geodetic,
    contact_points: Vec<(ContactPointSpec, ContactPointState)>,
) -> MultibodyVehicleBlock {
    footpad_lander_with(body, pos, contact_points, VehicleIntegrator::REAL_TIME)
}

fn footpad_lander_with(
    body: BodyEllipsoid,
    pos: Geodetic,
    contact_points: Vec<(ContactPointSpec, ContactPointState)>,
    integrator: VehicleIntegrator,
) -> MultibodyVehicleBlock {
    let period = TickSpan::from_ticks(TICKS_PER_SECOND / 50);
    let pos_fixed = body.geodetic_to_fixed(&pos);
    let frame = LocalFrame::at(&pos);
    let attitude =
        DQuat::from_mat3(&DMat3::from_cols(frame.north(), frame.east(), -frame.up())).normalize();
    let initial = VehicleState {
        position_ecef: pos_fixed,
        velocity_ecef: DVec3::ZERO,
        attitude,
        angular_rate_body: DVec3::ZERO,
    };
    let tree = ComponentTree::new([rigid_part(
        "lander",
        Placement::ROOT,
        1_000.0,
        DVec3::splat(400.0),
    )])
    .unwrap();
    let subsystems = MultibodySubsystems {
        contact_points,
        ..lunar_lander_subsystems()
    };
    MultibodyVehicleBlock::new(
        MultibodyVehicleConfig {
            body,
            period,
            epoch: UtcTime::from_unix_seconds(0.0),
            initial,
            integrator,
            time_scale: MotionTimeScale::SurfaceGravity,
        },
        tree,
        Vec::new(),
        subsystems,
        Box::new(VacuumAtmosphere),
        std::sync::Arc::new(body),
    )
    .unwrap()
}

fn lunar_lander_block(pos: Geodetic) -> MultibodyVehicleBlock {
    let block = footpad_lander(BodyEllipsoid::MOON_IAU, pos, Vec::new());
    let start = block.output();
    block.with_pinned_rest(PinnedSurfaceRest::Pinned {
        world_pos_m: start.position_ecef,
        world_rot: start.attitude,
    })
}

#[test]
fn snapshot_restore_replays_identical_and_tps_plus_pinned_breakout_work() {
    let mut block = lunar_lander_block(Geodetic::from_degrees(0.0, 0.0, 1.0));

    // Zero thrust stays pinned cleanly without jitter.
    block.step(&MultibodyVehicleInput::default()).unwrap();
    assert!(block.telemetry().is_pinned_rest);
    let snap = block.snapshot();

    // Ignite main engine (> lunar weight ~1940 N) to break free and climb!
    let burn = MultibodyVehicleInput {
        propulsion: PropulsionCommands {
            liquid_throttles: vec![1.0],
            ..PropulsionCommands::default()
        },
        ..MultibodyVehicleInput::default()
    };
    for _ in 0..25 {
        block.step(&burn).unwrap();
    }
    assert!(!block.telemetry().is_pinned_rest);
    assert!(block.telemetry().total_propellant_kg < 200.0);
    let end1 = block.output();

    block.restore(&snap);
    for _ in 0..25 {
        block.step(&burn).unwrap();
    }
    assert_eq!(block.output(), end1);
}

#[test]
fn detached_site_kinematics_is_rejected() {
    let mut block = MultibodyVehicleBlock::new(
        MultibodyVehicleConfig {
            body: BodyEllipsoid::WGS84,
            period: TickSpan::from_ticks(TICKS_PER_SECOND / 50),
            epoch: UtcTime::from_unix_seconds(0.0),
            initial: VehicleState {
                position_ecef: DVec3::new(7_000_000.0, 0.0, 0.0),
                velocity_ecef: DVec3::ZERO,
                attitude: DQuat::IDENTITY,
                angular_rate_body: DVec3::ZERO,
            },
            integrator: VehicleIntegrator::REAL_TIME,
            time_scale: MotionTimeScale::SurfaceGravity,
        },
        revolute_arm_tree(),
        Vec::new(),
        MultibodySubsystems::default(),
        Box::new(VacuumAtmosphere),
        std::sync::Arc::new(BodyEllipsoid::WGS84),
    )
    .unwrap();
    let mut states = block.tree().states().to_vec();
    if let Some(st) = states.get_mut(1) {
        st.attached = false;
    }
    block.set_component_states(&states).unwrap();
    assert!(matches!(
        block.site_kinematics(ComponentId(1)),
        Err(MbdError::DetachedComponent { id }) if id == ComponentId(1)
    ));
}
