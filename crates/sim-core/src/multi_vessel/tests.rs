use std::collections::BTreeMap;

use glam::{DMat3, DQuat, DVec3};
use mbd_core::{ComponentId, ComponentSpec, ComponentState, ComponentTree, JointSpec, Placement};
use mbd_multibody::{
    DockingPhase, DockingSpec, DockingState, MultiBodyDynamics, RailSlideSpec, RailSlideState,
};
use mbd_systems::{
    FeedRegime, FluidNetwork, FluidSpecies, TankGeometry, TankId, TankSpec, TankState,
};
use sim_atmosphere::VacuumAtmosphere;
use sim_geo::{BodyEllipsoid, UtcTime};

use super::*;

mod mechanism_reference;
use crate::multibody_vehicle::{
    MultibodySubsystems, MultibodyVehicleBlock, MultibodyVehicleConfig,
};
use crate::{MotionTimeScale, TICKS_PER_SECOND, TickSpan, VehicleIntegrator, VehicleState};

fn zero_grav_body() -> BodyEllipsoid {
    BodyEllipsoid {
        mu_m3_s2: 0.0,
        omega_rad_s: 0.0,
        ..BodyEllipsoid::WGS84
    }
}

fn part(
    name: &str,
    placement: Placement,
    joint: JointSpec,
    mass: f64,
    inertia: f64,
) -> (ComponentSpec, ComponentState) {
    (
        ComponentSpec {
            name: name.to_owned(),
            placement,
            dry_mass_kg: mass,
            dry_inertia_body: DMat3::from_diagonal(DVec3::splat(inertia)),
            thermal_capacity_j_k: 500.0,
            nominal_power_w: 0.0,
            joint,
        },
        ComponentState::at_temperature(290.0),
    )
}

fn tank_block(
    tree: ComponentTree,
    tank_comp: ComponentId,
    tank_id: u16,
    mass_kg: f64,
    pos_ecef: DVec3,
) -> MultibodyVehicleBlock {
    let mut fluids = FluidNetwork::new();
    fluids
        .add_tank(
            TankSpec {
                id: TankId(tank_id),
                component: tank_comp,
                center_local_m: DVec3::ZERO,
                geometry: TankGeometry::Sphere { radius_m: 0.5 },
                species: FluidSpecies::Hydrazine,
                fluid_density_kg_m3: 1004.0,
                capacity_mass_kg: 500.0,
                minimum_unusable_kg: 0.0,
                feed_regime: FeedRegime::BladderOrDiaphragm,
                slosh: None,
                passive_heat_leak_w: 0.0,
            },
            TankState::nominal(mass_kg, 200_000.0, 290.0),
        )
        .unwrap();
    let subs = MultibodySubsystems {
        fluids,
        ..MultibodySubsystems::default()
    };
    MultibodyVehicleBlock::new(
        MultibodyVehicleConfig {
            body: zero_grav_body(),
            period: TickSpan::from_ticks(TICKS_PER_SECOND / 50),
            epoch: UtcTime::from_unix_seconds(0.0),
            initial: VehicleState {
                position_ecef: pos_ecef,
                velocity_ecef: DVec3::ZERO,
                attitude: DQuat::IDENTITY,
                angular_rate_body: DVec3::ZERO,
            },
            integrator: VehicleIntegrator::REAL_TIME,
            // No field sets a time here; the mechanisms act over about a
            // second.
            time_scale: MotionTimeScale::Seconds(1.0),
        },
        tree,
        Vec::new(),
        subs,
        Box::new(VacuumAtmosphere),
    )
    .unwrap()
}

/// The soft-capture ring between vessel 1 and `target`, 0.2 m apart.
fn docking_link(target: SimVesselId) -> InterVesselLink {
    InterVesselLink::Docking {
        id: 1,
        chaser: SimVesselId(1),
        port_a: ComponentId(0),
        offset_a_local_m: DVec3::ZERO,
        axis_a_local: DVec3::X,
        target,
        port_b: ComponentId(0),
        offset_b_local_m: DVec3::ZERO,
        mate_placement: Placement::child_of(ComponentId(0), DVec3::ZERO),
        spec: DockingSpec {
            capture_radius_m: 0.5,
            capture_angle_rad: 0.2,
            max_closing_speed_m_s: 1.0,
            damping_vel_threshold_m_s: 0.2,
            damping_rate_threshold_rad_s: 0.1,
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
        },
        state: DockingState {
            phase: DockingPhase::Open,
            standoff_m: 0.2,
            dwell_s: 0.0,
        },
    }
}

/// A 2,000 kg airframe with a 100 kg seat on a 2 m rail along `+z`.
fn rail_carrier() -> MultibodyVehicleBlock {
    let rail = JointSpec::prismatic(DVec3::Z, -0.1, 2.0);
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
            rail,
            100.0,
            10.0,
        ),
    ])
    .unwrap();
    tank_block(
        tree,
        ComponentId(0),
        0,
        10.0,
        DVec3::new(6_378_137.0, 0.0, 0.0),
    )
}

/// The seat's catapult, already fired.
fn rail_link() -> InterVesselLink {
    InterVesselLink::RailSlide {
        id: 1,
        vessel: SimVesselId(1),
        spec: RailSlideSpec {
            component: ComponentId(1),
            rail_length_m: 1.0,
            catapult_force_n: 4_000.0,
            guide_friction_n: 50.0,
        },
        state: RailSlideState {
            fired: true,
            ready_to_release: false,
        },
    }
}

#[test]
fn stage_separation_partitions_tanks_conserves_momentum_and_switches_focus() {
    let tree = ComponentTree::new([
        part("core", Placement::ROOT, JointSpec::FIXED, 800.0, 300.0),
        part(
            "booster",
            Placement::child_of(ComponentId(0), DVec3::new(-4.0, 0.0, 0.0)),
            JointSpec::FIXED,
            400.0,
            150.0,
        ),
    ])
    .unwrap();
    let parent = tank_block(
        tree,
        ComponentId(1),
        7,
        120.0,
        DVec3::new(7_000_000.0, 0.0, 0.0),
    );
    let mom_before = {
        let m = parent.dynamic_mass_contributions().unwrap();
        MultiBodyDynamics::system_momentum(parent.tree(), &m, &parent.floating_base_fixed())
            .unwrap()
    };

    let mut coord = MultiVesselCoordinator::new(parent);
    let child_id = coord
        .separate_stage_impulse(SimVesselId(1), ComponentId(1), 500.0, DVec3::NEG_X)
        .unwrap();
    assert_eq!(coord.vessel_ids(), vec![SimVesselId(1), child_id]);

    let p = coord.vessel(SimVesselId(1)).unwrap();
    let c = coord.vessel(child_id).unwrap();
    assert!((p.telemetry().total_propellant_kg).abs() < 1e-12);
    assert!((c.telemetry().total_propellant_kg - 120.0).abs() < 1e-12);

    let mp = MultiBodyDynamics::system_momentum(
        p.tree(),
        &p.dynamic_mass_contributions().unwrap(),
        &p.floating_base_fixed(),
    )
    .unwrap();
    let mc = MultiBodyDynamics::system_momentum(
        c.tree(),
        &c.dynamic_mass_contributions().unwrap(),
        &c.floating_base_fixed(),
    )
    .unwrap();
    assert!(
        ((mp.linear_world_kg_m_s + mc.linear_world_kg_m_s) - mom_before.linear_world_kg_m_s)
            .length()
            < 1e-9
    );

    assert_eq!(coord.cycle_active_vessel(), child_id);
    assert_eq!(coord.cycle_active_vessel(), SimVesselId(1));
}

#[test]
fn docking_soft_capture_converges_to_hard_mate_and_merges_colliding_tanks() {
    let chaser = tank_block(
        ComponentTree::new([part(
            "chaser",
            Placement::ROOT,
            JointSpec::FIXED,
            1_000.0,
            400.0,
        )])
        .unwrap(),
        ComponentId(0),
        0,
        50.0,
        DVec3::new(7_000_000.0, 0.0, 0.0),
    );
    let target = tank_block(
        ComponentTree::new([part(
            "target",
            Placement::ROOT,
            JointSpec::FIXED,
            800.0,
            300.0,
        )])
        .unwrap(),
        ComponentId(0),
        0,
        30.0,
        DVec3::new(7_000_000.2, 0.0, 0.0),
    );
    let mut coord = MultiVesselCoordinator::new(chaser);
    let target_id = coord.spawn_independent(target);
    coord.add_link(docking_link(target_id));

    let mut mated = false;
    for _ in 0..80 {
        let rep = coord.step_all(&BTreeMap::new()).unwrap();
        if !rep.merged_vessels.is_empty() {
            mated = true;
            break;
        }
    }
    assert!(mated, "expected soft capture to retract and hard-mate");
    assert_eq!(coord.vessel_ids(), vec![SimVesselId(1)]);
    let merged = coord.vessel(SimVesselId(1)).unwrap();
    assert!((merged.telemetry().total_propellant_kg - 80.0).abs() < 1e-12);
}

#[test]
fn rail_slide_ejection_fires_along_guide_and_spawns_free_seat() {
    let mut coord = MultiVesselCoordinator::new(rail_carrier());
    coord.add_link(rail_link());

    let mut ejected = None;
    for _ in 0..40 {
        let rep = coord.step_all(&BTreeMap::new()).unwrap();
        if let Some(&(_, child)) = rep.spawned_vessels.first() {
            ejected = Some(child);
            break;
        }
    }
    assert!(ejected.is_some(), "expected rail seat ejection release");
    assert_eq!(coord.vessel_ids().len(), 2);
}
