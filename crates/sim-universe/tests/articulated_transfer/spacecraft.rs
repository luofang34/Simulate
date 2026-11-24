//! An articulated interplanetary spacecraft built in code: a bus with a
//! xenon tank, a battery, a solar array, an electric thruster and one
//! thermal node, carrying a high-gain antenna on a driven revolute gimbal.

use glam::{DMat3, DVec3};
use mbd_core::{ComponentId, ComponentSpec, ComponentState, ComponentTree, JointSpec, Placement};
use mbd_multibody::JointDriveInput;
use mbd_systems::{
    BatterySpec, BatteryState, ElectricThrusterSpec, ElectricThrusterState, ElectricalBus,
    FeedRegime, FluidNetwork, FluidSpecies, SolarArraySpec, SolarArrayState, TankGeometry, TankId,
    TankSpec, TankState, ThermalNetwork, ThermalNodeSpec,
};
use sim_atmosphere::VacuumAtmosphere;
use sim_bodies::BodyDescriptor;
use sim_core::{
    MotionTimeScale, MultibodySubsystems, MultibodyVehicleBlock, MultibodyVehicleConfig,
    MultibodyVehicleInput, PropulsionCommands, SimInstant, TICKS_PER_SECOND, TickSpan,
    VehicleIntegrator, VehicleState,
};
use sim_spacetime::SimEpoch;
use sim_universe::{MultibodyLocalPlant, UniverseError};

/// The plant's step. A minute resolves the slow gimbal loop and keeps a
/// voyage of months to a few hundred thousand carried steps.
pub const BLOCK_PERIOD: TickSpan = TickSpan::from_ticks(60 * TICKS_PER_SECOND);
/// How often the plant exchanges with the orbital tier.
pub const CARRY_PERIOD: TickSpan = TickSpan::from_ticks(3_600 * TICKS_PER_SECOND);
pub const BUS: ComponentId = ComponentId(0);
pub const ANTENNA: ComponentId = ComponentId(1);
pub const TANK: TankId = TankId(1);
pub const XENON_KG: f64 = 20.0;
/// Antenna inertia about the gimbal axis, kg·m².
pub const ANTENNA_INERTIA_KG_M2: f64 = 2.0;
/// Natural frequency of the gimbal's position loop, rad/s: a quarter of a
/// radian per plant step, so the sampled loop behaves as the continuous one.
pub const GIMBAL_LOOP_RAD_S: f64 = 0.004;

fn part(
    name: &str,
    placement: Placement,
    mass_kg: f64,
    inertia: DVec3,
    joint: JointSpec,
) -> (ComponentSpec, ComponentState) {
    (
        ComponentSpec {
            name: name.to_owned(),
            placement,
            dry_mass_kg: mass_kg,
            dry_inertia_body: DMat3::from_diagonal(inertia),
            thermal_capacity_j_k: 4.0e5,
            nominal_power_w: 0.0,
            joint,
        },
        ComponentState::at_temperature(290.0),
    )
}

fn tree() -> Result<ComponentTree, UniverseError> {
    ComponentTree::new([
        part(
            "bus",
            Placement::ROOT,
            480.0,
            DVec3::new(200.0, 300.0, 250.0),
            JointSpec::FIXED,
        ),
        part(
            "antenna",
            Placement::child_of(BUS, DVec3::new(0.0, 0.0, -1.2)),
            12.0,
            DVec3::new(1.5, 1.5, ANTENNA_INERTIA_KG_M2),
            JointSpec::revolute(DVec3::Z, -2.5, 2.5),
        ),
    ])
    .map_err(local)
}

fn local(error: impl std::error::Error + Send + Sync + 'static) -> UniverseError {
    UniverseError::LocalPlant(Box::new(error))
}

fn fluids() -> Result<FluidNetwork, UniverseError> {
    let mut fluids = FluidNetwork::new();
    fluids
        .add_tank(
            TankSpec {
                id: TANK,
                component: BUS,
                center_local_m: DVec3::ZERO,
                geometry: TankGeometry::Sphere { radius_m: 0.25 },
                species: FluidSpecies::Xenon,
                fluid_density_kg_m3: FluidSpecies::Xenon.nominal_density_kg_m3(),
                capacity_mass_kg: 40.0,
                minimum_unusable_kg: 0.5,
                feed_regime: FeedRegime::BladderOrDiaphragm,
                slosh: None,
                passive_heat_leak_w: 0.0,
            },
            TankState::nominal(XENON_KG, 250_000.0, 290.0),
        )
        .map_err(local)?;
    Ok(fluids)
}

fn electrical() -> Result<ElectricalBus, UniverseError> {
    let mut electrical = ElectricalBus::new(100.0).map_err(local)?;
    electrical
        .add_solar_array(
            SolarArraySpec {
                component: BUS,
                area_m2: 12.0,
                normal_local: DVec3::Y,
                efficiency: 0.3,
                packing_factor: 1.0,
                annual_degradation: 0.0,
                nominal_voltage_v: 100.0,
            },
            SolarArrayState {
                deployed_fraction: 1.0,
                ..SolarArrayState::default()
            },
        )
        .map_err(local)?;
    let battery = BatterySpec {
        component: BUS,
        capacity_wh: 600.0,
        nominal_voltage_v: 100.0,
        max_charge_w: 2_000.0,
        max_discharge_w: 3_000.0,
        charge_efficiency: 0.95,
        discharge_efficiency: 0.95,
        min_soc: 0.2,
        internal_resistance_ohm: 0.05,
    };
    electrical
        .add_battery(battery, BatteryState::full(&battery))
        .map_err(local)?;
    Ok(electrical)
}

fn thermal() -> Result<ThermalNetwork, UniverseError> {
    let mut thermal = ThermalNetwork::new();
    thermal
        .add_node(
            ThermalNodeSpec {
                component: BUS,
                heat_capacity_j_k: 4.0e5,
                radiative_area_m2: 6.0,
                emissivity: 0.8,
                solar_absorptivity: 0.3,
                solar_area_m2: 2.0,
                convective_area_m2: 0.0,
                min_operating_temp_k: 100.0,
                max_operating_temp_k: 400.0,
            },
            290.0,
        )
        .map_err(local)?;
    Ok(thermal)
}

fn subsystems() -> Result<MultibodySubsystems, UniverseError> {
    let mut subsystems = MultibodySubsystems {
        fluids: fluids()?,
        electrical: electrical()?,
        thermal: thermal()?,
        ..MultibodySubsystems::default()
    };
    subsystems.propulsion.electric_thrusters.push((
        ElectricThrusterSpec {
            site: BUS,
            mount_offset_local_m: DVec3::ZERO,
            thrust_axis_local: DVec3::X,
            propellant_tank: TANK,
            isp_s: 2_000.0,
            total_efficiency: 0.6,
            max_power_w: 2_000.0,
            priority: 1,
        },
        ElectricThrusterState::default(),
    ));
    Ok(subsystems)
}

/// The input that runs the thruster at `throttle` and drives the gimbal
/// with `gimbal_effort_nm`.
pub fn input(
    base: &MultibodyVehicleInput,
    throttle: f64,
    gimbal_effort_nm: f64,
) -> MultibodyVehicleInput {
    MultibodyVehicleInput {
        propulsion: PropulsionCommands {
            electric_throttles: vec![throttle],
            ..PropulsionCommands::default()
        },
        joint_drives: match gimbal_effort_nm == 0.0 {
            true => Vec::new(),
            false => vec![JointDriveInput::scalar(ANTENNA, gimbal_effort_nm)],
        },
        ..base.clone()
    }
}

/// The torque a critically damped position loop asks of the gimbal motor.
pub fn gimbal_effort_nm(command_rad: f64, angle_rad: f64, rate_rad_s: f64) -> f64 {
    let stiffness = ANTENNA_INERTIA_KG_M2 * GIMBAL_LOOP_RAD_S * GIMBAL_LOOP_RAD_S;
    let damping = 2.0 * ANTENNA_INERTIA_KG_M2 * GIMBAL_LOOP_RAD_S;
    stiffness * (command_rad - angle_rad) - damping * rate_rad_s
}

/// The spacecraft at `state`, fixed to `earth`, on that entry's gravity.
pub fn plant(
    earth: &BodyDescriptor,
    state: &VehicleState,
    start: SimInstant,
    epoch: &SimEpoch,
) -> Result<MultibodyLocalPlant, UniverseError> {
    let block = MultibodyVehicleBlock::new(
        MultibodyVehicleConfig {
            body: earth.ellipsoid,
            period: BLOCK_PERIOD,
            epoch: epoch.utc_at(start)?,
            initial: *state,
            integrator: VehicleIntegrator::FixedRk4 { substeps: 6 },
            time_scale: MotionTimeScale::SurfaceGravity,
        },
        tree()?,
        Vec::new(),
        subsystems()?,
        Box::new(VacuumAtmosphere),
    )?
    .with_gravity(earth.fields().gravity);
    Ok(
        MultibodyLocalPlant::new(block, earth.ellipsoid, start, 3.0)
            .with_carry_period(CARRY_PERIOD),
    )
}
