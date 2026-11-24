//! Spacecraft models, orbital test world, and physical invariants for
//! independent carried-spin verification tests.

use std::sync::Arc;

use glam::{DMat3, DQuat, DVec3, EulerRot};
use mbd_core::{ComponentId, ComponentSpec, ComponentState, ComponentTree, JointSpec, Placement};
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
use sim_geo::UtcTime;
use sim_orbital::{
    BodyOrientationCatalog, BuiltinSolarSystem2026_2028, ContinuousPlantState, Dop853Tolerances,
    EphemerisCatalog, ForceModel, NaifId, VesselId,
};
use sim_spacetime::SimEpoch;
use sim_universe::{
    LocalPlant, MultibodyLocalPlant, OrbitalContext, Tier, Universe, UniverseError, to_body_fixed,
};

pub const TEST_VESSEL: VesselId = VesselId(77);
pub const BUS: ComponentId = ComponentId(0);
pub const APPENDAGE: ComponentId = ComponentId(1);
pub const TANK: TankId = TankId(1);
pub const BLOCK_PERIOD: TickSpan = TickSpan::from_ticks(2 * TICKS_PER_SECOND);
pub const CLASSIC_BLOCK_PERIOD: TickSpan = TickSpan::from_ticks(60 * TICKS_PER_SECOND);
const START_UTC_UNIX_S: f64 = 1_792_281_600.0;
const ORBIT_RADIUS_M: f64 = 6_978_137.0;

#[derive(Clone, Debug)]
pub struct CraftSpec {
    pub block_period: TickSpan,
    pub carry_period: TickSpan,
    pub bus_mass_kg: f64,
    pub bus_inertia_body: DMat3,
    pub appendage_mass_kg: f64,
    pub appendage_inertia_body: DMat3,
    pub appendage_offset_m: DVec3,
    pub joint_axis: Option<DVec3>,
    pub min_rad: f64,
    pub max_rad: f64,
    pub thruster_offset_m: Option<DVec3>,
    pub xenon_kg: f64,
    pub rate_body_rad_s: DVec3,
    pub integrator: VehicleIntegrator,
}

impl CraftSpec {
    /// Asymmetric bus + tilted/offset appendage (`delta_i_eff != 0`, moderate
    /// asymmetry avoiding separatrix tumbling).
    pub fn asymmetric_coupled(rate_body_rad_s: DVec3, integrator: VehicleIntegrator) -> Self {
        let r_bus = DQuat::from_euler(EulerRot::XYZ, 0.35, -0.25, 0.40);
        let bus_i = rot_diag(r_bus, DVec3::new(180.0, 290.0, 230.0));
        let axis = DVec3::new(0.20, -0.10, 0.974_679_434_480_896_3).normalize();
        let r_ant = DQuat::from_rotation_arc(DVec3::Z, axis);
        let ant_i = rot_diag(r_ant, DVec3::new(2.5, 2.55, 4.0));
        Self {
            block_period: BLOCK_PERIOD,
            carry_period: TickSpan::from_ticks(300 * TICKS_PER_SECOND),
            bus_mass_kg: 450.0,
            bus_inertia_body: bus_i,
            appendage_mass_kg: 24.0,
            appendage_inertia_body: ant_i,
            appendage_offset_m: DVec3::new(0.01, -0.01, -1.2),
            joint_axis: Some(axis),
            min_rad: -100.0,
            max_rad: 100.0,
            thruster_offset_m: None,
            xenon_kg: 0.0,
            rate_body_rad_s,
            integrator,
        }
    }

    /// Heavy appendage where `(M_bb - I_eff) / M_bb` is ~0.25.
    pub fn heavy_appendage(rate_body_rad_s: DVec3, integrator: VehicleIntegrator) -> Self {
        let r_bus = DQuat::from_euler(EulerRot::XYZ, 0.18, -0.12, 0.25);
        let bus_i = rot_diag(r_bus, DVec3::new(75.0, 115.0, 95.0));
        let axis = DVec3::new(0.15, 0.10, 0.983_615_778_645_299_9).normalize();
        let r_ant = DQuat::from_rotation_arc(DVec3::Z, axis);
        let ant_i = rot_diag(r_ant, DVec3::new(22.0, 22.0, 32.0));
        Self {
            block_period: TickSpan::from_ticks(10 * TICKS_PER_SECOND),
            carry_period: TickSpan::from_ticks(120 * TICKS_PER_SECOND),
            bus_mass_kg: 200.0,
            bus_inertia_body: bus_i,
            appendage_mass_kg: 45.0,
            appendage_inertia_body: ant_i,
            appendage_offset_m: DVec3::new(0.0, 0.0, -1.0),
            joint_axis: Some(axis),
            min_rad: -100.0,
            max_rad: 100.0,
            thruster_offset_m: None,
            xenon_kg: 0.0,
            rate_body_rad_s,
            integrator,
        }
    }

    /// Matches the `articulated_transfer` geometry for direct drift checks across spin rates.
    pub fn classic_gimbal(rate_body_rad_s: DVec3, integrator: VehicleIntegrator) -> Self {
        Self {
            block_period: CLASSIC_BLOCK_PERIOD,
            carry_period: TickSpan::from_ticks(600 * TICKS_PER_SECOND),
            bus_mass_kg: 480.0,
            bus_inertia_body: DMat3::from_diagonal(DVec3::new(200.0, 300.0, 250.0)),
            appendage_mass_kg: 12.0,
            appendage_inertia_body: DMat3::from_diagonal(DVec3::new(1.5, 1.5, 2.0)),
            appendage_offset_m: DVec3::new(0.0, 0.0, -1.2),
            joint_axis: Some(DVec3::Z),
            min_rad: -100.0,
            max_rad: 100.0,
            thruster_offset_m: None,
            xenon_kg: 0.0,
            rate_body_rad_s,
            integrator,
        }
    }
}

fn rot_diag(q: DQuat, diag: DVec3) -> DMat3 {
    let r = DMat3::from_quat(q.normalize());
    r * DMat3::from_diagonal(diag) * r.transpose()
}

fn part(
    name: &str,
    placement: Placement,
    mass_kg: f64,
    inertia: DMat3,
    joint: JointSpec,
) -> (ComponentSpec, ComponentState) {
    (
        ComponentSpec {
            name: name.to_owned(),
            placement,
            dry_mass_kg: mass_kg,
            dry_inertia_body: inertia,
            thermal_capacity_j_k: 4.0e5,
            nominal_power_w: 0.0,
            joint,
        },
        ComponentState::at_temperature(290.0),
    )
}

fn build_tree(spec: &CraftSpec) -> ComponentTree {
    let bus = part(
        "bus",
        Placement::ROOT,
        spec.bus_mass_kg,
        spec.bus_inertia_body,
        JointSpec::FIXED,
    );
    let joint = match spec.joint_axis {
        Some(axis) => JointSpec::revolute(axis, spec.min_rad, spec.max_rad),
        None => JointSpec::FIXED,
    };
    let app = part(
        "appendage",
        Placement::child_of(BUS, spec.appendage_offset_m),
        spec.appendage_mass_kg,
        spec.appendage_inertia_body,
        joint,
    );
    ComponentTree::new([bus, app]).unwrap()
}

fn build_fluids(spec: &CraftSpec) -> FluidNetwork {
    let mut fluids = FluidNetwork::new();
    fluids
        .add_tank(
            TankSpec {
                id: TANK,
                component: BUS,
                center_local_m: DVec3::new(-0.15, 0.10, 0.20),
                geometry: TankGeometry::Sphere { radius_m: 0.25 },
                species: FluidSpecies::Xenon,
                fluid_density_kg_m3: FluidSpecies::Xenon.nominal_density_kg_m3(),
                capacity_mass_kg: 40.0,
                minimum_unusable_kg: 0.5,
                feed_regime: FeedRegime::BladderOrDiaphragm,
                slosh: None,
                passive_heat_leak_w: 0.0,
            },
            TankState::nominal(spec.xenon_kg, 250_000.0, 290.0),
        )
        .unwrap();
    fluids
}

fn build_electrical() -> ElectricalBus {
    let mut electrical = ElectricalBus::new(100.0).unwrap();
    electrical
        .add_solar_array(
            SolarArraySpec {
                component: BUS,
                area_m2: 18.0,
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
        .unwrap();
    let bat = BatterySpec {
        component: BUS,
        capacity_wh: 2_500.0,
        nominal_voltage_v: 100.0,
        max_charge_w: 3_000.0,
        max_discharge_w: 4_000.0,
        charge_efficiency: 0.95,
        discharge_efficiency: 0.95,
        min_soc: 0.1,
        internal_resistance_ohm: 0.04,
    };
    electrical
        .add_battery(bat, BatteryState::full(&bat))
        .unwrap();
    electrical
}

fn build_subsystems(spec: &CraftSpec) -> MultibodySubsystems {
    let Some(mount_offset) = spec.thruster_offset_m else {
        return MultibodySubsystems::default();
    };
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
        .unwrap();
    let mut sub = MultibodySubsystems {
        fluids: build_fluids(spec),
        electrical: build_electrical(),
        thermal,
        ..MultibodySubsystems::default()
    };
    sub.propulsion.electric_thrusters.push((
        ElectricThrusterSpec {
            site: BUS,
            mount_offset_local_m: mount_offset,
            thrust_axis_local: DVec3::X,
            propellant_tank: TANK,
            isp_s: 2_000.0,
            total_efficiency: 0.6,
            max_power_w: 2_000.0,
            priority: 1,
        },
        ElectricThrusterState::default(),
    ));
    sub
}

pub struct ReferenceWorld {
    pub universe: Universe,
    pub orientation: Arc<dyn BodyOrientationCatalog>,
    pub epoch: SimEpoch,
}

impl ReferenceWorld {
    pub fn spawn(spec: &CraftSpec) -> Self {
        Self::spawn_with_tolerances(
            spec,
            Dop853Tolerances {
                rtol: 1.0e-12,
                atol_att_rad: 1.0e-13,
                atol_omega_rad_s: 1.0e-15,
                ..Dop853Tolerances::default()
            },
        )
    }

    pub fn spawn_with_tolerances(spec: &CraftSpec, tolerances: Dop853Tolerances) -> Self {
        super::ensure_logger();
        let solar = BuiltinSolarSystem2026_2028::load_verified().unwrap();
        let force_model = ForceModel::standard_solar_system(solar.constants.clone()).unwrap();
        let mu_earth = force_model.constants().require_mu(NaifId::EARTH).unwrap();
        let ephemeris: Arc<dyn EphemerisCatalog> = Arc::new(solar.ephemeris);
        let orientation: Arc<dyn BodyOrientationCatalog> = Arc::new(solar.orientation);
        let epoch =
            SimEpoch::builtin_from_utc(UtcTime::from_unix_seconds(START_UTC_UNIX_S)).unwrap();
        let context = OrbitalContext {
            ephemeris,
            orientation: Arc::clone(&orientation),
            force_model,
            quantum: TickSpan::from_ticks(60 * TICKS_PER_SECOND),
            tolerances,
        };
        let mut universe = Universe::new(epoch.clone(), context);
        let start = epoch.tdb_at(SimInstant::START);
        let initial_icrf = ContinuousPlantState {
            primary_body: NaifId::EARTH,
            pos_rel_icrf_m: DVec3::X * ORBIT_RADIUS_M,
            vel_rel_icrf_m_s: DVec3::Y * (mu_earth / ORBIT_RADIUS_M).sqrt(),
            q_body_to_icrf: DQuat::IDENTITY,
            omega_body_rad_s: spec.rate_body_rad_s,
            propellant_mass_kg: 0.0,
            h_rw_body_nms: DVec3::ZERO,
        };
        let rotation = orientation.body_rotation(NaifId::EARTH, start).unwrap();
        let state = to_body_fixed(&initial_icrf, NaifId::EARTH, &rotation)
            .unwrap()
            .0;
        let earth = universe.bodies().get(NaifId::EARTH).unwrap().clone();
        let plant = build_plant(&earth, &state, spec, &epoch);
        universe
            .insert_local(
                TEST_VESSEL,
                Box::new(plant),
                Box::new(|_, _, _, _| Err(UniverseError::NotCarryable)),
            )
            .unwrap();
        Self {
            universe,
            orientation,
            epoch,
        }
    }

    pub fn ship(&self) -> &MultibodyLocalPlant {
        self.universe
            .carried::<MultibodyLocalPlant>(TEST_VESSEL)
            .or_else(|| self.universe.local::<MultibodyLocalPlant>(TEST_VESSEL))
            .unwrap()
    }

    pub fn ship_mut(&mut self) -> &mut MultibodyLocalPlant {
        if self
            .universe
            .carried::<MultibodyLocalPlant>(TEST_VESSEL)
            .is_some()
        {
            return self.universe.carried_mut(TEST_VESSEL).unwrap();
        }
        self.universe.local_mut(TEST_VESSEL).unwrap()
    }

    pub fn joint_state(&self) -> (f64, f64) {
        let j = self.ship().block().tree().state(APPENDAGE).unwrap().joint;
        (j.q_scalar, j.q_vel_scalar)
    }

    /// Instantaneous ICRF angular momentum using the catalog rotation at `now`.
    pub fn angular_momentum_icrf(&mut self) -> DVec3 {
        let tier = self.universe.tier(TEST_VESSEL).unwrap();
        let now = self.universe.now();
        let tdb = self.epoch.tdb_at(now);
        let rot = self.orientation.body_rotation(NaifId::EARTH, tdb).unwrap();
        match tier {
            Tier::Orbital => {
                let orb = self.universe.inertial_state(TEST_VESSEL).unwrap();
                let (i_orb, h_held) = self
                    .universe
                    .orbital_held_rotational_state_body(TEST_VESSEL)
                    .unwrap();
                orb.q_body_to_icrf * (i_orb * orb.omega_body_rad_s + h_held)
            }
            Tier::Local => {
                let state = self.ship().state();
                let q_icrf = (DQuat::from_mat3(&rot.r_icrf_to_fixed.transpose()) * state.attitude)
                    .normalize();
                let w_inertial =
                    state.angular_rate_body + state.attitude.conjugate() * rot.omega_body_rad_s;
                let block = self.ship_mut().block_mut();
                let m_bb = block.mass_properties().inertia_cg_body;
                let internal = block.internal_momentum_cg_body().unwrap();
                q_icrf * (m_bb * w_inertial + internal)
            }
        }
    }

    /// Inertial angular momentum in `BodyEllipsoid`'s exact inertial frame
    /// (for checking local-tier conservation without IAU pole precession).
    pub fn local_ellipsoid_momentum_icrf(&mut self) -> DVec3 {
        let now = self.universe.now();
        let state = self.ship().state();
        let body = self.ship().body();
        let elapsed_s = now.duration_since(SimInstant::START).as_seconds();
        let rot0 = self
            .orientation
            .body_rotation(NaifId::EARTH, self.epoch.tdb_at(SimInstant::START))
            .unwrap();
        let q_fixed_to_icrf0 = DQuat::from_mat3(&rot0.r_icrf_to_fixed.transpose());
        let q_body_to_inertial =
            DQuat::from_axis_angle(DVec3::Z, body.omega_rad_s * elapsed_s) * state.attitude;
        let q_icrf = (q_fixed_to_icrf0 * q_body_to_inertial).normalize();
        let w_inertial =
            state.angular_rate_body + state.attitude.conjugate() * (DVec3::Z * body.omega_rad_s);
        let block = self.ship_mut().block_mut();
        let m_bb = block.mass_properties().inertia_cg_body;
        let internal = block.internal_momentum_cg_body().unwrap();
        q_icrf * (m_bb * w_inertial + internal)
    }

    /// Exact multibody inertial angular momentum evaluated from the plant's own
    /// joint state and the orbital base state (valid at exchange instants).
    pub fn plant_multibody_momentum_icrf(&mut self) -> DVec3 {
        let orb = self.universe.inertial_state(TEST_VESSEL).unwrap();
        let block = self.ship_mut().block_mut();
        let m_bb = block.mass_properties().inertia_cg_body;
        let internal = block.internal_momentum_cg_body().unwrap();
        orb.q_body_to_icrf * (m_bb * orb.omega_body_rad_s + internal)
    }

    pub fn xenon_kg(&self) -> f64 {
        self.ship()
            .block()
            .subsystems()
            .fluids
            .tank_state(TANK)
            .unwrap()
            .mass_kg
    }
}

fn build_plant(
    earth: &BodyDescriptor,
    state: &VehicleState,
    spec: &CraftSpec,
    epoch: &SimEpoch,
) -> MultibodyLocalPlant {
    let block = MultibodyVehicleBlock::new(
        MultibodyVehicleConfig {
            body: earth.ellipsoid,
            period: spec.block_period,
            epoch: epoch.utc_at(SimInstant::START).unwrap(),
            initial: *state,
            integrator: spec.integrator,
            time_scale: MotionTimeScale::SurfaceGravity,
        },
        build_tree(spec),
        Vec::new(),
        build_subsystems(spec),
        Box::new(VacuumAtmosphere),
        earth.fields().gravity,
    )
    .unwrap();
    MultibodyLocalPlant::new(block, earth.ellipsoid, SimInstant::START, 3.0)
        .with_carry_period(spec.carry_period)
}

pub fn at_s(seconds: u64) -> SimInstant {
    SimInstant::START.advanced(TickSpan::from_ticks(seconds * TICKS_PER_SECOND))
}

pub fn motor_input(
    base: &MultibodyVehicleInput,
    throttle: f64,
    torque_nm: f64,
) -> MultibodyVehicleInput {
    MultibodyVehicleInput {
        propulsion: PropulsionCommands {
            electric_throttles: vec![throttle],
            ..PropulsionCommands::default()
        },
        joint_drives: if torque_nm == 0.0 {
            Vec::new()
        } else {
            vec![sim_core::JointDriveInput::scalar(APPENDAGE, torque_nm)]
        },
        ..base.clone()
    }
}
