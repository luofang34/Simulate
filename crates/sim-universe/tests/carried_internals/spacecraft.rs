//! An electric-propulsion spacecraft built in code, and the orbit it flies:
//! one body with a xenon tank, a bus with a battery and a solar array, a
//! thruster through the centre of gravity, and one thermal node.
//!
//! The orbit is laid out from the Sun's direction at the start: the array
//! faces along the orbit normal, 45° from the Sun, so it produces steady
//! power in sunlight; the vessel turns once per orbit about that normal, so
//! its thruster stays along the velocity; and it starts at orbital noon, so
//! the first eclipse comes a third of an orbit in.

use std::cell::Cell;
use std::rc::Rc;
use std::sync::Arc;

use glam::{DMat3, DQuat, DVec3};
use mbd_core::{
    ComponentId, ComponentSpec, ComponentState, ComponentTree, JointSpec, Placement, VacuumAero,
};
use mbd_systems::{
    BatterySpec, BatteryState, ElectricThrusterSpec, ElectricThrusterState, ElectricalBus,
    FeedRegime, FluidNetwork, FluidSpecies, SolarArraySpec, SolarArrayState, TankGeometry, TankId,
    TankSpec, TankState, ThermalNetwork, ThermalNodeSpec,
};
use sim_atmosphere::VacuumAtmosphere;
use sim_core::{
    MultibodySubsystems, MultibodyVehicleBlock, MultibodyVehicleConfig, MultibodyVehicleInput,
    PropulsionCommands, SimInstant, TICKS_PER_SECOND, TickSpan, VehicleState,
};
use sim_geo::{BodyEllipsoid, UtcTime};
use sim_orbital::{
    BuiltinSolarSystem2026_2028, ContinuousPlantState, DerivativeOrder, Dop853Tolerances,
    ForceModel, NaifId,
};
use sim_spacetime::SimEpoch;
use sim_universe::{
    LocalFactory, LocalPlant, MultibodyLocalPlant, OrbitalContext, UniverseError, to_body_fixed,
};

/// `2027-03-01T00:00:00Z`.
const START_UTC_UNIX_S: f64 = 1_803_859_200.0;
pub const BLOCK_PERIOD: TickSpan = TickSpan::from_ticks(TICKS_PER_SECOND);
pub const CARRY_PERIOD: TickSpan = TickSpan::from_seconds_and_subsec_ticks(10, 0);
pub const TANK: TankId = TankId(1);
pub const ROOT: ComponentId = ComponentId(0);
pub const DRY_MASS_KG: f64 = 480.0;
pub const XENON_KG: f64 = 20.0;
pub const THRUSTER_POWER_W: f64 = 2_000.0;
pub const THRUSTER_EFFICIENCY: f64 = 0.6;
pub const EXHAUST_VELOCITY_M_S: f64 = 2_000.0 * 9.806_65;
/// Thrust at full power, `2 η P / v_e`.
pub const FULL_THRUST_N: f64 = 2.0 * THRUSTER_EFFICIENCY * THRUSTER_POWER_W / EXHAUST_VELOCITY_M_S;
pub const BATTERY_CAPACITY_WH: f64 = 600.0;
pub const BATTERY_MIN_SOC: f64 = 0.2;
pub const ORBIT_RADIUS_M: f64 = 6_778_137.0;

/// Where and when the scenario starts.
pub struct Launch {
    pub epoch: SimEpoch,
    pub state: VehicleState,
    /// Unit vector to the Sun in Earth-fixed axes at the start.
    pub sun_fixed: DVec3,
}

#[cfg(test)]
pub fn seconds(s: u64) -> SimInstant {
    SimInstant::START.advanced(TickSpan::from_seconds_and_subsec_ticks(s, 0))
}

#[cfg(test)]
pub fn context() -> OrbitalContext {
    let solar = BuiltinSolarSystem2026_2028::load_verified().unwrap();
    let force_model = ForceModel::standard_solar_system(solar.constants.clone()).unwrap();
    OrbitalContext {
        ephemeris: Arc::new(solar.ephemeris),
        orientation: Arc::new(solar.orientation),
        force_model,
        quantum: CARRY_PERIOD,
        tolerances: Dop853Tolerances {
            rtol: 1e-12,
            atol_pos_m: 1e-6,
            atol_vel_m_s: 1e-9,
            ..Dop853Tolerances::default()
        },
    }
}

#[cfg(test)]
pub fn launch() -> Launch {
    let epoch = SimEpoch::builtin_from_utc(UtcTime::from_unix_seconds(START_UTC_UNIX_S)).unwrap();
    let context = context();
    let tdb = epoch.tdb_at(SimInstant::START);
    let sun = context
        .ephemeris
        .geometric_state_icrf(NaifId::SUN, NaifId::EARTH, tdb, DerivativeOrder::Position)
        .unwrap()
        .pos_m
        .normalize();
    let across = sun.cross(DVec3::Z).normalize();
    let normal = ((sun + across) * std::f64::consts::FRAC_1_SQRT_2).normalize();
    let noon = (sun - normal * sun.dot(normal)).normalize();
    let along = normal.cross(noon);
    let speed = (BodyEllipsoid::WGS84.mu_m3_s2 / ORBIT_RADIUS_M).sqrt();
    let inertial = ContinuousPlantState {
        primary_body: NaifId::EARTH,
        pos_rel_icrf_m: noon * ORBIT_RADIUS_M,
        vel_rel_icrf_m_s: along * speed,
        // Body +X (thrust) along the velocity, +Y (array normal) along the
        // orbit normal, turning about it at the orbital rate.
        q_body_to_icrf: DQuat::from_mat3(&DMat3::from_cols(along, normal, noon)).normalize(),
        omega_body_rad_s: DVec3::Y * (speed / ORBIT_RADIUS_M),
        propellant_mass_kg: 0.0,
        h_rw_body_nms: DVec3::ZERO,
    };
    let rotation = context
        .orientation
        .body_rotation(NaifId::EARTH, tdb)
        .unwrap();
    let (state, _) = to_body_fixed(&inertial, NaifId::EARTH, &rotation).unwrap();
    Launch {
        epoch,
        state,
        sun_fixed: rotation.r_icrf_to_fixed * sun,
    }
}

#[cfg(test)]
fn tree() -> ComponentTree {
    ComponentTree::new([(
        ComponentSpec {
            name: "bus".to_owned(),
            placement: Placement::ROOT,
            dry_mass_kg: DRY_MASS_KG,
            // Largest about +Y, so the once-per-orbit turn about it is stable.
            dry_inertia_body: DMat3::from_diagonal(DVec3::new(200.0, 300.0, 250.0)),
            thermal_capacity_j_k: 4.0e5,
            nominal_power_w: 0.0,
            joint: JointSpec::FIXED,
        },
        ComponentState::at_temperature(290.0),
    )])
    .unwrap()
}

#[cfg(test)]
fn fluids() -> FluidNetwork {
    let mut fluids = FluidNetwork::new();
    fluids
        .add_tank(
            TankSpec {
                id: TANK,
                component: ROOT,
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
        .unwrap();
    fluids
}

/// An array giving about 3.5 kW at 45° to the Sun, and a battery that
/// carries the 2 kW thruster for a quarter of an hour: less than an eclipse.
#[cfg(test)]
fn electrical() -> ElectricalBus {
    let mut electrical = ElectricalBus::new(100.0).unwrap();
    electrical
        .add_solar_array(
            SolarArraySpec {
                component: ROOT,
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
        .unwrap();
    let battery = BatterySpec {
        component: ROOT,
        capacity_wh: BATTERY_CAPACITY_WH,
        nominal_voltage_v: 100.0,
        max_charge_w: 2_000.0,
        max_discharge_w: 3_000.0,
        charge_efficiency: 0.95,
        discharge_efficiency: 0.95,
        min_soc: BATTERY_MIN_SOC,
        internal_resistance_ohm: 0.05,
    };
    electrical
        .add_battery(battery, BatteryState::full(&battery))
        .unwrap();
    electrical
}

#[cfg(test)]
fn thermal() -> ThermalNetwork {
    let mut thermal = ThermalNetwork::new();
    thermal
        .add_node(
            ThermalNodeSpec {
                component: ROOT,
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
    thermal
}

#[cfg(test)]
fn subsystems() -> MultibodySubsystems {
    let mut subsystems = MultibodySubsystems {
        fluids: fluids(),
        electrical: electrical(),
        thermal: thermal(),
        ..MultibodySubsystems::default()
    };
    subsystems.propulsion.electric_thrusters.push((
        ElectricThrusterSpec {
            site: ROOT,
            mount_offset_local_m: DVec3::ZERO,
            thrust_axis_local: DVec3::X,
            propellant_tank: TANK,
            isp_s: EXHAUST_VELOCITY_M_S / 9.806_65,
            total_efficiency: THRUSTER_EFFICIENCY,
            max_power_w: THRUSTER_POWER_W,
            priority: 1,
        },
        ElectricThrusterState::default(),
    ));
    subsystems
}

/// The spacecraft at `state`, thruster at full throttle. `carried` chooses
/// between a plant that goes to orbit with its vessel and one the universe
/// drops there.
#[cfg(test)]
pub fn plant(
    body: BodyEllipsoid,
    state: &VehicleState,
    start: SimInstant,
    epoch: &SimEpoch,
    sun_fixed: DVec3,
    carried: bool,
) -> Result<Box<dyn LocalPlant>, UniverseError> {
    let block = MultibodyVehicleBlock::with_aero_model(
        MultibodyVehicleConfig {
            body,
            period: BLOCK_PERIOD,
            epoch: epoch.utc_at(start)?,
            initial: *state,
        },
        tree(),
        Box::new(VacuumAero),
        subsystems(),
        Box::new(VacuumAtmosphere),
    )?;
    let input = MultibodyVehicleInput {
        propulsion: PropulsionCommands {
            electric_throttles: vec![1.0],
            ..PropulsionCommands::default()
        },
        sun_dir_world: sun_fixed,
        ..MultibodyVehicleInput::default()
    };
    let plant = MultibodyLocalPlant::new(block, body, start, 3.0)
        .with_input(input)
        .with_carry_period(CARRY_PERIOD);
    Ok(Box::new(if carried {
        plant
    } else {
        plant.without_carrying()
    }))
}

/// A factory that builds a fresh spacecraft and counts how often it is asked.
#[cfg(test)]
pub fn counting_factory(calls: Rc<Cell<u32>>, sun_fixed: DVec3, carried: bool) -> LocalFactory {
    Box::new(move |body, state, start, epoch| {
        calls.set(calls.get().wrapping_add(1));
        plant(body, state, start, epoch, sun_fixed, carried)
    })
}

/// Specific orbital energy, J/kg, of an Earth-fixed state under the point
/// mass and `J2`: the part of the field that does not average out over a
/// revolution, so the energy holds still for a coasting vessel.
#[cfg(test)]
pub fn orbital_energy_j_kg(state: &VehicleState) -> f64 {
    let earth = BodyEllipsoid::WGS84;
    let r = state.position_ecef;
    let velocity = state.velocity_ecef + (DVec3::Z * earth.omega_rad_s).cross(r);
    let radius = r.length();
    let sin_lat = r.z / radius;
    let oblateness =
        earth.j2 * (earth.semi_major_m / radius).powi(2) * (1.5 * sin_lat * sin_lat - 0.5);
    0.5 * velocity.length_squared() - earth.mu_m3_s2 / radius * (1.0 - oblateness)
}
