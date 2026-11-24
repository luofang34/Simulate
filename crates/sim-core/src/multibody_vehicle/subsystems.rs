use glam::DVec3;
use mbd_core::{ComponentId, ComponentTree, ElectricalLoad, FaultState, ThermalLoad, WrenchAtSite};
use mbd_multibody::{
    ContactPointSpec, ContactPointState, HydrodynamicsModel, JointDriveInput, LoopConstraint,
    ManipulatorChain, ModelScale, ModelState, PinnedSurfaceRest, SurfaceMedium, SurfaceSample,
    WaterState, WheelCommand, WheelSpec, WheelState,
};
use mbd_systems::{
    AbepThrusterSpec, AbepThrusterState, AirbreathingEngineSpec, AirbreathingEngineState,
    AtomicOxygenErosionSpec, AtomicOxygenErosionState, CgTrimController, ElectricThrusterSpec,
    ElectricThrusterState, ElectricalBus, FluidNetwork, LiquidRocketSpec, LiquidRocketState,
    SolidMotorSpec, SolidMotorState, ThermalNetwork, TpsAblationSpec, TpsAblationState,
};
use sim_geo::{BodyEllipsoid, Geodetic, LocalFrame, UtcTime};

use super::step::invalid_input;
use crate::SimInstant;
use crate::flight_model::coupled_mechanism::CoupledMechanism;
use crate::flight_model::integration::{MotionTimeScale, VehicleIntegrator};
use crate::flight_model::loads::AeroLoad;

/// Fixed setup of a [`super::MultibodyVehicleBlock`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MultibodyVehicleConfig {
    /// Central planetary/lunar body ellipsoid and reference gravity field.
    pub body: BodyEllipsoid,
    /// Fixed integration step on the master simulation clock.
    pub period: crate::TickSpan,
    /// UTC instant at [`SimInstant::START`].
    pub epoch: UtcTime,
    /// Initial 6-DoF state at the vehicle's instantaneous centre of gravity.
    pub initial: crate::VehicleState,
    /// How the equations of motion are integrated across a period.
    pub integrator: VehicleIntegrator,
    /// The characteristic time of the vehicle's motion. Its characteristic
    /// length and mass are taken from the component tree.
    pub time_scale: MotionTimeScale,
}

/// Planetary ground / liquid ocean geometry sampled at body-fixed coordinates
/// (`ECEF` / `IAU_BODY`).
pub trait SurfaceContactProvider {
    /// Samples solid/liquid contact surface underneath `point_fixed_m`.
    fn sample_surface(&self, point_fixed_m: DVec3) -> Option<SurfaceSample>;

    /// Samples liquid ocean/hydrocarbon state underneath `point_fixed_m`, if wet.
    fn sample_water(&self, point_fixed_m: DVec3) -> Option<WaterState>;
}

/// Ellipsoidal planetary ground or global liquid ocean surface at datum height `0 m`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct EllipsoidSurfaceProvider {
    /// Target planetary body ellipsoid and gravity field.
    pub body: BodyEllipsoid,
    /// Surface medium (`Solid` ground/regolith or `Liquid` ocean).
    pub medium: SurfaceMedium,
    /// Liquid density when `medium == SurfaceMedium::Liquid` (`1025.0` kg/m³ seawater).
    pub fluid_density_kg_m3: f64,
}

impl EllipsoidSurfaceProvider {
    /// Solid planetary surface at datum height `0 m` on `body`.
    #[must_use]
    pub const fn solid(body: BodyEllipsoid) -> Self {
        Self {
            body,
            medium: SurfaceMedium::Solid,
            fluid_density_kg_m3: 0.0,
        }
    }

    /// Global seawater ocean surface at datum height `0 m` on `body`.
    #[must_use]
    pub const fn ocean(body: BodyEllipsoid) -> Self {
        Self {
            body,
            medium: SurfaceMedium::Liquid,
            fluid_density_kg_m3: 1_025.0,
        }
    }
}

impl SurfaceContactProvider for EllipsoidSurfaceProvider {
    fn sample_surface(&self, point_fixed_m: DVec3) -> Option<SurfaceSample> {
        let geo = self.body.fixed_to_geodetic(point_fixed_m);
        let surf_geo = Geodetic {
            height_m: 0.0,
            ..geo
        };
        Some(SurfaceSample {
            surface_point_world_m: self.body.geodetic_to_fixed(&surf_geo),
            outward_normal_world: LocalFrame::at(&surf_geo).up(),
            velocity_world_m_s: DVec3::ZERO,
            medium: self.medium,
            mu_static_override: None,
            mu_dynamic_override: None,
        })
    }

    fn sample_water(&self, point_fixed_m: DVec3) -> Option<WaterState> {
        if self.medium != SurfaceMedium::Liquid || self.fluid_density_kg_m3 <= 0.0 {
            return None;
        }
        let geo = self.body.fixed_to_geodetic(point_fixed_m);
        let surf_geo = Geodetic {
            height_m: 0.0,
            ..geo
        };
        let origin = self.body.geodetic_to_fixed(&surf_geo);
        Some(WaterState {
            surface_origin_world_m: origin,
            outward_up_world: LocalFrame::at(&surf_geo).up(),
            gravity_accel_m_s2: self.body.gravity_fixed(origin).length(),
            fluid_density_kg_m3: self.fluid_density_kg_m3,
            fluid_vel_world_m_s: DVec3::ZERO,
        })
    }
}

/// Complete collection of propulsion devices mounted across a vehicle's components.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct PropulsionAssembly {
    /// Liquid monopropellant, bipropellant, or cold-gas engines and RCS thrusters.
    pub liquid_rockets: Vec<(LiquidRocketSpec, LiquidRocketState)>,
    /// Solid rocket motors (ejection, retro, ullage, SRBs).
    pub solid_motors: Vec<(SolidMotorSpec, SolidMotorState)>,
    /// Power-limited ion or Hall thrusters.
    pub electric_thrusters: Vec<(ElectricThrusterSpec, ElectricThrusterState)>,
    /// VLEO atomic-oxygen-collecting ABEP units.
    pub abep_thrusters: Vec<(AbepThrusterSpec, AbepThrusterState)>,
    /// Airbreathing jet, afterburning, turboprop, or piston engines.
    pub airbreathing_engines: Vec<(AirbreathingEngineSpec, AirbreathingEngineState)>,
}

/// Closed-loop center-of-gravity fluid trim transfer configuration.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CgTrimConfig {
    /// Identifier of the bidirectional [`FluidNetwork`] transfer line.
    pub line_id: u16,
    /// Proportional controller regulating longitudinal root CG coordinate.
    pub controller: CgTrimController,
}

/// Complete non-aerodynamic subsystem bundle attached to a [`super::MultibodyVehicleBlock`].
#[derive(Debug, Clone, Default)]
pub struct MultibodySubsystems {
    /// Multi-species tanks and crossfeed/trim transfer lines.
    pub fluids: FluidNetwork,
    /// Registered chemical, solid, electric, ABEP, and airbreathing propulsion units.
    pub propulsion: PropulsionAssembly,
    /// Prioritized electrical distribution bus with arrays, batteries, and generators.
    pub electrical: ElectricalBus,
    /// Lumped thermal conductance, convection, and radiation network.
    pub thermal: ThermalNetwork,
    /// Optional closed-loop longitudinal CG fuel trim controller.
    pub cg_trim: Option<CgTrimConfig>,
    /// Ablative TPS entry heatshields.
    pub tps_shields: Vec<(TpsAblationSpec, TpsAblationState)>,
    /// VLEO atomic-oxygen-eroding surface coatings.
    pub ao_coatings: Vec<(AtomicOxygenErosionSpec, AtomicOxygenErosionState)>,
    /// Articulated robotic manipulator chains keyed by manifest arm name.
    pub manipulators: Vec<(String, ManipulatorChain)>,
    /// Compliant footpads / skids / airframe hardpoints with bristle stiction.
    /// The states here are those of the last completed step and are taken
    /// up again at the start of the next.
    pub contact_points: Vec<(ContactPointSpec, ContactPointState)>,
    /// Driven, steered, and braked landing-gear or rover wheels, with states
    /// kept as those of the footpads are.
    pub wheels: Vec<(WheelSpec, WheelState)>,
    /// Optional ocean splashdown hydrodynamics hull stations.
    pub hydrodynamics: Option<HydrodynamicsModel>,
    /// Closed-loop internal joint ratios, welds, or tether distance limits.
    pub loop_constraints: Vec<LoopConstraint>,
    /// Maximum thrust breakout fraction or force threshold (`>= 0` N; default `0.98 * weight`).
    pub pinned_breakout_threshold_n: Option<f64>,
    /// Continuous dwell time before locking settled landing rest, s.
    pub pinned_dwell_s: f64,
}

/// Per-step commands directed to [`PropulsionAssembly`].
#[derive(Debug, Clone, PartialEq, Default)]
pub struct PropulsionCommands {
    /// Throttles `[0..=1]` indexed to `liquid_rockets`.
    pub liquid_throttles: Vec<f64>,
    /// Gimbal `[pitch_rad, yaw_rad]` indexed to `liquid_rockets`.
    pub liquid_gimbals_rad: Vec<[f64; 2]>,
    /// Manifold isolation flags (`true` cuts propellant feed even if stuck on).
    pub liquid_isolated: Vec<bool>,
    /// Optional fault overrides (`Some` updates thruster [`FaultState`]).
    pub liquid_faults: Vec<Option<FaultState>>,
    /// One-shot ignition triggers indexed to `solid_motors`.
    pub solid_ignites: Vec<bool>,
    /// Power throttles `[0..=1]` indexed to `electric_thrusters`.
    pub electric_throttles: Vec<f64>,
    /// Power throttles `[0..=1]` indexed to `abep_thrusters`.
    pub abep_throttles: Vec<f64>,
    /// Optional intake ram alignment override (`None` computes from body `+X` vs air velocity).
    pub abep_ram_alignment: Option<f64>,
    /// Dry throttles `[0..=1]` indexed to `airbreathing_engines`.
    pub airbreathing_throttles: Vec<f64>,
    /// Afterburner engagements `[0..=1]` indexed to `airbreathing_engines`.
    pub airbreathing_afterburners: Vec<f64>,
    /// Master fuel shutoff flags indexed to `airbreathing_engines`.
    pub airbreathing_fuel_cutoffs: Vec<bool>,
}

/// Full input vector consumed by [`super::MultibodyVehicleBlock`] each step.
#[derive(Debug, Clone, PartialEq)]
pub struct MultibodyVehicleInput {
    /// Propulsion commands across all engines and thrusters.
    pub propulsion: PropulsionCommands,
    /// Generalized actuator efforts applied to articulated joints.
    pub joint_drives: Vec<JointDriveInput>,
    /// External/mechanism/tether wrenches applied at component sites.
    pub external_wrenches: Vec<WrenchAtSite>,
    /// Control channels the aerodynamic elements read, in the order they
    /// name them: surface deflections, rad; rotor speeds, rad/s.
    pub control_surfaces_rad: [f64; 8],
    /// Steer/drive/brake commands indexed to `wheels`.
    pub wheel_commands: Vec<WheelCommand>,
    /// Unit solar vector in central-body fixed world axes.
    pub sun_dir_world: DVec3,
    /// Local solar irradiance magnitude, W/m² (`1361.0` at 1 AU).
    pub sun_irradiance_w_m2: f64,
    /// Sunlit visibility (`1.0` sunlit, `0.0` eclipsed).
    pub sun_visibility: f64,
    /// Deep-space/sky radiative sink temperature, K.
    pub sink_temperature_k: f64,
    /// Additional electrical loads placed on [`ElectricalBus`].
    pub extra_electrical_loads: Vec<ElectricalLoad>,
    /// Additional thermal loads placed on [`ThermalNetwork`].
    pub extra_thermal_loads: Vec<ThermalLoad>,
    /// Optional override for longitudinal CG trim target in root frame, m.
    pub cg_trim_target_x_m: Option<f64>,
    /// Mechanisms acting on the vehicle over this step with their state
    /// supplied here; [`super::MultibodyVehicleBlock::mechanism_states`]
    /// reports where each ended, in the same order.
    pub mechanisms: Vec<CoupledMechanism>,
}

impl Default for MultibodyVehicleInput {
    fn default() -> Self {
        Self {
            propulsion: PropulsionCommands::default(),
            joint_drives: Vec::new(),
            external_wrenches: Vec::new(),
            control_surfaces_rad: [0.0; 8],
            wheel_commands: Vec::new(),
            sun_dir_world: DVec3::X,
            sun_irradiance_w_m2: 1_361.0,
            sun_visibility: 1.0,
            sink_temperature_k: 2.725,
            extra_electrical_loads: Vec::new(),
            extra_thermal_loads: Vec::new(),
            cg_trim_target_x_m: None,
            mechanisms: Vec::new(),
        }
    }
}

/// Diagnostic and subsystem telemetry published after each [`super::MultibodyVehicleBlock`] step.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MultibodyVehicleTelemetry {
    /// Net aerodynamic load of every element, in body axes.
    pub aero_wrench: AeroLoad,
    /// Instantaneous freestream dynamic pressure $q_\infty$, Pa.
    pub dynamic_pressure_pa: f64,
    /// Non-gravitational specific force at center of gravity in root frame, m/s².
    pub specific_force_cg_body: DVec3,
    /// Sum of propulsion thrust forces expressed in root axes, N.
    pub total_thrust_body_n: DVec3,
    /// Sum of propulsion moments about the centre of gravity in root axes, N·m.
    pub thrust_torque_cg_body_nm: DVec3,
    /// Total liquid + solid propellant mass remaining across tanks and motors, kg.
    pub total_propellant_kg: f64,
    /// Nominal active bus voltage on root rail (or `0.0` if collapsed), V.
    pub bus_voltage_v: f64,
    /// Total electrical power generated in latest step, W.
    pub total_electrical_gen_w: f64,
    /// Electrical demand shed due to power shortage in latest step, W.
    pub electrical_shed_w: f64,
    /// True during hypersonic entry RF plasma blackout.
    pub rf_blackout: bool,
    /// Peak cold-wall stagnation heat flux recorded across heatshields, W/m².
    pub peak_heat_flux_w_m2: f64,
    /// Maximum eroded thickness across VLEO atomic-oxygen coatings, m.
    pub ao_erosion_depth_m: f64,
    /// Minimum remaining coating fraction (`1.0` untouched -> `0.0` eroded).
    pub ao_remaining_fraction: f64,
    /// True while any compliant foot or wheel carries ground normal load.
    pub in_ground_contact: bool,
    /// True while latched in zero-jitter [`PinnedSurfaceRest::Pinned`].
    pub is_pinned_rest: bool,
    /// True while hull stations experience positive ocean buoyancy.
    pub is_splashed_down: bool,
    /// Total ocean buoyant force acting upward, N.
    pub total_buoyancy_n: f64,
    /// True while no joint of the vehicle has a degree of freedom, so it
    /// moves as one rigid body.
    pub used_rigid_fast_path: bool,
}

impl Default for MultibodyVehicleTelemetry {
    fn default() -> Self {
        Self {
            aero_wrench: AeroLoad::ZERO,
            dynamic_pressure_pa: 0.0,
            specific_force_cg_body: DVec3::ZERO,
            total_thrust_body_n: DVec3::ZERO,
            thrust_torque_cg_body_nm: DVec3::ZERO,
            total_propellant_kg: 0.0,
            bus_voltage_v: 0.0,
            total_electrical_gen_w: 0.0,
            electrical_shed_w: 0.0,
            rf_blackout: false,
            peak_heat_flux_w_m2: 0.0,
            ao_erosion_depth_m: 0.0,
            ao_remaining_fraction: 1.0,
            in_ground_contact: false,
            is_pinned_rest: false,
            is_splashed_down: false,
            total_buoyancy_n: 0.0,
            used_rigid_fast_path: false,
        }
    }
}

/// Complete rollback/replay snapshot of a [`super::MultibodyVehicleBlock`].
#[derive(Debug, Clone)]
pub struct MultibodyVehicleSnapshot {
    pub(super) now: SimInstant,
    /// The model's state with its base in fixed axes: joints, and the
    /// continuous states and modes of every element.
    pub(super) state: ModelState,
    pub(super) last_alpha_root_rad_s2: DVec3,
    pub(super) component_states: Vec<mbd_core::ComponentState>,
    pub(super) fluids: FluidNetwork,
    pub(super) propulsion: PropulsionAssembly,
    pub(super) electrical: ElectricalBus,
    pub(super) thermal: ThermalNetwork,
    pub(super) tps_states: Vec<TpsAblationState>,
    pub(super) ao_states: Vec<AtomicOxygenErosionState>,
    pub(super) pinned_rest: PinnedSurfaceRest,
    pub(super) telemetry: MultibodyVehicleTelemetry,
}

impl MultibodyVehicleSnapshot {
    /// The model state at capture: base in fixed axes, joints, element
    /// states and modes.
    #[must_use]
    pub const fn model_state(&self) -> &ModelState {
        &self.state
    }

    /// Component runtime states at snapshot capture.
    #[must_use]
    pub fn component_states(&self) -> &[mbd_core::ComponentState] {
        &self.component_states
    }

    /// Snapshot timestamp on the simulation master clock.
    #[must_use]
    pub const fn now(&self) -> SimInstant {
        self.now
    }
}

/// Complete topology-preserving snapshot of a [`super::MultibodyVehicleBlock`],
/// including tree topology, subsystem specifications, aerodynamic elements,
/// configuration, and continuous/discrete dynamic state.
#[derive(Debug, Clone)]
pub struct MultibodyStructuralSnapshot {
    pub(super) inner: MultibodyVehicleSnapshot,
    pub(super) config: MultibodyVehicleConfig,
    pub(super) tree: ComponentTree,
    pub(super) aero: super::VehicleAero,
    pub(super) subsystems: MultibodySubsystems,
    pub(super) scale: ModelScale,
    pub(super) mechanism_states: Vec<crate::flight_model::coupled_mechanism::CoupledMechanismState>,
}

pub(super) fn lookup_mapped_comp(
    map: &[(ComponentId, ComponentId)],
    old: ComponentId,
) -> Option<ComponentId> {
    map.iter().find(|&&(k, _)| k == old).map(|&(_, v)| v)
}

pub(super) fn validate_input(input: &MultibodyVehicleInput) -> Result<(), crate::BlockError> {
    if !input.control_surfaces_rad.iter().all(|v| v.is_finite()) {
        return Err(invalid_input(
            "control_surfaces_rad",
            "non-finite deflection".to_owned(),
        ));
    }
    if !(input.sun_dir_world.is_finite()
        && input.sun_irradiance_w_m2.is_finite()
        && input.sun_visibility.is_finite()
        && input.sink_temperature_k.is_finite())
    {
        return Err(invalid_input(
            "environment",
            "non-finite solar/sink input".to_owned(),
        ));
    }
    for w in &input.external_wrenches {
        if !(w.point_local_m.is_finite()
            && w.force_local_n.is_finite()
            && w.torque_local_nm.is_finite())
        {
            return Err(invalid_input("external_wrenches", format!("{w:?}")));
        }
    }
    Ok(())
}
