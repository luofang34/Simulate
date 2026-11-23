use glam::{DQuat, DVec3};
use mbd_aero::CompositeAeroModel;
use mbd_core::{
    AeroModel, AeroWrench, ComponentId, ElectricalLoad, FaultState, MassProperties, ThermalLoad,
    WrenchAtSite,
};
use mbd_multibody::{
    ContactPointSpec, ContactPointState, FloatingBaseState, HydrodynamicsModel, JointDriveInput,
    LoopConstraint, ManipulatorChain, PinnedSurfaceRest, SurfaceMedium, SurfaceSample, WaterState,
    WheelCommand, WheelSpec, WheelState,
};
use mbd_systems::{
    AbepThrusterSpec, AbepThrusterState, AirbreathingEngineSpec, AirbreathingEngineState,
    AtomicOxygenErosionSpec, AtomicOxygenErosionState, CgTrimController, ElectricThrusterSpec,
    ElectricThrusterState, ElectricalBus, FluidNetwork, LiquidRocketSpec, LiquidRocketState,
    SolidMotorSpec, SolidMotorState, ThermalNetwork, TpsAblationSpec, TpsAblationState,
};
use sim_geo::{BodyEllipsoid, Geodetic, LocalFrame, UtcTime};

use super::step::invalid_input;
use crate::{SimInstant, VehicleState};

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
    pub contact_points: Vec<(ContactPointSpec, ContactPointState)>,
    /// Driven, steered, and braked landing-gear or rover wheels.
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
    /// Aerodynamic control surface deflections passed to `AeroModel`, rad.
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
        }
    }
}

/// Diagnostic and subsystem telemetry published after each [`super::MultibodyVehicleBlock`] step.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MultibodyVehicleTelemetry {
    /// Net aerodynamic wrench resolved to vehicle root frame.
    pub aero_wrench: AeroWrench,
    /// Instantaneous freestream dynamic pressure $q_\infty$, Pa.
    pub dynamic_pressure_pa: f64,
    /// Non-gravitational specific force at center of gravity in root frame, m/s².
    pub specific_force_cg_body: DVec3,
    /// Sum of propulsion thrust forces expressed in root axes, N.
    pub total_thrust_body_n: DVec3,
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
    /// True if forward dynamics dispatched through zero-DoF `RigidFastPath`.
    pub used_rigid_fast_path: bool,
}

impl Default for MultibodyVehicleTelemetry {
    fn default() -> Self {
        Self {
            aero_wrench: AeroWrench::ZERO,
            dynamic_pressure_pa: 0.0,
            specific_force_cg_body: DVec3::ZERO,
            total_thrust_body_n: DVec3::ZERO,
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
    pub(super) base_fixed: FloatingBaseState,
    pub(super) last_alpha_root_rad_s2: DVec3,
    pub(super) component_states: Vec<mbd_core::ComponentState>,
    pub(super) fluids: FluidNetwork,
    pub(super) propulsion: PropulsionAssembly,
    pub(super) electrical: ElectricalBus,
    pub(super) thermal: ThermalNetwork,
    pub(super) tps_states: Vec<TpsAblationState>,
    pub(super) ao_states: Vec<AtomicOxygenErosionState>,
    pub(super) contact_states: Vec<ContactPointState>,
    pub(super) wheel_states: Vec<WheelState>,
    pub(super) pinned_rest: PinnedSurfaceRest,
    pub(super) telemetry: MultibodyVehicleTelemetry,
}

impl MultibodyVehicleSnapshot {
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

pub(crate) enum AeroStorage {
    Composite(CompositeAeroModel),
    Dynamic(Box<dyn AeroModel>),
}

impl AeroStorage {
    pub(super) fn evaluate_root(
        &self,
        flow: &mbd_core::AeroFlowState,
        poses: &mbd_core::PoseTable,
    ) -> Result<AeroWrench, mbd_core::MbdError> {
        match self {
            Self::Composite(c) => c.evaluate_with_poses(flow, poses),
            Self::Dynamic(d) => d.evaluate(flow),
        }
    }
}

pub(super) fn wheel_inertia_estimate(wheel: &WheelSpec) -> f64 {
    (0.5 * 8.0 * wheel.wheel_radius_m * wheel.wheel_radius_m).max(0.05)
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

pub(super) fn fixed_base_from_vehicle(
    vehicle: &VehicleState,
    mass: &MassProperties,
) -> FloatingBaseState {
    let p_root = vehicle.position_ecef - vehicle.attitude * mass.cg_body_m;
    let v_root_fixed =
        vehicle.velocity_ecef - vehicle.attitude * vehicle.angular_rate_body.cross(mass.cg_body_m);
    FloatingBaseState {
        position_world_m: p_root,
        rotation_world_from_root: vehicle.attitude,
        linear_vel_root_m_s: vehicle.attitude.conjugate() * v_root_fixed,
        angular_vel_root_rad_s: vehicle.angular_rate_body,
    }
}

pub(super) fn vehicle_from_fixed_base(
    base: &FloatingBaseState,
    mass: &MassProperties,
) -> VehicleState {
    let cg_world = base.rotation_world_from_root * mass.cg_body_m;
    let v_root_fixed = base.rotation_world_from_root * base.linear_vel_root_m_s;
    let v_cg_fixed = v_root_fixed
        + base.rotation_world_from_root * base.angular_vel_root_rad_s.cross(mass.cg_body_m);
    VehicleState {
        position_ecef: base.position_world_m + cg_world,
        velocity_ecef: v_cg_fixed,
        attitude: base.rotation_world_from_root,
        angular_rate_body: base.angular_vel_root_rad_s,
    }
}

pub(crate) fn fixed_to_instantaneous_inertial(
    base_fixed: &FloatingBaseState,
    body: &BodyEllipsoid,
) -> FloatingBaseState {
    let omega_p = DVec3::Z * body.omega_rad_s;
    let v_fixed = base_fixed.rotation_world_from_root * base_fixed.linear_vel_root_m_s;
    let v_inertial = v_fixed + omega_p.cross(base_fixed.position_world_m);
    let q_conj = base_fixed.rotation_world_from_root.conjugate();
    FloatingBaseState {
        position_world_m: base_fixed.position_world_m,
        rotation_world_from_root: base_fixed.rotation_world_from_root,
        linear_vel_root_m_s: q_conj * v_inertial,
        angular_vel_root_rad_s: base_fixed.angular_vel_root_rad_s + q_conj * omega_p,
    }
}

pub(crate) fn inertial_to_fixed_after_step(
    base_i_next: &FloatingBaseState,
    body: &BodyEllipsoid,
    dt_s: f64,
) -> FloatingBaseState {
    let q_f_from_i = DQuat::from_axis_angle(DVec3::Z, -body.omega_rad_s * dt_s);
    let pos_fixed = q_f_from_i * base_i_next.position_world_m;
    let rot_fixed = (q_f_from_i * base_i_next.rotation_world_from_root).normalize();
    let v_i_in_f = q_f_from_i * base_i_next.linear_vel_world_m_s();
    let omega_p = DVec3::Z * body.omega_rad_s;
    let v_rel_fixed = v_i_in_f - omega_p.cross(pos_fixed);
    let q_conj = rot_fixed.conjugate();
    FloatingBaseState {
        position_world_m: pos_fixed,
        rotation_world_from_root: rot_fixed,
        linear_vel_root_m_s: q_conj * v_rel_fixed,
        angular_vel_root_rad_s: base_i_next.angular_vel_root_rad_s - q_conj * omega_p,
    }
}
