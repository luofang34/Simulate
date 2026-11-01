use aviate_core::control::{Command, CommandSource, ControlMode, Setpoint};
use aviate_core::math::Quaternion;
use aviate_core::types::{Meters, NormalizedThrust};
use aviate_hal_io::SystemCommand;
use glam::{DMat3, DVec3, EulerRot, Quat};
use mbd_core::{ComponentSpec, ComponentState, ComponentTree, Placement, VacuumAero};
use sim_atmosphere::StandardAtmosphere;
use sim_aviate::{
    AviateDirectConfig, AviateDirectRunner, DirectFixedWingKernel, DirectX500Kernel, RotorLayout,
    build_direct_fixed_wing_kernel, build_direct_x500_kernel, direct_fixed_wing_controls,
};
use sim_core::{
    Block, ControlInputs, EngineStartMode, EngineStatus, JSBSIM_DEFAULT_PERIOD, JsbsimFdmBlock,
    JsbsimFdmConfig, KinematicInput, KinematicModel, RigidBody6DofBlock, RigidBodyConfig,
    RigidBodyInput, SimInstant, SystemControls, TICKS_PER_SECOND, TickSpan, VehicleState,
};
use sim_geo::{BodyEllipsoid, Geodetic, UtcTime};
use sim_pack::{LoadedSyspack, builtin_c172_package, builtin_light_trainer_package};
use world_core::{GeodeticPosition, normal_gravity};

use crate::error::AppSimError;
use crate::supervisor_rig::{DELAWARE_RUNWAY_BEARING_RAD, default_delaware_spawn};

const X500_RATE_HZ: u64 = 400;
const _: () = assert!(TICKS_PER_SECOND.is_multiple_of(X500_RATE_HZ));
const X500_PERIOD: TickSpan = TickSpan::from_ticks(TICKS_PER_SECOND / X500_RATE_HZ);
const X500_XY_SPEED_M_S: f32 = 12.0;
const X500_CLIMB_SPEED_M_S: f32 = 5.0;
const X500_YAW_RATE_RAD_S: f32 = 1.2;

/// Selectable interactive free-flight vehicle and flight-control plant.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Hash)]
pub enum FreeFlightVehicle {
    /// SHA-256-verified JSBSim Cessna 172 Skyhawk (`vehicles/c172`) with `SystemControls`.
    #[default]
    JsbsimC172,
    /// SHA-256-verified JSBSim Light Trainer (`vehicles/light-trainer`) with `SystemControls`.
    JsbsimLightTrainer,
    /// Tier-0 `AviateDirectRunner` (`Ekf` + `MultirotorController` + `QuadXMixerX500`) on 6-DoF X500.
    AviateX500Direct,
    /// Tier-0 `AviateDirectRunner` (`Ekf` + `FixedWingController`) driving JSBSim Light Trainer.
    AviateFixedWingDirect,
    /// Fast kinematic 6-DoF free-flight model.
    Kinematic,
}

impl FreeFlightVehicle {
    /// Cycles to the next free-flight vehicle.
    #[must_use]
    pub const fn next(self) -> Self {
        match self {
            Self::JsbsimC172 => Self::JsbsimLightTrainer,
            Self::JsbsimLightTrainer => Self::AviateX500Direct,
            Self::AviateX500Direct => Self::AviateFixedWingDirect,
            Self::AviateFixedWingDirect => Self::Kinematic,
            Self::Kinematic => Self::JsbsimC172,
        }
    }

    /// Short title displayed in the window title bar and logs.
    #[must_use]
    pub const fn title(self) -> &'static str {
        match self {
            Self::JsbsimC172 => "C172 Skyhawk (JSBSim)",
            Self::JsbsimLightTrainer => "Light Trainer (JSBSim)",
            Self::AviateX500Direct => "X500 Quadrotor (Aviate Direct)",
            Self::AviateFixedWingDirect => "Fixed-Wing FBW (Aviate + JSBSim)",
            Self::Kinematic => "Kinematic Free Flight",
        }
    }
}

/// Active free-flight plant stepped by the interactive clock.
pub enum FreeFlightPlant {
    /// Direct pilot-flown JSBSim fixed-wing aircraft (`C172` or `LightTrainer`).
    Jsbsim {
        /// Loaded JSBSim flight dynamics block.
        fdm: Box<JsbsimFdmBlock>,
        /// Latest body FRD specific force (`m/s²`).
        specific_force_frd_m_s2: DVec3,
    },
    /// Tier-0 Aviate quadrotor flight controller closed around a `400 Hz` 6-DoF rigid body.
    AviateX500(Box<AviateX500Rig>),
    /// Tier-0 Aviate fixed-wing controller closed around a `120 Hz` JSBSim plant.
    AviateFixedWing(Box<AviateFixedWingRig>),
    /// Lightweight kinematic model.
    Kinematic(KinematicModel),
}

/// Closed-loop Tier-0 Aviate X500 quadrotor plant.
pub struct AviateX500Rig {
    body: RigidBody6DofBlock,
    rotors: RotorLayout,
    runner: AviateDirectRunner<
        aviate_core::control::multirotor::MultirotorController,
        aviate_core::mixer::QuadXMixerX500,
    >,
    sim_clock_offset: SimInstant,
    target_ned_m: [f32; 3],
    yaw_rad: f32,
    seq: u32,
}

/// Closed-loop Tier-0 Aviate fixed-wing plant.
pub struct AviateFixedWingRig {
    fdm: JsbsimFdmBlock,
    specific_force_frd_m_s2: DVec3,
    runner: AviateDirectRunner<
        aviate_core::control::fixed_wing::FixedWingController,
        sim_aviate::PassthroughFixedWingMixer,
    >,
    sim_clock_offset: SimInstant,
    heading_rad: f32,
    seq: u32,
}

impl FreeFlightPlant {
    /// Instantiates `vehicle` over the Delaware corridor runway alignment.
    pub fn new(vehicle: FreeFlightVehicle) -> Result<Self, AppSimError> {
        let spawn = default_delaware_spawn();
        match vehicle {
            FreeFlightVehicle::JsbsimC172 => {
                let pkg = builtin_c172_package().map_err(sim_scenario::ScenarioError::from)?;
                build_jsbsim_plant(&pkg, spawn, DELAWARE_RUNWAY_BEARING_RAD, 52.0)
            }
            FreeFlightVehicle::JsbsimLightTrainer => {
                let pkg =
                    builtin_light_trainer_package().map_err(sim_scenario::ScenarioError::from)?;
                build_jsbsim_plant(&pkg, spawn, DELAWARE_RUNWAY_BEARING_RAD, 48.0)
            }
            FreeFlightVehicle::AviateX500Direct => {
                let hover_spawn = Geodetic {
                    height_m: 120.0,
                    ..spawn
                };
                Ok(Self::AviateX500(Box::new(build_aviate_x500(hover_spawn)?)))
            }
            FreeFlightVehicle::AviateFixedWingDirect => Ok(Self::AviateFixedWing(Box::new(
                build_aviate_fixed_wing(spawn)?,
            ))),
            FreeFlightVehicle::Kinematic => Ok(Self::Kinematic(KinematicModel::new(
                VehicleState::level_flight(&spawn, DELAWARE_RUNWAY_BEARING_RAD, 55.0),
            ))),
        }
    }

    /// Fixed integration period of this plant.
    #[must_use]
    pub fn period(&self) -> TickSpan {
        match self {
            Self::Jsbsim { fdm, .. } => fdm.period(),
            Self::AviateX500(rig) => rig.body.period(),
            Self::AviateFixedWing(rig) => rig.fdm.period(),
            Self::Kinematic(model) => model.period(),
        }
    }

    /// Current `f64` ECEF vehicle state.
    #[must_use]
    pub fn state(&self) -> VehicleState {
        match self {
            Self::Jsbsim { fdm, .. } => fdm.output(),
            Self::AviateX500(rig) => rig.body.output(),
            Self::AviateFixedWing(rig) => rig.fdm.output(),
            Self::Kinematic(model) => model.output(),
        }
    }

    /// Body FRD specific force (`m/s²`) for turn/slip and G-load instrumentation.
    #[must_use]
    pub fn specific_force_frd_m_s2(&self) -> Option<DVec3> {
        match self {
            Self::Jsbsim {
                specific_force_frd_m_s2,
                ..
            } => Some(*specific_force_frd_m_s2),
            Self::AviateX500(rig) => Some(rig.body.specific_force_cg_body()),
            Self::AviateFixedWing(rig) => Some(rig.specific_force_frd_m_s2),
            Self::Kinematic(_) => None,
        }
    }

    /// Latched aircraft [`SystemControls`] when flying a JSBSim aircraft.
    #[must_use]
    pub fn system_controls(&self) -> Option<SystemControls> {
        match self {
            Self::Jsbsim { fdm, .. } => Some(fdm.system_controls()),
            Self::AviateFixedWing(rig) => Some(rig.fdm.system_controls()),
            Self::AviateX500(_) | Self::Kinematic(_) => None,
        }
    }

    /// Updates [`SystemControls`] on JSBSim-backed aircraft.
    pub fn set_system_controls(&mut self, controls: SystemControls) {
        match self {
            Self::Jsbsim { fdm, .. } => fdm.set_system_controls(controls),
            Self::AviateFixedWing(rig) => rig.fdm.set_system_controls(controls),
            Self::AviateX500(_) | Self::Kinematic(_) => {}
        }
    }

    /// Current engine running state and propeller RPM on JSBSim-backed aircraft.
    #[must_use]
    pub fn engine_status(&self) -> Option<EngineStatus> {
        match self {
            Self::Jsbsim { fdm, .. } => Some(fdm.engine_status()),
            Self::AviateFixedWing(rig) => Some(rig.fdm.engine_status()),
            Self::AviateX500(_) | Self::Kinematic(_) => None,
        }
    }

    /// Whether an in-process Aviate flight controller is armed (`None` on non-Aviate plants).
    #[must_use]
    pub fn aviate_armed(&self) -> Option<bool> {
        match self {
            Self::AviateX500(rig) => Some(rig.runner.is_armed()),
            Self::AviateFixedWing(rig) => Some(rig.runner.is_armed()),
            Self::Jsbsim { .. } | Self::Kinematic(_) => None,
        }
    }

    /// Advances the plant by one integration period under `controls`.
    pub fn step(&mut self, controls: ControlInputs) -> Result<(), AppSimError> {
        match self {
            Self::Jsbsim {
                fdm,
                specific_force_frd_m_s2,
            } => step_jsbsim_with_force(fdm, specific_force_frd_m_s2, controls),
            Self::AviateX500(rig) => step_aviate_x500(rig, controls),
            Self::AviateFixedWing(rig) => step_aviate_fixed_wing(rig, controls),
            Self::Kinematic(model) => {
                model
                    .step(&KinematicInput { controls })
                    .map_err(sim_scenario::ScenarioError::from)?;
                Ok(())
            }
        }
    }
}

fn build_jsbsim_fdm(
    pkg: &LoadedSyspack,
    initial: Geodetic,
    heading_rad: f64,
    speed_m_s: f64,
) -> Result<(JsbsimFdmBlock, DVec3), AppSimError> {
    let resolver = pkg
        .build_jsbsim_resolver()
        .map_err(sim_scenario::ScenarioError::from)?;
    let (aircraft_path, model) = pkg
        .jsbsim_model()
        .map_err(sim_scenario::ScenarioError::from)?;
    let mut fdm = JsbsimFdmBlock::new(&JsbsimFdmConfig {
        resolver: &resolver,
        aircraft_path,
        model,
        period: JSBSIM_DEFAULT_PERIOD,
        initial,
        heading_rad,
        speed_m_s,
        alpha_rad: 0.04,
        engine_start: EngineStartMode::Running,
    })
    .map_err(sim_scenario::ScenarioError::from)?;
    drop(fdm.trim_level_flight());
    let state = fdm.output();
    let g_ecef = BodyEllipsoid::WGS84.gravity_fixed(state.position_ecef);
    let specific_force = -(DMat3::from_quat(state.attitude.inverse()) * g_ecef);
    Ok((fdm, specific_force))
}

fn build_jsbsim_plant(
    pkg: &LoadedSyspack,
    initial: Geodetic,
    heading_rad: f64,
    speed_m_s: f64,
) -> Result<FreeFlightPlant, AppSimError> {
    let (fdm, specific_force_frd_m_s2) = build_jsbsim_fdm(pkg, initial, heading_rad, speed_m_s)?;
    Ok(FreeFlightPlant::Jsbsim {
        fdm: Box::new(fdm),
        specific_force_frd_m_s2,
    })
}

fn step_jsbsim_with_force(
    fdm: &mut JsbsimFdmBlock,
    specific_force_frd_m_s2: &mut DVec3,
    controls: ControlInputs,
) -> Result<(), AppSimError> {
    let prev = fdm.output();
    fdm.step(&KinematicInput { controls })
        .map_err(sim_scenario::ScenarioError::from)?;
    let next = fdm.output();
    let dt_s = fdm.period().as_seconds();
    let accel_ecef = (next.velocity_ecef - prev.velocity_ecef) / dt_s;
    let g_ecef = BodyEllipsoid::WGS84.gravity_fixed(next.position_ecef);
    *specific_force_frd_m_s2 = DMat3::from_quat(next.attitude.inverse()) * (accel_ecef - g_ecef);
    Ok(())
}

fn build_aviate_x500(origin: Geodetic) -> Result<AviateX500Rig, AppSimError> {
    let g = normal_gravity(GeodeticPosition::wgs84(
        origin.lat_rad.to_degrees(),
        origin.lon_rad.to_degrees(),
        origin.height_m,
    ))
    .unwrap_or(9.806_65);
    let spec = ComponentSpec {
        name: "X500 Airframe".to_owned(),
        placement: Placement::ROOT,
        dry_mass_kg: 20.25 / g,
        dry_inertia_body: DMat3::from_diagonal(DVec3::new(0.029, 0.029, 0.055)),
        thermal_capacity_j_k: 0.0,
        nominal_power_w: 0.0,
    };
    let tree = ComponentTree::new([(spec, ComponentState::at_temperature(293.15))])
        .map_err(sim_scenario::ScenarioError::from)?;
    let yaw_rad = DELAWARE_RUNWAY_BEARING_RAD;
    let body = RigidBody6DofBlock::new(
        RigidBodyConfig {
            body: BodyEllipsoid::WGS84,
            period: X500_PERIOD,
            epoch: UtcTime::from_unix_seconds(0.0),
            initial: VehicleState::level_flight(&origin, yaw_rad, 0.0),
        },
        tree,
        Box::new(VacuumAero),
        Box::new(StandardAtmosphere),
    )
    .map_err(sim_scenario::ScenarioError::from)?;
    let kernel: DirectX500Kernel =
        build_direct_x500_kernel().ok_or(AppSimError::AviateKernel { detail: "x500" })?;
    let mut runner = AviateDirectRunner::new(kernel, AviateDirectConfig::ideal(origin))?;
    let mut at = SimInstant::START;
    let rest_force = DVec3::new(0.0, 0.0, -g);
    for _ in 0..400 {
        let _act = runner.step_cg(at, &body.output(), rest_force)?;
        at = at.advanced(X500_PERIOD);
    }
    runner.send_command(SystemCommand::Arm);
    let _act = runner.step_cg(at, &body.output(), rest_force)?;
    at = at.advanced(X500_PERIOD);
    Ok(AviateX500Rig {
        body,
        rotors: RotorLayout::x500(),
        runner,
        sim_clock_offset: at,
        target_ned_m: [0.0, 0.0, 0.0],
        yaw_rad: yaw_rad as f32,
        seq: 1,
    })
}

fn step_aviate_x500(rig: &mut AviateX500Rig, controls: ControlInputs) -> Result<(), AppSimError> {
    let dt = X500_PERIOD.as_seconds() as f32;
    rig.yaw_rad += (controls.yaw as f32) * X500_YAW_RATE_RAD_S * dt;
    let (sin_y, cos_y) = rig.yaw_rad.sin_cos();
    let fwd_cmd = -(controls.pitch as f32) * X500_XY_SPEED_M_S * dt;
    let right_cmd = (controls.roll as f32) * X500_XY_SPEED_M_S * dt;
    let climb_cmd = ((controls.throttle as f32) - 0.5) * 2.0 * X500_CLIMB_SPEED_M_S * dt;
    let [mut n, mut e, mut d] = rig.target_ned_m;
    n += fwd_cmd * cos_y - right_cmd * sin_y;
    e += fwd_cmd * sin_y + right_cmd * cos_y;
    d -= climb_cmd;
    rig.target_ned_m = [n, e, d];

    if rig.seq.is_multiple_of(8) {
        rig.runner
            .send_command(SystemCommand::FlightControl(Command {
                mode: ControlMode::PositionHold,
                setpoint: Setpoint {
                    position: Some([Meters(n), Meters(e), Meters(d)]),
                    heading: Some(aviate_core::types::Radians(rig.yaw_rad)),
                    ..Setpoint::default()
                },
                config_mode_request: None,
                sensor_overrides: None,
                sequence: rig.seq,
                source: CommandSource::Gcs,
            }));
    }
    rig.seq = rig.seq.wrapping_add(1);
    let act = rig.runner.step_cg(
        rig.sim_clock_offset,
        &rig.body.output(),
        rig.body.specific_force_cg_body(),
    )?;
    rig.sim_clock_offset = rig.sim_clock_offset.advanced(X500_PERIOD);
    let thrusters = rig.rotors.direct_thrusters(&act)?;
    rig.body
        .step(&RigidBodyInput {
            thrusters,
            ..RigidBodyInput::default()
        })
        .map_err(sim_scenario::ScenarioError::from)?;
    Ok(())
}

fn build_aviate_fixed_wing(origin: Geodetic) -> Result<AviateFixedWingRig, AppSimError> {
    let pkg = builtin_light_trainer_package().map_err(sim_scenario::ScenarioError::from)?;
    let (fdm, specific_force_frd_m_s2) =
        build_jsbsim_fdm(&pkg, origin, DELAWARE_RUNWAY_BEARING_RAD, 48.0)?;
    let kernel: DirectFixedWingKernel =
        build_direct_fixed_wing_kernel().ok_or(AppSimError::AviateKernel {
            detail: "fixed_wing",
        })?;
    let mut runner = AviateDirectRunner::new(kernel, AviateDirectConfig::ideal(origin))?;
    let mut at = SimInstant::START;
    let state = fdm.output();
    for _ in 0..480 {
        let _act = runner.step_cg(at, &state, specific_force_frd_m_s2)?;
        at = at.advanced(JSBSIM_DEFAULT_PERIOD);
        if runner.is_ready() {
            break;
        }
    }
    runner.send_command(SystemCommand::Arm);
    let _act = runner.step_cg(at, &state, specific_force_frd_m_s2)?;
    at = at.advanced(JSBSIM_DEFAULT_PERIOD);
    Ok(AviateFixedWingRig {
        fdm,
        specific_force_frd_m_s2,
        runner,
        sim_clock_offset: at,
        heading_rad: DELAWARE_RUNWAY_BEARING_RAD as f32,
        seq: 1,
    })
}

fn step_aviate_fixed_wing(
    rig: &mut AviateFixedWingRig,
    controls: ControlInputs,
) -> Result<(), AppSimError> {
    let dt = rig.fdm.period().as_seconds() as f32;
    rig.heading_rad += (controls.yaw as f32) * 0.45 * dt;
    let roll_rad = (controls.roll as f32) * 35.0_f32.to_radians();
    let pitch_rad = (controls.pitch as f32) * 18.0_f32.to_radians();
    let q = Quat::from_euler(EulerRot::ZYX, rig.heading_rad, pitch_rad, roll_rad).normalize();
    let thrust = if controls.throttle > 0.01 {
        controls.throttle as f32
    } else {
        0.62
    };
    if rig.seq.is_multiple_of(4) {
        rig.runner
            .send_command(SystemCommand::FlightControl(Command {
                mode: ControlMode::Attitude,
                setpoint: Setpoint {
                    attitude: Some(Quaternion::new(q.w, q.x, q.y, q.z).normalize()),
                    collective_thrust: NormalizedThrust(thrust),
                    ..Setpoint::default()
                },
                config_mode_request: None,
                sensor_overrides: None,
                sequence: rig.seq,
                source: CommandSource::Gcs,
            }));
    }
    rig.seq = rig.seq.wrapping_add(1);
    let act = rig.runner.step_cg(
        rig.sim_clock_offset,
        &rig.fdm.output(),
        rig.specific_force_frd_m_s2,
    )?;
    rig.sim_clock_offset = rig.sim_clock_offset.advanced(rig.fdm.period());
    let fbw_controls = direct_fixed_wing_controls(&act);
    step_jsbsim_with_force(&mut rig.fdm, &mut rig.specific_force_frd_m_s2, fbw_controls)
}
