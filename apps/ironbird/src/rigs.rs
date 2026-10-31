use aviate_core::control::{Command, CommandSource, ControlMode, Setpoint};
use aviate_core::math::Quaternion;
use aviate_core::types::{Meters, NormalizedThrust};
use aviate_hal_io::SystemCommand;
use glam::DVec3;
use sim_aviate::{
    AviateDirectConfig, AviateDirectRunner, AviateSharedMemoryEndpoint, AviateStimulus,
    DirectNoiseTier, EnuWorld, SharedStateV2, build_direct_fixed_wing_kernel,
    build_direct_x500_kernel, direct_fixed_wing_controls, new_shared_state,
};
use sim_core::{BridgeStats, ExternalNodeBridge, HitlSyncMode, SimInstant, VehicleState};
#[cfg(not(target_arch = "wasm32"))]
use sim_core::{ControlInputs, ExternalEndpoint};
use sim_geo::{BodyEllipsoid, Geodetic, ecef_to_geodetic};
#[cfg(not(target_arch = "wasm32"))]
use sim_xil::{
    XilStimulus,
    betaflight::{BetaflightServoPacket, BetaflightUdpEndpoint},
    mavlink_hil::{HilActuatorControls, HilControlMapping},
};
use web_time::Instant;

use crate::IronbirdError;
use crate::plant::{FixedWingVehicle, JsbsimPlant, X500RigidPlant};

/// Execution telemetry and performance summary of an ironbird closed-loop run.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RigSummary {
    /// Total physics steps executed (excluding pre-arm sensor warmup).
    pub steps_executed: u64,
    /// Simulated flight duration (`s`).
    pub simulated_seconds: f64,
    /// Host wall-clock elapsed duration (`s`).
    pub wall_seconds: f64,
    /// `simulated_seconds / wall_seconds` speedup ratio.
    pub realtime_speedup: f64,
    /// Initial WGS84 geodetic position.
    pub initial_geodetic: Geodetic,
    /// Final WGS84 geodetic position.
    pub final_geodetic: Geodetic,
    /// Final 6-DoF ECEF `VehicleState`.
    pub final_state: VehicleState,
    /// Bridge delivery/drop counters (zeroed for direct Tier-0 runs).
    pub bridge_stats: BridgeStats,
}

/// Runs Tier-0 zero-overhead `AviateDirectRunner` (`build_x500_kernel`) driving
/// `X500RigidPlant` to climb `target_climb_m` over `duration_s`.
pub fn run_tier0_x500_hover_climb(
    origin: Geodetic,
    target_climb_m: f64,
    duration_s: f64,
    noise: DirectNoiseTier,
    noise_seed: u64,
) -> Result<RigSummary, IronbirdError> {
    let mut plant = X500RigidPlant::new_at(origin)?;
    let period = plant.period();
    let total_steps = steps_for(duration_s, period.as_seconds());
    let kernel = build_direct_x500_kernel().ok_or(IronbirdError::X500KernelBuild)?;
    let mut fc = AviateDirectRunner::new(
        kernel,
        AviateDirectConfig {
            noise,
            noise_seed,
            ..AviateDirectConfig::ideal(origin)
        },
    )?;

    let wall_start = Instant::now();
    let mut now = SimInstant::START;
    let rest_f = DVec3::new(0.0, 0.0, -9.806_65);
    for _ in 0..500 {
        if fc.is_ready() {
            break;
        }
        let _ = fc.step_cg(now, &plant.state(), rest_f)?;
        now = now.advanced(period);
    }

    let mut seq = 1_u32;
    let hold_cmd = |sequence: u32| {
        SystemCommand::FlightControl(Command {
            mode: ControlMode::PositionHold,
            setpoint: Setpoint {
                position: Some([Meters(0.0), Meters(0.0), Meters(-(target_climb_m as f32))]),
                ..Setpoint::default()
            },
            config_mode_request: None,
            sensor_overrides: None,
            sequence,
            source: CommandSource::Gcs,
        })
    };
    fc.send_command(hold_cmd(seq));
    fc.send_command(SystemCommand::Arm);
    let act0 = fc.step_cg(now, &plant.state(), rest_f)?;
    let _ = plant.step(plant.rotors().direct_thrusters(&act0)?)?;
    now = now.advanced(period);

    for k in 0..total_steps {
        if k > 0 && k % 40 == 0 {
            seq = seq.wrapping_add(1);
            fc.send_command(hold_cmd(seq));
        }
        let stim = AviateStimulus {
            vehicle: plant.state(),
            site: plant.site_kinematics()?,
        };
        let act = fc.step_site(now, &stim)?;
        let thrusters = plant.rotors().direct_thrusters(&act)?;
        let _ = plant.step(thrusters)?;
        now = now.advanced(period);
    }

    Ok(finish_summary(
        total_steps,
        period.as_seconds(),
        wall_start.elapsed().as_secs_f64(),
        origin,
        plant.state(),
        BridgeStats::default(),
    ))
}

/// Runs Tier-0 zero-overhead `AviateDirectRunner` (`FixedWingController`) driving
/// JSBSim (`c172` or `light-trainer`) in stabilized cruise over `duration_s`.
pub fn run_tier0_fixed_wing_cruise(
    vehicle: FixedWingVehicle,
    origin: Geodetic,
    duration_s: f64,
    noise: DirectNoiseTier,
    noise_seed: u64,
) -> Result<RigSummary, IronbirdError> {
    let mut plant = JsbsimPlant::new_cruise(vehicle, origin)?;
    let period = plant.period();
    let total_steps = steps_for(duration_s, period.as_seconds());
    let kernel = build_direct_fixed_wing_kernel().ok_or(IronbirdError::FixedWingKernelBuild)?;
    let mut fc = AviateDirectRunner::new(
        kernel,
        AviateDirectConfig {
            noise,
            noise_seed,
            ..AviateDirectConfig::ideal(origin)
        },
    )?;

    let wall_start = Instant::now();
    let mut now = SimInstant::START;
    for _ in 0..500 {
        if fc.is_ready() {
            break;
        }
        let _ = fc.step_cg(now, &plant.state(), plant.specific_force_frd_m_s2())?;
        now = now.advanced(period);
    }

    let mut seq = 1_u32;
    let cruise_cmd = |sequence: u32| {
        SystemCommand::FlightControl(Command {
            mode: ControlMode::Attitude,
            setpoint: Setpoint {
                attitude: Some(Quaternion::IDENTITY),
                collective_thrust: NormalizedThrust(0.68),
                ..Setpoint::default()
            },
            config_mode_request: None,
            sensor_overrides: None,
            sequence,
            source: CommandSource::Gcs,
        })
    };
    fc.send_command(SystemCommand::Arm);
    let _ = fc.step_cg(now, &plant.state(), plant.specific_force_frd_m_s2())?;
    now = now.advanced(period);
    fc.send_command(cruise_cmd(seq));

    for k in 0..total_steps {
        if k > 0 && k % 12 == 0 {
            seq = seq.wrapping_add(1);
            fc.send_command(cruise_cmd(seq));
        }
        let act = fc.step_cg(now, &plant.state(), plant.specific_force_frd_m_s2())?;
        let controls = direct_fixed_wing_controls(&act);
        let _ = plant.step(controls)?;
        now = now.advanced(period);
    }

    Ok(finish_summary(
        total_steps,
        period.as_seconds(),
        wall_start.elapsed().as_secs_f64(),
        origin,
        plant.state(),
        BridgeStats::default(),
    ))
}

/// Runs Tier-1 [`AviateSharedMemoryEndpoint`] (`SharedStateV2` seqlock) driving
/// `X500RigidPlant` through [`ExternalNodeBridge`].
pub fn run_tier1_shm_x500<F: FnMut(&mut SharedStateV2)>(
    origin: Geodetic,
    duration_s: f64,
    mode: HitlSyncMode,
    peer: F,
) -> Result<RigSummary, IronbirdError> {
    let mut plant = X500RigidPlant::new_at(origin)?;
    let period = plant.period();
    let total_steps = steps_for(duration_s, period.as_seconds());
    let world = EnuWorld::new(&BodyEllipsoid::WGS84, &origin);
    let ep = AviateSharedMemoryEndpoint::attach(new_shared_state(1), world, peer)?;
    let mut bridge = ExternalNodeBridge::new(ep, mode);

    let wall_start = Instant::now();
    let mut now = SimInstant::START;
    let mut thrusters = Vec::new();
    for _ in 0..total_steps {
        let stim = AviateStimulus {
            vehicle: plant.state(),
            site: plant.site_kinematics()?,
        };
        bridge.stimulate(now, &stim)?;
        if let Some(delivered) = bridge.poll(now)?.last() {
            thrusters = plant.rotors().thrusters(&delivered.response)?;
        }
        let _ = plant.step(thrusters.clone())?;
        now = now.advanced(period);
    }

    Ok(finish_summary(
        total_steps,
        period.as_seconds(),
        wall_start.elapsed().as_secs_f64(),
        origin,
        plant.state(),
        bridge.stats(),
    ))
}

/// Runs Tier-2 MAVLink v2 HIL (`MavlinkHilTcpEndpoint` or `MavlinkHilUdpEndpoint`)
/// driving `JsbsimPlant` through [`ExternalNodeBridge`].
#[cfg(not(target_arch = "wasm32"))]
pub fn run_tier2_mavlink_fixed_wing<E>(
    vehicle: FixedWingVehicle,
    origin: Geodetic,
    duration_s: f64,
    mode: HitlSyncMode,
    mapping: HilControlMapping,
    endpoint: E,
) -> Result<RigSummary, IronbirdError>
where
    E: ExternalEndpoint<Stimulus = XilStimulus, Response = HilActuatorControls>,
{
    let mut plant = JsbsimPlant::new_cruise(vehicle, origin)?;
    let period = plant.period();
    let total_steps = steps_for(duration_s, period.as_seconds());
    let mut bridge = ExternalNodeBridge::new(endpoint, mode);

    let wall_start = Instant::now();
    let mut now = SimInstant::START;
    let mut controls = ControlInputs {
        throttle: 0.65,
        ..ControlInputs::default()
    };
    for _ in 0..total_steps {
        let stim = plant.xil_stimulus()?;
        bridge.stimulate(now, &stim)?;
        if let Some(delivered) = bridge.poll(now)?.last() {
            controls = delivered.response.to_control_inputs(&mapping);
        }
        let _ = plant.step(controls)?;
        now = now.advanced(period);
    }

    Ok(finish_summary(
        total_steps,
        period.as_seconds(),
        wall_start.elapsed().as_secs_f64(),
        origin,
        plant.state(),
        bridge.stats(),
    ))
}

/// Runs Tier-2 [`BetaflightUdpEndpoint`] driving `X500RigidPlant` through
/// [`ExternalNodeBridge`].
#[cfg(not(target_arch = "wasm32"))]
pub fn run_tier2_betaflight_x500(
    origin: Geodetic,
    duration_s: f64,
    mode: HitlSyncMode,
    endpoint: BetaflightUdpEndpoint,
) -> Result<RigSummary, IronbirdError> {
    let mut plant = X500RigidPlant::new_at(origin)?;
    let period = plant.period();
    let total_steps = steps_for(duration_s, period.as_seconds());
    let mut bridge = ExternalNodeBridge::new(endpoint, mode);

    let wall_start = Instant::now();
    let mut now = SimInstant::START;
    let mut latest_servo: Option<BetaflightServoPacket> = None;
    for _ in 0..total_steps {
        let stim = plant.xil_stimulus()?;
        bridge.stimulate(now, &stim)?;
        if let Some(delivered) = bridge.poll(now)?.last() {
            latest_servo = Some(delivered.response);
        }
        let thrusters = match &latest_servo {
            Some(servo) => betaflight_thrusters(&plant, servo),
            None => Vec::new(),
        };
        let _ = plant.step(thrusters)?;
        now = now.advanced(period);
    }

    Ok(finish_summary(
        total_steps,
        period.as_seconds(),
        wall_start.elapsed().as_secs_f64(),
        origin,
        plant.state(),
        bridge.stats(),
    ))
}

#[cfg(not(target_arch = "wasm32"))]
fn betaflight_thrusters(
    plant: &X500RigidPlant,
    servo: &BetaflightServoPacket,
) -> Vec<sim_core::ThrusterWrench> {
    let mut cmd = sim_aviate::DirectActuatorCmd {
        count: 4,
        armed: true,
        ..sim_aviate::DirectActuatorCmd::default()
    };
    for (idx, val) in servo.motors.into_iter().enumerate() {
        if let Some(slot) = cmd.outputs.get_mut(idx) {
            *slot = val;
        }
    }
    plant.rotors().direct_thrusters(&cmd).unwrap_or_default()
}

fn steps_for(duration_s: f64, dt_s: f64) -> u64 {
    ((duration_s.max(dt_s)) / dt_s).round() as u64
}

fn finish_summary(
    steps_executed: u64,
    dt_s: f64,
    wall_seconds: f64,
    initial_geodetic: Geodetic,
    final_state: VehicleState,
    bridge_stats: BridgeStats,
) -> RigSummary {
    let simulated_seconds = (steps_executed as f64) * dt_s;
    let safe_wall = wall_seconds.max(1.0e-9);
    RigSummary {
        steps_executed,
        simulated_seconds,
        wall_seconds,
        realtime_speedup: simulated_seconds / safe_wall,
        initial_geodetic,
        final_geodetic: ecef_to_geodetic(final_state.position_ecef),
        final_state,
        bridge_stats,
    }
}
