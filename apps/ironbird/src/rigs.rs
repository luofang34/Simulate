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
#[cfg(not(target_arch = "wasm32"))]
use sim_core::ExternalEndpoint;
use sim_core::{
    Block, BridgeStats, ControlInputs, ExternalNodeBridge, HitlSyncMode, SensorExposureEvent,
    SimInstant, VehicleState,
};
use sim_geo::{BodyEllipsoid, Geodetic, ecef_to_geodetic};
use sim_pack::{LoadedSyspack, builtin_c172p_package, builtin_c172s_package};
#[cfg(not(target_arch = "wasm32"))]
use sim_xil::{
    XilStimulus,
    betaflight::{BetaflightServoPacket, BetaflightUdpEndpoint},
    mavlink_hil::{HilActuatorControls, HilControlMapping},
};
use std::time::Duration;

use sim_xil::{PacerStats, RealtimePacer};
use web_time::Instant;

use crate::IronbirdError;
use crate::plant::{FixedWingVehicle, JsbsimPlant, X500RigidPlant};

mod rig_session;

use rig_session::RigSession;
pub use rig_session::{OPTICAL_CHANNEL, OPTICAL_PERIOD};

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
    /// Wall-clock pacing health; zeroed unless the run used
    /// [`HitlSyncMode::RealTimeStamped`].
    pub pacer_stats: PacerStats,
    /// Exposures of the 30 Hz [`OPTICAL_CHANNEL`] over every plant step the
    /// run's [`sim_core::SimSession`] ran (controller warm-up never steps the
    /// plant).
    pub optical_exposures: u64,
    /// The newest of those exposures, with the interpolated vehicle state it
    /// saw.
    pub last_optical_exposure: Option<SensorExposureEvent>,
}

/// Lateness a real-time step may accumulate before it counts as an overrun.
const REALTIME_OVERRUN_TOLERANCE: Duration = Duration::from_millis(2);

/// Paces real-time runs to wall-clock; lockstep runs go as fast as the peer.
struct ModePacer(Option<RealtimePacer>);

impl ModePacer {
    fn for_mode(mode: HitlSyncMode) -> Self {
        Self((mode == HitlSyncMode::RealTimeStamped).then(RealtimePacer::new))
    }

    fn pace_until(&mut self, now: SimInstant) {
        if let Some(pacer) = &mut self.0 {
            pacer.pace_until(now, REALTIME_OVERRUN_TOLERANCE);
        }
    }

    fn stats(&self) -> PacerStats {
        self.0
            .as_ref()
            .map(RealtimePacer::stats)
            .unwrap_or_default()
    }
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
    let mut rig = RigSession::new(X500RigidPlant::new_at(origin)?, Vec::new())?;
    let period = rig.period();
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
    let mut warm_up_end = SimInstant::START;
    let rest_f = DVec3::new(0.0, 0.0, -9.806_65);
    for _ in 0..500 {
        if fc.is_ready() {
            break;
        }
        let _ = fc.step_cg(warm_up_end, &rig.truth_state(), rest_f)?;
        warm_up_end = warm_up_end.advanced(period);
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
    let act0 = fc.step_cg(rig.clock_from(warm_up_end), &rig.truth_state(), rest_f)?;
    let thrusters = rig.plant().rotors().direct_thrusters(&act0)?;
    rig.step(thrusters)?;

    for k in 0..total_steps {
        if k > 0 && k % 40 == 0 {
            seq = seq.wrapping_add(1);
            fc.send_command(hold_cmd(seq));
        }
        let stim = AviateStimulus {
            vehicle: rig.truth_state(),
            site: rig.plant().site_kinematics()?,
        };
        let act = fc.step_site(rig.clock_from(warm_up_end), &stim)?;
        let thrusters = rig.plant().rotors().direct_thrusters(&act)?;
        rig.step(thrusters)?;
    }

    Ok(finish_summary(
        &rig,
        total_steps,
        origin,
        wall_start,
        BridgeStats::default(),
        PacerStats::default(),
    ))
}

/// Runs Tier-0 zero-overhead `AviateDirectRunner` (`FixedWingController`) driving
/// a built-in JSBSim aircraft in stabilized cruise over `duration_s`.
pub fn run_tier0_fixed_wing_cruise(
    vehicle: FixedWingVehicle,
    origin: Geodetic,
    duration_s: f64,
    noise: DirectNoiseTier,
    noise_seed: u64,
) -> Result<RigSummary, IronbirdError> {
    let package = match vehicle {
        FixedWingVehicle::C172 => builtin_c172s_package()?,
        FixedWingVehicle::C172p => builtin_c172p_package()?,
    };
    run_tier0_package_cruise(&package, origin, duration_s, noise, noise_seed)
}

/// Runs Tier-0 zero-overhead `AviateDirectRunner` (`FixedWingController`) driving
/// any verified fixed-wing [`LoadedSyspack`] in stabilized cruise over `duration_s`.
pub fn run_tier0_package_cruise(
    package: &LoadedSyspack,
    origin: Geodetic,
    duration_s: f64,
    noise: DirectNoiseTier,
    noise_seed: u64,
) -> Result<RigSummary, IronbirdError> {
    let speed_m_s = package
        .airspeed_envelope()
        .map_or(50.0, |env| f64::from(env.vno_kt) * 0.40);
    let plant = JsbsimPlant::from_package(package, origin, speed_m_s)?;
    let mut rig = RigSession::new(plant, ControlInputs::default())?;
    let period = rig.period();
    let total_steps = steps_for(duration_s, period.as_seconds());
    let kernel = build_direct_fixed_wing_kernel(&sim_aviate::c172_airframe())
        .ok_or(IronbirdError::FixedWingKernelBuild)?;
    let mut fc = AviateDirectRunner::new(
        kernel,
        AviateDirectConfig {
            noise,
            noise_seed,
            ..AviateDirectConfig::ideal(origin)
        },
    )?;

    let wall_start = Instant::now();
    let mut warm_up_end = SimInstant::START;
    for _ in 0..500 {
        if fc.is_ready() {
            break;
        }
        let _ = fc.step_cg(warm_up_end, &rig.truth_state(), specific_force(&rig))?;
        warm_up_end = warm_up_end.advanced(period);
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
    let _ = fc.step_cg(warm_up_end, &rig.truth_state(), specific_force(&rig))?;
    let armed_at = warm_up_end.advanced(period);
    fc.send_command(cruise_cmd(seq));

    for k in 0..total_steps {
        if k > 0 && k % 12 == 0 {
            seq = seq.wrapping_add(1);
            fc.send_command(cruise_cmd(seq));
        }
        let fc_now = rig.clock_from(armed_at);
        let act = fc.step_cg(fc_now, &rig.truth_state(), specific_force(&rig))?;
        rig.step(direct_fixed_wing_controls(&act))?;
    }

    Ok(finish_summary(
        &rig,
        total_steps,
        origin,
        wall_start,
        BridgeStats::default(),
        PacerStats::default(),
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
    let mut rig = RigSession::new(X500RigidPlant::new_at(origin)?, Vec::new())?;
    let total_steps = steps_for(duration_s, rig.period().as_seconds());
    let world = EnuWorld::new(&BodyEllipsoid::WGS84, &origin);
    let ep = AviateSharedMemoryEndpoint::attach(new_shared_state(1), world, peer)?;
    let mut bridge = ExternalNodeBridge::new(ep, mode);
    let mut pacer = ModePacer::for_mode(mode);

    let wall_start = Instant::now();
    let mut thrusters = Vec::new();
    for _ in 0..total_steps {
        let now = rig.now();
        let stim = AviateStimulus {
            vehicle: rig.truth_state(),
            site: rig.plant().site_kinematics()?,
        };
        bridge.stimulate(now, &stim)?;
        if let Some(delivered) = bridge.poll(now)?.last() {
            thrusters = rig.plant().rotors().thrusters(&delivered.response)?;
        }
        rig.step(thrusters.clone())?;
        pacer.pace_until(rig.now());
    }

    Ok(finish_summary(
        &rig,
        total_steps,
        origin,
        wall_start,
        bridge.stats(),
        pacer.stats(),
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
    let mut controls = ControlInputs {
        throttle: 0.65,
        ..ControlInputs::default()
    };
    let mut rig = RigSession::new(JsbsimPlant::new_cruise(vehicle, origin)?, controls)?;
    let total_steps = steps_for(duration_s, rig.period().as_seconds());
    let mut bridge = ExternalNodeBridge::new(endpoint, mode);
    let mut pacer = ModePacer::for_mode(mode);

    let wall_start = Instant::now();
    for _ in 0..total_steps {
        let now = rig.now();
        let stim = rig.plant().xil_stimulus()?;
        bridge.stimulate(now, &stim)?;
        if let Some(delivered) = bridge.poll(now)?.last() {
            controls = delivered.response.to_control_inputs(&mapping);
        }
        rig.step(controls)?;
        pacer.pace_until(rig.now());
    }

    Ok(finish_summary(
        &rig,
        total_steps,
        origin,
        wall_start,
        bridge.stats(),
        pacer.stats(),
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
    let mut rig = RigSession::new(X500RigidPlant::new_at(origin)?, Vec::new())?;
    let total_steps = steps_for(duration_s, rig.period().as_seconds());
    let mut bridge = ExternalNodeBridge::new(endpoint, mode);
    let mut pacer = ModePacer::for_mode(mode);

    let wall_start = Instant::now();
    let mut latest_servo: Option<BetaflightServoPacket> = None;
    for _ in 0..total_steps {
        let now = rig.now();
        let stim = rig.plant().xil_stimulus()?;
        bridge.stimulate(now, &stim)?;
        if let Some(delivered) = bridge.poll(now)?.last() {
            latest_servo = Some(delivered.response);
        }
        let thrusters = match &latest_servo {
            Some(servo) => betaflight_thrusters(rig.plant(), servo),
            None => Vec::new(),
        };
        rig.step(thrusters)?;
        pacer.pace_until(rig.now());
    }

    Ok(finish_summary(
        &rig,
        total_steps,
        origin,
        wall_start,
        bridge.stats(),
        pacer.stats(),
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

fn specific_force(rig: &RigSession<JsbsimPlant>) -> DVec3 {
    rig.plant().specific_force_frd_m_s2()
}

fn finish_summary<B>(
    rig: &RigSession<B>,
    steps_executed: u64,
    initial_geodetic: Geodetic,
    wall_start: Instant,
    bridge_stats: BridgeStats,
    pacer_stats: PacerStats,
) -> RigSummary
where
    B: Block<Output = VehicleState>,
    B::Input: Clone,
{
    let wall_seconds = wall_start.elapsed().as_secs_f64();
    let final_state = rig.truth_state();
    let simulated_seconds = (steps_executed as f64) * rig.period().as_seconds();
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
        pacer_stats,
        optical_exposures: rig.optical_exposures(),
        last_optical_exposure: rig.last_optical_exposure(),
    }
}
