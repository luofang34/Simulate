//! The Aviate flight controller and the X500 behind Pilotage's
//! `VehicleAdapter`.
//!
//! Commands reach the flight controller as MAVLink bytes in its receive ring
//! and its telemetry comes back as MAVLink bytes from its transmit ring; the
//! adapter holds no control law. Truth moves the airframe and feeds the
//! flight controller's sensor synthesis, and leaves upward only as a
//! [`pilotage_adapter_api::SimTruthSample`].

mod downlink;
mod truth;
mod uplink;

#[cfg(test)]
mod tests;

use aviate_core::control::ControlMode;
use aviate_core::control::multirotor::MultirotorController;
use aviate_core::mixer::QuadXMixerX500;
use glam::DVec3;
use pilotage_adapter_api::{
    ActionCapability, ActionResult, AdapterCapabilities, ApplyOutcome, Disposition, ExecutionMode,
    IntentCapability, LinkLossEnactError, LinkLossPolicy, RejectReason, ScopeDescriptor,
    StepBudget, StepOutcome, TelemetryBatch, VehicleAdapter, VehicleDescriptor, VideoSource,
};
use pilotage_protocol::{
    ActionKind, ControlAction, ControlIntent, DIRECT_SCOPE, HeadingReference, IntentFamily,
    MOTION_SCOPE, ReferenceFrame, ScopeId, ScopedControlFrame, VehicleClass, VehicleId,
};
use pilotage_timing::SimTick;
use sim_aviate::{
    AviateDirectConfig, AviateDirectRunner, DirectNoiseTier, build_direct_x500_kernel,
};
use sim_core::{SimInstant, TICKS_PER_SECOND, TickSpan, VehicleState};
use sim_geo::Geodetic;

use crate::{ChainError, PlantStimulus, X500Plant};
use downlink::Downlink;
use truth::TruthReporter;
use uplink::Uplink;

/// Rate of the airframe and of the flight controller's cycle, hertz.
pub const FLIGHT_RATE_HZ: u64 = 480;

const ADAPTER_VERSION: &str = env!("CARGO_PKG_VERSION");
/// Largest transmit burst drained from the flight controller per cycle.
const TELEMETRY_BUFFER_BYTES: usize = 2_048;

/// One X500 on its pad.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AviateVehicleConfig {
    /// The vehicle this adapter exposes.
    pub vehicle: VehicleId,
    /// The pad, which is also the origin of the flight controller's local
    /// north-east-down frame.
    pub pad: Geodetic,
    /// Heading on the pad from true north, radians.
    pub heading_rad: f64,
    /// Noise of the sensors the flight controller synthesises.
    pub noise: DirectNoiseTier,
    /// Seed of that noise.
    pub noise_seed: u64,
}

/// The Aviate X500 kernel flying the [`X500Plant`], commanded and observed
/// through Pilotage's adapter API.
pub struct AviateX500Vehicle {
    config: AviateVehicleConfig,
    plant: X500Plant,
    /// On the heap: the board carries its rings and filter by value.
    controller: Box<AviateDirectRunner<MultirotorController, QuadXMixerX500>>,
    uplink: Uplink,
    downlink: Downlink,
    truth: TruthReporter,
    tick: SimTick,
    now: SimInstant,
    refused_uplink_bytes: u64,
    fault: Option<ChainError>,
}

impl AviateX500Vehicle {
    /// The vehicle at rest on its pad, flight controller booting.
    pub fn new(config: AviateVehicleConfig) -> Result<Self, ChainError> {
        if !TICKS_PER_SECOND.is_multiple_of(FLIGHT_RATE_HZ) {
            return Err(ChainError::Rate {
                rate_hz: FLIGHT_RATE_HZ,
            });
        }
        let period = TickSpan::from_ticks(TICKS_PER_SECOND / FLIGHT_RATE_HZ);
        let plant = X500Plant::on_pad(config.pad, config.heading_rad, period)?;
        let kernel = build_direct_x500_kernel().ok_or(ChainError::KernelBuild)?;
        let controller = Box::new(
            AviateDirectRunner::new(
                kernel,
                AviateDirectConfig {
                    noise: config.noise,
                    noise_seed: config.noise_seed,
                    ..AviateDirectConfig::ideal(config.pad)
                },
            )
            .map_err(|source| ChainError::FlightController { at_s: 0.0, source })?,
        );
        Ok(Self {
            truth: TruthReporter::new(&config),
            config,
            plant,
            controller,
            uplink: Uplink::default(),
            downlink: Downlink::new(),
            tick: SimTick::new(0),
            now: SimInstant::START,
            refused_uplink_bytes: 0,
            fault: None,
        })
    }

    /// The instant the vehicle state describes.
    pub const fn now(&self) -> SimInstant {
        self.now
    }

    /// The true vehicle state.
    pub fn true_state(&self) -> VehicleState {
        self.plant.state()
    }

    /// Whether the pad still carries the vehicle.
    pub const fn is_on_pad(&self) -> bool {
        self.plant.is_on_pad()
    }

    /// The true specific force at the centre of gravity in body
    /// forward-right-down axes, m/s².
    pub fn true_specific_force_frd_m_s2(&self) -> DVec3 {
        self.plant.specific_force_frd_m_s2()
    }

    /// Whether the flight controller's last heartbeat reported it armed.
    pub fn reports_armed(&self) -> Option<bool> {
        self.downlink.reports_armed()
    }

    /// The control mode the flight controller flew on its last cycle.
    pub fn flight_mode(&self) -> ControlMode {
        self.controller.board().last_effective_command().mode
    }

    /// The flight controller's estimate of the vehicle state, when it has a
    /// usable one.
    pub fn estimated_state(&self) -> Option<VehicleState> {
        self.controller.estimated_state()
    }

    /// Command bytes the flight controller's receive ring had no room for.
    pub const fn refused_uplink_bytes(&self) -> u64 {
        self.refused_uplink_bytes
    }

    /// Why stepping stopped, once it has.
    pub const fn fault(&self) -> Option<&ChainError> {
        self.fault.as_ref()
    }

    /// Takes the reason stepping stopped, leaving the vehicle stopped.
    pub(crate) fn take_fault(&mut self) -> Option<ChainError> {
        self.fault.replace(ChainError::Stopped)
    }

    fn send(&mut self, frame: &[u8]) {
        let accepted = self.controller.push_mavlink_rx(frame);
        let refused = frame.len().saturating_sub(accepted) as u64;
        self.refused_uplink_bytes = self.refused_uplink_bytes.wrapping_add(refused);
    }

    fn refuse(&self, reason: RejectReason) -> ApplyOutcome {
        ApplyOutcome::new(self.tick, Disposition::Rejected(reason))
    }

    /// Sends the frame's discrete actions; only arm and disarm reach Aviate.
    fn send_actions(&mut self, actions: &[ControlAction]) -> Vec<ActionResult> {
        actions
            .iter()
            .map(|action| match action {
                ControlAction::Arm | ControlAction::Disarm => {
                    let frame = self.uplink.arm_frame(*action == ControlAction::Arm);
                    self.send(&frame);
                    ActionResult::accepted(*action)
                }
                _ => ActionResult::rejected(*action, "Aviate takes only arm and disarm"),
            })
            .collect()
    }

    fn step_once(&mut self) -> Result<(), ChainError> {
        let at_s = self.now.duration_since(SimInstant::START).as_seconds();
        let command = match self.plant.stimulus()? {
            PlantStimulus::Resting {
                vehicle,
                specific_force_frd_m_s2,
            } => self
                .controller
                .step_cg(self.now, &vehicle, specific_force_frd_m_s2),
            PlantStimulus::Flying(stimulus) => self.controller.step_site(self.now, &stimulus),
        }
        .map_err(|source| ChainError::FlightController { at_s, source })?;
        self.plant.step(&command)?;
        self.tick = self.tick.next();
        self.now = self.now.advanced(self.plant.period());
        let mut buffer = [0_u8; TELEMETRY_BUFFER_BYTES];
        loop {
            let read = self.controller.pop_mavlink_tx(&mut buffer);
            let Some(bytes) = buffer.get(..read).filter(|bytes| !bytes.is_empty()) else {
                break;
            };
            self.downlink.receive(bytes, self.now);
        }
        Ok(())
    }
}

fn action(action: ActionKind) -> ActionCapability {
    ActionCapability {
        action,
        mode_targets: Vec::new(),
        feel_targets: Vec::new(),
        takeoff_start_state: None,
    }
}

fn scope(name: &str, intent: IntentCapability) -> ScopeDescriptor {
    ScopeDescriptor {
        payload: None,
        authority_group: Some(MOTION_SCOPE.to_owned()),
        scope: ScopeId::new(name),
        axes: Vec::new(),
        intents: vec![intent],
        actions: vec![action(ActionKind::Arm), action(ActionKind::Disarm)],
        legacy: None,
    }
}

impl VehicleAdapter for AviateX500Vehicle {
    fn capabilities(&self) -> AdapterCapabilities {
        // The envelope is the flight controller's own configured limits.
        let limits = &self.controller.board().kernel().cfg().limits;
        let velocity = IntentCapability {
            family: IntentFamily::Velocity,
            frames: vec![ReferenceFrame::BodyFrd, ReferenceFrame::LocalNed],
            max_linear: limits.max_horizontal_speed.0,
            max_vertical: limits.max_climb_rate.0.min(limits.max_descent_rate.0),
            max_angular: limits.max_yaw_rate.0,
            max_yaw_rate: 0.0,
        };
        let attitude = IntentCapability {
            family: IntentFamily::AttitudeThrust,
            frames: vec![ReferenceFrame::LocalNed],
            max_linear: 0.0,
            max_vertical: 0.0,
            max_angular: limits.max_roll.0.min(limits.max_pitch.0),
            max_yaw_rate: limits.max_yaw_rate.0,
        };
        AdapterCapabilities {
            execution: ExecutionMode {
                real_time: false,
                stepped: true,
                accelerated: true,
                deterministic: true,
                render_capable: false,
                physically_embodied: false,
            },
            vehicles: vec![VehicleDescriptor {
                id: self.config.vehicle,
                class: VehicleClass::Multicopter,
                heading_reference: Some(HeadingReference::True),
                display_name: "X500 (Aviate)".to_owned(),
                scopes: vec![scope(MOTION_SCOPE, velocity), scope(DIRECT_SCOPE, attitude)],
                link_loss_actions: Vec::new(),
            }],
            adapter_version: ADAPTER_VERSION.to_owned(),
            control_feel: None,
        }
    }

    fn apply_control(&mut self, frame: &ScopedControlFrame) -> ApplyOutcome {
        if frame.vehicle != self.config.vehicle {
            return self.refuse(RejectReason::UnknownVehicle);
        }
        let scope = frame.scope.as_str();
        if scope != MOTION_SCOPE && scope != DIRECT_SCOPE {
            return self.refuse(RejectReason::UnknownScope);
        }
        // Like Pilotage's own Aviate adapter, nothing is commanded without
        // the flight controller's authorized pose: the heading every frame
        // is built on comes from it.
        let Some(pose) = self.downlink.pose() else {
            return self.refuse(RejectReason::MeasurementUnavailable);
        };
        let action_results = self.send_actions(&frame.actions);
        match (scope, frame.intent) {
            (_, None) => {}
            (MOTION_SCOPE, Some(ControlIntent::Velocity(velocity))) => {
                let yaw_rad = f64::from(pose.yaw_rad);
                let bytes = self.uplink.velocity_frame(&velocity, yaw_rad, self.now);
                self.send(&bytes);
            }
            (DIRECT_SCOPE, Some(ControlIntent::AttitudeThrust(attitude))) => {
                let bytes = self.uplink.attitude_frame(&attitude, self.now);
                self.send(&bytes);
            }
            _ => {
                let detail = "intent family does not belong to this scope".to_owned();
                return self.refuse(RejectReason::Other(detail));
            }
        }
        ApplyOutcome {
            tick: self.tick,
            disposition: Disposition::Accepted,
            action_results,
        }
    }

    fn sample_telemetry(&mut self) -> TelemetryBatch {
        let mut sample = self.downlink.sample(self.config.vehicle, self.tick);
        sample.sim_truth = Some(self.truth.sample(self.now, &self.plant.state()));
        TelemetryBatch {
            samples: vec![sample],
        }
    }

    fn video_sources(&self) -> Vec<VideoSource> {
        Vec::new()
    }

    fn set_link_loss_policy(
        &mut self,
        vehicle: VehicleId,
        _scope: &ScopeId,
        _policy: Option<LinkLossPolicy>,
    ) -> Result<(), LinkLossEnactError> {
        if vehicle != self.config.vehicle {
            return Err(LinkLossEnactError::UnknownVehicle { vehicle });
        }
        Err(LinkLossEnactError::NoActuationChannel)
    }

    fn step(&mut self, budget: StepBudget) -> StepOutcome {
        let mut advanced = 0_u32;
        while advanced < budget.ticks && self.fault.is_none() {
            match self.step_once() {
                Ok(()) => advanced = advanced.wrapping_add(1),
                Err(error) => {
                    tracing::error!(%error, "flight chain vehicle stopped stepping");
                    self.fault = Some(error);
                }
            }
        }
        StepOutcome {
            advanced,
            now: self.tick,
        }
    }
}
