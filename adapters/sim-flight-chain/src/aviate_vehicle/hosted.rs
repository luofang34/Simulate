//! Any Aviate board on its airframe behind Pilotage's `VehicleAdapter`.
//!
//! The board takes its sensors from the vehicle's own suite, so truth
//! reaches the flight controller through the devices alone and leaves
//! upward only as a [`pilotage_adapter_api::SimTruthSample`].

use aviate_core::control::{ControlMode, VehicleController};
use aviate_core::mixer::Mixer;
use pilotage_adapter_api::{
    ActionResult, AdapterCapabilities, ApplyOutcome, Disposition, ExecutionMode, IntentCapability,
    LinkLossEnactError, LinkLossPolicy, RejectReason, StepBudget, StepOutcome, TelemetryBatch,
    VehicleAdapter, VehicleDescriptor, VideoSource,
};
use pilotage_protocol::{
    ControlAction, ControlIntent, DIRECT_SCOPE, HeadingReference, IntentFamily, MOTION_SCOPE,
    ReferenceFrame, ScopeId, ScopedControlFrame, VehicleClass, VehicleId,
};
use pilotage_timing::SimTick;
use sim_core::{SimInstant, TickSpan, VehicleState};
use sim_geo::Geodetic;

use super::downlink::Downlink;
use super::truth::TruthReporter;
use super::uplink::Uplink;
use super::{ADAPTER_VERSION, TELEMETRY_BUFFER_BYTES, scope};
use crate::{Airframe, ChainError, HostedBoard};

/// How a vehicle presents itself to Pilotage.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct VehicleIdentity {
    /// The vehicle the adapter exposes.
    pub vehicle: VehicleId,
    /// Its class, which decides how a mission ends on it.
    pub class: VehicleClass,
    /// The name an operator reads.
    pub display_name: &'static str,
    /// Tells one run's simulation oracle from another's.
    pub run_seed: u64,
}

/// A [`HostedBoard`] commanded and observed through Pilotage's adapter API:
/// MAVLink bytes into the flight controller's receive ring, MAVLink bytes
/// out of its transmit ring, and no control law in between.
pub struct HostedVehicle<A, C, M>
where
    C: VehicleController,
    M: Mixer,
{
    identity: VehicleIdentity,
    board: HostedBoard<A, C, M>,
    uplink: Uplink,
    downlink: Downlink,
    truth: TruthReporter,
    tick: SimTick,
    refused_uplink_bytes: u64,
    fault: Option<ChainError>,
}

impl<A, C, M> HostedVehicle<A, C, M>
where
    A: Airframe,
    C: VehicleController,
    M: Mixer,
{
    /// `board`, whose flight controller measures its local frame from
    /// `origin`, presented as `identity`.
    pub fn new(identity: VehicleIdentity, board: HostedBoard<A, C, M>, origin: &Geodetic) -> Self {
        Self {
            truth: TruthReporter::new(identity.vehicle, identity.run_seed, origin),
            identity,
            board,
            uplink: Uplink::default(),
            downlink: Downlink::new(),
            tick: SimTick::new(0),
            refused_uplink_bytes: 0,
            fault: None,
        }
    }

    /// The board and its airframe.
    pub const fn board(&self) -> &HostedBoard<A, C, M> {
        &self.board
    }

    /// The board, to reach its devices.
    pub const fn board_mut(&mut self) -> &mut HostedBoard<A, C, M> {
        &mut self.board
    }

    /// The instant the vehicle state describes.
    pub const fn now(&self) -> SimInstant {
        self.board.now()
    }

    /// The flight controller's cycle.
    pub fn period(&self) -> TickSpan {
        self.board.airframe().period()
    }

    /// The true vehicle state.
    pub fn true_state(&self) -> VehicleState {
        self.board.true_state()
    }

    /// Whether the flight controller's last heartbeat reported it armed.
    pub fn reports_armed(&self) -> Option<bool> {
        self.downlink.reports_armed()
    }

    /// The control mode the flight controller flew on its last cycle.
    pub fn flight_mode(&self) -> ControlMode {
        self.board.flight_mode()
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
        let accepted = self.board.uplink(frame);
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

    /// Sends what `frame` carries. A velocity is turned into the flight
    /// controller's local frame with `yaw_rad`, the heading it reports.
    fn apply(&mut self, frame: &ScopedControlFrame, yaw_rad: Option<f64>) -> ApplyOutcome {
        let action_results = self.send_actions(&frame.actions);
        let now = self.now();
        match (frame.intent, yaw_rad) {
            (Some(ControlIntent::Velocity(velocity)), Some(yaw_rad)) => {
                let bytes = self.uplink.velocity_frame(&velocity, yaw_rad, now);
                self.send(&bytes);
            }
            (Some(ControlIntent::AttitudeThrust(attitude)), _) => {
                let bytes = self.uplink.attitude_frame(&attitude, now);
                self.send(&bytes);
            }
            _ => {}
        }
        ApplyOutcome {
            tick: self.tick,
            disposition: Disposition::Accepted,
            action_results,
        }
    }

    fn step_once(&mut self) -> Result<(), ChainError> {
        self.board.step()?;
        self.tick = self.tick.next();
        let mut buffer = [0_u8; TELEMETRY_BUFFER_BYTES];
        loop {
            let read = self.board.downlink(&mut buffer);
            let Some(bytes) = buffer.get(..read).filter(|bytes| !bytes.is_empty()) else {
                break;
            };
            self.downlink.receive(bytes, self.board.now());
        }
        Ok(())
    }
}

impl<A, C, M> VehicleAdapter for HostedVehicle<A, C, M>
where
    A: Airframe,
    C: VehicleController,
    M: Mixer,
{
    fn capabilities(&self) -> AdapterCapabilities {
        // The envelope is the flight controller's own configured limits.
        let limits = &self.board.controller().board().kernel().cfg().limits;
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
                id: self.identity.vehicle,
                class: self.identity.class,
                heading_reference: Some(HeadingReference::True),
                display_name: self.identity.display_name.to_owned(),
                scopes: vec![scope(MOTION_SCOPE, velocity), scope(DIRECT_SCOPE, attitude)],
                link_loss_actions: Vec::new(),
            }],
            adapter_version: ADAPTER_VERSION.to_owned(),
            control_feel: None,
        }
    }

    fn apply_control(&mut self, frame: &ScopedControlFrame) -> ApplyOutcome {
        if frame.vehicle != self.identity.vehicle {
            return self.refuse(RejectReason::UnknownVehicle);
        }
        match (frame.scope.as_str(), frame.intent) {
            // Velocity is turned into the flight controller's local frame
            // with the heading of its own authorized pose.
            (MOTION_SCOPE, None | Some(ControlIntent::Velocity(_))) => match self.downlink.pose() {
                Some(pose) => self.apply(frame, Some(f64::from(pose.yaw_rad))),
                None => self.refuse(RejectReason::MeasurementUnavailable),
            },
            // Attitude and thrust need the attitude estimate and nothing of
            // the position, so they still pass when the receiver is silent.
            (DIRECT_SCOPE, None | Some(ControlIntent::AttitudeThrust(_))) => {
                if self.downlink.attitude_authorized() {
                    self.apply(frame, None)
                } else {
                    self.refuse(RejectReason::MeasurementUnavailable)
                }
            }
            (MOTION_SCOPE | DIRECT_SCOPE, _) => {
                let detail = "intent family does not belong to this scope".to_owned();
                self.refuse(RejectReason::Other(detail))
            }
            _ => self.refuse(RejectReason::UnknownScope),
        }
    }

    fn sample_telemetry(&mut self) -> TelemetryBatch {
        let mut sample = self.downlink.sample(self.identity.vehicle, self.tick);
        sample.sim_truth = Some(self.truth.sample(self.now(), &self.true_state()));
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
        if vehicle != self.identity.vehicle {
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
                    tracing::error!(%error, "hosted vehicle stopped stepping");
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
