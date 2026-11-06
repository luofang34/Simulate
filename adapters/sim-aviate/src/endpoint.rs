use std::collections::VecDeque;
use std::time::Duration;

use aviate_xil_contract::{FcState, SharedStateV2, unpack_fc_status};
use mbd_core::LocalSiteKinematics;
use sim_core::{
    EndpointError, ExternalEndpoint, ExternalNodeBridge, SimInstant, StampedResponse,
    TICKS_PER_SECOND, VehicleState,
};

use crate::wire::{publish_model_state, read_motor_command};
use crate::{AviateBridgeError, EnuWorld, MotorCommand, validate_shared_state};

mod boundary;
#[cfg(test)]
mod tests;

pub use boundary::{AviateExecutionBoundary, Tier1StepMetrics};

const TICKS_PER_MICROSECOND: u64 = TICKS_PER_SECOND / 1_000_000;
// `time_us` truncates only what a tick count below one microsecond can hold.
const _: () = assert!(TICKS_PER_SECOND.is_multiple_of(1_000_000));

/// The vehicle at one instant, as the flight controller's sensors would feel
/// it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AviateStimulus {
    /// Centre-of-gravity state.
    pub vehicle: VehicleState,
    /// The mounting site the published lanes describe, from
    /// [`sim_core::RigidBody6DofBlock::site_kinematics`] or
    /// [`mbd_core::compute_site_kinematics`]. Its specific force has no lane:
    /// the contract's reader derives its sensors from the pose and velocity.
    pub site: LocalSiteKinematics,
}

/// What the flight controller left in the block for one published step.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FlightControllerCommand {
    /// The step this answers.
    pub sim_step: u64,
    /// The rotor setpoints.
    pub motors: MotorCommand,
    /// The controller's published state.
    pub fc_state: FcState,
    /// Whether `fc_state` is [`FcState::Ready`] for the current world epoch;
    /// the contract counts `Ready` for no other.
    pub fc_ready: bool,
}

/// [`ExternalEndpoint`] over a [`SharedStateV2`] this endpoint owns, playing
/// the Gazebo plugin against `peer`, the flight controller.
///
/// `peer` runs between each publish and read with exclusive access to the
/// block, and may touch only what the contract gives a flight controller. The
/// header is validated before every publish and again before the command is
/// interpreted, so a peer that corrupts the block fails the exchange.
///
/// The peer is a callback on the calling thread, so this endpoint's
/// [`Self::execution_boundary`] is always
/// [`AviateExecutionBoundary::InProcessSynchronous`]: harnesses must not read
/// its timing as evidence about an out-of-process controller.
pub struct AviateSharedMemoryEndpoint<F> {
    shared: SharedStateV2,
    world: EnuWorld,
    peer: F,
    sim_step: u64,
    answered: VecDeque<StampedResponse<FlightControllerCommand>>,
    metrics: Tier1StepMetrics,
    step_budget: Option<Duration>,
}

/// A simulation's link to a flight controller over the Aviate contract.
pub type AviateXilBridge<F> = ExternalNodeBridge<AviateSharedMemoryEndpoint<F>>;

impl<F: FnMut(&mut SharedStateV2)> AviateSharedMemoryEndpoint<F> {
    /// Attaches to `shared`, refusing a block that fails
    /// [`validate_shared_state`]. Steps continue from the block's own count,
    /// which stays monotonic across world resets.
    pub fn attach(
        shared: SharedStateV2,
        world: EnuWorld,
        peer: F,
    ) -> Result<Self, AviateBridgeError> {
        validate_shared_state(&shared)?;
        Ok(Self {
            sim_step: shared.state.sim_step,
            shared,
            world,
            peer,
            answered: VecDeque::new(),
            metrics: Tier1StepMetrics::default(),
            step_budget: None,
        })
    }

    /// Times every peer call against `budget`, counting
    /// [`Tier1StepMetrics::budget_overruns`]. Timing is diagnostic only and
    /// never changes what the simulation sees.
    #[must_use]
    pub fn with_step_budget(mut self, budget: Duration) -> Self {
        self.step_budget = Some(budget);
        self
    }

    /// Where the peer runs: always on the calling thread.
    #[must_use]
    pub const fn execution_boundary(&self) -> AviateExecutionBoundary {
        AviateExecutionBoundary::InProcessSynchronous
    }

    /// Publish, peer, and read counters since attaching.
    #[must_use]
    pub const fn metrics(&self) -> Tier1StepMetrics {
        self.metrics
    }

    /// The block as the last exchange left it.
    pub fn shared(&self) -> &SharedStateV2 {
        &self.shared
    }

    /// One publish, peer and read cycle.
    pub fn round_trip(
        &mut self,
        at: SimInstant,
        stimulus: &AviateStimulus,
    ) -> Result<FlightControllerCommand, AviateBridgeError> {
        validate_shared_state(&self.shared)?;
        let step = self.sim_step.wrapping_add(1);
        let lanes = self.world.model_lanes(stimulus);
        publish_model_state(&mut self.shared, step, micros_since_start(at)?, &lanes)?;
        self.sim_step = step;
        self.metrics.on_publish(step);
        self.run_peer();

        validate_shared_state(&self.shared)?;
        let acknowledged = self.shared.command.fc_step_ack;
        self.metrics.on_acknowledgement(step, acknowledged);
        if self.shared.control.lockstep_enabled != 0 && acknowledged != step {
            return Err(AviateBridgeError::StepNotAcknowledged {
                expected: step,
                acknowledged,
            });
        }
        let motors = read_motor_command(&mut self.shared)?;
        // Control words are single atomic fields, outside any seqlock.
        let (generation, fc_state) = unpack_fc_status(self.shared.control.fc_status);
        Ok(FlightControllerCommand {
            sim_step: step,
            motors,
            fc_state,
            fc_ready: fc_state == FcState::Ready
                && generation == self.shared.header.reset_generation,
        })
    }
}

impl<F: FnMut(&mut SharedStateV2)> AviateSharedMemoryEndpoint<F> {
    fn run_peer(&mut self) {
        let Some(budget) = self.step_budget else {
            (self.peer)(&mut self.shared);
            return;
        };
        let started = web_time::Instant::now();
        (self.peer)(&mut self.shared);
        self.metrics.on_peer_wall(started.elapsed(), budget);
    }
}

fn micros_since_start(at: SimInstant) -> Result<u64, AviateBridgeError> {
    let (seconds, subsec_ticks) = at.seconds_and_subsec_ticks();
    seconds
        .checked_mul(1_000_000)
        .and_then(|micros| micros.checked_add(subsec_ticks / TICKS_PER_MICROSECOND))
        .ok_or(AviateBridgeError::TimeOutOfRange { seconds })
}

fn endpoint_error(error: AviateBridgeError) -> EndpointError {
    EndpointError {
        detail: error.to_string(),
    }
}

impl<F: FnMut(&mut SharedStateV2)> ExternalEndpoint for AviateSharedMemoryEndpoint<F> {
    type Stimulus = AviateStimulus;
    type Response = FlightControllerCommand;

    fn exchange_blocking(
        &mut self,
        at: SimInstant,
        stimulus: &AviateStimulus,
    ) -> Result<FlightControllerCommand, EndpointError> {
        self.round_trip(at, stimulus).map_err(endpoint_error)
    }

    /// Runs the same cycle, since the peer shares this thread, and stamps the
    /// answer for `at`: an in-process peer has no latency of its own to report.
    fn post(&mut self, at: SimInstant, stimulus: &AviateStimulus) -> Result<(), EndpointError> {
        let response = self.round_trip(at, stimulus).map_err(endpoint_error)?;
        self.answered.push_back(StampedResponse {
            deliver_at: at,
            response,
        });
        Ok(())
    }

    fn try_receive(
        &mut self,
    ) -> Result<Option<StampedResponse<FlightControllerCommand>>, EndpointError> {
        Ok(self.answered.pop_front())
    }
}
