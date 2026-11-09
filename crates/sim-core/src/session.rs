use crate::{Block, BlockError, RateAdapter, SimInstant, TickSpan, VehicleState};

mod input_queue;
mod recovery;
mod sensors;
#[cfg(test)]
mod tests;

pub use input_queue::StampedInputQueue;
pub use recovery::{ExactSessionSnapshot, TimelineCheckpoint};
pub use sensors::{SensorChannelId, SensorExposureEvent, SensorSchedule};

/// Why a session operation failed.
#[derive(Debug, Clone, PartialEq, thiserror::Error)]
pub enum SessionError {
    /// An ordered submission was stamped before an input already queued.
    #[error("input stamped {at:?} precedes last queued input at {last:?}")]
    InputOutOfOrder {
        /// The refused instant.
        at: SimInstant,
        /// The latest instant already queued.
        last: SimInstant,
    },
    /// A sensor channel was registered with a zero period.
    #[error("sensor channel {channel:?} has a zero period")]
    ZeroSensorPeriod {
        /// The refused channel.
        channel: SensorChannelId,
    },
    /// A sensor channel id was registered twice.
    #[error("sensor channel {channel:?} is already registered")]
    DuplicateSensorChannel {
        /// The refused channel.
        channel: SensorChannelId,
    },
    /// The plant block failed to step.
    #[error("block step starting at {at:?} failed")]
    Block {
        /// Start of the failed step.
        at: SimInstant,
        /// The block's failure.
        #[source]
        source: BlockError,
    },
}

/// Blends two vehicle states for display between physics steps: linear in
/// position, velocity, and body rate, spherical in attitude. `phase` is
/// clamped to `[0, 1]`; a non-finite phase yields `prev`.
#[must_use]
pub fn interpolate_vehicle_state(
    prev: &VehicleState,
    curr: &VehicleState,
    phase: f64,
) -> VehicleState {
    let phase = if phase.is_finite() {
        phase.clamp(0.0, 1.0)
    } else {
        0.0
    };
    VehicleState {
        position_ecef: prev.position_ecef.lerp(curr.position_ecef, phase),
        velocity_ecef: prev.velocity_ecef.lerp(curr.velocity_ecef, phase),
        attitude: prev.attitude.slerp(curr.attitude, phase).normalize(),
        angular_rate_body: prev.angular_rate_body.lerp(curr.angular_rate_body, phase),
    }
}

/// A deterministic, rendering-free execution kernel around one plant block.
///
/// Shells hand it elapsed master-clock time in whatever slices their frame
/// rate produces; the session turns that into whole fixed steps, latching
/// stamped inputs and emitting sensor exposures per step, so a 30 FPS window,
/// a 144 FPS window, and a headless batch run reach bit-identical truth.
#[derive(Debug)]
pub struct SimSession<B>
where
    B: Block<Output = VehicleState>,
    B::Input: Clone,
{
    block: B,
    period: TickSpan,
    rate: RateAdapter,
    now: SimInstant,
    physics_time: SimInstant,
    prev_output: VehicleState,
    curr_output: VehicleState,
    latched_input: B::Input,
    inputs: StampedInputQueue<B::Input>,
    sensors: SensorSchedule,
    exposures: Vec<SensorExposureEvent>,
    source_epoch: u32,
    episode_id: u64,
}

impl<B> SimSession<B>
where
    B: Block<Output = VehicleState>,
    B::Input: Clone,
{
    /// A session at [`SimInstant::START`] holding `initial_input` until the
    /// first queued input falls due.
    pub fn new(block: B, initial_input: B::Input) -> Self {
        let period = block.period();
        let output = block.output();
        Self {
            block,
            period,
            rate: RateAdapter::new(period),
            now: SimInstant::START,
            physics_time: SimInstant::START,
            prev_output: output,
            curr_output: output,
            latched_input: initial_input,
            inputs: StampedInputQueue::new(),
            sensors: SensorSchedule::new(),
            exposures: Vec::new(),
            source_epoch: 0,
            episode_id: 0,
        }
    }

    /// Registers a fixed-rate sensor channel, first exposing one `period`
    /// after the current physics instant.
    pub fn register_sensor(
        &mut self,
        id: SensorChannelId,
        period: TickSpan,
    ) -> Result<(), SessionError> {
        self.sensors.register(id, period, self.physics_time)
    }

    /// Queues `input` at `at`, clamped forward to the last queued instant;
    /// returns the instant used.
    pub fn submit_at(&mut self, at: SimInstant, input: B::Input) -> SimInstant {
        self.inputs.submit_at(at, input)
    }

    /// Queues `input` at `at`, refusing an out-of-order instant.
    pub fn submit_ordered(&mut self, at: SimInstant, input: B::Input) -> Result<(), SessionError> {
        self.inputs.submit_ordered(at, input)
    }

    /// Queues `input` at the session's current instant.
    pub fn latch_now(&mut self, input: B::Input) -> SimInstant {
        self.inputs.submit_at(self.now, input)
    }

    /// Advances the master clock by `elapsed`, running every fixed step now
    /// due, and returns how many ran. After an error the session is left
    /// mid-advance and should be restored, rewound, or reset.
    pub fn advance(&mut self, elapsed: TickSpan) -> Result<u64, SessionError> {
        let steps = self.rate.steps_for(elapsed);
        self.now = self.now.advanced(elapsed);
        for _ in 0..steps {
            self.step_once()?;
        }
        Ok(steps)
    }

    /// Advances to `target`; does nothing if `target` is not after `now`.
    pub fn advance_to(&mut self, target: SimInstant) -> Result<u64, SessionError> {
        self.advance(target.duration_since(self.now))
    }

    fn step_once(&mut self) -> Result<(), SessionError> {
        let step_start = self.physics_time;
        if let Some(input) = self.inputs.latch_due(step_start) {
            self.latched_input = input;
        }
        self.block
            .step(&self.latched_input)
            .map_err(|source| SessionError::Block {
                at: step_start,
                source,
            })?;
        let step_end = step_start.advanced(self.period);
        self.prev_output = self.curr_output;
        self.curr_output = self.block.output();
        self.physics_time = step_end;
        self.sensors.on_step(
            step_start,
            step_end,
            &self.prev_output,
            &self.curr_output,
            &mut self.exposures,
        );
        Ok(())
    }

    /// Master-clock time handed to the session so far.
    #[must_use]
    pub fn now(&self) -> SimInstant {
        self.now
    }

    /// End of the last completed physics step (at most one period before
    /// [`Self::now`]).
    #[must_use]
    pub fn physics_time(&self) -> SimInstant {
        self.physics_time
    }

    /// Exact plant output after the last completed step.
    #[must_use]
    pub fn truth_state(&self) -> VehicleState {
        self.curr_output
    }

    /// Exact plant output before the last completed step: the start of the
    /// span [`Self::display_state`] interpolates across.
    #[must_use]
    pub fn previous_truth_state(&self) -> VehicleState {
        self.prev_output
    }

    /// Fraction in `[0, 1)` of the next physics step that [`Self::now`] has
    /// reached past [`Self::physics_time`].
    #[must_use]
    pub fn display_phase(&self) -> f64 {
        self.rate.phase()
    }

    /// Plant output interpolated to [`Self::now`] for smooth display; never
    /// feed this back into the simulation.
    #[must_use]
    pub fn display_state(&self) -> VehicleState {
        interpolate_vehicle_state(&self.prev_output, &self.curr_output, self.rate.phase())
    }

    /// The input the next step will use unless a queued one falls due first.
    #[must_use]
    pub fn latched_input(&self) -> &B::Input {
        &self.latched_input
    }

    /// Inputs queued for future steps.
    #[must_use]
    pub fn pending_inputs(&self) -> &StampedInputQueue<B::Input> {
        &self.inputs
    }

    /// Exposures emitted since the last [`Self::take_exposures`], oldest
    /// first, left in place for a later take.
    #[must_use]
    pub fn exposures(&self) -> &[SensorExposureEvent] {
        &self.exposures
    }

    /// Hands over and clears the emitted exposures.
    pub fn take_exposures(&mut self) -> Vec<SensorExposureEvent> {
        std::mem::take(&mut self.exposures)
    }

    /// Incremented on every discontinuity of the timeline (rewind or reset)
    /// so consumers can discard estimates from an abandoned history.
    #[must_use]
    pub fn source_epoch(&self) -> u32 {
        self.source_epoch
    }

    /// Incremented on every [`Self::reset_episode`].
    #[must_use]
    pub fn episode_id(&self) -> u64 {
        self.episode_id
    }

    /// The plant block, for reading state beyond [`VehicleState`].
    #[must_use]
    pub fn block(&self) -> &B {
        &self.block
    }

    /// The plant block, for latched settings that are not step inputs (an
    /// engine switch, a failure injection). Changing its dynamic state here
    /// bypasses the clock: [`Self::truth_state`] and [`Self::display_state`]
    /// keep reporting the last stepped output until the next step. Jump the
    /// state with [`Self::rewind_timeline`] or [`Self::reset_episode`]
    /// instead.
    pub fn block_mut(&mut self) -> &mut B {
        &mut self.block
    }
}
