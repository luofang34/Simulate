use crate::{Block, RateAdapter, SimInstant, VehicleState};

use super::{SensorExposureEvent, SensorSchedule, SimSession, StampedInputQueue};

/// Everything needed to resume a session bit-for-bit, including its clock
/// carry, queued inputs, sensor phases, exposures emitted but not yet taken,
/// and timeline identity. Restoring one is not a discontinuity:
/// [`SimSession::source_epoch`] is preserved.
#[derive(Debug, Clone, PartialEq)]
pub struct ExactSessionSnapshot<S, I> {
    block_snapshot: S,
    now: SimInstant,
    physics_time: SimInstant,
    prev_output: VehicleState,
    curr_output: VehicleState,
    rate_adapter: RateAdapter,
    latched_input: I,
    sensor_channels: SensorSchedule,
    pending_inputs: StampedInputQueue<I>,
    pending_exposures: Vec<SensorExposureEvent>,
    source_epoch: u32,
    episode_id: u64,
}

impl<S, I> ExactSessionSnapshot<S, I> {
    /// Session time when captured.
    #[must_use]
    pub fn now(&self) -> SimInstant {
        self.now
    }

    /// Timeline epoch when captured.
    #[must_use]
    pub fn source_epoch(&self) -> u32 {
        self.source_epoch
    }

    /// Episode when captured.
    #[must_use]
    pub fn episode_id(&self) -> u64 {
        self.episode_id
    }

    /// Exposures emitted but not yet taken when captured, in emission order.
    #[must_use]
    pub fn pending_exposures(&self) -> &[SensorExposureEvent] {
        &self.pending_exposures
    }
}

/// A plant state on the timeline to jump back to. Rewinding to one starts a
/// new timeline: queued inputs and sub-step carry from the abandoned one are
/// dropped and [`SimSession::source_epoch`] advances.
#[derive(Debug, Clone, PartialEq)]
pub struct TimelineCheckpoint<S> {
    /// The plant block's complete state.
    pub block_snapshot: S,
    /// The physics instant the plant state belongs to.
    pub at: SimInstant,
}

impl<B> SimSession<B>
where
    B: Block<Output = VehicleState>,
    B::Input: Clone,
{
    /// Captures the session exactly.
    #[must_use]
    pub fn snapshot_exact(&self) -> ExactSessionSnapshot<B::Snapshot, B::Input> {
        ExactSessionSnapshot {
            block_snapshot: self.block.snapshot(),
            now: self.now,
            physics_time: self.physics_time,
            prev_output: self.prev_output,
            curr_output: self.curr_output,
            rate_adapter: self.rate,
            latched_input: self.latched_input.clone(),
            sensor_channels: self.sensors.clone(),
            pending_inputs: self.inputs.clone(),
            pending_exposures: self.exposures.clone(),
            source_epoch: self.source_epoch,
            episode_id: self.episode_id,
        }
    }

    /// Returns to `snap` so subsequent steps, inputs, and exposures repeat
    /// bit-for-bit. Exposures emitted but not yet taken when `snap` was
    /// captured come back too, so a consumer draining after the restore sees
    /// the same sequence it would have seen without the detour.
    pub fn restore_exact(&mut self, snap: &ExactSessionSnapshot<B::Snapshot, B::Input>) {
        self.block.restore(&snap.block_snapshot);
        self.now = snap.now;
        self.physics_time = snap.physics_time;
        self.prev_output = snap.prev_output;
        self.curr_output = snap.curr_output;
        self.rate = snap.rate_adapter;
        self.latched_input = snap.latched_input.clone();
        self.sensors.clone_from(&snap.sensor_channels);
        self.inputs.clone_from(&snap.pending_inputs);
        self.source_epoch = snap.source_epoch;
        self.episode_id = snap.episode_id;
        self.exposures.clone_from(&snap.pending_exposures);
    }

    /// A checkpoint of the plant at the current physics instant.
    #[must_use]
    pub fn checkpoint(&self) -> TimelineCheckpoint<B::Snapshot> {
        TimelineCheckpoint {
            block_snapshot: self.block.snapshot(),
            at: self.physics_time,
        }
    }

    /// Jumps the plant to `checkpoint` holding `latched_input`, starting a new
    /// timeline: the clock carry is zeroed, queued inputs and untaken
    /// exposures are dropped, sensors re-phase to the checkpoint, and
    /// [`Self::source_epoch`] advances.
    pub fn rewind_timeline(
        &mut self,
        checkpoint: &TimelineCheckpoint<B::Snapshot>,
        latched_input: B::Input,
    ) {
        self.block.restore(&checkpoint.block_snapshot);
        self.restart_at(checkpoint.at, latched_input);
        self.sensors.rephase(checkpoint.at);
        self.source_epoch = self.source_epoch.wrapping_add(1);
    }

    /// Starts a new episode from `initial` at [`SimInstant::START`]: queued
    /// inputs and untaken exposures are dropped, sensors restart their
    /// sequences, and both [`Self::episode_id`] and
    /// [`Self::source_epoch`] advance.
    pub fn reset_episode(&mut self, initial: &B::Snapshot, initial_input: B::Input) {
        self.block.restore(initial);
        self.restart_at(SimInstant::START, initial_input);
        self.sensors.restart(SimInstant::START);
        self.episode_id = self.episode_id.wrapping_add(1);
        self.source_epoch = self.source_epoch.wrapping_add(1);
    }

    fn restart_at(&mut self, at: SimInstant, latched_input: B::Input) {
        let output = self.block.output();
        self.now = at;
        self.physics_time = at;
        self.prev_output = output;
        self.curr_output = output;
        self.rate = RateAdapter::new(self.period);
        self.latched_input = latched_input;
        self.inputs.clear();
        self.exposures.clear();
    }
}
