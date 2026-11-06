use std::collections::VecDeque;

use crate::SimInstant;

use super::SessionError;

/// Inputs stamped with the master-clock instant they take effect, held in
/// non-decreasing instant order so a session latches them identically however
/// its shell slices wall-clock frames.
#[derive(Debug, Clone, PartialEq)]
pub struct StampedInputQueue<I> {
    pending: VecDeque<(SimInstant, I)>,
}

impl<I> Default for StampedInputQueue<I> {
    fn default() -> Self {
        Self {
            pending: VecDeque::new(),
        }
    }
}

impl<I> StampedInputQueue<I> {
    /// An empty queue.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// The latest instant queued, if any.
    #[must_use]
    pub fn last_instant(&self) -> Option<SimInstant> {
        self.pending.back().map(|(at, _)| *at)
    }

    /// Queues `input` at `at`, moving `at` forward to the last queued instant
    /// if it is earlier, and returns the instant actually used. Clamping keeps
    /// FIFO order without letting a late event reorder history.
    pub fn submit_at(&mut self, at: SimInstant, input: I) -> SimInstant {
        let effective = self.last_instant().map_or(at, |last| at.max(last));
        self.pending.push_back((effective, input));
        effective
    }

    /// Queues `input` at `at`, refusing an instant earlier than the last
    /// queued one instead of clamping it.
    pub fn submit_ordered(&mut self, at: SimInstant, input: I) -> Result<(), SessionError> {
        if let Some(last) = self.last_instant()
            && at < last
        {
            return Err(SessionError::InputOutOfOrder { at, last });
        }
        self.pending.push_back((at, input));
        Ok(())
    }

    /// Removes every input stamped at or before `step_start`, oldest first,
    /// and returns the newest of them: the one in force for the step.
    pub fn latch_due(&mut self, step_start: SimInstant) -> Option<I> {
        let mut latched = None;
        while self
            .pending
            .front()
            .is_some_and(|(at, _)| *at <= step_start)
        {
            latched = self.pending.pop_front().map(|(_, input)| input).or(latched);
        }
        latched
    }

    /// Number of inputs still waiting.
    #[must_use]
    pub fn len(&self) -> usize {
        self.pending.len()
    }

    /// Whether no inputs are waiting.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.pending.is_empty()
    }

    /// Discards every waiting input.
    pub fn clear(&mut self) {
        self.pending.clear();
    }

    /// Waiting inputs, oldest first.
    pub fn iter(&self) -> impl Iterator<Item = &(SimInstant, I)> {
        self.pending.iter()
    }
}
