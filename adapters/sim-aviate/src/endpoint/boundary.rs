use std::time::Duration;

/// Where a Tier-1 flight controller runs relative to the simulation thread,
/// which decides what its timing and step correlation can be trusted for.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum AviateExecutionBoundary {
    /// The peer is a callback on the simulation thread: every publish is
    /// answered before the call returns, so latency is whatever the bridge
    /// charges and wall-clock time measures only host speed.
    InProcessSynchronous,
    /// The peer is another process stepping the same shared-memory block in
    /// lockstep: answers are causal per step but cost real inter-process
    /// wake-up time.
    SharedMemoryLockstep,
    /// The peer is reached over an asynchronous transport: answers may lag,
    /// arrive out of step, or not arrive.
    AsyncExternalTransport,
}

/// Counters of a Tier-1 endpoint's publish, peer, and read cycles.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Tier1StepMetrics {
    /// Publish cycles started, including ones that failed.
    pub step_calls: u64,
    /// Cycles whose command acknowledged the step just published.
    pub causal_steps: u64,
    /// Cycles whose command acknowledged some other step: tolerated with
    /// lockstep off, fatal with it on.
    pub acausal_steps: u64,
    /// The newest step published.
    pub last_published_step: u64,
    /// The step the newest command acknowledged.
    pub last_acknowledged_step: u64,
    /// Peer calls that took longer than the configured wall-clock budget.
    pub budget_overruns: u64,
    /// Longest peer call measured; measured only while a budget is set.
    pub max_peer_wall: Duration,
}

impl Tier1StepMetrics {
    pub(super) fn on_publish(&mut self, step: u64) {
        self.step_calls = self.step_calls.wrapping_add(1);
        self.last_published_step = step;
    }

    pub(super) fn on_acknowledgement(&mut self, published: u64, acknowledged: u64) {
        self.last_acknowledged_step = acknowledged;
        if acknowledged == published {
            self.causal_steps = self.causal_steps.wrapping_add(1);
        } else {
            self.acausal_steps = self.acausal_steps.wrapping_add(1);
        }
    }

    pub(super) fn on_peer_wall(&mut self, elapsed: Duration, budget: Duration) {
        self.max_peer_wall = self.max_peer_wall.max(elapsed);
        if elapsed > budget {
            self.budget_overruns = self.budget_overruns.wrapping_add(1);
        }
    }
}
