//! Wall-clock pacing for real-time XIL runs.

use std::time::Duration;
#[cfg(not(target_arch = "wasm32"))]
use std::time::Instant;

use sim_core::SimInstant;
#[cfg(not(target_arch = "wasm32"))]
use sim_core::TICKS_PER_SECOND;

#[cfg(all(test, not(target_arch = "wasm32")))]
mod tests;

/// Lag beyond which the pacer stops trying to catch up and re-anchors, so a
/// stalled peer or debugger pause does not trigger an unbounded burst.
#[cfg(not(target_arch = "wasm32"))]
const MAX_CATCH_UP: Duration = Duration::from_millis(250);
/// How long before a deadline the pacer stops sleeping and spins, absorbing
/// the OS scheduler's wake-up jitter.
#[cfg(not(target_arch = "wasm32"))]
const SPIN_WINDOW: Duration = Duration::from_micros(250);

/// Timing health of a paced run.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct PacerStats {
    /// Calls to [`RealtimePacer::pace_until`].
    pub steps: u64,
    /// Steps that arrived later than their deadline plus the tolerance.
    pub overrun_count: u64,
    /// Largest lateness of any step, microseconds.
    pub max_overrun_us: u64,
    /// Total wall-clock time spent waiting for deadlines, microseconds.
    pub total_sleep_us: u64,
}

/// Holds a simulation to wall-clock time: each master-clock instant is mapped
/// to a wall-clock deadline relative to an anchor taken at construction.
///
/// On `wasm32` the main thread may not block and has no monotonic clock in
/// `std`, so pacing is left to the host's frame loop and only steps are counted.
#[derive(Debug, Clone)]
pub struct RealtimePacer {
    #[cfg(not(target_arch = "wasm32"))]
    origin_wall: Instant,
    origin_sim: SimInstant,
    stats: PacerStats,
}

impl Default for RealtimePacer {
    fn default() -> Self {
        Self::new()
    }
}

impl RealtimePacer {
    /// A pacer mapping [`SimInstant::START`] to now.
    #[must_use]
    pub fn new() -> Self {
        Self::anchored_at(SimInstant::START)
    }

    /// A pacer mapping `origin_sim` to now.
    #[must_use]
    pub fn anchored_at(origin_sim: SimInstant) -> Self {
        Self {
            #[cfg(not(target_arch = "wasm32"))]
            origin_wall: Instant::now(),
            origin_sim,
            stats: PacerStats::default(),
        }
    }

    /// The master-clock instant mapped to the wall-clock anchor.
    #[must_use]
    pub const fn origin(&self) -> SimInstant {
        self.origin_sim
    }

    /// Timing counters so far.
    #[must_use]
    pub const fn stats(&self) -> PacerStats {
        self.stats
    }

    /// Waits until the wall-clock deadline of `target_sim`. A step later than
    /// its deadline by more than `overrun_tolerance` counts as an overrun; a
    /// lag beyond 250 ms re-anchors the clock instead of bursting to catch up.
    pub fn pace_until(&mut self, target_sim: SimInstant, overrun_tolerance: Duration) {
        self.stats.steps = self.stats.steps.wrapping_add(1);
        #[cfg(not(target_arch = "wasm32"))]
        self.pace_native(target_sim, overrun_tolerance);
        #[cfg(target_arch = "wasm32")]
        {
            let _unpaced = (target_sim, overrun_tolerance);
        }
    }

    #[cfg(not(target_arch = "wasm32"))]
    fn pace_native(&mut self, target_sim: SimInstant, overrun_tolerance: Duration) {
        let offset = sim_offset(self.origin(), target_sim);
        let Some(deadline) = self.origin_wall.checked_add(offset) else {
            return;
        };
        let now = Instant::now();
        if let Some(lag) = now.checked_duration_since(deadline) {
            if lag > overrun_tolerance {
                self.stats.overrun_count = self.stats.overrun_count.wrapping_add(1);
                self.stats.max_overrun_us = self.stats.max_overrun_us.max(micros(lag));
            }
            if lag > MAX_CATCH_UP {
                self.origin_wall = self.origin_wall.checked_add(lag).unwrap_or(now);
            }
            return;
        }
        let ahead = deadline.saturating_duration_since(now);
        if let Some(sleep_for) = ahead.checked_sub(SPIN_WINDOW) {
            std::thread::sleep(sleep_for);
        }
        while Instant::now() < deadline {
            std::hint::spin_loop();
        }
        let waited = now.elapsed();
        self.stats.total_sleep_us = self.stats.total_sleep_us.saturating_add(micros(waited));
    }
}

#[cfg(not(target_arch = "wasm32"))]
/// Wall-clock offset of `target` from `origin` at 1x real time; zero if
/// `target` precedes `origin`.
fn sim_offset(origin: SimInstant, target: SimInstant) -> Duration {
    let (seconds, subsec_ticks) = target.duration_since(origin).seconds_and_subsec_ticks();
    let nanos = u128::from(subsec_ticks) * 1_000_000_000 / u128::from(TICKS_PER_SECOND);
    Duration::new(seconds, u32::try_from(nanos).unwrap_or(999_999_999))
}

#[cfg(not(target_arch = "wasm32"))]
fn micros(duration: Duration) -> u64 {
    u64::try_from(duration.as_micros()).unwrap_or(u64::MAX)
}
