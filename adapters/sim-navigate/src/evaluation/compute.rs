//! Host-side cost of navigation code. Wall time and memory vary between runs
//! and machines, so nothing here may reach simulation state, a replay digest,
//! or a score; [`ComputeMeter`] only hands its readings back to the caller.

use std::collections::BTreeMap;
use std::time::Duration;

use super::{NavigationOrigin, Percentiles};

#[cfg(test)]
mod tests;

/// The cost of one metered call. Each reading is `None` where the platform
/// cannot provide it, such as in a browser.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct ComputeSample {
    /// Wall-clock time the call took.
    pub wall: Option<Duration>,
    /// Process resident memory just before the call, bytes.
    pub rss_before_bytes: Option<u64>,
    /// Process resident memory just after the call, bytes.
    pub rss_after_bytes: Option<u64>,
}

impl ComputeSample {
    /// Resident memory gained over the call, bytes; negative when it shrank.
    pub fn rss_delta_bytes(&self) -> Option<i128> {
        Some(i128::from(self.rss_after_bytes?) - i128::from(self.rss_before_bytes?))
    }

    /// The larger of the two memory readings. Memory is read only around
    /// the call, so a transient peak inside it is not seen.
    pub fn rss_sampled_max_bytes(&self) -> Option<u64> {
        match (self.rss_before_bytes, self.rss_after_bytes) {
            (Some(before), Some(after)) => Some(before.max(after)),
            (before, after) => before.or(after),
        }
    }
}

/// The cost of one origin's calls over a run.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct ComputeStats {
    /// Calls metered.
    pub calls: u64,
    /// Wall time per call, seconds.
    pub wall_s: Option<Percentiles>,
    /// Largest resident-memory gain over one call, bytes.
    pub max_rss_delta_bytes: Option<i128>,
    /// Largest resident memory read around any call, bytes.
    pub rss_sampled_max_bytes: Option<u64>,
}

/// Times and weighs navigation calls, per origin.
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct ComputeMeter {
    samples: BTreeMap<NavigationOrigin, Vec<ComputeSample>>,
}

impl ComputeMeter {
    /// Runs `call` as work of `origin`, keeps its cost, and returns its
    /// result untouched.
    pub fn measure<R>(&mut self, origin: NavigationOrigin, call: impl FnOnce() -> R) -> R {
        let rss_before_bytes = probe::resident_bytes();
        let started = probe::now();
        let result = call();
        let wall = probe::elapsed(started);
        let sample = ComputeSample {
            wall,
            rss_before_bytes,
            rss_after_bytes: probe::resident_bytes(),
        };
        self.samples.entry(origin).or_default().push(sample);
        result
    }

    /// Every sample of `origin`, in call order.
    pub fn samples(&self, origin: NavigationOrigin) -> &[ComputeSample] {
        self.samples.get(&origin).map_or(&[], Vec::as_slice)
    }

    /// The cost of `origin`'s calls, summarised.
    pub fn stats(&self, origin: NavigationOrigin) -> ComputeStats {
        let samples = self.samples(origin);
        let wall: Vec<f64> = samples
            .iter()
            .filter_map(|s| s.wall.map(|w| w.as_secs_f64()))
            .collect();
        ComputeStats {
            calls: samples.len() as u64,
            wall_s: Percentiles::of(&wall),
            max_rss_delta_bytes: samples
                .iter()
                .filter_map(ComputeSample::rss_delta_bytes)
                .max(),
            rss_sampled_max_bytes: samples
                .iter()
                .filter_map(ComputeSample::rss_sampled_max_bytes)
                .max(),
        }
    }
}

#[cfg(not(target_arch = "wasm32"))]
mod probe {
    use std::time::{Duration, Instant};

    pub(super) fn now() -> Option<Instant> {
        Some(Instant::now())
    }

    pub(super) fn elapsed(started: Option<Instant>) -> Option<Duration> {
        started.map(|s| s.elapsed())
    }

    pub(super) fn resident_bytes() -> Option<u64> {
        memory_stats::memory_stats().map(|m| m.physical_mem as u64)
    }
}

/// The browser has no monotonic `Instant` and no process memory to read.
#[cfg(target_arch = "wasm32")]
mod probe {
    use std::time::Duration;

    pub(super) fn now() -> Option<()> {
        None
    }

    pub(super) fn elapsed(_started: Option<()>) -> Option<Duration> {
        None
    }

    pub(super) fn resident_bytes() -> Option<u64> {
        None
    }
}
