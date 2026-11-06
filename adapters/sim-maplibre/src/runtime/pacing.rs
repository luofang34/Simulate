//! Minimum-interval gating shared by instrument map surfaces.

use std::time::Duration;

/// Whether `min_interval` has passed since `prev` at `at`. A clock that ran backwards
/// (scenario reset or replay seek) always counts as elapsed so the surface redraws at once.
#[must_use]
pub(crate) fn interval_elapsed(
    prev: Option<Duration>,
    at: Duration,
    min_interval: Duration,
) -> bool {
    match prev {
        Some(prev) if at >= prev => at.saturating_sub(prev) >= min_interval,
        _ => true,
    }
}

#[cfg(test)]
mod tests;
