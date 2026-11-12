//! The page's latest published instants, and the chain at any route time between them:
//! the true aeroplane interpolated between the instants on either side, everything else
//! the flight controller and the mission host had said by then.

use std::collections::VecDeque;

use sim_core::interpolate_vehicle_state;

use super::PageSnapshot;

/// Instants kept besides the newest: about a second of a page publishing at 60 Hz,
/// longer than the display ever runs behind.
#[cfg(any(target_arch = "wasm32", test))]
pub(super) const KEPT: usize = 63;

/// The page's latest instants.
#[derive(Debug, Clone)]
pub(super) struct SnapshotRing {
    /// Older instants, oldest first.
    older: VecDeque<PageSnapshot>,
    newest: PageSnapshot,
}

impl SnapshotRing {
    #[cfg(any(target_arch = "wasm32", test))]
    pub(super) fn new(first: PageSnapshot) -> Self {
        Self {
            older: VecDeque::new(),
            newest: first,
        }
    }

    /// Keeps `snapshot` as the newest; one older than the newest starts the ring again,
    /// as a flight started again does.
    #[cfg(any(target_arch = "wasm32", test))]
    pub(super) fn push(&mut self, snapshot: PageSnapshot) {
        if snapshot.route_s() < self.newest.route_s() {
            self.older.clear();
        } else {
            if self.older.len() >= KEPT {
                self.older.pop_front();
            }
            self.older.push_back(self.newest.clone());
        }
        self.newest = snapshot;
    }

    /// Route times of the oldest and newest instants kept, seconds.
    pub(super) fn span_s(&self) -> (f64, f64) {
        let newest = self.newest.route_s();
        (
            self.older.front().map_or(newest, PageSnapshot::route_s),
            newest,
        )
    }

    /// The newest instant.
    pub(super) const fn newest(&self) -> &PageSnapshot {
        &self.newest
    }

    /// The chain at `route_s`: the latest instant at or before it, with the true
    /// aeroplane interpolated toward the next one. The flight controller's account, the
    /// mission host's words and the flight model's other outputs stay those of the
    /// earlier instant, so the displays never see a sample from their future.
    pub(super) fn at(&self, route_s: f64) -> PageSnapshot {
        let all = || self.older.iter().chain(std::iter::once(&self.newest));
        let Some(after) = all().position(|s| s.route_s() > route_s) else {
            return self.newest.clone();
        };
        let (Some(before), Some(next)) = (
            after.checked_sub(1).and_then(|index| all().nth(index)),
            all().nth(after),
        ) else {
            // Before the oldest instant kept: draw the oldest.
            return self.older.front().unwrap_or(&self.newest).clone();
        };
        let mut snapshot = before.clone();
        let span = next.route_s() - before.route_s();
        if span > 0.0 {
            let phase = ((route_s - before.route_s()) / span).clamp(0.0, 1.0);
            snapshot.truth.state =
                interpolate_vehicle_state(&before.truth.state, &next.truth.state, phase);
        }
        snapshot
    }
}
