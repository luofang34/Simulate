//! The chain flown by a link's page and shown by the app: the page owns the one trial and
//! steps it at its own pace; the app only draws what the page published, at a display
//! time that runs smoothly a little behind the newest instant. The world outside is drawn
//! at the truth; the displays read the flight controller's account only.

use std::sync::{Arc, Mutex, PoisonError};

use sim_chain_trial::{ChainTrial, TruthSnapshot};
use sim_flight_chain::{FcAvionics, NavigationDisplay};

use crate::page_map::PageMapLink;
use display_clock::DisplayClock;
use ring::SnapshotRing;

mod display_clock;
mod ring;
#[cfg(test)]
mod tests;

/// The chain at one instant: the truth for the world outside, the flight controller's
/// account for the displays, and Pilotage's mission phase and navigation display.
#[derive(Debug, Clone, PartialEq)]
pub struct PageSnapshot {
    /// TRUTH: the true aeroplane, for drawing the world outside it only.
    pub truth: TruthSnapshot,
    /// The flight controller's account of its last cycle.
    pub avionics: FcAvionics,
    /// Pilotage's mission phase, in its own words.
    pub phase: Option<String>,
    /// Pilotage's navigation display: the plan, the leg flown and the holding pattern.
    pub navigation: Option<NavigationDisplay>,
}

impl PageSnapshot {
    /// The chain of `trial` now, in mission `phase`.
    #[must_use]
    pub fn of(trial: &ChainTrial, phase: Option<String>) -> Self {
        Self {
            truth: trial.truth(),
            avionics: trial.avionics(),
            phase,
            navigation: trial.navigation().cloned(),
        }
    }

    /// Route time, seconds.
    #[must_use]
    pub fn route_s(&self) -> f64 {
        super::route_s(&self.truth)
    }
}

/// What the page publishes and the app reads: the latest instants of the page's flight,
/// the display time the app draws them at, and the civil time the flight starts at.
#[derive(Debug, Clone)]
pub struct PageFeed {
    shared: Arc<Mutex<Published>>,
    epoch_utc_s: f64,
    map: PageMapLink,
}

/// The instants published and the display clock following them.
#[derive(Debug)]
struct Published {
    ring: SnapshotRing,
    clock: DisplayClock,
}

impl PageFeed {
    /// A feed of a flight starting at `epoch_utc_s`, Unix seconds, from `first`.
    #[cfg(any(target_arch = "wasm32", test))]
    #[must_use]
    pub fn new(epoch_utc_s: f64, first: PageSnapshot) -> Self {
        Self {
            shared: Arc::new(Mutex::new(Published {
                ring: SnapshotRing::new(first),
                clock: DisplayClock::default(),
            })),
            epoch_utc_s,
            map: PageMapLink::default(),
        }
    }

    fn lock(&self) -> std::sync::MutexGuard<'_, Published> {
        self.shared.lock().unwrap_or_else(PoisonError::into_inner)
    }

    /// Adds the newest snapshot.
    #[cfg(any(target_arch = "wasm32", test))]
    pub fn publish(&self, snapshot: PageSnapshot) {
        self.lock().ring.push(snapshot);
    }

    /// The newest snapshot.
    #[must_use]
    pub fn latest(&self) -> PageSnapshot {
        self.lock().ring.newest().clone()
    }

    /// The route time to draw at after `wall_dt_s` wall seconds, seconds: smooth at the
    /// flight's pace, a little behind the newest instant.
    pub fn display_route_s(&self, wall_dt_s: f64) -> f64 {
        let mut shared = self.lock();
        let (oldest_s, newest_s) = shared.ring.span_s();
        shared.clock.advance(wall_dt_s, newest_s, oldest_s)
    }

    /// The chain at `route_s`: the true aeroplane interpolated between the published
    /// instants on either side, the flight controller's account as it stood then.
    #[must_use]
    pub fn at(&self, route_s: f64) -> PageSnapshot {
        self.lock().ring.at(route_s)
    }

    /// The link to the page's own map canvas, which the app's EFB map draws on.
    #[must_use]
    pub const fn map(&self) -> &PageMapLink {
        &self.map
    }

    /// The flight's civil time at its start, Unix seconds.
    #[must_use]
    pub const fn epoch_utc_s(&self) -> f64 {
        self.epoch_utc_s
    }
}
