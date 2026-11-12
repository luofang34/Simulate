//! The chain flown by a link's page and shown by the app: the page owns the one trial and
//! steps it at its own pace; the app only draws what the page last published. The world
//! outside is drawn at the truth; the displays read the flight controller's account only.

use std::sync::{Arc, Mutex, PoisonError};

use sim_chain_trial::{ChainTrial, TruthSnapshot};
use sim_flight_chain::{FcAvionics, NavigationDisplay};

use crate::page_map::PageMapLink;

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

/// What the page publishes and the app reads: the latest snapshot of the page's flight,
/// and the civil time the flight starts at.
#[derive(Debug, Clone)]
pub struct PageFeed {
    latest: Arc<Mutex<PageSnapshot>>,
    epoch_utc_s: f64,
    map: PageMapLink,
}

impl PageFeed {
    /// A feed of a flight starting at `epoch_utc_s`, Unix seconds, from `first`.
    #[cfg(any(target_arch = "wasm32", test))]
    #[must_use]
    pub fn new(epoch_utc_s: f64, first: PageSnapshot) -> Self {
        Self {
            latest: Arc::new(Mutex::new(first)),
            epoch_utc_s,
            map: PageMapLink::default(),
        }
    }

    /// Replaces the latest snapshot.
    #[cfg(any(target_arch = "wasm32", test))]
    pub fn publish(&self, snapshot: PageSnapshot) {
        *self.latest.lock().unwrap_or_else(PoisonError::into_inner) = snapshot;
    }

    /// The latest snapshot.
    #[must_use]
    pub fn latest(&self) -> PageSnapshot {
        self.latest
            .lock()
            .unwrap_or_else(PoisonError::into_inner)
            .clone()
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
