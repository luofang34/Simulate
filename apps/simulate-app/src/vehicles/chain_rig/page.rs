//! The chain flown by a link's page and shown by the app: the page owns the
//! one trial and steps it at its own pace; the app only draws what the page
//! last published. The outside world is drawn at the true state, which is
//! what the world looks like; no onboard consumer reads it from here.

use std::sync::{Arc, Mutex, PoisonError};

use glam::DVec3;
use sim_chain_trial::ChainTrial;
use sim_core::{AirframeArticulation, EngineStatus, VehicleState};

/// The chain at one instant of the page's flight.
#[derive(Debug, Clone, PartialEq)]
pub struct ChainSnapshot {
    /// Route time, seconds.
    pub route_s: f64,
    /// TRUTH: the vehicle's true state.
    pub state: VehicleState,
    /// TRUTH: the specific force in body forward-right-down axes, m/s².
    pub specific_force_frd_m_s2: DVec3,
    /// The flight model's engine.
    pub engine: EngineStatus,
    /// The flight model's surfaces, gear and propeller.
    pub articulation: AirframeArticulation,
    /// Whether the flight controller is armed.
    pub armed: bool,
    /// Pilotage's mission phase, in its own words.
    pub phase: Option<String>,
}

impl ChainSnapshot {
    /// The chain of `trial` now, in mission `phase`.
    #[must_use]
    pub fn of(trial: &ChainTrial, phase: Option<String>) -> Self {
        let chain = trial.chain();
        let board = chain.board();
        Self {
            route_s: trial.now_s(),
            state: chain.true_state(),
            specific_force_frd_m_s2: board.airframe().specific_force_body_m_s2(),
            engine: board.airframe().fdm().engine_status(),
            articulation: board.airframe().fdm().articulation(),
            armed: board.controller().is_armed(),
            phase,
        }
    }
}

/// What the page publishes and the app reads: the latest snapshot of the
/// page's flight, and the civil time the flight starts at.
#[derive(Debug, Clone)]
pub struct PageFeed {
    latest: Arc<Mutex<ChainSnapshot>>,
    epoch_utc_s: f64,
}

impl PageFeed {
    /// A feed of a flight starting at `epoch_utc_s`, Unix seconds, from
    /// `first`.
    #[cfg(any(target_arch = "wasm32", test))]
    #[must_use]
    pub fn new(epoch_utc_s: f64, first: ChainSnapshot) -> Self {
        Self {
            latest: Arc::new(Mutex::new(first)),
            epoch_utc_s,
        }
    }

    /// Replaces the latest snapshot.
    #[cfg(any(target_arch = "wasm32", test))]
    pub fn publish(&self, snapshot: ChainSnapshot) {
        *self.latest.lock().unwrap_or_else(PoisonError::into_inner) = snapshot;
    }

    /// The latest snapshot.
    #[must_use]
    pub fn latest(&self) -> ChainSnapshot {
        self.latest
            .lock()
            .unwrap_or_else(PoisonError::into_inner)
            .clone()
    }

    /// The flight's civil time at its start, Unix seconds.
    #[must_use]
    pub const fn epoch_utc_s(&self) -> f64 {
        self.epoch_utc_s
    }
}
