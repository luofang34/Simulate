//! What a host draws from the trial at a rate it asks for: the flight
//! controller's account for the avionics, and the truth for the world
//! outside, kept apart.

use glam::DVec3;
use sim_core::{AirframeArticulation, EngineStatus, SimInstant, TICKS_PER_SECOND, VehicleState};
use sim_flight_chain::FcAvionics;

use super::C172Chain;
use crate::ChainTrialError;

/// The true aeroplane at one instant, for drawing the world outside the
/// aeroplane only: no display of the aeroplane's own systems may read it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TruthSnapshot {
    /// The instant.
    pub at: SimInstant,
    /// The true state.
    pub state: VehicleState,
    /// The true specific force at the centre of gravity, body forward-right-down, m/s².
    pub specific_force_body_m_s2: DVec3,
    /// The flight model's surfaces, gear and propeller.
    pub articulation: AirframeArticulation,
    /// The flight model's engine.
    pub engine: EngineStatus,
}

/// The truth of `chain` now.
pub(super) fn truth(chain: &C172Chain) -> TruthSnapshot {
    let fdm = chain.board().airframe().fdm();
    TruthSnapshot {
        at: chain.now(),
        state: chain.true_state(),
        specific_force_body_m_s2: chain.board().airframe().specific_force_body_m_s2(),
        articulation: fdm.articulation(),
        engine: fdm.engine_status(),
    }
}

/// The flight controller's account of `chain`'s last cycle.
pub(super) fn avionics(chain: &C172Chain) -> FcAvionics {
    chain.board().avionics()
}

/// When the next feed sample is due.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) struct FeedClock {
    period_ticks: u128,
    next_tick: u128,
}

impl FeedClock {
    /// A sample every `1 / rate_hz` seconds of route time, the first at the
    /// next cycle's end.
    pub(super) fn new(rate_hz: f64, now: SimInstant) -> Result<Self, ChainTrialError> {
        if !(rate_hz.is_finite() && rate_hz > 0.0) {
            return Err(ChainTrialError::Invalid {
                detail: format!("a feed rate of {rate_hz} Hz"),
            });
        }
        let period_ticks = ((TICKS_PER_SECOND as f64 / rate_hz).round() as u128).max(1);
        Ok(Self {
            period_ticks,
            next_tick: now.ticks_u128(),
        })
    }

    /// Whether a sample is due at `now`, the end of a cycle; a due sample
    /// moves the next one a period on from the boundary it fell due at, so
    /// the samples keep their rate whatever the cycle.
    pub(super) fn due(&mut self, now: SimInstant) -> bool {
        let tick = now.ticks_u128();
        if tick < self.next_tick {
            return false;
        }
        let behind = (tick - self.next_tick) / self.period_ticks;
        self.next_tick = self
            .next_tick
            .saturating_add(self.period_ticks.saturating_mul(behind.saturating_add(1)));
        true
    }
}
