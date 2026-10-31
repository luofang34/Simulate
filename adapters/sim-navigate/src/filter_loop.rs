use std::collections::BTreeMap;

use navigate_contract::{DurationNanos, MonotonicNanos, NavigationSolution};
use navigate_fusion::{FusionConfig, IngestOutcome, NavigationFilter, Observation};
use sim_core::{SimInstant, TickSpan};

use crate::{MasterClockBridge, NavigateBridgeError};

#[cfg(test)]
mod tests;

/// Fusion settings for a vehicle moving at up to `max_speed_m_s` and
/// manoeuvring at up to `max_accel_m_s2`, with velocity aiding every
/// `aiding_period_s`.
///
/// The defaults suit neither. The filter starts at zero velocity, and the 10 m/s
/// prior refuses the first velocity fix of anything faster at the innovation
/// gate. Its constant-velocity model then has to cover a turn through process
/// noise: with too little, one velocity fix is refused, the next is further off,
/// and aiding drops out while position dead-reckons away.
pub fn airborne_fusion_config(
    max_speed_m_s: f64,
    max_accel_m_s2: f64,
    aiding_period_s: f64,
) -> FusionConfig {
    FusionConfig {
        initial_velocity_variance_m2_per_s2: max_speed_m_s * max_speed_m_s,
        process_noise_accel_psd: max_accel_m_s2 * max_accel_m_s2 * aiding_period_s,
        ..FusionConfig::default()
    }
}

/// Feeds a [`NavigationFilter`] in acquisition order.
///
/// The filter applies a measurement acquired before its state time at the
/// present state, so a 300 ms old anchor applied after fresher velocity aiding
/// lands 300 ms of flight away from where it was taken. Every observation is
/// therefore held for one delay, then replayed in acquisition order; a solution
/// is the filter's state propagated from there to the present.
#[derive(Debug)]
pub struct NavigationLoop {
    clock: MasterClockBridge,
    filter: NavigationFilter,
    hold_ns: u64,
    pending: BTreeMap<(MonotonicNanos, u64), Observation>,
    next_arrival: u64,
    accepted: u64,
}

impl NavigationLoop {
    /// A loop holding each observation for `hold`, which must cover the slowest
    /// source's delay and stay inside the filter's staleness bound.
    pub fn new(
        config: FusionConfig,
        clock: MasterClockBridge,
        hold: TickSpan,
    ) -> Result<Self, NavigateBridgeError> {
        let hold_ns = SimInstant::START.advanced(hold).to_nanos().ok_or(
            NavigateBridgeError::InstantOutOfRange {
                instant: SimInstant::START.advanced(hold),
            },
        )?;
        if DurationNanos::from_nanos(hold_ns) >= config.staleness_bound {
            return Err(NavigateBridgeError::HoldReachesStaleness {
                hold: DurationNanos::from_nanos(hold_ns),
                bound: config.staleness_bound,
            });
        }
        Ok(Self {
            clock,
            filter: NavigationFilter::new(config, clock.domain()),
            hold_ns,
            pending: BTreeMap::new(),
            next_arrival: 0,
            accepted: 0,
        })
    }

    /// The filter, for its rejection counters and configuration.
    pub const fn filter(&self) -> &NavigationFilter {
        &self.filter
    }

    /// Observations the filter has admitted.
    pub const fn accepted(&self) -> u64 {
        self.accepted
    }

    /// Queues one observation. Arrival order breaks ties in acquisition time.
    pub fn offer(&mut self, observation: Observation) {
        let key = (observation.stamp.acquired_at, self.next_arrival);
        self.next_arrival = self.next_arrival.wrapping_add(1);
        self.pending.insert(key, observation);
    }

    /// Replays everything held long enough and publishes the solution for
    /// `now`, or `None` before a first position fix.
    pub fn advance(
        &mut self,
        now: SimInstant,
    ) -> Result<Option<NavigationSolution>, NavigateBridgeError> {
        let now = self.clock.to_monotonic(now)?;
        while let Some(entry) = self.pending.first_entry() {
            let held = now.as_nanos().checked_sub(entry.key().0.as_nanos());
            if held.is_none_or(|held| held < self.hold_ns) {
                break;
            }
            let observation = entry.remove();
            if let IngestOutcome::Accepted = self.filter.ingest(&observation, now) {
                self.accepted = self.accepted.wrapping_add(1);
            }
        }
        Ok(self.filter.tick(now))
    }
}
