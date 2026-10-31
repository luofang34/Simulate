use navigate_contract::{
    ClockDomainId, MonotonicNanos, ObservationStamp, SourceEpoch, SourceId, WrappingSequence,
};
use sim_core::SimInstant;

use crate::NavigateBridgeError;

#[cfg(test)]
mod tests;

/// Clock domain of every stamp the bridge makes by default.
pub const MASTER_CLOCK_DOMAIN: ClockDomainId = ClockDomainId::new(0x4653_4730);

/// Converts between the 46.08 GHz master clock and Navigate's nanoseconds.
///
/// Conversion to nanoseconds rounds down, because Navigate refuses a stamp
/// ahead of its admission time, so a stamp must never read later than the
/// instant it describes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MasterClockBridge {
    domain: ClockDomainId,
}

impl Default for MasterClockBridge {
    fn default() -> Self {
        Self::new(MASTER_CLOCK_DOMAIN)
    }
}

impl MasterClockBridge {
    /// A bridge stamping on `domain`.
    pub const fn new(domain: ClockDomainId) -> Self {
        Self { domain }
    }

    /// The clock domain of this bridge's stamps.
    pub const fn domain(&self) -> ClockDomainId {
        self.domain
    }

    /// `instant` as monotonic time since simulation start.
    pub fn to_monotonic(&self, instant: SimInstant) -> Result<MonotonicNanos, NavigateBridgeError> {
        instant
            .to_nanos()
            .map(MonotonicNanos::from_nanos)
            .ok_or(NavigateBridgeError::InstantOutOfRange { instant })
    }

    /// The first master-clock instant of `time`.
    pub const fn to_instant(&self, time: MonotonicNanos) -> SimInstant {
        SimInstant::from_nanos(time.as_nanos())
    }

    /// The stamp of one observation acquired at `instant`.
    pub fn stamp(
        &self,
        source: SourceId,
        epoch: SourceEpoch,
        sequence: WrappingSequence,
        instant: SimInstant,
    ) -> Result<ObservationStamp, NavigateBridgeError> {
        Ok(ObservationStamp::new(
            source,
            epoch,
            sequence,
            self.to_monotonic(instant)?,
            self.domain,
        ))
    }
}
