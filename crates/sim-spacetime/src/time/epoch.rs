//! The one mapping from master-clock ticks to physical and civil time for a run.

use std::sync::Arc;

use sim_geo::UtcTime;

use super::{SimClockAdapter, TdbEpoch, TimeError, TimeScaleCatalog, Ut1Sample};
use crate::clock::SimInstant;
use crate::hash::{ContentHash, DomainHasher};

/// `TT - TAI`, seconds (exact by definition).
const TT_MINUS_TAI_S: f64 = 32.184;

/// Where a run's [`SimInstant::START`] sits in absolute time, and the
/// time-scale record that names it in UTC and UT1.
///
/// Flight dynamics, orbital propagation, the sky, air data and every display
/// clock derive their time from one `SimEpoch`. TDB is the uniform scale:
/// ticks advance TDB exactly, and UTC and UT1 are read from it, so a flight
/// that crosses a leap second or a pre-1972 rate change stays on the same line
/// as the ephemerides.
#[derive(Clone, Debug)]
pub struct SimEpoch {
    clock: SimClockAdapter,
    scales: Arc<TimeScaleCatalog>,
}

impl SimEpoch {
    /// A run whose first tick is `start` TDB.
    #[must_use]
    pub fn from_tdb(start: TdbEpoch, scales: Arc<TimeScaleCatalog>) -> Self {
        Self {
            clock: SimClockAdapter::new(start),
            scales,
        }
    }

    /// A run whose first tick is `start` UTC.
    pub fn from_utc(start: UtcTime, scales: Arc<TimeScaleCatalog>) -> Result<Self, TimeError> {
        let tdb = scales.utc_to_tdb(start)?;
        Ok(Self::from_tdb(tdb, scales))
    }

    /// A run whose first tick is `start` UTC, on [`TimeScaleCatalog::builtin`].
    pub fn builtin_from_utc(start: UtcTime) -> Result<Self, TimeError> {
        Self::from_utc(start, Arc::new(TimeScaleCatalog::builtin()?))
    }

    /// The tick-to-TDB adapter orbital propagation steps on.
    #[must_use]
    pub const fn clock(&self) -> SimClockAdapter {
        self.clock
    }

    /// The time-scale record this run reads UTC and UT1 from.
    #[must_use]
    pub fn scales(&self) -> &TimeScaleCatalog {
        &self.scales
    }

    /// TDB at `instant`.
    #[must_use]
    pub fn tdb_at(&self, instant: SimInstant) -> TdbEpoch {
        self.clock.instant_to_tdb(instant)
    }

    /// Terrestrial Time at `instant`, seconds past `J2000.0 TT`: the scale
    /// clock rates near Earth are referred to, which departs from TDB by an
    /// annual term of up to 1.7 ms.
    #[must_use]
    pub fn tt_j2000_seconds_at(&self, instant: SimInstant) -> f64 {
        self.scales.tdb_to_tai_j2000_seconds(self.tdb_at(instant)) + TT_MINUS_TAI_S
    }

    /// UTC at `instant`.
    pub fn utc_at(&self, instant: SimInstant) -> Result<UtcTime, TimeError> {
        self.scales.tdb_to_utc(self.tdb_at(instant))
    }

    /// UTC at the run's first tick.
    pub fn start_utc(&self) -> Result<UtcTime, TimeError> {
        self.utc_at(SimInstant::START)
    }

    /// UT1 and polar motion at `instant`, which Earth's rotation angle follows.
    pub fn ut1_at(&self, instant: SimInstant) -> Result<Ut1Sample, TimeError> {
        self.scales.utc_to_ut1(self.utc_at(instant)?)
    }

    /// The tick at which `utc` falls, or `None` before the run started.
    pub fn instant_at_utc(&self, utc: UtcTime) -> Result<Option<SimInstant>, TimeError> {
        Ok(self.clock.tdb_to_instant(self.scales.utc_to_tdb(utc)?))
    }

    /// Identifies the anchor and the time-scale record, for replay manifests
    /// and snapshot checks.
    #[must_use]
    pub fn content_hash(&self) -> ContentHash {
        let mut hasher = DomainHasher::new(b"sim-spacetime:sim-epoch:v1");
        hasher.write_hash(self.clock.content_hash());
        hasher.write_hash(self.scales.content_hash());
        hasher.finish()
    }
}

impl PartialEq for SimEpoch {
    fn eq(&self, other: &Self) -> bool {
        self.clock == other.clock && self.scales.content_hash() == other.scales.content_hash()
    }
}

#[cfg(test)]
mod tests;
