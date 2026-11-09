//! Exact J2000-anchored TDB epochs, coverage intervals, and time-scale catalogs.
//!
//! Authoritative simulation ticks (`SimInstant`) advance Barycentric Dynamical
//! Time (`TDB`) uniformly from an explicit [`TdbEpoch`] scenario anchor.

use serde::{Deserialize, Serialize};

use crate::{
    clock::{SimInstant, TICKS_PER_SECOND, TickSpan},
    hash::{ContentHash, DomainHasher},
};

mod coverage;
mod scales;

pub use coverage::{CoverageSet, TdbSpanNs};
pub use scales::{
    EarthEopEntry, EarthEopTable, FutureUtcPolicy, LeapSecondEntry, LeapSecondTable,
    OutOfEopPolicy, TimeError, TimeScaleCatalog, Ut1Sample,
};

const NANOS_PER_SECOND_I128: i128 = 1_000_000_000;
const TPS_U128: u128 = TICKS_PER_SECOND as u128;

/// Exact coordinate timestamp in Barycentric Dynamical Time (`TDB`) relative to
/// `J2000.0 TDB` (`2000-01-01T12:00:00 TDB`, JD `2451545.0`).
#[derive(
    Clone, Copy, Debug, Default, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize,
)]
pub struct TdbEpoch {
    j2000_seconds: i64,
    subsec_ticks: u64,
}

impl TdbEpoch {
    /// `J2000.0 TDB` (`2000-01-01T12:00:00 TDB`).
    pub const J2000: Self = Self {
        j2000_seconds: 0,
        subsec_ticks: 0,
    };

    /// `2026-01-01T00:00:00 TDB` (`+820,497,600 s` past J2000).
    pub const JAN_1_2026_00Z_TDB: Self = Self {
        j2000_seconds: 820_497_600,
        subsec_ticks: 0,
    };

    /// `2028-01-01T00:00:00 TDB` (`+883,569,600 s` past J2000).
    pub const JAN_1_2028_00Z_TDB: Self = Self {
        j2000_seconds: 883_569_600,
        subsec_ticks: 0,
    };

    /// Constructs a canonical epoch from whole seconds past `J2000.0 TDB` and `subsec_ticks`.
    #[must_use]
    pub const fn new(j2000_seconds: i64, subsec_ticks: u64) -> Self {
        let extra_s = (subsec_ticks / TICKS_PER_SECOND) as i64;
        let rem_ticks = subsec_ticks % TICKS_PER_SECOND;
        Self {
            j2000_seconds: j2000_seconds.saturating_add(extra_s),
            subsec_ticks: rem_ticks,
        }
    }

    /// Constructs an epoch from signed nanoseconds relative to `J2000.0 TDB`.
    #[must_use]
    pub const fn from_j2000_nanos(nanos: i128) -> Self {
        let secs = nanos.div_euclid(NANOS_PER_SECOND_I128) as i64;
        let rem_ns = nanos.rem_euclid(NANOS_PER_SECOND_I128) as u128;
        let ticks = (rem_ns * TPS_U128).div_ceil(NANOS_PER_SECOND_I128 as u128) as u64;
        Self::new(secs, ticks)
    }

    /// Constructs an epoch from floating-point seconds past `J2000.0 TDB`.
    #[must_use]
    pub fn from_j2000_seconds_f64(seconds: f64) -> Option<Self> {
        if !seconds.is_finite() || seconds < -4.0e15 || seconds > 4.0e15 {
            return None;
        }
        let whole = seconds.floor();
        let frac = (seconds - whole).clamp(0.0, 1.0);
        let ticks = (frac * (TICKS_PER_SECOND as f64)).round() as u64;
        Some(Self::new(whole as i64, ticks))
    }

    /// Whole seconds past `J2000.0 TDB`.
    #[must_use]
    pub const fn j2000_seconds(self) -> i64 {
        self.j2000_seconds
    }

    /// Alias for [`Self::j2000_seconds`].
    #[must_use]
    pub const fn secs_j2000(self) -> i64 {
        self.j2000_seconds
    }

    /// Canonical subsecond master-clock ticks (`0..TICKS_PER_SECOND`).
    #[must_use]
    pub const fn subsec_ticks(self) -> u64 {
        self.subsec_ticks
    }

    /// Returns `(whole_seconds_f64, subsec_seconds_f64)` past `J2000.0 TDB`.
    #[must_use]
    pub fn split_j2000_seconds(self) -> (f64, f64) {
        (
            self.j2000_seconds as f64,
            (self.subsec_ticks as f64) / (TICKS_PER_SECOND as f64),
        )
    }

    /// Approximate single `f64` seconds past `J2000.0 TDB`.
    #[must_use]
    pub fn as_j2000_seconds_f64(self) -> f64 {
        let (w, s) = self.split_j2000_seconds();
        w + s
    }

    /// Signed nanoseconds from `J2000.0 TDB` (rounded down).
    #[must_use]
    pub const fn to_j2000_nanos_floor(self) -> i128 {
        let sub_ns = ((self.subsec_ticks as u128) * (NANOS_PER_SECOND_I128 as u128)) / TPS_U128;
        (self.j2000_seconds as i128) * NANOS_PER_SECOND_I128 + (sub_ns as i128)
    }

    /// Signed nanoseconds from `J2000.0 TDB` (rounded up).
    #[must_use]
    pub const fn to_j2000_nanos_ceil(self) -> i128 {
        let sub_ns =
            ((self.subsec_ticks as u128) * (NANOS_PER_SECOND_I128 as u128)).div_ceil(TPS_U128);
        (self.j2000_seconds as i128) * NANOS_PER_SECOND_I128 + (sub_ns as i128)
    }

    /// Advances by `span` master-clock ticks in uniform `TDB`.
    #[must_use]
    pub const fn add_span(self, span: TickSpan) -> Self {
        let (ds, dt) = span.seconds_and_subsec_ticks();
        let sum_ticks = (self.subsec_ticks as u128) + (dt as u128);
        let carry = (sum_ticks / TPS_U128) as i64;
        let rem = (sum_ticks % TPS_U128) as u64;
        Self {
            j2000_seconds: self
                .j2000_seconds
                .saturating_add(ds as i64)
                .saturating_add(carry),
            subsec_ticks: rem,
        }
    }

    /// Unsigned master-clock ticks `(self - anchor)` when `self >= anchor`, or `0` otherwise.
    #[must_use]
    pub const fn ticks_since(self, anchor: Self) -> u128 {
        let ds = (self.j2000_seconds as i128) - (anchor.j2000_seconds as i128);
        let dt = (self.subsec_ticks as i128) - (anchor.subsec_ticks as i128);
        let total = ds * (TICKS_PER_SECOND as i128) + dt;
        if total > 0 { total as u128 } else { 0 }
    }

    /// Signed segment-local offset `(self - anchor)` in `f64` seconds.
    #[must_use]
    pub fn signed_offset_seconds_from(self, anchor: Self) -> f64 {
        let ds = (self.j2000_seconds as i128) - (anchor.j2000_seconds as i128);
        let dt = (self.subsec_ticks as i128) - (anchor.subsec_ticks as i128);
        (ds as f64) + (dt as f64) / (TICKS_PER_SECOND as f64)
    }

    /// Advances by `delta_s` seconds (`f64`).
    #[must_use]
    pub fn advanced_seconds(self, delta_s: f64) -> Self {
        Self::from_j2000_seconds_f64(self.as_j2000_seconds_f64() + delta_s).unwrap_or(self)
    }
}

/// Pure adapter mapping `SimInstant` ticks onto `TDB` coordinate epochs.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct SimClockAdapter {
    base_tdb: TdbEpoch,
}

impl SimClockAdapter {
    /// Anchors [`SimInstant::START`] (`tick 0`) to `base_tdb`.
    #[must_use]
    pub const fn new(base_tdb: TdbEpoch) -> Self {
        Self { base_tdb }
    }

    /// The `TDB` epoch at [`SimInstant::START`].
    #[must_use]
    pub const fn base_tdb(self) -> TdbEpoch {
        self.base_tdb
    }

    /// Alias for [`Self::base_tdb`].
    #[must_use]
    pub const fn anchor_tdb(self) -> TdbEpoch {
        self.base_tdb
    }

    /// Canonical SHA-256 content hash of this clock anchor.
    #[must_use]
    pub fn content_hash(self) -> ContentHash {
        let mut h = DomainHasher::new(b"sim-spacetime:sim-clock-adapter:v1");
        h.write_i64(self.base_tdb.j2000_seconds());
        h.write_u64(self.base_tdb.subsec_ticks());
        h.finish()
    }

    /// Maps a simulation instant to its exact `TDB` coordinate epoch.
    #[must_use]
    pub fn instant_to_tdb(self, instant: SimInstant) -> TdbEpoch {
        self.base_tdb
            .add_span(instant.duration_since(SimInstant::START))
    }

    /// Maps a `TdbEpoch` at or after `base_tdb` back to [`SimInstant`], returning
    /// `None` if `epoch < base_tdb`.
    #[must_use]
    pub fn tdb_to_instant(self, epoch: TdbEpoch) -> Option<SimInstant> {
        if epoch < self.base_tdb {
            return None;
        }
        let mut ds = (epoch.j2000_seconds() - self.base_tdb.j2000_seconds()) as u128;
        let dt = if epoch.subsec_ticks() >= self.base_tdb.subsec_ticks() {
            (epoch.subsec_ticks() - self.base_tdb.subsec_ticks()) as u128
        } else {
            ds = ds.saturating_sub(1);
            (TICKS_PER_SECOND - (self.base_tdb.subsec_ticks() - epoch.subsec_ticks())) as u128
        };
        let total_ticks = ds
            .saturating_mul(TICKS_PER_SECOND as u128)
            .saturating_add(dt);
        Some(SimInstant::START.advanced(TickSpan::from_ticks_u128(total_ticks)))
    }
}
