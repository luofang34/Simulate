//! Exact integer master simulation clock (`SimInstant`, `TickSpan`, `TICKS_PER_SECOND`).

#[cfg(test)]
mod tests;

/// Default master-clock resolution: 46.08 GHz (`2^16 × 3^2 × 5^7` ticks/s).
pub const TICKS_PER_SECOND: u64 = 46_080_000_000;

const _: () = assert!((TICKS_PER_SECOND as u128 * 12).is_multiple_of(1_024_000));

const NANOS_PER_SECOND_U64: u64 = 1_000_000_000;
const TPS_U128: u128 = TICKS_PER_SECOND as u128;

/// A duration in master-clock ticks (`u128` for multi-century time-warp headroom at picosecond resolution).
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub struct TickSpan(u128);

impl TickSpan {
    /// No time.
    pub const ZERO: Self = Self(0);
    /// One second on the master clock.
    pub const SECOND: Self = Self(TICKS_PER_SECOND as u128);
    /// One microsecond on the master clock (`46_080` ticks).
    pub const MICROSECOND: Self = Self((TICKS_PER_SECOND / 1_000_000) as u128);
    /// One millisecond on the master clock (`46_080,000` ticks).
    pub const MILLISECOND: Self = Self((TICKS_PER_SECOND / 1_000) as u128);
    /// One Apollo Block II AGC memory cycle time (`12 / 1,024,000 s`).
    pub const AGC_MCT: Self = Self((TICKS_PER_SECOND as u128 * 12) / 1_024_000);

    /// A span of `ticks` master-clock ticks.
    #[must_use]
    pub const fn from_ticks(ticks: u64) -> Self {
        Self(ticks as u128)
    }

    /// A span of `ticks` master-clock ticks (`u128`).
    #[must_use]
    pub const fn from_ticks_u128(ticks: u128) -> Self {
        Self(ticks)
    }

    /// A span of whole `seconds` plus `subsec_ticks` master-clock ticks.
    #[must_use]
    pub const fn from_seconds_and_subsec_ticks(seconds: u64, subsec_ticks: u64) -> Self {
        Self((seconds as u128) * (TICKS_PER_SECOND as u128) + (subsec_ticks as u128))
    }

    /// The period of an integer rate in hertz, or `None` if that rate does not divide [`TICKS_PER_SECOND`] exactly.
    #[must_use]
    pub const fn period_of_hz(hz: u64) -> Option<Self> {
        Self::period_of_ratio(hz, 1)
    }

    /// The period of a rational rate `num_hz / denom_s` hertz, or `None` if not an exact integer number of ticks.
    #[must_use]
    pub const fn period_of_ratio(num_hz: u64, denom_s: u64) -> Option<Self> {
        if num_hz == 0 || denom_s == 0 {
            return None;
        }
        let total = (TICKS_PER_SECOND as u128).saturating_mul(denom_s as u128);
        if !total.is_multiple_of(num_hz as u128) {
            return None;
        }
        Some(Self(total / (num_hz as u128)))
    }

    /// Tick count as `u64`, saturating at [`u64::MAX`] if the span exceeds 12.68 years.
    #[must_use]
    pub const fn ticks(self) -> u64 {
        if self.0 > u64::MAX as u128 {
            u64::MAX
        } else {
            self.0 as u64
        }
    }

    /// Exact 128-bit tick count.
    #[must_use]
    pub const fn ticks_u128(self) -> u128 {
        self.0
    }

    /// Decomposes this span into `(whole_seconds, subsec_ticks)` where `subsec_ticks < TICKS_PER_SECOND`.
    #[must_use]
    pub const fn seconds_and_subsec_ticks(self) -> (u64, u64) {
        let secs = self.0 / TPS_U128;
        let rem = (self.0 % TPS_U128) as u64;
        let secs_u64 = if secs > u64::MAX as u128 {
            u64::MAX
        } else {
            secs as u64
        };
        (secs_u64, rem)
    }

    /// Length in seconds (`f64`).
    #[must_use]
    pub fn as_seconds(self) -> f64 {
        let whole_s = self.0 / TPS_U128;
        let rem_ticks = (self.0 % TPS_U128) as u64;
        whole_s as f64 + (rem_ticks as f64 / TICKS_PER_SECOND as f64)
    }
}

/// An instant on the master clock, counted from simulation start in `u128` ticks.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub struct SimInstant(u128);

impl SimInstant {
    /// Simulation start.
    pub const START: Self = Self(0);

    /// Creates an instant at `ticks` master-clock ticks from [`Self::START`].
    #[must_use]
    pub const fn from_ticks_u128(ticks: u128) -> Self {
        Self(ticks)
    }

    /// Ticks since simulation start, saturating at [`u64::MAX`].
    #[must_use]
    pub const fn ticks(self) -> u64 {
        if self.0 > u64::MAX as u128 {
            u64::MAX
        } else {
            self.0 as u64
        }
    }

    /// Exact 128-bit ticks since simulation start.
    #[must_use]
    pub const fn ticks_u128(self) -> u128 {
        self.0
    }

    /// Elapsed `(whole_seconds, subsec_ticks)` since [`Self::START`].
    #[must_use]
    pub const fn seconds_and_subsec_ticks(self) -> (u64, u64) {
        TickSpan(self.0).seconds_and_subsec_ticks()
    }

    /// Whole nanoseconds since [`Self::START`], rounded down, or `None` on `u64` overflow.
    #[must_use]
    pub const fn to_nanos(self) -> Option<u64> {
        let (seconds, subsec_ticks) = self.seconds_and_subsec_ticks();
        let subsec_ns = (subsec_ticks as u128 * NANOS_PER_SECOND_U64 as u128 / TPS_U128) as u64;
        match seconds.checked_mul(NANOS_PER_SECOND_U64) {
            Some(whole_ns) => whole_ns.checked_add(subsec_ns),
            None => None,
        }
    }

    /// The first instant whose [`Self::to_nanos`] is `nanos`.
    #[must_use]
    pub const fn from_nanos(nanos: u64) -> Self {
        Self((nanos as u128 * TPS_U128).div_ceil(NANOS_PER_SECOND_U64 as u128))
    }

    /// Elapsed time since [`Self::START`] in `f64` seconds.
    #[must_use]
    pub fn as_seconds(self) -> f64 {
        TickSpan(self.0).as_seconds()
    }

    /// Duration elapsed since `earlier`, or [`TickSpan::ZERO`] if `earlier` is after `self`.
    #[must_use]
    pub const fn duration_since(self, earlier: Self) -> TickSpan {
        TickSpan(self.0.saturating_sub(earlier.0))
    }

    /// This instant moved forward by `span`.
    #[must_use]
    pub const fn advanced(self, span: TickSpan) -> Self {
        Self(self.0.wrapping_add(span.0))
    }
}

/// Lossless `(whole_seconds, subsec_ticks)` serde helper for [`SimInstant`].
pub mod sim_instant_serde {
    use serde::{Deserialize, Deserializer, Serialize, Serializer};

    use super::{SimInstant, TickSpan};

    /// Serializes `SimInstant` as `(u64, u64)` `(seconds, subsec_ticks)`.
    pub fn serialize<S: Serializer>(inst: &SimInstant, s: S) -> Result<S::Ok, S::Error> {
        inst.seconds_and_subsec_ticks().serialize(s)
    }

    /// Deserializes `SimInstant` from `(u64, u64)` `(seconds, subsec_ticks)`.
    pub fn deserialize<'de, D: Deserializer<'de>>(d: D) -> Result<SimInstant, D::Error> {
        let (secs, sub) = <(u64, u64)>::deserialize(d)?;
        Ok(SimInstant::START.advanced(TickSpan::from_seconds_and_subsec_ticks(secs, sub)))
    }
}
