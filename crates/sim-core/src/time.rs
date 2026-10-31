#[cfg(test)]
mod tests;

/// Default master-clock resolution: 46.08 GHz (`2^16 × 3^2 × 5^7` ticks/s).
///
/// Chosen as a highly composite rate that divides evenly into:
/// - Avionics data buses: ARINC 429 high-speed (100 kbps / 200 kHz RZ) and
///   low-speed (12.5 kbps / 25 kHz RZ), MIL-STD-1553 (1 Mbps / 2 MHz),
///   CAN / CAN-FD (125k, 250k, 500k, 1M, 2M, 4M, 5M, 8M, 10M), I2C (100k,
///   400k, 1M, 3.2M), and standard UART baud rates (300..921,600 bps from
///   1.8432 / 7.3728 / 14.7456 MHz crystals).
/// - Historical guidance computers: Apollo Block II AGC 2.048 MHz crystal,
///   1.024 MHz clock, memory cycle time (12 / 1,024,000 s), PIPA/gyro
///   pulses (3,200 Hz), scaler (1,600 Hz), and AGS (1 MHz).
/// - Common MCU, sensor, and RTC oscillators: 32.768 kHz RTC (`2^15` Hz),
///   65.536 kHz (`2^16` Hz), sensor ODRs (12.5 Hz..32 kHz), and 1, 2, 4, 5,
///   8, 10, 12, 16, 20, 24, 32, 40, 48, 60, 64, 72, 80, 96, 120, 144, 160,
///   240, 480 MHz clocks.
/// - Display and physics frame rates: 24, 25, 30, 36, 45, 48, 50, 60, 72,
///   75, 90, 100, 120, 144, 180, 200, 240, 300, 360, 400, 480, 600, 1,000 Hz.
///
/// [`SimInstant`] and [`TickSpan`] store tick counts in `u128` (16 bytes,
/// matching [`core::time::Duration`]), providing `2.34 × 10^20` years of
/// range at 21.7 ps resolution so century-scale fast time-warping never
/// overflows.
pub const TICKS_PER_SECOND: u64 = 46_080_000_000;

const _: () = assert!((TICKS_PER_SECOND as u128 * 12).is_multiple_of(1_024_000));

const NANOS_PER_SECOND: u64 = 1_000_000_000;

/// A duration in master-clock ticks (`u128` for multi-century time-warp
/// headroom at picosecond resolution).
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub struct TickSpan(u128);

impl TickSpan {
    /// No time.
    pub const ZERO: Self = Self(0);
    /// One second on the master clock.
    pub const SECOND: Self = Self(TICKS_PER_SECOND as u128);
    /// One Apollo Block II AGC memory cycle time, 12 / 1,024,000 s, derived
    /// from [`TICKS_PER_SECOND`] so it adapts automatically if the master-clock
    /// constant changes.
    pub const AGC_MCT: Self = Self((TICKS_PER_SECOND as u128 * 12) / 1_024_000);

    /// A span of `ticks` master-clock ticks.
    pub const fn from_ticks(ticks: u64) -> Self {
        Self(ticks as u128)
    }

    /// A span of `ticks` master-clock ticks (`u128`, for multi-decade or
    /// multi-century intervals).
    pub const fn from_ticks_u128(ticks: u128) -> Self {
        Self(ticks)
    }

    /// A span of whole `seconds` plus `subsec_ticks` master-clock ticks.
    pub const fn from_seconds_and_subsec_ticks(seconds: u64, subsec_ticks: u64) -> Self {
        Self((seconds as u128) * (TICKS_PER_SECOND as u128) + (subsec_ticks as u128))
    }

    /// The period of an integer rate in hertz, or `None` if that rate does not
    /// divide [`TICKS_PER_SECOND`] exactly.
    pub const fn period_of_hz(hz: u64) -> Option<Self> {
        Self::period_of_ratio(hz, 1)
    }

    /// The period of a rational rate `num_hz / denom_s` hertz (for example,
    /// ARINC 429 low-speed 12.5 kHz as `(12_500, 1)` or a 12.5 Hz sensor ODR
    /// as `(25, 2)`), or `None` if the period is not an exact integer number
    /// of master-clock ticks.
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

    /// Tick count as `u64`, saturating at [`u64::MAX`] if the span exceeds
    /// 12.68 years. Prefer [`Self::ticks_u128`] or [`Self::seconds_and_subsec_ticks`]
    /// for century-scale intervals.
    pub const fn ticks(self) -> u64 {
        if self.0 > u64::MAX as u128 {
            u64::MAX
        } else {
            self.0 as u64
        }
    }

    /// Exact 128-bit tick count.
    pub const fn ticks_u128(self) -> u128 {
        self.0
    }

    /// Decomposes this span into `(whole_seconds, subsec_ticks)` where
    /// `subsec_ticks < TICKS_PER_SECOND`. Lossless across multi-century
    /// durations.
    pub const fn seconds_and_subsec_ticks(self) -> (u64, u64) {
        let tps = TICKS_PER_SECOND as u128;
        let secs = self.0 / tps;
        let rem = (self.0 % tps) as u64;
        let secs_u64 = if secs > u64::MAX as u128 {
            u64::MAX
        } else {
            secs as u64
        };
        (secs_u64, rem)
    }

    /// Length in seconds, for physics that integrates in SI units. Splits
    /// whole seconds from sub-second ticks so sub-second precision is preserved
    /// across arbitrary durations.
    pub fn as_seconds(self) -> f64 {
        let tps = TICKS_PER_SECOND as u128;
        let whole_s = self.0 / tps;
        let rem_ticks = (self.0 % tps) as u64;
        whole_s as f64 + (rem_ticks as f64 / TICKS_PER_SECOND as f64)
    }
}

/// An instant on the master clock, counted from simulation start in `u128`
/// ticks (`2.34 × 10^20` years of range at 46.08 GHz).
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub struct SimInstant(u128);

impl SimInstant {
    /// Simulation start.
    pub const START: Self = Self(0);

    /// Ticks since simulation start, saturating at [`u64::MAX`] if past
    /// 12.68 years. Prefer [`Self::ticks_u128`] or
    /// [`Self::seconds_and_subsec_ticks`] for deep-space / century time scales.
    pub const fn ticks(self) -> u64 {
        if self.0 > u64::MAX as u128 {
            u64::MAX
        } else {
            self.0 as u64
        }
    }

    /// Exact 128-bit ticks since simulation start.
    pub const fn ticks_u128(self) -> u128 {
        self.0
    }

    /// Elapsed `(whole_seconds, subsec_ticks)` since [`Self::START`].
    pub const fn seconds_and_subsec_ticks(self) -> (u64, u64) {
        TickSpan(self.0).seconds_and_subsec_ticks()
    }

    /// Whole nanoseconds since [`Self::START`], rounded down, or `None` past
    /// the 584 years a `u64` count holds. A tick is about 21.7 ps, so instants
    /// less than a nanosecond apart can share a value; the order never inverts.
    pub const fn to_nanos(self) -> Option<u64> {
        let (seconds, subsec_ticks) = self.seconds_and_subsec_ticks();
        let subsec_ns =
            (subsec_ticks as u128 * NANOS_PER_SECOND as u128 / TICKS_PER_SECOND as u128) as u64;
        match seconds.checked_mul(NANOS_PER_SECOND) {
            Some(whole_ns) => whole_ns.checked_add(subsec_ns),
            None => None,
        }
    }

    /// The first instant whose [`Self::to_nanos`] is `nanos`, so nanoseconds
    /// survive the round trip exactly.
    pub const fn from_nanos(nanos: u64) -> Self {
        Self((nanos as u128 * TICKS_PER_SECOND as u128).div_ceil(NANOS_PER_SECOND as u128))
    }

    /// Duration elapsed since `earlier`, or [`TickSpan::ZERO`] if `earlier` is
    /// after `self`.
    pub const fn duration_since(self, earlier: Self) -> TickSpan {
        TickSpan(self.0.saturating_sub(earlier.0))
    }

    /// This instant moved forward by `span`. The 128-bit counter wraps only
    /// after `2.34 × 10^20` simulated years.
    #[must_use]
    pub const fn advanced(self, span: TickSpan) -> Self {
        Self(self.0.wrapping_add(span.0))
    }
}
