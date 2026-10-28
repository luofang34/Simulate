#[cfg(test)]
mod tests;

/// Master-clock resolution. 15.36 MHz is the smallest rate that every
/// Apollo AGC timing source except its 2.048 MHz crystal divides into, while
/// also dividing evenly into common frame rates (60, 120, 240 Hz).
pub const TICKS_PER_SECOND: u64 = 15_360_000;

/// A duration in master-clock ticks.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub struct TickSpan(u64);

impl TickSpan {
    /// No time.
    pub const ZERO: Self = Self(0);
    /// One AGC memory cycle time, 12/1,024,000 s.
    pub const AGC_MCT: Self = Self(180);

    /// A span of exactly `ticks` ticks.
    pub const fn from_ticks(ticks: u64) -> Self {
        Self(ticks)
    }

    /// The period of a rate in hertz, or `None` if that rate does not divide
    /// the master clock exactly (an inexact period would drift).
    pub const fn period_of_hz(hz: u64) -> Option<Self> {
        if hz == 0 || !TICKS_PER_SECOND.is_multiple_of(hz) {
            None
        } else {
            Some(Self(TICKS_PER_SECOND / hz))
        }
    }

    /// Tick count.
    pub const fn ticks(self) -> u64 {
        self.0
    }

    /// Length in seconds, for physics that integrates in SI units; exact
    /// while the count stays below 2^53 ticks (about 18 years).
    pub fn as_seconds(self) -> f64 {
        self.0 as f64 / TICKS_PER_SECOND as f64
    }
}

/// An instant on the master clock, counted from simulation start.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub struct SimInstant(u64);

impl SimInstant {
    /// Simulation start.
    pub const START: Self = Self(0);

    /// Ticks since simulation start.
    pub const fn ticks(self) -> u64 {
        self.0
    }

    /// This instant moved forward by `span`. The counter is monotonic and wraps
    /// only after about 38,000 simulated years.
    #[must_use]
    pub const fn advanced(self, span: TickSpan) -> Self {
        Self(self.0.wrapping_add(span.0))
    }
}
