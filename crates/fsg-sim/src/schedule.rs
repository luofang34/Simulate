use crate::{TICKS_PER_SECOND, TickSpan};

#[cfg(test)]
mod tests;

/// Converts elapsed master-clock time into whole steps of one block,
/// carrying the remainder so no time is lost or gained across frames.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RateAdapter {
    period: TickSpan,
    carry: u64,
}

impl RateAdapter {
    /// An adapter for a block with `period`; a zero period never steps.
    pub fn new(period: TickSpan) -> Self {
        Self { period, carry: 0 }
    }

    /// Steps due after `elapsed` more ticks.
    pub fn steps_for(&mut self, elapsed: TickSpan) -> u64 {
        let period = self.period.ticks();
        if period == 0 {
            return 0;
        }
        let total = self.carry.saturating_add(elapsed.ticks());
        self.carry = total % period;
        total / period
    }
}

/// Converts wall-clock frame durations into master-clock ticks for real-time
/// play, keeping the sub-tick remainder and refusing to catch up more than
/// `max_catch_up` after a stall (a debugger pause, a backgrounded tab).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RealTimeClock {
    max_catch_up: TickSpan,
    fractional_ticks: f64,
}

impl RealTimeClock {
    /// A clock that advances at most `max_catch_up` per call.
    pub fn new(max_catch_up: TickSpan) -> Self {
        Self {
            max_catch_up,
            fractional_ticks: 0.0,
        }
    }

    /// Ticks to simulate for `real_seconds` of wall-clock time at
    /// `time_scale` (1.0 is real time). Negative or non-finite input advances
    /// nothing.
    pub fn advance(&mut self, real_seconds: f64, time_scale: f64) -> TickSpan {
        let wanted = real_seconds * time_scale * TICKS_PER_SECOND as f64 + self.fractional_ticks;
        if !wanted.is_finite() || wanted <= 0.0 {
            self.fractional_ticks = 0.0;
            return TickSpan::ZERO;
        }
        let cap = self.max_catch_up.ticks() as f64;
        if wanted >= cap {
            self.fractional_ticks = 0.0;
            return self.max_catch_up;
        }
        let whole = wanted.floor();
        self.fractional_ticks = wanted - whole;
        TickSpan::from_ticks(whole as u64)
    }
}
