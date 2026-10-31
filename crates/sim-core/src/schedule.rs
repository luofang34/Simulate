use crate::{TICKS_PER_SECOND, TickSpan};

#[cfg(test)]
mod tests;

/// Converts elapsed master-clock time into whole steps of one block,
/// carrying the remainder so no time is lost or gained across frames.
///
/// Supports both exact tick periods ([`Self::new`]) and arbitrary rational
/// oscillator frequencies `num_hz / denom_s` ([`Self::from_hz_ratio`]) that do
/// not divide [`TICKS_PER_SECOND`] evenly (such as GPS 10.23 MHz, NTSC
/// `60_000 / 1_001` Hz, or crystals with ppm thermal drift).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RateAdapter {
    num: u128,
    den: u128,
    carry: u128,
}

impl RateAdapter {
    /// An adapter for a block with an exact tick `period`; a zero period never
    /// steps.
    pub const fn new(period: TickSpan) -> Self {
        Self {
            num: 1,
            den: period.ticks_u128(),
            carry: 0,
        }
    }

    /// An adapter for an oscillator or bus running at `num_hz / denom_s` steps
    /// per second, using exact rational remainder accumulation so even
    /// non-dividing frequencies never drift. A zero numerator or denominator
    /// never steps.
    pub const fn from_hz_ratio(num_hz: u64, denom_s: u64) -> Self {
        if num_hz == 0 || denom_s == 0 {
            return Self {
                num: 0,
                den: 0,
                carry: 0,
            };
        }
        if let Some(period) = TickSpan::period_of_ratio(num_hz, denom_s) {
            return Self::new(period);
        }
        Self {
            num: num_hz as u128,
            den: (TICKS_PER_SECOND as u128).saturating_mul(denom_s as u128),
            carry: 0,
        }
    }

    /// Steps due after `elapsed` more ticks (`u64`, saturating at [`u64::MAX`]).
    pub fn steps_for(&mut self, elapsed: TickSpan) -> u64 {
        let steps = self.steps_for_u128(elapsed);
        u64::try_from(steps).unwrap_or(u64::MAX)
    }

    /// Exact 128-bit step count due after `elapsed` more ticks, supporting
    /// century-scale time-warp jumps without saturation.
    pub fn steps_for_u128(&mut self, elapsed: TickSpan) -> u128 {
        if self.den == 0 || self.num == 0 {
            return 0;
        }
        let ticks = elapsed.ticks_u128();
        let whole_cycles = ticks / self.den;
        let rem_ticks = ticks % self.den;
        let gained = rem_ticks.saturating_mul(self.num);
        let total = self.carry.saturating_add(gained);
        self.carry = total % self.den;
        whole_cycles
            .saturating_mul(self.num)
            .saturating_add(total / self.den)
    }
}

/// Converts wall-clock frame durations into master-clock ticks for real-time
/// and time-warped play, keeping the sub-tick remainder and refusing to catch
/// up more than `max_catch_up` of wall-clock time after a stall (a debugger
/// pause, a backgrounded tab).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RealTimeClock {
    max_catch_up: TickSpan,
    fractional_ticks: f64,
}

impl RealTimeClock {
    /// A clock that caps wall-clock frame gaps at `max_catch_up` (at 1.0x
    /// real time; scaled proportionally when `time_scale > 1.0` so fast
    /// time-warping is never throttled by the stall guard).
    pub fn new(max_catch_up: TickSpan) -> Self {
        Self {
            max_catch_up,
            fractional_ticks: 0.0,
        }
    }

    /// Ticks to simulate for `real_seconds` of wall-clock time at
    /// `time_scale` (1.0 is real time; e.g. `1e8` for century-scale fast warp).
    /// Splits whole simulated seconds from fractional seconds before
    /// multiplying by [`TICKS_PER_SECOND`] so `f64` significand limits never
    /// degrade tick accuracy during extreme time-warp.
    pub fn advance(&mut self, real_seconds: f64, time_scale: f64) -> TickSpan {
        if !real_seconds.is_finite()
            || !time_scale.is_finite()
            || real_seconds <= 0.0
            || time_scale <= 0.0
        {
            self.fractional_ticks = 0.0;
            return TickSpan::ZERO;
        }
        let max_real_s = self.max_catch_up.as_seconds();
        let capped_real_s = if real_seconds >= max_real_s {
            self.fractional_ticks = 0.0;
            max_real_s
        } else {
            real_seconds
        };
        let sim_seconds = capped_real_s * time_scale;
        if !sim_seconds.is_finite() || sim_seconds <= 0.0 {
            self.fractional_ticks = 0.0;
            return TickSpan::ZERO;
        }
        let whole_sim_s = sim_seconds.trunc();
        let fract_sim_s = sim_seconds.fract();
        let subsec_wanted = fract_sim_s * (TICKS_PER_SECOND as f64) + self.fractional_ticks;
        let subsec_whole = subsec_wanted.floor();
        self.fractional_ticks = subsec_wanted - subsec_whole;
        let whole_ticks = (whole_sim_s as u128).saturating_mul(TICKS_PER_SECOND as u128);
        TickSpan::from_ticks_u128(whole_ticks.saturating_add(subsec_whole as u128))
    }
}
