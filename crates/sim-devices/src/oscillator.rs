//! A frequency standard that counts its own proper time: a GNSS satellite's
//! clock, a spacecraft's oscillator, a station's maser. It is told how fast
//! proper time runs where it is, and reports how far its count has drifted
//! from the coordinate time the master clock ticks.

use glam::DVec3;
use sim_core::{Block, BlockError, TickSpan};
use sim_spacetime::{ProperTimeClock, orbit_periodic_clock_term_s};

const BLOCK: &str = "oscillator";

/// One oscillator.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct OscillatorConfig {
    /// Fractional frequency the oscillator is built off its nominal by. A
    /// GPS clock is set `-4.4647e-10` low on the ground so that it keeps
    /// system time in orbit.
    pub fractional_frequency_offset: f64,
    /// How often the count is brought up to date.
    pub period: TickSpan,
}

/// An oscillator's count against coordinate time, and the rate it last ran
/// at.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct OscillatorSnapshot {
    clock: ProperTimeClock,
    rate_offset: f64,
}

/// An oscillator as a block.
///
/// Each step it takes `dτ/dt - 1` at its position and velocity, against the
/// coordinate time its period is measured in, and publishes its reading
/// minus that coordinate time, in seconds. Master-clock ticks are uniform
/// TDB, so a barycentric rate integrates over the period exactly as it
/// stands.
#[derive(Debug, Clone, PartialEq)]
pub struct OscillatorBlock {
    config: OscillatorConfig,
    state: OscillatorSnapshot,
}

impl OscillatorBlock {
    /// An oscillator reading `reading_minus_coordinate_s` ahead of
    /// coordinate time, where proper time runs at `1 + rate_offset`.
    #[must_use]
    pub const fn new(
        config: OscillatorConfig,
        reading_minus_coordinate_s: f64,
        rate_offset: f64,
    ) -> Self {
        Self {
            config,
            state: OscillatorSnapshot {
                clock: ProperTimeClock::with_offset(reading_minus_coordinate_s),
                rate_offset,
            },
        }
    }

    /// The oscillator's reading minus coordinate time after removing the
    /// orbital eccentricity periodic term `-2 (r·v) / c²` (IS-GPS-200 `Δt_r`).
    #[must_use]
    pub fn system_time_offset_s(&self, position_m: DVec3, velocity_m_s: DVec3) -> f64 {
        self.output() - orbit_periodic_clock_term_s(position_m, velocity_m_s)
    }

    /// The oscillator's own rate against coordinate time where proper time
    /// runs at `1 + rate_offset`.
    fn counted(&self, rate_offset: f64) -> f64 {
        let built = self.config.fractional_frequency_offset;
        rate_offset + built + rate_offset * built
    }
}

impl Block for OscillatorBlock {
    type Input = f64;
    type Output = f64;
    type Snapshot = OscillatorSnapshot;

    fn period(&self) -> TickSpan {
        self.config.period
    }

    fn step(&mut self, rate_offset: &f64) -> Result<(), BlockError> {
        if !rate_offset.is_finite() {
            return Err(BlockError::InputOutOfRange {
                block: BLOCK,
                input: "proper-time rate offset",
                detail: format!("{rate_offset}"),
            });
        }
        let (before, after) = (
            self.counted(self.state.rate_offset),
            self.counted(*rate_offset),
        );
        self.state
            .clock
            .advance(self.config.period.as_seconds(), before, after);
        self.state.rate_offset = *rate_offset;
        Ok(())
    }

    fn output(&self) -> f64 {
        self.state.clock.tau_minus_tt_s()
    }

    fn snapshot(&self) -> OscillatorSnapshot {
        self.state
    }

    fn restore(&mut self, snapshot: &OscillatorSnapshot) {
        self.state = *snapshot;
    }
}

#[cfg(test)]
mod tests;
