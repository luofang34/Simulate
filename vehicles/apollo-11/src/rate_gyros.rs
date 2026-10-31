use glam::{BVec3, DVec3};
use sim_core::{Block, BlockError, TickSpan};

use crate::InertialSiteSample;
use crate::site::require_period;

#[cfg(test)]
mod tests;

const BLOCK: &str = "rate_gyro_assembly";

/// Rate at which the assembly's output pins, rad/s: 20°/s.
const NOMINAL_RANGE_RAD_S: f64 = 20.0 * std::f64::consts::PI / 180.0;

/// Setup of a [`RateGyroAssemblyBlock`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RateGyroConfig {
    /// Sample period.
    pub period: TickSpan,
    /// Bias fixed at turn-on, site axes, rad/s.
    pub turn_on_bias_rad_s: DVec3,
    /// Output range per axis, rad/s.
    pub range_rad_s: f64,
}

impl RateGyroConfig {
    /// A representative unit with `turn_on_bias_rad_s` and the nominal range.
    pub fn nominal(period: TickSpan, turn_on_bias_rad_s: DVec3) -> Self {
        Self {
            period,
            turn_on_bias_rad_s,
            range_rad_s: NOMINAL_RANGE_RAD_S,
        }
    }
}

/// What a [`RateGyroAssemblyBlock`] publishes.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RateGyroOutput {
    /// Body rate, site axes, rad/s.
    pub rate_rad_s: DVec3,
    /// Per axis, whether the output has pinned at its range.
    pub saturated: BVec3,
}

/// Three body-rate gyros for stabilisation feedback: the `BMAG`s of the CM and
/// the `RGA` of the LM.
#[derive(Debug, Clone, PartialEq)]
pub struct RateGyroAssemblyBlock {
    config: RateGyroConfig,
    output: RateGyroOutput,
}

impl RateGyroAssemblyBlock {
    /// A unit reading zero. A zero period or nonpositive range is an error.
    pub fn new(config: RateGyroConfig) -> Result<Self, BlockError> {
        require_period(BLOCK, config.period)?;
        if !(config.range_rad_s.is_finite() && config.range_rad_s > 0.0) {
            return Err(BlockError::InputOutOfRange {
                block: BLOCK,
                input: "range_rad_s",
                detail: format!("{} is not a positive range", config.range_rad_s),
            });
        }
        Ok(Self {
            config,
            output: RateGyroOutput {
                rate_rad_s: DVec3::ZERO,
                saturated: BVec3::FALSE,
            },
        })
    }
}

impl Block for RateGyroAssemblyBlock {
    type Input = InertialSiteSample;
    type Output = RateGyroOutput;
    type Snapshot = RateGyroOutput;

    fn period(&self) -> TickSpan {
        self.config.period
    }

    fn step(&mut self, input: &InertialSiteSample) -> Result<(), BlockError> {
        input.validate(BLOCK)?;
        let range = DVec3::splat(self.config.range_rad_s);
        let raw = input.angular_rate_site_rad_s + self.config.turn_on_bias_rad_s;
        self.output = RateGyroOutput {
            rate_rad_s: raw.clamp(-range, range),
            saturated: raw.abs().cmpge(range),
        };
        Ok(())
    }

    fn output(&self) -> RateGyroOutput {
        self.output
    }

    fn snapshot(&self) -> RateGyroOutput {
        self.output
    }

    fn restore(&mut self, snapshot: &RateGyroOutput) {
        self.output = *snapshot;
    }
}
