use std::f64::consts::PI;

use glam::DVec3;
use mbd_core::LocalSiteKinematics;
use sim_core::{Block, BlockError, TickSpan};

use super::{RadarValidity, require_fixed_site};
use crate::site::require_period;

const RENDEZVOUS_BLOCK: &str = "rendezvous_radar";
const VHF_BLOCK: &str = "vhf_ranging";

/// Rendezvous radar acquisition range, metres: 400 nautical miles.
const RR_MAX_RANGE_M: f64 = 740_800.0;
/// Rendezvous radar shaft travel either side of `+z`, rad: 70°.
const RR_MAX_SHAFT_RAD: f64 = 70.0 * PI / 180.0;
/// Rendezvous radar trunnion travel either side of the shaft plane, rad: 55°.
const RR_MAX_TRUNNION_RAD: f64 = 55.0 * PI / 180.0;
/// VHF ranging lock limit, metres: 327 nautical miles.
const VHF_MAX_RANGE_M: f64 = 605_600.0;
/// VHF range readout resolution, metres: 0.01 nautical mile.
const VHF_RESOLUTION_M: f64 = 18.52;

/// Where a ranging system and its target are, in one body-fixed frame.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RelativeTrackInput {
    /// The antenna site; see [`crate::site_in_fixed_frame`].
    pub site: LocalSiteKinematics,
    /// Target position, metres.
    pub target_position_fixed_m: DVec3,
    /// Target velocity, m/s.
    pub target_velocity_fixed_m_s: DVec3,
}

impl RelativeTrackInput {
    fn validate(&self, block: &'static str) -> Result<(), BlockError> {
        require_fixed_site(block, &self.site)?;
        let separation = self.target_position_fixed_m - self.site.position_m;
        if self.target_velocity_fixed_m_s.is_finite() && separation.length() > 0.0 {
            return Ok(());
        }
        Err(BlockError::InputOutOfRange {
            block,
            input: "target",
            detail: format!("{self:?} is not a distinct finite target"),
        })
    }

    fn line_of_sight(&self) -> DVec3 {
        self.target_position_fixed_m - self.site.position_m
    }
}

/// Setup of a [`RendezvousRadarBlock`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RendezvousRadarConfig {
    /// Sample period.
    pub period: TickSpan,
    /// Highest usable range, metres.
    pub max_range_m: f64,
    /// Shaft travel either side of site `+z`, rad.
    pub max_shaft_rad: f64,
    /// Trunnion travel either side of the shaft plane, rad.
    pub max_trunnion_rad: f64,
}

impl RendezvousRadarConfig {
    /// The flown limits, sampled every `period`.
    pub fn nominal(period: TickSpan) -> Self {
        Self {
            period,
            max_range_m: RR_MAX_RANGE_M,
            max_shaft_rad: RR_MAX_SHAFT_RAD,
            max_trunnion_rad: RR_MAX_TRUNNION_RAD,
        }
    }
}

/// What a [`RendezvousRadarBlock`] publishes.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RendezvousRadarOutput {
    /// Whether the data can be used.
    pub validity: RadarValidity,
    /// Slant range, metres.
    pub range_m: f64,
    /// Range rate, m/s; positive opening.
    pub range_rate_m_s: f64,
    /// Line-of-sight rotation about site `+y` from `+z`, rad.
    pub shaft_rad: f64,
    /// Line-of-sight elevation out of the shaft plane, towards `+y`, rad.
    pub trunnion_rad: f64,
}

/// The LM's rendezvous radar tracking the CSM's transponder.
#[derive(Debug, Clone, PartialEq)]
pub struct RendezvousRadarBlock {
    config: RendezvousRadarConfig,
    output: RendezvousRadarOutput,
}

impl RendezvousRadarBlock {
    /// A radar with no track yet. A zero period is an error.
    pub fn new(config: RendezvousRadarConfig) -> Result<Self, BlockError> {
        require_period(RENDEZVOUS_BLOCK, config.period)?;
        Ok(Self {
            config,
            output: RendezvousRadarOutput {
                validity: RadarValidity::OutOfLimits,
                range_m: 0.0,
                range_rate_m_s: 0.0,
                shaft_rad: 0.0,
                trunnion_rad: 0.0,
            },
        })
    }
}

impl Block for RendezvousRadarBlock {
    type Input = RelativeTrackInput;
    type Output = RendezvousRadarOutput;
    type Snapshot = RendezvousRadarOutput;

    fn period(&self) -> TickSpan {
        self.config.period
    }

    fn step(&mut self, input: &RelativeTrackInput) -> Result<(), BlockError> {
        input.validate(RENDEZVOUS_BLOCK)?;
        let line_of_sight = input.line_of_sight();
        let range_m = line_of_sight.length();
        let direction = line_of_sight / range_m;
        let closing = input.target_velocity_fixed_m_s - input.site.velocity_m_s;
        let in_site = input.site.attitude.inverse() * direction;
        let shaft_rad = in_site.x.atan2(in_site.z);
        let trunnion_rad = in_site.y.clamp(-1.0, 1.0).asin();
        let valid = range_m <= self.config.max_range_m
            && shaft_rad.abs() <= self.config.max_shaft_rad
            && trunnion_rad.abs() <= self.config.max_trunnion_rad;
        self.output = RendezvousRadarOutput {
            validity: if valid {
                RadarValidity::Valid
            } else {
                RadarValidity::OutOfLimits
            },
            range_m,
            range_rate_m_s: closing.dot(direction),
            shaft_rad,
            trunnion_rad,
        };
        Ok(())
    }

    fn output(&self) -> RendezvousRadarOutput {
        self.output
    }

    fn snapshot(&self) -> RendezvousRadarOutput {
        self.output
    }

    fn restore(&mut self, snapshot: &RendezvousRadarOutput) {
        self.output = *snapshot;
    }
}

/// Setup of a [`VhfRangingBlock`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct VhfRangingConfig {
    /// Sample period.
    pub period: TickSpan,
    /// Highest usable range, metres.
    pub max_range_m: f64,
    /// Readout resolution, metres.
    pub resolution_m: f64,
    /// Fixed range error, metres.
    pub bias_m: f64,
}

impl VhfRangingConfig {
    /// The flown limits with a fixed error of `bias_m`, sampled every
    /// `period`.
    pub fn nominal(period: TickSpan, bias_m: f64) -> Self {
        Self {
            period,
            max_range_m: VHF_MAX_RANGE_M,
            resolution_m: VHF_RESOLUTION_M,
            bias_m,
        }
    }
}

/// What a [`VhfRangingBlock`] publishes.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct VhfRangingOutput {
    /// Whether the data can be used.
    pub validity: RadarValidity,
    /// Measured range, metres.
    pub range_m: f64,
}

/// The CSM's VHF ranging: an omnidirectional range measurement over the voice
/// link, independent of the LM's radar, so the CSM can cross-check it.
#[derive(Debug, Clone, PartialEq)]
pub struct VhfRangingBlock {
    config: VhfRangingConfig,
    output: VhfRangingOutput,
}

impl VhfRangingBlock {
    /// A ranging set with no lock yet. A zero period or nonpositive
    /// resolution is an error.
    pub fn new(config: VhfRangingConfig) -> Result<Self, BlockError> {
        require_period(VHF_BLOCK, config.period)?;
        if !(config.resolution_m.is_finite() && config.resolution_m > 0.0) {
            return Err(BlockError::InputOutOfRange {
                block: VHF_BLOCK,
                input: "resolution_m",
                detail: format!("{} is not a positive resolution", config.resolution_m),
            });
        }
        Ok(Self {
            config,
            output: VhfRangingOutput {
                validity: RadarValidity::OutOfLimits,
                range_m: 0.0,
            },
        })
    }
}

impl Block for VhfRangingBlock {
    type Input = RelativeTrackInput;
    type Output = VhfRangingOutput;
    type Snapshot = VhfRangingOutput;

    fn period(&self) -> TickSpan {
        self.config.period
    }

    fn step(&mut self, input: &RelativeTrackInput) -> Result<(), BlockError> {
        input.validate(VHF_BLOCK)?;
        let config = &self.config;
        let true_range_m = input.line_of_sight().length();
        let measured = true_range_m + config.bias_m;
        self.output = VhfRangingOutput {
            validity: if true_range_m <= config.max_range_m {
                RadarValidity::Valid
            } else {
                RadarValidity::OutOfLimits
            },
            range_m: (measured / config.resolution_m).round() * config.resolution_m,
        };
        Ok(())
    }

    fn output(&self) -> VhfRangingOutput {
        self.output
    }

    fn snapshot(&self) -> VhfRangingOutput {
        self.output
    }

    fn restore(&mut self, snapshot: &VhfRangingOutput) {
        self.output = *snapshot;
    }
}
