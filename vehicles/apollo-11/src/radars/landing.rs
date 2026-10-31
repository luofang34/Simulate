use std::f64::consts::{PI, TAU};

use glam::{DMat3, DQuat, DVec3};
use mbd_core::LocalSiteKinematics;
use sim_core::{Block, BlockError, TickSpan};

use super::{RadarValidity, require_fixed_site};
use crate::SurfacePlane;
use crate::site::require_period;

const BLOCK: &str = "landing_radar";

/// Highest slant range the radar locks on at, metres.
const MAX_SLANT_RANGE_M: f64 = 12_000.0;
/// Largest beam incidence on the surface that returns an echo, rad: 60°.
const MAX_INCIDENCE_RAD: f64 = PI / 3.0;
/// Boresight tilt from `-x` towards `+z` in the descent position, rad: 24°.
const DESCENT_TILT_RAD: f64 = 24.0 * PI / 180.0;
/// Boresight tilt from `-x` towards `+z` in the hover position, rad: 6°.
const HOVER_TILT_RAD: f64 = 6.0 * PI / 180.0;
/// Velocity beam splay from the boresight, rad: 20°.
const VELOCITY_BEAM_SPLAY_RAD: f64 = 20.0 * PI / 180.0;

/// The two positions of the tilting antenna.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LandingRadarAntenna {
    /// Position 1, tilted forward for the pitched-back braking phase.
    Descent,
    /// Position 2, close to the thrust axis for the near-vertical hover.
    Hover,
}

impl LandingRadarAntenna {
    fn boresight_site(self) -> DVec3 {
        let tilt = match self {
            Self::Descent => DESCENT_TILT_RAD,
            Self::Hover => HOVER_TILT_RAD,
        };
        DQuat::from_rotation_y(tilt) * DVec3::NEG_X
    }
}

/// Setup of a [`LandingRadarBlock`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LandingRadarConfig {
    /// Sample period.
    pub period: TickSpan,
    /// Highest usable slant range, metres.
    pub max_slant_range_m: f64,
    /// Largest usable beam incidence on the surface, rad.
    pub max_incidence_rad: f64,
}

impl LandingRadarConfig {
    /// The flown limits, sampled every `period`.
    pub fn nominal(period: TickSpan) -> Self {
        Self {
            period,
            max_slant_range_m: MAX_SLANT_RANGE_M,
            max_incidence_rad: MAX_INCIDENCE_RAD,
        }
    }
}

/// What a [`LandingRadarBlock`] reads each step.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LandingRadarInput {
    /// The antenna site, body-fixed; see [`crate::site_in_fixed_frame`].
    pub site: LocalSiteKinematics,
    /// The terrain beneath the vehicle.
    pub surface: SurfacePlane,
    /// Antenna position.
    pub antenna: LandingRadarAntenna,
}

/// What a [`LandingRadarBlock`] publishes.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LandingRadarOutput {
    /// Whether the data can be used.
    pub validity: RadarValidity,
    /// Range along the range beam, metres.
    pub slant_range_m: f64,
    /// Antenna speed towards the surface along each velocity beam, m/s.
    pub beam_doppler_m_s: [f64; 3],
    /// Antenna velocity over the surface, site axes, m/s, solved from the
    /// three beams.
    pub velocity_site_m_s: DVec3,
}

/// The LM's four-beam landing radar: one range beam on the boresight and three
/// Doppler velocity beams splayed about it, on an antenna with a descent and a
/// hover position.
///
/// Each beam is ray-cast onto the local [`SurfacePlane`]. The radar goes
/// [`RadarValidity::OutOfLimits`] above its range limit, or when any beam
/// misses or strikes the surface too obliquely, as at excessive tilt.
#[derive(Debug, Clone, PartialEq)]
pub struct LandingRadarBlock {
    config: LandingRadarConfig,
    output: LandingRadarOutput,
}

impl LandingRadarBlock {
    /// A radar with no lock yet. A zero period is an error.
    pub fn new(config: LandingRadarConfig) -> Result<Self, BlockError> {
        require_period(BLOCK, config.period)?;
        Ok(Self {
            config,
            output: LandingRadarOutput {
                validity: RadarValidity::OutOfLimits,
                slant_range_m: 0.0,
                beam_doppler_m_s: [0.0; 3],
                velocity_site_m_s: DVec3::ZERO,
            },
        })
    }
}

/// The three velocity beams, site axes.
fn velocity_beams(boresight: DVec3) -> [DVec3; 3] {
    let perpendicular = boresight.any_orthonormal_vector();
    [0.0, 1.0, 2.0].map(|index: f64| {
        let spoke = DQuat::from_axis_angle(boresight, index * TAU / 3.0) * perpendicular;
        DQuat::from_axis_angle(spoke, VELOCITY_BEAM_SPLAY_RAD) * boresight
    })
}

impl Block for LandingRadarBlock {
    type Input = LandingRadarInput;
    type Output = LandingRadarOutput;
    type Snapshot = LandingRadarOutput;

    fn period(&self) -> TickSpan {
        self.config.period
    }

    fn step(&mut self, input: &LandingRadarInput) -> Result<(), BlockError> {
        require_fixed_site(BLOCK, &input.site)?;
        input.surface.validate(BLOCK)?;
        let site_to_fixed = input.site.attitude;
        let normal_site = site_to_fixed.inverse() * input.surface.normal_fixed;
        let clearance_m = input.surface.clearance_m(input.site.position_m);
        let velocity_site = site_to_fixed.inverse() * input.site.velocity_m_s;

        let boresight = input.antenna.boresight_site();
        let beams = velocity_beams(boresight);
        let min_cosine = self.config.max_incidence_rad.cos();
        let in_limits = |beam: DVec3| -beam.dot(normal_site) >= min_cosine;
        let slant_range_m = clearance_m / -boresight.dot(normal_site);
        let beam_doppler_m_s = beams.map(|beam| beam.dot(velocity_site));

        let [first, second, third] = beams;
        let geometry = DMat3::from_cols(first, second, third).transpose();
        let valid = in_limits(boresight)
            && beams.into_iter().all(in_limits)
            && clearance_m > 0.0
            && slant_range_m <= self.config.max_slant_range_m;
        self.output = LandingRadarOutput {
            validity: if valid {
                RadarValidity::Valid
            } else {
                RadarValidity::OutOfLimits
            },
            slant_range_m,
            beam_doppler_m_s,
            velocity_site_m_s: geometry.inverse() * DVec3::from_array(beam_doppler_m_s),
        };
        Ok(())
    }

    fn output(&self) -> LandingRadarOutput {
        self.output
    }

    fn snapshot(&self) -> LandingRadarOutput {
        self.output
    }

    fn restore(&mut self, snapshot: &LandingRadarOutput) {
        self.output = *snapshot;
    }
}
