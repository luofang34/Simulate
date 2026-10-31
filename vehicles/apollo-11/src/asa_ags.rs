use glam::{DQuat, DVec3};
use sim_core::{Block, BlockError, TickSpan};

use crate::InertialSiteSample;
use crate::site::{require_period, rotation_over};

#[cfg(test)]
mod tests;

const BLOCK: &str = "strapdown_asa";

/// Gyro drift of 0.1°/h: an order of magnitude worse than the PGNCS platform.
const NOMINAL_GYRO_DRIFT_RAD_S: f64 = 0.1 * std::f64::consts::PI / 180.0 / 3_600.0;

/// Setup of a [`StrapdownAsaBlock`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct StrapdownAsaConfig {
    /// Sample period.
    pub period: TickSpan,
    /// Site-to-J2000 rotation at the start.
    pub initial_site_to_inertial: DQuat,
    /// Gyro bias, site axes, rad/s.
    pub gyro_bias_rad_s: DVec3,
    /// Accelerometer bias, site axes, m/s².
    pub accel_bias_m_s2: DVec3,
}

impl StrapdownAsaConfig {
    /// A representative unit initialised at `site_to_inertial`.
    pub fn nominal(period: TickSpan, site_to_inertial: DQuat) -> Self {
        Self {
            period,
            initial_site_to_inertial: site_to_inertial,
            gyro_bias_rad_s: DVec3::new(1.0, -0.7, 0.5) * NOMINAL_GYRO_DRIFT_RAD_S,
            accel_bias_m_s2: DVec3::ZERO,
        }
    }
}

/// What a [`StrapdownAsaBlock`] publishes, and all of its state.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AsaOutput {
    /// Integrated site-to-J2000 attitude.
    pub site_to_inertial: DQuat,
    /// Integrated velocity change in J2000 axes, m/s.
    pub delta_velocity_inertial_m_s: DVec3,
}

/// The LM Abort Guidance System's sensor assembly: three gyros and three
/// accelerometers bolted to the navigation base.
///
/// With no gimbals there is nothing to lock, so attitude stays available at
/// any orientation; that is why the abort system could take over from a
/// captured [`crate::ThreeGimbalImuBlock`]. The price is drift above the
/// platform's and a software integration of the body rate each sample.
#[derive(Debug, Clone, PartialEq)]
pub struct StrapdownAsaBlock {
    config: StrapdownAsaConfig,
    output: AsaOutput,
}

impl StrapdownAsaBlock {
    /// A unit at its initial attitude. A zero period is an error.
    pub fn new(config: StrapdownAsaConfig) -> Result<Self, BlockError> {
        require_period(BLOCK, config.period)?;
        Ok(Self {
            output: AsaOutput {
                site_to_inertial: config.initial_site_to_inertial,
                delta_velocity_inertial_m_s: DVec3::ZERO,
            },
            config,
        })
    }
}

impl Block for StrapdownAsaBlock {
    type Input = InertialSiteSample;
    type Output = AsaOutput;
    type Snapshot = AsaOutput;

    fn period(&self) -> TickSpan {
        self.config.period
    }

    fn step(&mut self, input: &InertialSiteSample) -> Result<(), BlockError> {
        input.validate(BLOCK)?;
        let dt_s = self.config.period.as_seconds();
        let rate = input.angular_rate_site_rad_s + self.config.gyro_bias_rad_s;
        let attitude = (self.output.site_to_inertial * rotation_over(rate, dt_s)).normalize();
        let force = input.specific_force_site_m_s2 + self.config.accel_bias_m_s2;
        self.output = AsaOutput {
            site_to_inertial: attitude,
            delta_velocity_inertial_m_s: self.output.delta_velocity_inertial_m_s
                + attitude * force * dt_s,
        };
        Ok(())
    }

    fn output(&self) -> AsaOutput {
        self.output
    }

    fn snapshot(&self) -> AsaOutput {
        self.output
    }

    fn restore(&mut self, snapshot: &AsaOutput) {
        self.output = *snapshot;
    }
}
