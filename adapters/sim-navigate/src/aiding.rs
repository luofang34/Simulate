use glam::DVec3;
use navigate_contract::{
    NedVelocity, SensorClass, SourceComposition, SourceEpoch, SourceId, SymmetricCov3,
    WrappingSequence,
};
use navigate_fusion::{Observation, ObservationValue};
use sim_core::SimInstant;
use sim_geo::{LocalFrame, ecef_to_geodetic};

use crate::{MasterClockBridge, NavigateBridgeError};

#[cfg(test)]
mod tests;

/// Turns a measured ECEF velocity into the velocity fix a navigation filter
/// takes from an inertial source. The filter has no specific-force model, so
/// velocity is the form inertial aiding reaches it in.
#[derive(Debug, Clone, PartialEq)]
pub struct KinematicAiding {
    clock: MasterClockBridge,
    source: SourceId,
    epoch: SourceEpoch,
    next: WrappingSequence,
    variance_m2_s2: f64,
}

impl KinematicAiding {
    /// An aiding stream whose velocity has `sigma_m_s` of error on each axis.
    pub const fn new(
        clock: MasterClockBridge,
        source: SourceId,
        epoch: SourceEpoch,
        sigma_m_s: f64,
    ) -> Self {
        Self {
            clock,
            source,
            epoch,
            next: WrappingSequence::new(0),
            variance_m2_s2: sigma_m_s * sigma_m_s,
        }
    }

    /// The observation of `velocity_ecef` measured at `at`, resolved into
    /// local north, east, down at `position_ecef`.
    pub fn observe(
        &mut self,
        at: SimInstant,
        position_ecef: DVec3,
        velocity_ecef: DVec3,
    ) -> Result<Observation, NavigateBridgeError> {
        let stamp = self.clock.stamp(self.source, self.epoch, self.next, at)?;
        self.next = self.next.next();
        let ned =
            LocalFrame::at(&ecef_to_geodetic(position_ecef)).ecef_to_ned_vector(velocity_ecef);
        let variance = self.variance_m2_s2;
        Ok(Observation::new(
            stamp,
            ObservationValue::VelocityFix {
                velocity: NedVelocity::new(ned.x, ned.y, ned.z),
                covariance: SymmetricCov3::from_diagonal(variance, variance, variance),
            },
            SourceComposition::of(SensorClass::Inertial),
        ))
    }
}
