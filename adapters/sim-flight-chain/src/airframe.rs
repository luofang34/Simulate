//! What a flight controller's board is bolted to: a vehicle whose dynamics
//! answer the board's actuator command, in air that its sensors also sit in.

use std::sync::Arc;

use glam::DVec3;
use sim_atmosphere::{AtmosphereError, AtmosphereField, AtmosphereSample};
use sim_aviate::DirectActuatorCmd;
use sim_core::{TickSpan, VehicleState};
use sim_devices::SensedTruth;
use sim_geo::{BodyEllipsoid, Geodetic, UtcTime};

use crate::ChainError;

/// Rotation rate of the Earth about its axis, rad/s.
const EARTH_RATE_RAD_S: f64 = 7.292_115e-5;

/// A vehicle's dynamics under a flight controller's actuator command.
pub trait Airframe {
    /// The integration period, which is the flight controller's cycle.
    fn period(&self) -> TickSpan;

    /// The true state.
    fn state(&self) -> VehicleState;

    /// What the vehicle's motion and the air present to its sensors now.
    fn sensed(&self) -> Result<SensedTruth, ChainError>;

    /// One period under `command`.
    fn fly(&mut self, command: &DirectActuatorCmd) -> Result<(), ChainError>;
}

/// One air field shared by a vehicle's dynamics and its air-data devices,
/// so both meet the same air and the same wind.
#[derive(Clone)]
pub struct SharedAir(Arc<dyn AtmosphereField + Send + Sync>);

impl SharedAir {
    /// `field` behind a handle its users share.
    pub fn new(field: impl AtmosphereField + Send + Sync + 'static) -> Self {
        Self(Arc::new(field))
    }
}

impl AtmosphereField for SharedAir {
    fn sample(
        &self,
        position: &Geodetic,
        utc: UtcTime,
    ) -> Result<AtmosphereSample, AtmosphereError> {
        self.0.sample(position, utc)
    }
}

/// Specific force in body axes of a vehicle whose Earth-fixed velocity went
/// from `before` to `after` over `dt_s`: its acceleration in inertial space,
/// which adds the rotating frame's Coriolis and centrifugal terms, less the
/// mass attraction.
pub(crate) fn specific_force_body_m_s2(
    before: &VehicleState,
    after: &VehicleState,
    dt_s: f64,
) -> DVec3 {
    let earth_rate = DVec3::new(0.0, 0.0, EARTH_RATE_RAD_S);
    let acceleration = (after.velocity_ecef - before.velocity_ecef) / dt_s
        + 2.0 * earth_rate.cross(after.velocity_ecef)
        + earth_rate.cross(earth_rate.cross(after.position_ecef));
    let attraction = BodyEllipsoid::WGS84.gravity_fixed(after.position_ecef);
    after.attitude.inverse() * (acceleration - attraction)
}
