use sim_geo::UtcTime;

use super::dynamics::{Derivative, RigidState};
use crate::BlockError;

impl RigidState {
    /// This state moved along `rate` for `h` seconds, back on the unit
    /// quaternion sphere that the additive update leaves.
    fn advanced(&self, rate: &Derivative, h: f64) -> Self {
        Self {
            position_fixed_m: self.position_fixed_m + rate.velocity * h,
            velocity_fixed_m_s: self.velocity_fixed_m_s + rate.acceleration * h,
            attitude: (self.attitude + rate.attitude_rate * h).normalize(),
            omega_inertial_body: self.omega_inertial_body + rate.alpha * h,
        }
    }
}

/// Classic fourth-order Runge-Kutta over `dt_s` from `start` at `utc`.
pub(super) fn rk4(
    start: &RigidState,
    utc: UtcTime,
    dt_s: f64,
    rate_at: impl Fn(&RigidState, UtcTime) -> Result<Derivative, BlockError>,
) -> Result<RigidState, BlockError> {
    let half_s = dt_s / 2.0;
    let middle = utc.advanced_by(half_s);
    let k1 = rate_at(start, utc)?;
    let k2 = rate_at(&start.advanced(&k1, half_s), middle)?;
    let k3 = rate_at(&start.advanced(&k2, half_s), middle)?;
    let k4 = rate_at(&start.advanced(&k3, dt_s), utc.advanced_by(dt_s))?;
    let combined = Derivative {
        velocity: (k1.velocity + (k2.velocity + k3.velocity) * 2.0 + k4.velocity) / 6.0,
        acceleration: (k1.acceleration
            + (k2.acceleration + k3.acceleration) * 2.0
            + k4.acceleration)
            / 6.0,
        attitude_rate: (k1.attitude_rate
            + (k2.attitude_rate + k3.attitude_rate) * 2.0
            + k4.attitude_rate)
            * (1.0 / 6.0),
        alpha: (k1.alpha + (k2.alpha + k3.alpha) * 2.0 + k4.alpha) / 6.0,
    };
    Ok(start.advanced(&combined, dt_s))
}
