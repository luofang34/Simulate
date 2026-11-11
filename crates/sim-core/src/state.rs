use glam::{DMat3, DQuat, DVec3};
use sim_geo::{BodyEllipsoid, Geodetic, LocalFrame, geodetic_to_ecef};

#[cfg(test)]
mod tests;

/// Rigid-body vehicle state in `f64` ECEF.
///
/// Body axes follow the aerospace convention: `x` forward, `y` right, `z`
/// down. `attitude` rotates body components into ECEF components.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct VehicleState {
    /// Centre-of-gravity position, ECEF metres.
    pub position_ecef: DVec3,
    /// Velocity relative to the rotating Earth, ECEF m/s.
    pub velocity_ecef: DVec3,
    /// Body-to-ECEF rotation.
    pub attitude: DQuat,
    /// Angular rate relative to the Earth, body axes, rad/s.
    pub angular_rate_body: DVec3,
}

impl VehicleState {
    /// Wings-level, level flight at `position`, heading `heading_rad` from true
    /// north, moving forward at `speed_m_s`.
    pub fn level_flight(position: &Geodetic, heading_rad: f64, speed_m_s: f64) -> Self {
        let frame = LocalFrame::at(position);
        let (sin_h, cos_h) = heading_rad.sin_cos();
        let forward = frame.north() * cos_h + frame.east() * sin_h;
        let right = frame.east() * cos_h - frame.north() * sin_h;
        let down = -frame.up();
        Self {
            position_ecef: geodetic_to_ecef(position),
            velocity_ecef: forward * speed_m_s,
            attitude: DQuat::from_mat3(&DMat3::from_cols(forward, right, down)).normalize(),
            angular_rate_body: DVec3::ZERO,
        }
    }

    /// The angular rate a gyroscope fixed to the body senses on `body`, body
    /// axes, rad/s: [`Self::angular_rate_body`] is relative to the turning
    /// body, and an inertial sensor adds the body's own spin.
    pub fn inertial_rate_body(&self, body: &BodyEllipsoid) -> DVec3 {
        self.angular_rate_body + self.attitude.inverse() * (DVec3::Z * body.omega_rad_s)
    }

    /// The body forward axis in ECEF.
    pub fn forward_ecef(&self) -> DVec3 {
        self.attitude * DVec3::X
    }

    /// The body down axis in ECEF.
    pub fn down_ecef(&self) -> DVec3 {
        self.attitude * DVec3::Z
    }
}
