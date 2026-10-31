//! The crossing from Simulate's frames into Indicate's.

use std::f64::consts::FRAC_PI_2;

use glam::{DMat3, DQuat, DVec3};
use indicate_instrument_state::Quat;
use sim_geo::{BodyEllipsoid, Geodetic, InertialTransform, LocalFrame};

#[cfg(test)]
mod tests;

/// Horizontal share of the nose axis below which heading has no meaning: the
/// nose within about 0.06° of the vertical.
const MIN_NOSE_HORIZONTAL: f64 = 1e-3;

/// How a vehicle's axes sit against the forward-right-down axes a display
/// assumes.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DisplayAxes {
    display_to_vehicle: DQuat,
}

impl DisplayAxes {
    /// A vehicle already on forward-right-down axes.
    pub const AEROSPACE: Self = Self {
        display_to_vehicle: DQuat::IDENTITY,
    };

    /// Axes related to the vehicle's by `display_to_vehicle`.
    pub const fn new(display_to_vehicle: DQuat) -> Self {
        Self { display_to_vehicle }
    }

    /// The Lunar Module, whose `+x` is up the thrust axis and `+z` forward out
    /// of the hatch. A crew flying it upright reads wings level with the
    /// thrust axis vertical, so forward is `+z` and down is `-x`.
    pub fn apollo_lm() -> Self {
        Self::new(DQuat::from_rotation_y(-FRAC_PI_2))
    }

    /// `vector` in vehicle axes, in display axes.
    pub fn vector_in_display(&self, vector: DVec3) -> DVec3 {
        self.display_to_vehicle.inverse() * vector
    }

    fn compose(&self, vehicle_to_ned: DQuat) -> DQuat {
        (vehicle_to_ned * self.display_to_vehicle).normalize()
    }
}

/// The local north-east-down frame at a position over `body`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LocalLevel {
    geodetic: Geodetic,
    frame: LocalFrame,
    fixed_to_ned: DQuat,
}

impl LocalLevel {
    /// The frame at `position_fixed_m`, body-fixed metres.
    pub fn at(body: &BodyEllipsoid, position_fixed_m: DVec3) -> Self {
        let geodetic = body.fixed_to_geodetic(position_fixed_m);
        let frame = LocalFrame::at(&geodetic);
        Self {
            geodetic,
            frame,
            fixed_to_ned: DQuat::from_mat3(&frame.ned_to_ecef()).inverse(),
        }
    }

    /// The position this frame stands at.
    pub fn geodetic(&self) -> Geodetic {
        self.geodetic
    }

    /// Body-fixed vector in north, east, down components.
    pub fn vector_in_ned(&self, fixed: DVec3) -> DVec3 {
        self.frame.ecef_to_ned_vector(fixed)
    }

    /// Display-to-NED attitude from a vehicle-to-body-fixed one.
    pub fn attitude_from_fixed(&self, vehicle_to_fixed: DQuat, axes: &DisplayAxes) -> DQuat {
        axes.compose(self.fixed_to_ned * vehicle_to_fixed)
    }

    /// Display-to-NED attitude from a vehicle-to-J2000 one, with `central_body`
    /// the orientation of the body at the same instant.
    pub fn attitude_from_inertial(
        &self,
        vehicle_to_inertial: DQuat,
        central_body: &InertialTransform,
        axes: &DisplayAxes,
    ) -> DQuat {
        let inertial_to_fixed = central_body.fixed_to_j2000_rotation().inverse();
        self.attitude_from_fixed(inertial_to_fixed * vehicle_to_inertial, axes)
    }
}

/// Bank and pitch of a display-to-NED attitude, radians.
pub fn bank_and_pitch_rad(display_to_ned: DQuat) -> (f64, f64) {
    let rotation = DMat3::from_quat(display_to_ned);
    (
        rotation.y_axis.z.atan2(rotation.z_axis.z),
        (-rotation.x_axis.z).clamp(-1.0, 1.0).asin(),
    )
}

/// True heading of a display-to-NED attitude on `[-π, π]`, or `None` with the
/// nose so near the vertical that it points nowhere.
pub fn true_heading_rad(display_to_ned: DQuat) -> Option<f64> {
    let nose = display_to_ned * DVec3::X;
    (nose.x.hypot(nose.y) >= MIN_NOSE_HORIZONTAL).then(|| nose.y.atan2(nose.x))
}

/// Heading rate from display-axis body rates, rad/s, positive right, or `None`
/// where heading itself is undefined.
pub fn heading_rate_rad_s(display_to_ned: DQuat, rate_display_rad_s: DVec3) -> Option<f64> {
    let (bank, pitch) = bank_and_pitch_rad(display_to_ned);
    let cos_pitch = pitch.cos();
    let (sin_bank, cos_bank) = bank.sin_cos();
    (cos_pitch >= MIN_NOSE_HORIZONTAL)
        .then(|| (rate_display_rad_s.y * sin_bank + rate_display_rad_s.z * cos_bank) / cos_pitch)
}

/// The `f32` quaternion Indicate reads.
pub fn indicate_quat(rotation: DQuat) -> Quat {
    Quat {
        w: rotation.w as f32,
        x: rotation.x as f32,
        y: rotation.y as f32,
        z: rotation.z as f32,
    }
}
