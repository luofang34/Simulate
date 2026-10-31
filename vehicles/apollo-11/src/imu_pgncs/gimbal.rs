use std::f64::consts::TAU;

use glam::{DMat3, DQuat, DVec3};

use super::CDU_COUNTS_PER_TURN;

/// The three gimbal angles between the stable member and the navigation base.
///
/// The gimbal order runs stable member, inner (about `y`), middle (about `z`),
/// outer (about `x`), navigation base, so the base's orientation in stable
/// axes is `Ry(iga) Rz(mga) Rx(oga)`. At `mga = ±90°` the outer and inner axes
/// coincide: that is gimbal lock. The middle angle is reported on `±90°`, so
/// a vehicle turning past 90° reads as falling back from it.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct GimbalAngles {
    /// Outer gimbal angle, rad.
    pub oga_rad: f64,
    /// Middle gimbal angle, rad.
    pub mga_rad: f64,
    /// Inner gimbal angle, rad.
    pub iga_rad: f64,
}

impl GimbalAngles {
    /// Solves the gimbals for `navbase_to_platform`.
    pub fn of_navbase_to_platform(navbase_to_platform: DQuat) -> Self {
        let rotation = DMat3::from_quat(navbase_to_platform);
        let DVec3 {
            x: x_x,
            y: x_y,
            z: x_z,
        } = rotation.x_axis;
        let y_y = rotation.y_axis.y;
        let z_y = rotation.z_axis.y;
        Self {
            oga_rad: (-z_y).atan2(y_y),
            mga_rad: x_y.clamp(-1.0, 1.0).asin(),
            iga_rad: (-x_z).atan2(x_x),
        }
    }

    /// The rotation these angles describe, the inverse of
    /// [`Self::of_navbase_to_platform`].
    pub fn navbase_to_platform(&self) -> DQuat {
        DQuat::from_rotation_y(self.iga_rad)
            * DQuat::from_rotation_z(self.mga_rad)
            * DQuat::from_rotation_x(self.oga_rad)
    }

    /// 15-bit shaft encodings: outer, inner, middle.
    pub(super) fn cdu_counts(&self) -> [u16; 3] {
        [self.oga_rad, self.iga_rad, self.mga_rad].map(|angle| {
            let counts = (angle / TAU * f64::from(CDU_COUNTS_PER_TURN)).round() as i32;
            counts.rem_euclid(CDU_COUNTS_PER_TURN) as u16
        })
    }
}
