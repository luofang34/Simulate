use glam::{DMat4, DVec3, DVec4, Mat4};
use sim_geo::SensorIntrinsics;

use crate::{CameraPose, RenderError};

#[cfg(test)]
mod tests;

/// Distance along the optical axis of a reversed-Z depth value, metres.
/// Depth zero is the far plane at infinity, so it has no range. Off-axis
/// pixels give the distance along the axis, not along their ray.
pub fn reversed_z_range_m(depth_ndc: f32, near_m: f64) -> Option<f64> {
    (depth_ndc > 0.0).then(|| near_m / f64::from(depth_ndc))
}

/// A reversed-Z, infinite-far projection for a camera's calibrated optics.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PinholeProjection {
    intrinsics: SensorIntrinsics,
    near_m: f64,
}

impl PinholeProjection {
    /// A projection for `intrinsics`. Fails on unusable optics or a near
    /// plane that is not a positive distance.
    pub fn new(intrinsics: SensorIntrinsics, near_m: f64) -> Result<Self, RenderError> {
        if !intrinsics.is_usable() {
            return Err(RenderError::InvalidCamera {
                detail: format!("{intrinsics:?}"),
            });
        }
        if !(near_m.is_finite() && near_m > 0.0) {
            return Err(RenderError::InvalidCamera {
                detail: format!("near plane {near_m} m"),
            });
        }
        Ok(Self { intrinsics, near_m })
    }

    /// The camera's optics.
    pub fn intrinsics(&self) -> SensorIntrinsics {
        self.intrinsics
    }

    /// Near plane distance, metres.
    pub fn near_m(&self) -> f64 {
        self.near_m
    }

    /// The principal point in continuous image coordinates, whose origin is
    /// the corner of the first pixel. These are half a pixel from the
    /// intrinsics', which put pixel centres on integers.
    pub(super) fn continuous_principal_point(&self) -> [f64; 2] {
        [self.intrinsics.cx + 0.5, self.intrinsics.cy + 0.5]
    }

    /// The projection from renderer camera axes (`+X` right, `+Y` up, `-Z`
    /// forward) to wgpu clip space: `+Y` up, depth `near / distance`.
    pub fn matrix(&self) -> DMat4 {
        let [px, py] = self.continuous_principal_point();
        let width = f64::from(self.intrinsics.width);
        let height = f64::from(self.intrinsics.height);
        DMat4::from_cols(
            DVec4::new(2.0 * self.intrinsics.fx / width, 0.0, 0.0, 0.0),
            DVec4::new(0.0, 2.0 * self.intrinsics.fy / height, 0.0, 0.0),
            DVec4::new(1.0 - 2.0 * px / width, 2.0 * py / height - 1.0, 0.0, -1.0),
            DVec4::new(0.0, 0.0, self.near_m, 0.0),
        )
    }

    /// The camera-relative view-projection: translation is omitted because
    /// all geometry reaches the shader already relative to the eye.
    pub fn view_projection(&self, pose: &CameraPose) -> Mat4 {
        (self.matrix() * DMat4::from_quat(pose.orientation.inverse())).as_mat4()
    }

    /// Where a direction in camera axes lands, in the intrinsics' pixel
    /// coordinates. `None` at or behind the image plane.
    pub fn pixel_of(&self, camera_direction: DVec3) -> Option<[f64; 2]> {
        let forward = -camera_direction.z;
        (forward > 0.0).then(|| {
            [
                self.intrinsics.fx * camera_direction.x / forward + self.intrinsics.cx,
                self.intrinsics.fy * -camera_direction.y / forward + self.intrinsics.cy,
            ]
        })
    }

    /// Distance along the optical axis of a depth-buffer value, metres.
    pub fn range_m(&self, depth_ndc: f32) -> Option<f64> {
        reversed_z_range_m(depth_ndc, self.near_m)
    }
}
