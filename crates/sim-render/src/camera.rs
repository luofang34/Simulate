use glam::{DMat3, DMat4, DQuat, DVec3, Mat4};

#[cfg(test)]
mod tests;

/// Where the eye is and which way it looks, in `f64` ECEF.
///
/// `orientation` rotates camera-local axes into ECEF: the camera looks along
/// local `-Z` with local `+Y` up, the usual right-handed view convention.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CameraPose {
    /// Eye position, ECEF metres.
    pub position_ecef: DVec3,
    /// Camera-local to ECEF rotation.
    pub orientation: DQuat,
}

impl CameraPose {
    /// A pose at `position_ecef` looking along `forward` with `up` as the
    /// approximate up direction. Both are ECEF directions and need not be
    /// normalised; `up` must not be parallel to `forward`.
    pub fn looking(position_ecef: DVec3, forward: DVec3, up: DVec3) -> Self {
        let back = -forward.normalize();
        let right = up.cross(back).normalize();
        let true_up = back.cross(right);
        Self {
            position_ecef,
            orientation: DQuat::from_mat3(&DMat3::from_cols(right, true_up, back)),
        }
    }

    /// The pose a fraction `t` of the way to `end`, moving at constant
    /// velocity and turning at constant angular rate.
    pub fn interpolated(&self, end: &Self, t: f64) -> Self {
        Self {
            position_ecef: self.position_ecef.lerp(end.position_ecef, t),
            orientation: self.orientation.slerp(end.orientation, t).normalize(),
        }
    }

    /// The poses at the middles of `count` equal time slices of a move to
    /// `end`, for rendering sub-frames of one exposure.
    pub fn exposure_sub_poses(&self, end: &Self, count: usize) -> Vec<Self> {
        (0..count)
            .map(|slice| self.interpolated(end, (slice as f64 + 0.5) / count as f64))
            .collect()
    }
}

/// A perspective camera with reversed-Z, infinite-far projection.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Camera {
    /// Eye pose.
    pub pose: CameraPose,
    /// Vertical field of view, radians.
    pub fov_y_rad: f64,
    /// Near plane distance, metres. Reversed-Z keeps depth precision nearly
    /// uniform in log space, so a few centimetres is fine even when the
    /// horizon is thousands of kilometres away.
    pub near_m: f64,
}

impl Camera {
    /// The camera-relative view-projection: translation is omitted because
    /// all geometry reaches the shader already relative to the eye.
    pub fn view_projection(&self, aspect: f64) -> Mat4 {
        let view = DMat4::from_quat(self.pose.orientation.inverse());
        let projection = glam::dcamera::rh::proj::directx::perspective_infinite_reverse(
            self.fov_y_rad,
            aspect,
            self.near_m,
        );
        (projection * view).as_mat4()
    }
}
