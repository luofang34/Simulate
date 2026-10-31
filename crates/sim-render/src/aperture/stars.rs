use sim_geo::{APOLLO_NAV_STARS, InertialTransform};

use crate::{CameraPose, PinholeProjection};

#[cfg(test)]
mod tests;

/// Star slots in the overlay shader's uniform.
pub(super) const STAR_SLOTS: usize = APOLLO_NAV_STARS.len();
/// Spread of a star's image, pixels: the optic's diffraction and focus blur.
const PSF_SIGMA_PX: f32 = 1.3;

/// One star for the shader: continuous pixel `x`, `y`, peak linear intensity,
/// and spread. A zero peak is a star out of view.
pub(super) type StarSpot = [f32; 4];

/// Peak intensity of a star, with magnitude 0 at full scale.
fn peak_intensity(visual_magnitude: f32) -> f32 {
    10.0_f32.powf(-0.4 * visual_magnitude).min(1.0)
}

/// Projects the Apollo navigation stars into the image. Stars are at
/// infinity, so only the camera's orientation matters.
pub(super) fn star_spots(
    projection: &PinholeProjection,
    pose: &CameraPose,
    inertial: &InertialTransform,
) -> [StarSpot; STAR_SLOTS] {
    let [offset_x, offset_y] = {
        let [px, py] = projection.continuous_principal_point();
        let intrinsics = projection.intrinsics();
        [px - intrinsics.cx, py - intrinsics.cy]
    };
    let to_camera = pose.orientation.inverse();
    APOLLO_NAV_STARS.each_ref().map(|star| {
        let fixed = inertial.j2000_to_fixed(star.direction_j2000);
        match projection.pixel_of(to_camera * fixed) {
            Some([x, y]) => [
                (x + offset_x) as f32,
                (y + offset_y) as f32,
                peak_intensity(star.visual_magnitude),
                PSF_SIGMA_PX,
            ],
            None => [0.0; 4],
        }
    })
}
