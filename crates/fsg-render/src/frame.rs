use glam::DVec3;

use crate::{Camera, MAX_PATCHES};

/// `view_proj` (16) + `sun_direction` (4) + one `vec4` per patch.
pub(crate) const FRAME_UNIFORM_FLOATS: usize = 16 + 4 + 4 * MAX_PATCHES;

/// Fills the per-frame uniform block in place, so drawing a frame never
/// allocates. Patch offsets are subtracted in `f64` before narrowing.
pub(crate) fn write_frame_uniform(
    out: &mut [f32; FRAME_UNIFORM_FLOATS],
    camera: &Camera,
    aspect: f64,
    sun_direction_ecef: DVec3,
    patch_centres: &[DVec3],
) {
    out.fill(0.0);
    let (header, offsets) = out.split_at_mut(20);
    let (matrix, sun) = header.split_at_mut(16);
    matrix.copy_from_slice(&camera.view_projection(aspect).to_cols_array());
    let sun_direction = sun_direction_ecef.normalize_or_zero().as_vec3();
    sun.copy_from_slice(&[sun_direction.x, sun_direction.y, sun_direction.z, 0.0]);
    for (slot, centre) in offsets.as_chunks_mut::<4>().0.iter_mut().zip(patch_centres) {
        let offset = (*centre - camera.pose.position_ecef).as_vec3();
        slot.copy_from_slice(&[offset.x, offset.y, offset.z, 0.0]);
    }
}
