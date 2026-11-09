use glam::{DVec3, Mat4};

use crate::MAX_PATCHES;

#[cfg(test)]
mod tests;

/// `view_proj` (16) + `sun_direction` and exposure (4) + `thermal` (4) +
/// `fog` (4).
const FRAME_HEADER_FLOATS: usize = 28;
const MATRIX_FLOATS: usize = 16;
const SUN_FLOATS: usize = 4;
const THERMAL_FLOATS: usize = 4;

/// The header plus one `vec4` per patch.
pub(crate) const FRAME_UNIFORM_FLOATS: usize = FRAME_HEADER_FLOATS + 4 * MAX_PATCHES;

/// Everything in the per-frame uniform block but the patch table.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct FrameParams {
    /// Camera-relative view-projection.
    pub(crate) view_proj: Mat4,
    /// Toward the Sun, body-fixed; need not be normalised.
    pub(crate) sun_direction: DVec3,
    /// Thermal display mapping: black exitance, white exitance (W/m²), NEDT
    /// (K), and noise seed.
    pub(crate) thermal: [f32; 4],
    /// Scale applied to every lit colour; `1.0` leaves it unchanged.
    pub(crate) exposure: f32,
    /// Fog colour (linear RGB, after exposure) and extinction per metre.
    pub(crate) fog: [f32; 4],
}

impl FrameParams {
    /// Params for drawing visible light under `environment`.
    pub(crate) fn visible(
        view_proj: Mat4,
        environment: &crate::EnvironmentFrame,
        thermal: [f32; 4],
    ) -> Self {
        let [r, g, b] = environment.fog_linear_rgb();
        Self {
            view_proj,
            sun_direction: environment.sun.direction_fixed,
            thermal,
            exposure: environment.exposure,
            fog: [r, g, b, environment.fog.extinction_per_m as f32],
        }
    }
}

/// Fills the per-frame uniform block in place, so drawing a frame never
/// allocates. Patch offsets are subtracted in `f64` before narrowing.
pub(crate) fn write_frame_uniform(
    out: &mut [f32; FRAME_UNIFORM_FLOATS],
    params: &FrameParams,
    eye: DVec3,
    patch_centres: &[DVec3],
) {
    out.fill(0.0);
    let (header, offsets) = out.split_at_mut(FRAME_HEADER_FLOATS);
    let (matrix, lighting) = header.split_at_mut(MATRIX_FLOATS);
    let (sun, rest) = lighting.split_at_mut(SUN_FLOATS);
    let (thermal, fog) = rest.split_at_mut(THERMAL_FLOATS);
    matrix.copy_from_slice(&params.view_proj.to_cols_array());
    let sun_direction = params.sun_direction.normalize_or_zero().as_vec3();
    sun.copy_from_slice(&[
        sun_direction.x,
        sun_direction.y,
        sun_direction.z,
        params.exposure,
    ]);
    thermal.copy_from_slice(&params.thermal);
    fog.copy_from_slice(&params.fog);
    for (slot, centre) in offsets.as_chunks_mut::<4>().0.iter_mut().zip(patch_centres) {
        let offset = (*centre - eye).as_vec3();
        slot.copy_from_slice(&[offset.x, offset.y, offset.z, 0.0]);
    }
}
