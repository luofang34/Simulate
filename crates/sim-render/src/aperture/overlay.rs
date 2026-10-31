use std::f32::consts::TAU;

use crate::PinholeProjection;
use crate::aperture::stars::{STAR_SLOTS, StarSpot};

#[cfg(test)]
mod tests;

/// Floats in the overlay uniform: eight `vec4` then the star table.
pub(super) const OVERLAY_UNIFORM_FLOATS: usize = 4 * 8 + 4 * STAR_SLOTS;

const RETICLE_NONE: f32 = 0.0;
const RETICLE_SXT: f32 = 1.0;
const RETICLE_AOT: f32 = 2.0;
const RETICLE_LPD: f32 = 3.0;

/// Azimuth between the Alignment Optical Telescope's detents.
const AOT_DETENT_RAD: f32 = TAU / 6.0;

/// The etched pattern of a navigation optic.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum OpticalReticle {
    /// Sextant: crossed double lines on the boresight.
    Sxt,
    /// Alignment Optical Telescope: crosshair, six detent marks, and a
    /// reading spiral.
    Aot {
        /// Selected detent, `0..6`; turns the whole pattern.
        detent: u8,
        /// Reticle rotation from the reading knob, radians.
        reticle_rotation_rad: f32,
    },
    /// Landing Point Designator: the window's vertical scale, marked in
    /// degrees below the boresight.
    Lpd {
        /// Angle between scale marks, degrees.
        mark_spacing_deg: f32,
    },
}

impl OpticalReticle {
    /// Shader kind, rotation, and mark spacing in radians.
    fn uniform(reticle: Option<&Self>) -> [f32; 4] {
        match reticle {
            None => [RETICLE_NONE, 0.0, 0.0, 0.0],
            Some(Self::Sxt) => [RETICLE_SXT, 0.0, 0.0, 0.0],
            Some(Self::Aot {
                detent,
                reticle_rotation_rad,
            }) => {
                let rotation = f32::from(*detent) * AOT_DETENT_RAD + reticle_rotation_rad;
                [RETICLE_AOT, rotation, 0.0, 0.0]
            }
            Some(Self::Lpd { mark_spacing_deg }) => {
                [RETICLE_LPD, 0.0, mark_spacing_deg.to_radians(), 0.0]
            }
        }
    }
}

/// A window frame around the view: everything outside an ellipse centred on
/// the image is hidden.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct WindowBezel {
    /// Ellipse semi-axes, pixels.
    pub semi_axes_px: [f32; 2],
    /// Width of the darkening inside the frame's edge, pixels.
    pub vignette_px: f32,
    /// Frame colour, linear RGB.
    pub colour: [f32; 3],
}

impl WindowBezel {
    fn occludes(&self, width: u32, height: u32, x: f32, y: f32) -> bool {
        let [semi_x, semi_y] = self.semi_axes_px;
        let dx = (x - width as f32 / 2.0) / semi_x;
        let dy = (y - height as f32 / 2.0) / semi_y;
        dx.hypot(dy) >= 1.0
    }
}

/// A spinning propeller in front of the lens.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PropellerDisk {
    /// Hub position, pixels.
    pub hub_px: [f32; 2],
    /// Blade tip radius, pixels.
    pub tip_radius_px: f32,
    /// Hub radius, pixels; solid at any phase.
    pub hub_radius_px: f32,
    /// Blades, evenly spaced.
    pub blade_count: u32,
    /// Half the angle a blade subtends, radians.
    pub blade_half_width_rad: f32,
    /// Blade angle this frame, radians; advance it with the shaft.
    pub phase_rad: f32,
    /// Blade colour, linear RGB.
    pub colour: [f32; 3],
}

impl PropellerDisk {
    fn occludes(&self, x: f32, y: f32) -> bool {
        let [hub_x, hub_y] = self.hub_px;
        let (dx, dy) = (x - hub_x, y - hub_y);
        let radius = dx.hypot(dy);
        if radius >= self.tip_radius_px || self.blade_count == 0 {
            return radius < self.hub_radius_px;
        }
        if radius < self.hub_radius_px {
            return true;
        }
        let sector = TAU / self.blade_count as f32;
        let within = (dy.atan2(dx) - self.phase_rad).rem_euclid(sector);
        within.min(sector - within) <= self.blade_half_width_rad
    }
}

/// Synthetic obstructions drawn over a frame, to stress-test image masking.
/// The mask each one produces is known exactly, from
/// [`Self::ground_truth_mask`].
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct OcclusionOverlayConfig {
    /// A window frame, if any.
    pub bezel: Option<WindowBezel>,
    /// A propeller, if any.
    pub propeller: Option<PropellerDisk>,
}

impl OcclusionOverlayConfig {
    /// Whether an obstruction covers the point `(x, y)` in continuous image
    /// coordinates, whose origin is the corner of the first pixel.
    pub fn occludes(&self, width: u32, height: u32, x: f32, y: f32) -> bool {
        self.bezel
            .is_some_and(|bezel| bezel.occludes(width, height, x, y))
            || self.propeller.is_some_and(|disk| disk.occludes(x, y))
    }

    /// One entry per pixel, row-major: `true` where the pixel centre is
    /// obstructed.
    pub fn ground_truth_mask(&self, width: u32, height: u32) -> Vec<bool> {
        (0..height)
            .flat_map(|y| (0..width).map(move |x| (x, y)))
            .map(|(x, y)| self.occludes(width, height, x as f32 + 0.5, y as f32 + 0.5))
            .collect()
    }
}

/// Fills the overlay uniform block; the layout is `Overlay` in
/// `shaders/aperture.wgsl`.
pub(super) fn overlay_uniform(
    projection: &PinholeProjection,
    occlusion: &OcclusionOverlayConfig,
    reticle: Option<&OpticalReticle>,
    stars: &[StarSpot; STAR_SLOTS],
) -> [f32; OVERLAY_UNIFORM_FLOATS] {
    let intrinsics = projection.intrinsics();
    let [px, py] = projection.continuous_principal_point();
    let (bezel, bezel_colour) = occlusion.bezel.map_or(([0.0; 4], [0.0; 4]), |bezel| {
        let [semi_x, semi_y] = bezel.semi_axes_px;
        let [r, g, b] = bezel.colour;
        ([semi_x, semi_y, bezel.vignette_px, 1.0], [r, g, b, 0.0])
    });
    let (hub, blades, blade_colour) =
        occlusion
            .propeller
            .map_or(([0.0; 4], [0.0; 4], [0.0; 4]), |disk| {
                let [hub_x, hub_y] = disk.hub_px;
                let [r, g, b] = disk.colour;
                (
                    [hub_x, hub_y, disk.tip_radius_px, 1.0],
                    [
                        disk.blade_count as f32,
                        disk.blade_half_width_rad,
                        disk.phase_rad,
                        disk.hub_radius_px,
                    ],
                    [r, g, b, 0.0],
                )
            });
    let header: [[f32; 4]; 8] = [
        [intrinsics.width as f32, intrinsics.height as f32, 0.0, 0.0],
        [
            intrinsics.fx as f32,
            intrinsics.fy as f32,
            px as f32,
            py as f32,
        ],
        OpticalReticle::uniform(reticle),
        bezel,
        bezel_colour,
        hub,
        blades,
        blade_colour,
    ];
    let mut out = [0.0; OVERLAY_UNIFORM_FLOATS];
    let (head, tail) = out.split_at_mut(header.as_flattened().len());
    head.copy_from_slice(header.as_flattened());
    tail.copy_from_slice(stars.as_flattened());
    out
}
