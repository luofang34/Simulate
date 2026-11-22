//! Handheld display resolution and pixel-ratio capping, plus window-to-surface
//! coordinate conversion when the rendering surface is smaller than raw Retina pixels.

use std::sync::Arc;

use sim_render::{RenderError, Renderer, VectorScenePass};
use winit::window::Window;

use super::FlightApp;
use crate::platform;

/// The window's renderer and the vector passes compiled on its device.
///
/// The passes are built here, asynchronously, because pipeline validation awaits a
/// browser promise on `WebGPU` and cannot block.
pub struct GpuReady {
    /// The window renderer.
    pub renderer: Renderer,
    /// Vector pass for the conformal HUD.
    pub hud_pass: VectorScenePass,
    /// Vector pass for the `Esc` menu.
    pub menu_pass: VectorScenePass,
}

impl GpuReady {
    /// Creates the renderer for `window` and compiles both vector passes on its device.
    pub async fn new(window: Arc<Window>, width: u32, height: u32) -> Result<Self, RenderError> {
        let renderer = Renderer::new(window, width, height).await?;
        let samples = if platform::is_handheld_browser() {
            1
        } else {
            sim_render::VECTOR_SAMPLE_COUNT
        };
        let hud_pass = VectorScenePass::with_sample_count(renderer.device(), samples).await?;
        let menu_pass = VectorScenePass::with_sample_count(renderer.device(), samples).await?;
        Ok(Self {
            renderer,
            hud_pass,
            menu_pass,
        })
    }
}

/// Maximum pixel ratio used on handheld browsers (`3×` Retina canvases still keep crisp `2×`
/// swapchain vector instruments without `3×` fill-rate cost).
pub const HANDHELD_MAX_PIXEL_RATIO: f64 = 2.0;

/// Maximum swapchain pixel budget on handheld browsers (`~2.07 MP`, `1920×1080`) for crisp
/// direct-to-swapchain PFD/HSI vector gauges and HUD.
pub const HANDHELD_MAX_PIXELS: u64 = 1_920 * 1_080;

/// Maximum offscreen 3D OTW pixel budget on handheld browsers (`~0.92 MP`, `1280×720`) so
/// WebGPU terrain/aircraft shading stays cool while overlay vectors remain sharp at `2×`.
pub const HANDHELD_OTW_MAX_PIXELS: u64 = 1_280 * 720;

/// Returns `(surface_px, effective_pixel_ratio)` for a window reporting `window_px`
/// physical pixels and `scale_factor`. Non-handheld devices keep native resolution.
#[must_use]
pub fn capped_surface_size(
    window_px: [u32; 2],
    scale_factor: f64,
    handheld: bool,
) -> ([u32; 2], f64) {
    let scale = if scale_factor.is_finite() && scale_factor > 0.0 {
        scale_factor
    } else {
        1.0
    };
    let clamped = [window_px[0].max(1), window_px[1].max(1)];
    if !handheld {
        return (clamped, scale.max(1.0));
    }
    let css_w = f64::from(clamped[0]) / scale;
    let css_h = f64::from(clamped[1]) / scale;
    let capped_ratio = scale.clamp(1.0, HANDHELD_MAX_PIXEL_RATIO);
    let mut w = (css_w * capped_ratio).round().max(1.0);
    let mut h = (css_h * capped_ratio).round().max(1.0);
    let max_px = HANDHELD_MAX_PIXELS as f64;
    let area = w * h;
    if area > max_px {
        let shrink = (max_px / area).sqrt();
        w = (w * shrink).floor().max(1.0);
        h = (h * shrink).floor().max(1.0);
    }
    let effective_ratio = (w / css_w.max(1.0)).clamp(1.0, capped_ratio);
    ([w as u32, h as u32], effective_ratio)
}

/// Returns the 3D OTW target size for `surface_px`, preserving aspect ratio and capping
/// handheld OTW area to [`HANDHELD_OTW_MAX_PIXELS`] while swapchain vectors stay full-res.
#[must_use]
pub fn capped_otw_size(surface_px: [u32; 2], handheld: bool) -> [u32; 2] {
    let clamped = [surface_px[0].max(1), surface_px[1].max(1)];
    if !handheld {
        return clamped;
    }
    let w = f64::from(clamped[0]);
    let h = f64::from(clamped[1]);
    let area = w * h;
    let max_px = HANDHELD_OTW_MAX_PIXELS as f64;
    if area <= max_px {
        return clamped;
    }
    let shrink = (max_px / area).sqrt();
    [
        (w * shrink).floor().max(1.0) as u32,
        (h * shrink).floor().max(1.0) as u32,
    ]
}

/// Converts a raw winit physical cursor/touch position inside `window_px` into
/// swapchain pixels inside `surface_px`.
#[must_use]
pub fn map_window_pos_to_surface(
    pos_px: [f64; 2],
    window_px: [u32; 2],
    surface_px: [u32; 2],
) -> [f64; 2] {
    let sx = f64::from(surface_px[0].max(1)) / f64::from(window_px[0].max(1));
    let sy = f64::from(surface_px[1].max(1)) / f64::from(window_px[1].max(1));
    [
        if pos_px[0].is_finite() {
            pos_px[0] * sx
        } else {
            0.0
        },
        if pos_px[1].is_finite() {
            pos_px[1] * sy
        } else {
            0.0
        },
    ]
}

impl FlightApp {
    /// Applies a capped swapchain resize from raw window physical pixels and syncs the
    /// cockpit rig's effective pixel ratio.
    pub(super) fn apply_window_resize(&mut self, width: u32, height: u32) {
        let scale = self
            .window
            .as_ref()
            .map_or(1.0, |window| window.scale_factor());
        let ([w, h], effective_ratio) = capped_surface_size([width, height], scale, self.handheld);
        if let Some(renderer) = self.renderer.as_mut() {
            renderer.resize(w, h);
        }
        if let Some(rig) = self.cockpit_rig.as_mut() {
            rig.set_pixel_ratio(effective_ratio);
        }
    }

    /// Recomputes swapchain size and effective pixel ratio using the window's current size
    /// and `scale_factor`.
    pub(super) fn sync_window_surface(&mut self) {
        let Some(size) = self.window.as_ref().map(|window| window.inner_size()) else {
            return;
        };
        self.apply_window_resize(size.width, size.height);
    }

    /// Maps a raw window physical position to swapchain coordinates.
    #[must_use]
    pub(super) fn window_pos_to_surface(&self, pos_px: [f64; 2]) -> [f64; 2] {
        let window_px = self.window.as_ref().map_or(self.surface_size(), |window| {
            let size = window.inner_size();
            [size.width, size.height]
        });
        map_window_pos_to_surface(pos_px, window_px, self.surface_size())
    }
}

#[cfg(test)]
mod tests {
    use super::{
        HANDHELD_MAX_PIXEL_RATIO, HANDHELD_MAX_PIXELS, HANDHELD_OTW_MAX_PIXELS, capped_otw_size,
        capped_surface_size, map_window_pos_to_surface,
    };

    #[test]
    fn desktop_keeps_native_resolution_and_scale() {
        let (px, ratio) = capped_surface_size([2560, 1440], 2.0, false);
        assert_eq!(px, [2560, 1440]);
        assert!((ratio - 2.0).abs() < 1e-12);
        assert_eq!(capped_otw_size([2560, 1440], false), [2560, 1440]);
    }

    #[test]
    fn iphone_retina_3x_caps_dpr_and_pixel_area_while_keeping_aspect() {
        let (px, ratio) = capped_surface_size([1290, 2796], 3.0, true);
        assert_eq!(px, [860, 1864]);
        assert!((ratio - HANDHELD_MAX_PIXEL_RATIO).abs() < 1e-12);

        let (landscape_px, _) = capped_surface_size([2796, 1290], 3.0, true);
        assert_eq!(landscape_px, [1864, 860]);

        let otw = capped_otw_size(landscape_px, true);
        assert!(u64::from(otw[0]) * u64::from(otw[1]) <= HANDHELD_OTW_MAX_PIXELS);
        let src_aspect = landscape_px[0] as f64 / landscape_px[1] as f64;
        let otw_aspect = otw[0] as f64 / otw[1] as f64;
        assert!((src_aspect - otw_aspect).abs() < 0.01);

        let (ipad_px, ipad_ratio) = capped_surface_size([2048, 2732], 2.0, true);
        assert!(u64::from(ipad_px[0]) * u64::from(ipad_px[1]) <= HANDHELD_MAX_PIXELS);
        assert!(ipad_ratio <= HANDHELD_MAX_PIXEL_RATIO);
    }

    #[test]
    fn window_pos_maps_proportionally_to_capped_surface() {
        let mapped = map_window_pos_to_surface([645.0, 1398.0], [1290, 2796], [645, 1398]);
        assert_eq!(mapped, [322.5, 699.0]);
        let invalid = map_window_pos_to_surface([f64::NAN, f64::INFINITY], [100, 100], [50, 50]);
        assert_eq!(invalid, [0.0, 0.0]);
    }
}
