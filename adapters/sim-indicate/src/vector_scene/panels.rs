//! GPU-vector primary flight display (PFD) and horizontal situation indicator (HSI)
//! tessellation at physical display resolution.

use indicate_alerts::AlertOutput;
use indicate_instrument_panels::{BUILTIN_FRAME, PANEL_H, PANEL_W, PfdConfig, draw_hsi, draw_pfd};
use indicate_instrument_scene::{Rgba8, SceneError, SceneWriter};
use indicate_instrument_state::PanelData;
use sim_render::VectorVertex;

use super::{Mesh, shapes, tessellate_mesh};
use crate::{PANEL_SCENE_BYTES, PanelError};

/// Compass-rose centre of Indicate's `480 × 360` HSI design frame.
const HSI_CENTRE_DESIGN: [f32; 2] = [240.0, 190.0];
/// Aircraft reference yellow (`[255, 210, 0, 255]`).
const OWNSHIP_YELLOW: Rgba8 = Rgba8::rgba(255, 210, 0, 255);
/// Dark contrast outline (`[12, 16, 22, 240]`).
const OWNSHIP_OUTLINE: Rgba8 = Rgba8::rgba(12, 16, 22, 240);
/// Outline width added around each stroke in design units.
const OUTLINE_EXTRA: f32 = 1.5;

/// Nose-up silhouette segments relative to [`HSI_CENTRE_DESIGN`]: `(a, b, half_width)`.
const OWNSHIP_STROKES: [([f32; 2], [f32; 2], f32); 3] = [
    ([0.0, -10.0], [0.0, 11.0], 1.7),
    ([-11.0, -1.0], [11.0, -1.0], 1.6),
    ([-5.0, 9.0], [5.0, 9.0], 1.4),
];

/// Reusable scene-encoding buffer that tessellates Indicate's PFD and HSI directly into
/// physical-pixel [`VectorVertex`] lists for `4×` MSAA GPU rendering, bypassing CPU scanline
/// rasterization and pixel staging uploads.
pub struct VectorFlightPanels {
    scene: Vec<u8>,
}

impl Default for VectorFlightPanels {
    fn default() -> Self {
        Self::new()
    }
}

impl VectorFlightPanels {
    /// Allocates a panel encoder with a reusable scene buffer sized for any valid panel scene.
    #[must_use]
    pub fn new() -> Self {
        Self {
            scene: vec![0_u8; PANEL_SCENE_BYTES],
        }
    }

    /// Encodes and tessellates the PFD (`BackgroundMode::Svs` or `BackgroundMode::Horizon`)
    /// into triangles covering a `target_w × target_h` physical [`sim_render::VectorTarget`].
    pub fn tessellate_pfd(
        &mut self,
        data: &PanelData,
        config: &PfdConfig,
        alerts: Option<&AlertOutput>,
        target_w: u32,
        target_h: u32,
    ) -> Result<Vec<VectorVertex>, PanelError> {
        let mesh = self.encode_and_tessellate("PFD", [target_w, target_h], |writer| {
            draw_pfd(data, config, alerts, BUILTIN_FRAME, writer)
        })?;
        Ok(mesh.into_vertices())
    }

    /// Encodes and tessellates the HSI (plus its yellow/dark-outlined ownship reference symbol
    /// when heading is valid) into triangles covering a `target_w × target_h` target.
    pub fn tessellate_hsi(
        &mut self,
        data: &PanelData,
        alerts: Option<&AlertOutput>,
        target_w: u32,
        target_h: u32,
    ) -> Result<Vec<VectorVertex>, PanelError> {
        let mut mesh = self.encode_and_tessellate("HSI", [target_w, target_h], |writer| {
            draw_hsi(data, alerts, BUILTIN_FRAME, writer)
        })?;
        if data.heading.value_rad.status.shows_value() {
            append_hsi_ownship(&mut mesh, [target_w as f32, target_h as f32]);
        }
        Ok(mesh.into_vertices())
    }

    fn encode_and_tessellate(
        &mut self,
        panel: &'static str,
        target: [u32; 2],
        draw: impl FnOnce(&mut SceneWriter<'_>) -> Result<(), SceneError>,
    ) -> Result<Mesh, PanelError> {
        let capacity = self.scene.len();
        let mut writer = SceneWriter::new(&mut self.scene).map_err(|cause| PanelError::Scene {
            panel,
            capacity,
            cause,
        })?;
        draw(&mut writer).map_err(|cause| PanelError::Scene {
            panel,
            capacity,
            cause,
        })?;
        let scene_bytes = writer.finish();
        let scene = self.scene.get(..scene_bytes).unwrap_or_default();
        tessellate_mesh(scene, [PANEL_W, PANEL_H], target)
            .map_err(|source| PanelError::VectorScene { source })
    }
}

fn append_hsi_ownship(mesh: &mut Mesh, [tw, th]: [f32; 2]) {
    let clip = [0.0, 0.0, tw, th];
    let [cx, cy] = HSI_CENTRE_DESIGN;
    if mesh.begin(OWNSHIP_OUTLINE, clip) {
        for ([ax, ay], [bx, by], half_w) in OWNSHIP_STROKES {
            shapes::stroke_path(
                mesh,
                &[[cx + ax, cy + ay], [cx + bx, cy + by]],
                false,
                half_w + OUTLINE_EXTRA,
            );
        }
    }
    if mesh.begin(OWNSHIP_YELLOW, clip) {
        for ([ax, ay], [bx, by], half_w) in OWNSHIP_STROKES {
            shapes::stroke_path(
                mesh,
                &[[cx + ax, cy + ay], [cx + bx, cy + by]],
                false,
                half_w,
            );
        }
    }
}
