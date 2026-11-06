//! GPU-vector primary flight display (PFD) and horizontal situation indicator (HSI)
//! tessellation at physical display resolution.

use indicate_alerts::AlertOutput;
use indicate_instrument_panels::{BUILTIN_FRAME, PANEL_H, PANEL_W, PfdConfig, draw_hsi, draw_pfd};
use indicate_instrument_scene::{Rgba8, SceneError, SceneWriter};
use indicate_instrument_state::PanelData;
use sim_render::VectorVertex;

use super::{Mesh, shapes, tessellate_mesh_into};
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

/// Reusable scene-encoding and vertex-tessellation buffers that turn Indicate's PFD and HSI
/// directly into physical-pixel [`VectorVertex`] lists for `4×` MSAA GPU rendering, bypassing
/// CPU scanline rasterization and per-frame vertex allocations.
pub struct VectorFlightPanels {
    scene: Vec<u8>,
    vertices: Vec<VectorVertex>,
}

impl Default for VectorFlightPanels {
    fn default() -> Self {
        Self::new()
    }
}

impl VectorFlightPanels {
    /// Allocates a panel encoder with reusable scene and vertex buffers.
    #[must_use]
    pub fn new() -> Self {
        Self {
            scene: vec![0_u8; PANEL_SCENE_BYTES],
            vertices: Vec::with_capacity(8_192),
        }
    }

    /// Encodes and tessellates the PFD into the panel encoder's reusable vertex buffer,
    /// returning a borrowed slice ready for [`sim_render::VectorScenePass::encode`].
    pub fn tessellate_pfd_slice(
        &mut self,
        data: &PanelData,
        config: &PfdConfig,
        alerts: Option<&AlertOutput>,
        target_w: u32,
        target_h: u32,
    ) -> Result<&[VectorVertex], PanelError> {
        self.encode_and_tessellate(
            "PFD",
            [target_w, target_h],
            |writer| draw_pfd(data, config, alerts, BUILTIN_FRAME, writer),
            |_| {},
        )?;
        Ok(&self.vertices)
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
        self.tessellate_pfd_slice(data, config, alerts, target_w, target_h)
            .map(<[VectorVertex]>::to_vec)
    }

    /// Encodes and tessellates the HSI into the panel encoder's reusable vertex buffer,
    /// returning a borrowed slice ready for [`sim_render::VectorScenePass::encode`].
    pub fn tessellate_hsi_slice(
        &mut self,
        data: &PanelData,
        alerts: Option<&AlertOutput>,
        target_w: u32,
        target_h: u32,
    ) -> Result<&[VectorVertex], PanelError> {
        let show_ownship = data.heading.value_rad.status.shows_value();
        self.encode_and_tessellate(
            "HSI",
            [target_w, target_h],
            |writer| draw_hsi(data, alerts, BUILTIN_FRAME, writer),
            |mesh| {
                if show_ownship {
                    append_hsi_ownship(mesh, [target_w as f32, target_h as f32]);
                }
            },
        )?;
        Ok(&self.vertices)
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
        self.tessellate_hsi_slice(data, alerts, target_w, target_h)
            .map(<[VectorVertex]>::to_vec)
    }

    fn encode_and_tessellate(
        &mut self,
        panel: &'static str,
        target: [u32; 2],
        draw: impl FnOnce(&mut SceneWriter<'_>) -> Result<(), SceneError>,
        post: impl FnOnce(&mut Mesh),
    ) -> Result<(), PanelError> {
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
        tessellate_mesh_into(scene, [PANEL_W, PANEL_H], target, &mut self.vertices, post)
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
