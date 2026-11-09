use indicate_alerts::AlertOutput;
use indicate_instrument_descriptor::DesignFrame;
use indicate_instrument_hud::{HudContext, draw_hud};
use indicate_instrument_scene::SceneWriter;
use indicate_instrument_state::PanelData;
use sim_render::VectorVertex;

use crate::PanelError;
use crate::panel::{
    HUD_MAX_ASPECT, HUD_MAX_WIDTH_PX, HUD_MIN_ASPECT, HUD_MIN_SIDE_PX, PANEL_SCENE_BYTES,
};

/// Logical height the vector HUD lays out at; the physical target scales it.
pub const VECTOR_HUD_DESIGN_HEIGHT: f32 = 720.0;

/// Indicate's screen head-up display tessellated for a target of any
/// resolution.
///
/// The HUD lays out once in a fixed-height logical frame of the target's
/// aspect, so symbology keeps its designed size in points while a Retina
/// target gets proportionally more pixels per line. Conformal cues register
/// with a world rendered through `context.projection` at the target aspect:
/// the projection carries only the vertical field of view, and the design
/// frame supplies the horizontal extent.
pub struct VectorHudPanel {
    scene: Vec<u8>,
    vertices: Vec<VectorVertex>,
}

impl Default for VectorHudPanel {
    fn default() -> Self {
        Self::new()
    }
}

impl VectorHudPanel {
    /// A panel with reusable scene and vertex buffers large enough for any accepted scene.
    #[must_use]
    pub fn new() -> Self {
        Self {
            scene: vec![0; PANEL_SCENE_BYTES],
            vertices: Vec::with_capacity(8_192),
        }
    }

    /// The logical `[width, height]` design frame for a physical target.
    ///
    /// The height is [`VECTOR_HUD_DESIGN_HEIGHT`] unless that would make the
    /// width narrower than the HUD lays out, in which case the width is held
    /// at the minimum and the height grows, keeping the target's aspect.
    pub fn design_frame(target_w: u32, target_h: u32) -> Result<[f32; 2], PanelError> {
        let out_of_range = PanelError::HudFrameOutOfRange {
            width: target_w,
            height: target_h,
        };
        if target_w == 0 || target_h == 0 {
            return Err(out_of_range);
        }
        let aspect = target_w as f32 / target_h as f32;
        if !(HUD_MIN_ASPECT..=HUD_MAX_ASPECT).contains(&aspect) {
            return Err(out_of_range);
        }
        let min_side = HUD_MIN_SIDE_PX as f32;
        let width = VECTOR_HUD_DESIGN_HEIGHT * aspect;
        if width >= min_side {
            Ok([width.min(HUD_MAX_WIDTH_PX as f32), VECTOR_HUD_DESIGN_HEIGHT])
        } else {
            Ok([min_side, min_side / aspect])
        }
    }

    /// Draws the HUD for `data` and tessellates it into the panel's reusable
    /// vertex buffer, returning a borrowed slice for [`sim_render::VectorScenePass::encode`].
    pub fn tessellate_slice(
        &mut self,
        data: &PanelData,
        context: &HudContext,
        alerts: Option<&AlertOutput>,
        target_w: u32,
        target_h: u32,
    ) -> Result<&[VectorVertex], PanelError> {
        let [width, height] = Self::design_frame(target_w, target_h)?;
        let capacity = self.scene.len();
        let mut writer = SceneWriter::new(&mut self.scene).map_err(|cause| PanelError::Scene {
            panel: "HUD",
            capacity,
            cause,
        })?;
        draw_hud(
            data,
            context,
            alerts,
            DesignFrame { width, height },
            &mut writer,
        )
        .map_err(|source| PanelError::HudDraw { source })?;
        let scene_bytes = writer.finish();
        let scene = self.scene.get(..scene_bytes).unwrap_or_default();
        super::tessellate_scene_cmds_into(
            scene,
            width,
            height,
            target_w,
            target_h,
            &mut self.vertices,
        )?;
        Ok(&self.vertices)
    }

    /// Draws the HUD for `data` and tessellates it into physical-pixel
    /// triangles for a `target_w × target_h` vector target.
    pub fn tessellate(
        &mut self,
        data: &PanelData,
        context: &HudContext,
        alerts: Option<&AlertOutput>,
        target_w: u32,
        target_h: u32,
    ) -> Result<Vec<VectorVertex>, PanelError> {
        self.tessellate_slice(data, context, alerts, target_w, target_h)
            .map(<[VectorVertex]>::to_vec)
    }
}
