use indicate_alerts::AlertOutput;
use indicate_instrument_panels::{BUILTIN_FRAME, PANEL_H, PANEL_W, PfdConfig, draw_hsi, draw_pfd};
use indicate_instrument_raster::{FrameId, FramebufferDims, RenderReport, render};
use indicate_instrument_scene::{MAX_SCENE_BYTES, SceneError, SceneWriter};
use indicate_instrument_state::PanelData;

use crate::PanelError;

#[cfg(test)]
mod tests;

/// Width every shipped panel draws at, pixels.
pub const PANEL_WIDTH_PX: u32 = PANEL_W as u32;
/// Height every shipped panel draws at, pixels.
pub const PANEL_HEIGHT_PX: u32 = PANEL_H as u32;
/// Bytes of one tightly packed RGBA8 panel frame.
pub const PANEL_RGBA_BYTES: usize = PANEL_WIDTH_PX as usize * PANEL_HEIGHT_PX as usize * 4;
/// Scene buffer that holds any scene the rasterizer accepts.
pub const PANEL_SCENE_BYTES: usize = MAX_SCENE_BYTES;

/// Where a panel is drawn, owned by the caller so a frame loop can reuse it.
#[derive(Debug)]
pub struct PanelBuffers<'a> {
    /// Receives the encoded scene.
    pub scene: &'a mut [u8],
    /// Receives the RGBA8 frame.
    pub pixels: &'a mut [u8],
    /// Shape of `pixels`.
    pub dims: FramebufferDims,
}

/// What a panel frame took.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PanelRender {
    /// Encoded scene length, bytes.
    pub scene_bytes: usize,
    /// The rasterizer's account of the frame.
    pub report: RenderReport,
}

fn paint(
    panel: &'static str,
    frame: FrameId,
    buffers: &mut PanelBuffers<'_>,
    draw: impl FnOnce(&mut SceneWriter<'_>) -> Result<(), SceneError>,
) -> Result<PanelRender, PanelError> {
    let dims = buffers.dims;
    if dims.width < PANEL_WIDTH_PX || dims.height < PANEL_HEIGHT_PX {
        return Err(PanelError::FramebufferClipsPanel {
            width: dims.width,
            height: dims.height,
            min_width: PANEL_WIDTH_PX,
            min_height: PANEL_HEIGHT_PX,
        });
    }
    let capacity = buffers.scene.len();
    let scene_error = |cause| PanelError::Scene {
        panel,
        capacity,
        cause,
    };
    let mut writer = SceneWriter::new(buffers.scene).map_err(scene_error)?;
    draw(&mut writer).map_err(scene_error)?;
    let scene_bytes = writer.finish();
    let scene = buffers.scene.get(..scene_bytes).unwrap_or_default();
    let report = render(scene, buffers.pixels, dims, frame)
        .map_err(|source| PanelError::Raster { panel, source })?;
    Ok(PanelRender {
        scene_bytes,
        report,
    })
}

/// Rasterizes Indicate's primary flight display for `data` into `buffers`.
pub fn render_pfd(
    data: &PanelData,
    config: &PfdConfig,
    alerts: Option<&AlertOutput>,
    frame: FrameId,
    buffers: &mut PanelBuffers<'_>,
) -> Result<PanelRender, PanelError> {
    paint("PFD", frame, buffers, |scene| {
        draw_pfd(data, config, alerts, BUILTIN_FRAME, scene)
    })
}

/// Rasterizes Indicate's horizontal situation indicator for `data` into
/// `buffers`.
pub fn render_hsi(
    data: &PanelData,
    alerts: Option<&AlertOutput>,
    frame: FrameId,
    buffers: &mut PanelBuffers<'_>,
) -> Result<PanelRender, PanelError> {
    paint("HSI", frame, buffers, |scene| {
        draw_hsi(data, alerts, BUILTIN_FRAME, scene)
    })
}
