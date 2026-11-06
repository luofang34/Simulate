use indicate_alerts::AlertOutput;
use indicate_instrument_descriptor::DesignFrame;
use indicate_instrument_hud::{HudContext, draw_hud};
use indicate_instrument_panels::{BUILTIN_FRAME, PANEL_H, PANEL_W, PfdConfig, draw_hsi, draw_pfd};
use indicate_instrument_raster::{FrameId, FramebufferDims, RenderReport, render};
use indicate_instrument_scene::{MAX_SCENE_BYTES, SceneError, SceneWriter};
use indicate_instrument_state::PanelData;

use crate::PanelError;

mod ownship_symbol;
#[cfg(test)]
mod tests;

/// Smallest HUD frame side `indicate-instrument-hud` lays out, pixels.
pub const HUD_MIN_SIDE_PX: u32 = 390;
/// Largest HUD frame width `indicate-instrument-hud` lays out, pixels.
pub const HUD_MAX_WIDTH_PX: u32 = 2560;
/// Largest HUD frame height `indicate-instrument-hud` lays out, pixels.
pub const HUD_MAX_HEIGHT_PX: u32 = 1600;
pub(crate) const HUD_MIN_ASPECT: f32 = 0.45;
pub(crate) const HUD_MAX_ASPECT: f32 = 2.4;

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
    let capacity = buffers.scene.len();
    paint_sized(
        panel,
        [PANEL_WIDTH_PX, PANEL_HEIGHT_PX],
        frame,
        buffers,
        |scene| {
            draw(scene).map_err(|cause| PanelError::Scene {
                panel,
                capacity,
                cause,
            })
        },
    )
}

fn paint_sized(
    panel: &'static str,
    [min_width, min_height]: [u32; 2],
    frame: FrameId,
    buffers: &mut PanelBuffers<'_>,
    draw: impl FnOnce(&mut SceneWriter<'_>) -> Result<(), PanelError>,
) -> Result<PanelRender, PanelError> {
    let dims = buffers.dims;
    if dims.width < min_width || dims.height < min_height {
        return Err(PanelError::FramebufferClipsPanel {
            width: dims.width,
            height: dims.height,
            min_width,
            min_height,
        });
    }
    let capacity = buffers.scene.len();
    let mut writer = SceneWriter::new(buffers.scene).map_err(|cause| PanelError::Scene {
        panel,
        capacity,
        cause,
    })?;
    draw(&mut writer)?;
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
    let render = paint("HSI", frame, buffers, |scene| {
        draw_hsi(data, alerts, BUILTIN_FRAME, scene)
    })?;
    if data.heading.value_rad.status.shows_value() {
        ownship_symbol::stamp_ownship(buffers.pixels, buffers.dims, ownship_symbol::HSI_CENTRE_PX);
    }
    Ok(render)
}

/// Rasterizes Indicate's screen head-up display over the whole of `buffers`.
///
/// The design frame is the framebuffer itself, so the conformal cues register
/// with a world rendered through `context.projection` at the same size. The
/// frame must lie inside the HUD's layout bounds.
pub fn render_hud(
    data: &PanelData,
    context: &HudContext,
    alerts: Option<&AlertOutput>,
    frame: FrameId,
    buffers: &mut PanelBuffers<'_>,
) -> Result<PanelRender, PanelError> {
    let FramebufferDims { width, height, .. } = buffers.dims;
    let aspect = width as f32 / height.max(1) as f32;
    if !(HUD_MIN_SIDE_PX..=HUD_MAX_WIDTH_PX).contains(&width)
        || !(HUD_MIN_SIDE_PX..=HUD_MAX_HEIGHT_PX).contains(&height)
        || !(HUD_MIN_ASPECT..=HUD_MAX_ASPECT).contains(&aspect)
    {
        return Err(PanelError::HudFrameOutOfRange { width, height });
    }
    let design = DesignFrame {
        width: width as f32,
        height: height as f32,
    };
    paint_sized("HUD", [width, height], frame, buffers, |scene| {
        draw_hud(data, context, alerts, design, scene)
            .map_err(|source| PanelError::HudDraw { source })
    })
}
