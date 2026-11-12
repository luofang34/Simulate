//! Panels presented beside the main view: a second canvas or window the
//! renderer's device draws overlay quads onto, so instruments need not lie
//! over the world.

use super::Renderer;
use crate::compositor::{CompositorTarget, OverlayQuad};
use crate::{FrameOutcome, RenderError};

/// Background behind a panel's quads.
const PANEL_CLEAR: wgpu::Color = wgpu::Color {
    r: 0.0,
    g: 0.0,
    b: 0.0,
    a: 1.0,
};

/// A surface of its own that the renderer presents panels to.
pub struct PanelSurface {
    surface: wgpu::Surface<'static>,
    config: wgpu::SurfaceConfiguration,
}

impl PanelSurface {
    /// Current size `[width, height]` in physical pixels.
    #[must_use]
    pub const fn size(&self) -> [u32; 2] {
        [self.config.width, self.config.height]
    }
}

impl Renderer {
    /// A panel surface on `target` at `width × height` pixels, sharing this renderer's
    /// device. It presents through the main view's format, so the compositor serves both.
    pub fn panel_surface(
        &self,
        target: impl Into<wgpu::SurfaceTarget<'static>>,
        width: u32,
        height: u32,
    ) -> Result<PanelSurface, RenderError> {
        let gpu = &self.gpu;
        let surface = gpu
            .instance
            .create_surface(target.into())
            .map_err(RenderError::CreateSurface)?;
        let unsupported = || RenderError::UnsupportedSurface {
            adapter: gpu.adapter.get_info().name,
        };
        let mut config = surface
            .get_default_config(&gpu.adapter, width.max(1), height.max(1))
            .ok_or_else(unsupported)?;
        let formats = surface.get_capabilities(&gpu.adapter).formats;
        config.format = formats
            .into_iter()
            .find(|format| format.add_srgb_suffix() == gpu.view_format)
            .ok_or_else(unsupported)?;
        if gpu.view_format != config.format {
            config.view_formats = vec![gpu.view_format];
        }
        surface.configure(&gpu.device, &config);
        Ok(PanelSurface { surface, config })
    }

    /// Resizes `panel`; zero sizes and the size it has are ignored.
    pub fn resize_panel(&self, panel: &mut PanelSurface, width: u32, height: u32) {
        if width == 0 || height == 0 || panel.size() == [width, height] {
            return;
        }
        panel.config.width = width;
        panel.config.height = height;
        panel.surface.configure(&self.gpu.device, &panel.config);
    }

    /// Draws `quads` onto `panel` over a black background and presents it.
    pub fn present_panel(
        &mut self,
        panel: &mut PanelSurface,
        quads: &[OverlayQuad<'_>],
    ) -> Result<FrameOutcome, RenderError> {
        let texture = match panel.surface.get_current_texture() {
            wgpu::CurrentSurfaceTexture::Success(texture)
            | wgpu::CurrentSurfaceTexture::Suboptimal(texture) => texture,
            wgpu::CurrentSurfaceTexture::Timeout | wgpu::CurrentSurfaceTexture::Occluded => {
                return Ok(FrameOutcome::Skipped);
            }
            wgpu::CurrentSurfaceTexture::Outdated | wgpu::CurrentSurfaceTexture::Lost => {
                panel.surface.configure(&self.gpu.device, &panel.config);
                return Ok(FrameOutcome::Skipped);
            }
            wgpu::CurrentSurfaceTexture::Validation => return Err(RenderError::SurfaceValidation),
        };
        let view = texture.texture.create_view(&wgpu::TextureViewDescriptor {
            format: Some(self.gpu.view_format),
            ..Default::default()
        });
        let mut encoder = self
            .gpu
            .device
            .create_command_encoder(&wgpu::CommandEncoderDescriptor {
                label: Some("panel"),
            });
        self.compositor.encode(
            &self.gpu.device,
            &self.gpu.queue,
            &mut encoder,
            CompositorTarget {
                view: &view,
                clear: Some(PANEL_CLEAR),
                size_px: panel.size(),
            },
            quads,
        );
        self.gpu.queue.submit([encoder.finish()]);
        self.gpu.queue.present(texture);
        Ok(FrameOutcome::Presented)
    }
}
