//! A second presentation surface on the [`Renderer`]'s device: a texture drawn
//! elsewhere (a map view, an instrument) shown on a canvas or window of its own,
//! so one GPU device serves several places on a page.

use crate::compositor::{CompositorPass, CompositorTarget, OverlayQuad};
use crate::{FrameOutcome, RenderError, Renderer};

/// A surface that shows one texture view per frame, filling it.
pub struct CanvasPresenter {
    surface: wgpu::Surface<'static>,
    config: wgpu::SurfaceConfiguration,
    view_format: wgpu::TextureFormat,
    compositor: CompositorPass,
}

impl CanvasPresenter {
    /// A surface on `target`, `width × height` physical pixels, presenting
    /// through `renderer`'s adapter and device.
    pub async fn new(
        renderer: &Renderer,
        target: impl Into<wgpu::SurfaceTarget<'static>>,
        width: u32,
        height: u32,
    ) -> Result<Self, RenderError> {
        let surface = renderer
            .instance()
            .create_surface(target.into())
            .map_err(RenderError::CreateSurface)?;
        let adapter = renderer.adapter();
        let mut config = surface
            .get_default_config(adapter, width.max(1), height.max(1))
            .ok_or_else(|| RenderError::UnsupportedSurface {
                adapter: adapter.get_info().name,
            })?;
        let capabilities = surface.get_capabilities(adapter);
        if let Some(srgb) = capabilities.formats.iter().copied().find(|f| f.is_srgb()) {
            config.format = srgb;
        }
        // As for the window: browser WebGPU offers non-sRGB swap chains only, and an
        // sRGB view of the same texture still gamma-encodes.
        let view_format = config.format.add_srgb_suffix();
        if view_format != config.format {
            config.view_formats.push(view_format);
        }
        surface.configure(renderer.device(), &config);
        let compositor = CompositorPass::new(renderer.device(), view_format).await?;
        Ok(Self {
            surface,
            config,
            view_format,
            compositor,
        })
    }

    /// The surface's size, physical pixels.
    #[must_use]
    pub const fn size(&self) -> [u32; 2] {
        [self.config.width, self.config.height]
    }

    /// Resizes the surface; zero or unchanged sizes are ignored.
    pub fn resize(&mut self, device: &wgpu::Device, width: u32, height: u32) {
        if width == 0 || height == 0 || [width, height] == self.size() {
            return;
        }
        self.config.width = width;
        self.config.height = height;
        self.surface.configure(device, &self.config);
    }

    /// Shows `quad` (normally [`OverlayQuad::full_map`]) on the surface.
    pub fn present(
        &mut self,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        quad: OverlayQuad<'_>,
    ) -> Result<FrameOutcome, RenderError> {
        let texture = match self.surface.get_current_texture() {
            wgpu::CurrentSurfaceTexture::Success(texture)
            | wgpu::CurrentSurfaceTexture::Suboptimal(texture) => texture,
            wgpu::CurrentSurfaceTexture::Timeout | wgpu::CurrentSurfaceTexture::Occluded => {
                return Ok(FrameOutcome::Skipped);
            }
            wgpu::CurrentSurfaceTexture::Outdated | wgpu::CurrentSurfaceTexture::Lost => {
                self.surface.configure(device, &self.config);
                return Ok(FrameOutcome::Skipped);
            }
            wgpu::CurrentSurfaceTexture::Validation => return Err(RenderError::SurfaceValidation),
        };
        let view = texture.texture.create_view(&wgpu::TextureViewDescriptor {
            format: Some(self.view_format),
            ..Default::default()
        });
        let mut encoder = device.create_command_encoder(&wgpu::CommandEncoderDescriptor {
            label: Some("canvas presenter"),
        });
        self.compositor.encode(
            device,
            queue,
            &mut encoder,
            CompositorTarget {
                view: &view,
                clear: Some(wgpu::Color::BLACK),
                size_px: self.size(),
            },
            &[quad],
        );
        queue.submit([encoder.finish()]);
        queue.present(texture);
        Ok(FrameOutcome::Presented)
    }
}
