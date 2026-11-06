use crate::RenderError;

/// Persistent `Rgba8UnormSrgb` GPU texture receiving CPU-rasterized `Indicate` RGBA8 frames.
pub struct RgbaPanelTexture {
    texture: wgpu::Texture,
    view: wgpu::TextureView,
    width: u32,
    height: u32,
}

impl RgbaPanelTexture {
    /// Allocates a `width × height` `Rgba8UnormSrgb` texture on `device`.
    pub fn new(
        device: &wgpu::Device,
        label: &'static str,
        width: u32,
        height: u32,
    ) -> Result<Self, RenderError> {
        let width = width.max(1);
        let height = height.max(1);
        let max = device.limits().max_texture_dimension_2d;
        if width > max || height > max {
            return Err(RenderError::TextureTooLarge {
                what: label,
                width,
                height,
                max,
            });
        }
        let texture = device.create_texture(&wgpu::TextureDescriptor {
            label: Some(label),
            size: wgpu::Extent3d {
                width,
                height,
                depth_or_array_layers: 1,
            },
            mip_level_count: 1,
            sample_count: 1,
            dimension: wgpu::TextureDimension::D2,
            format: wgpu::TextureFormat::Rgba8UnormSrgb,
            usage: wgpu::TextureUsages::TEXTURE_BINDING | wgpu::TextureUsages::COPY_DST,
            view_formats: &[],
        });
        let view = texture.create_view(&wgpu::TextureViewDescriptor::default());
        Ok(Self {
            texture,
            view,
            width,
            height,
        })
    }

    /// Uploads a tightly packed `width * height * 4` sRGB RGBA8 buffer into the GPU texture.
    pub fn upload(&self, queue: &wgpu::Queue, rgba: &[u8]) -> Result<(), RenderError> {
        let expected = (self.width as usize)
            .saturating_mul(self.height as usize)
            .saturating_mul(4);
        if rgba.len() != expected {
            return Err(RenderError::InvalidUploadBytes {
                expected,
                actual: rgba.len(),
            });
        }
        queue.write_texture(
            wgpu::TexelCopyTextureInfo {
                texture: &self.texture,
                mip_level: 0,
                origin: wgpu::Origin3d::ZERO,
                aspect: wgpu::TextureAspect::All,
            },
            rgba,
            wgpu::TexelCopyBufferLayout {
                offset: 0,
                bytes_per_row: Some(self.width.saturating_mul(4)),
                rows_per_image: Some(self.height),
            },
            wgpu::Extent3d {
                width: self.width,
                height: self.height,
                depth_or_array_layers: 1,
            },
        );
        Ok(())
    }

    /// GPU texture view ready to pass in an [`OverlayQuad`](crate::OverlayQuad).
    #[must_use]
    pub const fn view(&self) -> &wgpu::TextureView {
        &self.view
    }

    /// Texture width in pixels.
    #[must_use]
    pub const fn width(&self) -> u32 {
        self.width
    }

    /// Texture height in pixels.
    #[must_use]
    pub const fn height(&self) -> u32 {
        self.height
    }
}
