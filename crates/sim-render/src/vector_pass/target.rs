use crate::RenderError;
use crate::gpu::DEPTH_FORMAT;

/// Colour format of a [`VectorTarget`]; blending happens in linear light and
/// the resolve stores sRGB-encoded premultiplied colour.
pub const VECTOR_COLOUR_FORMAT: wgpu::TextureFormat = wgpu::TextureFormat::Rgba8UnormSrgb;

/// Samples per pixel of the multisampled vector attachments.
pub const VECTOR_SAMPLE_COUNT: u32 = 4;

/// Multisampled attachments plus the single-sample resolve texture a
/// [`VectorScenePass`](crate::VectorScenePass) draws into.
///
/// The resolve view holds premultiplied colour and is meant for
/// [`OverlayQuad::vector_overlay`](crate::OverlayQuad::vector_overlay).
pub struct VectorTarget {
    label: &'static str,
    msaa_view: wgpu::TextureView,
    depth_view: wgpu::TextureView,
    resolve: wgpu::Texture,
    resolve_view: wgpu::TextureView,
    size: [u32; 2],
}

impl VectorTarget {
    /// Allocates `width × height` attachments on `device`.
    pub fn new(
        device: &wgpu::Device,
        label: &'static str,
        width: u32,
        height: u32,
    ) -> Result<Self, RenderError> {
        check_size(device, label, width, height)?;
        let extent = wgpu::Extent3d {
            width,
            height,
            depth_or_array_layers: 1,
        };
        let msaa = texture(
            device,
            label,
            extent,
            VECTOR_SAMPLE_COUNT,
            VECTOR_COLOUR_FORMAT,
            wgpu::TextureUsages::RENDER_ATTACHMENT,
        );
        let depth = texture(
            device,
            label,
            extent,
            VECTOR_SAMPLE_COUNT,
            DEPTH_FORMAT,
            wgpu::TextureUsages::RENDER_ATTACHMENT,
        );
        let resolve = texture(
            device,
            label,
            extent,
            1,
            VECTOR_COLOUR_FORMAT,
            wgpu::TextureUsages::RENDER_ATTACHMENT
                | wgpu::TextureUsages::TEXTURE_BINDING
                | wgpu::TextureUsages::COPY_SRC,
        );
        let view = |t: &wgpu::Texture| t.create_view(&wgpu::TextureViewDescriptor::default());
        Ok(Self {
            label,
            msaa_view: view(&msaa),
            depth_view: view(&depth),
            resolve_view: view(&resolve),
            resolve,
            size: [width, height],
        })
    }

    /// Reallocates the attachments when the size differs from the current one.
    pub fn ensure_size(
        &mut self,
        device: &wgpu::Device,
        width: u32,
        height: u32,
    ) -> Result<(), RenderError> {
        if self.size == [width, height] {
            return Ok(());
        }
        *self = Self::new(device, self.label, width, height)?;
        Ok(())
    }

    /// Resolved single-sample premultiplied colour view.
    #[must_use]
    pub const fn view(&self) -> &wgpu::TextureView {
        &self.resolve_view
    }

    /// Resolved single-sample colour texture, for copies and readback.
    #[must_use]
    pub const fn texture(&self) -> &wgpu::Texture {
        &self.resolve
    }

    /// Attachment size `[width_px, height_px]`.
    #[must_use]
    pub const fn size_px(&self) -> [u32; 2] {
        self.size
    }

    pub(crate) const fn msaa_view(&self) -> &wgpu::TextureView {
        &self.msaa_view
    }

    pub(crate) const fn depth_view(&self) -> &wgpu::TextureView {
        &self.depth_view
    }
}

fn check_size(
    device: &wgpu::Device,
    label: &'static str,
    width: u32,
    height: u32,
) -> Result<(), RenderError> {
    if width == 0 || height == 0 {
        return Err(RenderError::EmptyTexture {
            what: label,
            width,
            height,
        });
    }
    let max = device.limits().max_texture_dimension_2d;
    if width > max || height > max {
        return Err(RenderError::TextureTooLarge {
            what: label,
            width,
            height,
            max,
        });
    }
    Ok(())
}

fn texture(
    device: &wgpu::Device,
    label: &'static str,
    size: wgpu::Extent3d,
    sample_count: u32,
    format: wgpu::TextureFormat,
    usage: wgpu::TextureUsages,
) -> wgpu::Texture {
    device.create_texture(&wgpu::TextureDescriptor {
        label: Some(label),
        size,
        mip_level_count: 1,
        sample_count,
        dimension: wgpu::TextureDimension::D2,
        format,
        usage,
        view_formats: &[],
    })
}
