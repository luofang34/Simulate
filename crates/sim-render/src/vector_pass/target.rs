use super::{MIN_VERTEX_CAPACITY, VectorVertex, too_large};
use crate::RenderError;
use crate::gpu::DEPTH_FORMAT;

/// Colour format of a [`VectorTarget`]; blending happens in linear light and
/// the resolve stores sRGB-encoded premultiplied colour.
pub const VECTOR_COLOUR_FORMAT: wgpu::TextureFormat = wgpu::TextureFormat::Rgba8UnormSrgb;

/// Samples per pixel of the multisampled vector attachments.
pub const VECTOR_SAMPLE_COUNT: u32 = 4;

struct Attachments {
    msaa: wgpu::Texture,
    msaa_view: wgpu::TextureView,
    depth: wgpu::Texture,
    depth_view: wgpu::TextureView,
    resolve: wgpu::Texture,
    resolve_view: wgpu::TextureView,
}

impl Attachments {
    /// Frees the textures now. On WebGPU dropping a texture frees nothing until the
    /// page's garbage collector runs, so a resize would otherwise hold both sizes.
    fn destroy(&self) {
        self.msaa.destroy();
        self.depth.destroy();
        self.resolve.destroy();
    }
}

/// Multisampled attachments, per-target GPU buffers, and the single-sample
/// resolve texture a [`VectorScenePass`](crate::VectorScenePass) draws into.
///
/// Each target owns its own viewport uniform and vertex buffer so multiple
/// targets (such as a cockpit's PFD, HSI, and HUD, or independent standalone
/// displays) can record into one `wgpu::CommandEncoder` and submit together.
///
/// The resolve view holds premultiplied colour and is meant for
/// [`OverlayQuad::vector_overlay`](crate::OverlayQuad::vector_overlay).
pub struct VectorTarget {
    label: &'static str,
    attachments: Attachments,
    viewport: wgpu::Buffer,
    bind_group: Option<wgpu::BindGroup>,
    vertices: Option<wgpu::Buffer>,
    capacity_bytes: u64,
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
        let att = allocate_attachments(device, label, width, height)?;
        let viewport = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some(label),
            size: std::mem::size_of::<[f32; 4]>() as u64,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        Ok(Self {
            label,
            attachments: att,
            viewport,
            bind_group: None,
            vertices: None,
            capacity_bytes: 0,
            size: [width, height],
        })
    }

    /// Reallocates the colour and depth attachments when the size differs from
    /// the current one, preserving the target's uniform and vertex buffers.
    pub fn ensure_size(
        &mut self,
        device: &wgpu::Device,
        width: u32,
        height: u32,
    ) -> Result<(), RenderError> {
        if self.size == [width, height] {
            return Ok(());
        }
        let att = allocate_attachments(device, self.label, width, height)?;
        std::mem::replace(&mut self.attachments, att).destroy();
        self.size = [width, height];
        Ok(())
    }

    /// Resolved single-sample premultiplied colour view.
    #[must_use]
    pub const fn view(&self) -> &wgpu::TextureView {
        &self.attachments.resolve_view
    }

    /// Resolved single-sample colour texture, for copies and readback.
    #[must_use]
    pub const fn texture(&self) -> &wgpu::Texture {
        &self.attachments.resolve
    }

    /// Attachment size `[width_px, height_px]`.
    #[must_use]
    pub const fn size_px(&self) -> [u32; 2] {
        self.size
    }

    pub(super) const fn msaa_view(&self) -> &wgpu::TextureView {
        &self.attachments.msaa_view
    }

    pub(super) const fn depth_view(&self) -> &wgpu::TextureView {
        &self.attachments.depth_view
    }

    pub(super) fn write_viewport(&self, queue: &wgpu::Queue) {
        let [w, h] = self.size;
        let viewport = [w as f32, h as f32, 0.0, 0.0];
        queue.write_buffer(&self.viewport, 0, bytemuck::bytes_of(&viewport));
    }

    pub(super) fn ensure_bind_group(
        &mut self,
        device: &wgpu::Device,
        layout: &wgpu::BindGroupLayout,
    ) {
        if self.bind_group.is_none() {
            self.bind_group = Some(device.create_bind_group(&wgpu::BindGroupDescriptor {
                label: Some(self.label),
                layout,
                entries: &[wgpu::BindGroupEntry {
                    binding: 0,
                    resource: self.viewport.as_entire_binding(),
                }],
            }));
        }
    }

    pub(super) const fn draw_resources(&self) -> Option<(&wgpu::BindGroup, &wgpu::Buffer)> {
        match (&self.bind_group, &self.vertices) {
            (Some(bg), Some(buf)) => Some((bg, buf)),
            _ => None,
        }
    }

    pub(super) fn upload_vertices(
        &mut self,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        vertices: &[VectorVertex],
    ) -> Result<(), RenderError> {
        let bytes: &[u8] = bytemuck::cast_slice(vertices);
        let needed = bytes.len() as u64;
        if self.vertices.is_none() || needed > self.capacity_bytes {
            let max = device.limits().max_buffer_size;
            let stride = std::mem::size_of::<VectorVertex>() as u64;
            let capacity = needed
                .max(MIN_VERTEX_CAPACITY * stride)
                .checked_next_power_of_two()
                .unwrap_or(needed)
                .min(max);
            if needed > capacity {
                return Err(too_large(device, vertices.len()));
            }
            self.vertices = Some(device.create_buffer(&wgpu::BufferDescriptor {
                label: Some(self.label),
                size: capacity,
                usage: wgpu::BufferUsages::VERTEX | wgpu::BufferUsages::COPY_DST,
                mapped_at_creation: false,
            }));
            self.capacity_bytes = capacity;
        }
        if let Some(buffer) = &self.vertices {
            queue.write_buffer(buffer, 0, bytes);
        }
        Ok(())
    }
}

fn allocate_attachments(
    device: &wgpu::Device,
    label: &'static str,
    width: u32,
    height: u32,
) -> Result<Attachments, RenderError> {
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
    Ok(Attachments {
        msaa_view: view(&msaa),
        depth_view: view(&depth),
        resolve_view: view(&resolve),
        msaa,
        depth,
        resolve,
    })
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
