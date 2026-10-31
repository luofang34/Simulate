use crate::{PinholeProjection, RenderError};

#[cfg(test)]
mod tests;

/// Colour format of an aperture. The stored bytes are sRGB, as `Navigate`'s
/// image pipeline expects of a camera frame.
pub const APERTURE_COLOUR_FORMAT: wgpu::TextureFormat = wgpu::TextureFormat::Rgba8UnormSrgb;
/// Depth format of an aperture: reversed-Z, cleared to `0.0` at infinity.
pub const APERTURE_DEPTH_FORMAT: wgpu::TextureFormat = crate::gpu::DEPTH_FORMAT;

/// Both RGBA8 and `Depth32Float` are four bytes per pixel.
#[cfg(not(target_arch = "wasm32"))]
const READBACK_BYTES_PER_PIXEL: u32 = 4;

/// Bytes per image row in a buffer texture copy: wgpu requires rows to start
/// on [`wgpu::COPY_BYTES_PER_ROW_ALIGNMENT`] boundaries. `None` on overflow.
pub fn padded_bytes_per_row(width: u32, bytes_per_pixel: u32) -> Option<u32> {
    width
        .checked_mul(bytes_per_pixel)?
        .checked_next_multiple_of(wgpu::COPY_BYTES_PER_ROW_ALIGNMENT)
}

/// Removes the row padding of a buffer texture copy.
#[cfg(not(target_arch = "wasm32"))]
fn strip_row_padding(padded: &[u8], padded_row: usize, tight_row: usize) -> Vec<u8> {
    padded
        .chunks_exact(padded_row)
        .flat_map(|row| row.iter().take(tight_row))
        .copied()
        .collect()
}

/// One rendered frame, tightly packed.
#[derive(Debug, Clone, PartialEq)]
pub struct ApertureImage {
    /// Width in pixels.
    pub width: u32,
    /// Height in pixels.
    pub height: u32,
    /// Row-major sRGB RGBA8 pixels, top row first.
    pub rgba8: Vec<u8>,
    /// Row-major reversed-Z depth, top row first; `0.0` is open sky.
    pub depth: Vec<f32>,
}

impl ApertureImage {
    /// The colour at pixel `(x, y)`, or `None` outside the image.
    pub fn pixel(&self, x: u32, y: u32) -> Option<[u8; 4]> {
        let start = self.index(x, y)?.checked_mul(4)?;
        self.rgba8.get(start..start + 4)?.first_chunk().copied()
    }

    /// The depth at pixel `(x, y)`, or `None` outside the image.
    pub fn depth_at(&self, x: u32, y: u32) -> Option<f32> {
        self.depth.get(self.index(x, y)?).copied()
    }

    fn index(&self, x: u32, y: u32) -> Option<usize> {
        (x < self.width && y < self.height).then(|| y as usize * self.width as usize + x as usize)
    }
}

/// An offscreen colour and depth target for one camera, with the staging
/// buffers to read it back.
pub struct OffscreenAperture {
    projection: PinholeProjection,
    colour: wgpu::Texture,
    colour_view: wgpu::TextureView,
    depth: wgpu::Texture,
    depth_view: wgpu::TextureView,
    #[cfg(not(target_arch = "wasm32"))]
    colour_staging: wgpu::Buffer,
    #[cfg(not(target_arch = "wasm32"))]
    depth_staging: wgpu::Buffer,
}

impl OffscreenAperture {
    /// Creates a target sized by `projection`'s intrinsics.
    pub fn new(device: &wgpu::Device, projection: PinholeProjection) -> Result<Self, RenderError> {
        let intrinsics = projection.intrinsics();
        let (width, height) = (intrinsics.width, intrinsics.height);
        let max = device.limits().max_texture_dimension_2d;
        if width > max || height > max {
            return Err(RenderError::TextureTooLarge {
                what: "aperture",
                width,
                height,
                max,
            });
        }
        let colour = target_texture(
            device,
            "aperture colour",
            width,
            height,
            APERTURE_COLOUR_FORMAT,
        );
        let depth = target_texture(
            device,
            "aperture depth",
            width,
            height,
            APERTURE_DEPTH_FORMAT,
        );
        Ok(Self {
            projection,
            colour_view: colour.create_view(&wgpu::TextureViewDescriptor::default()),
            colour,
            depth_view: depth.create_view(&wgpu::TextureViewDescriptor::default()),
            depth,
            #[cfg(not(target_arch = "wasm32"))]
            colour_staging: staging_buffer(device, "aperture colour readback", width, height)?,
            #[cfg(not(target_arch = "wasm32"))]
            depth_staging: staging_buffer(device, "aperture depth readback", width, height)?,
        })
    }

    /// The camera this target belongs to.
    pub fn projection(&self) -> &PinholeProjection {
        &self.projection
    }

    /// The colour texture, for sampling a frame on the GPU.
    pub fn colour_texture(&self) -> &wgpu::Texture {
        &self.colour
    }

    /// The depth texture, for sampling a frame on the GPU.
    pub fn depth_texture(&self) -> &wgpu::Texture {
        &self.depth
    }

    pub(super) fn colour_view(&self) -> &wgpu::TextureView {
        &self.colour_view
    }

    pub(super) fn depth_view(&self) -> &wgpu::TextureView {
        &self.depth_view
    }
}

fn target_texture(
    device: &wgpu::Device,
    label: &'static str,
    width: u32,
    height: u32,
    format: wgpu::TextureFormat,
) -> wgpu::Texture {
    device.create_texture(&wgpu::TextureDescriptor {
        label: Some(label),
        size: wgpu::Extent3d {
            width,
            height,
            depth_or_array_layers: 1,
        },
        mip_level_count: 1,
        sample_count: 1,
        dimension: wgpu::TextureDimension::D2,
        format,
        usage: wgpu::TextureUsages::RENDER_ATTACHMENT
            | wgpu::TextureUsages::COPY_SRC
            | wgpu::TextureUsages::TEXTURE_BINDING,
        view_formats: &[],
    })
}

// wgpu's GL backend copies through a colour attachment, which a depth texture
// cannot be, and a browser cannot block on a map at all; so this path is
// native only. In a browser, sample the textures on the GPU instead.
#[cfg(not(target_arch = "wasm32"))]
impl OffscreenAperture {
    /// Copies the last rendered frame to the CPU.
    pub fn read_blocking(
        &self,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
    ) -> Result<ApertureImage, RenderError> {
        let intrinsics = self.projection.intrinsics();
        let (width, height) = (intrinsics.width, intrinsics.height);
        let padded_row = readback_row(width)?;
        let tight_row = (width * READBACK_BYTES_PER_PIXEL) as usize;
        let mut encoder = device.create_command_encoder(&wgpu::CommandEncoderDescriptor {
            label: Some("aperture readback"),
        });
        copy_to_staging(
            &mut encoder,
            &self.colour,
            wgpu::TextureAspect::All,
            &self.colour_staging,
            padded_row,
        );
        copy_to_staging(
            &mut encoder,
            &self.depth,
            wgpu::TextureAspect::DepthOnly,
            &self.depth_staging,
            padded_row,
        );
        queue.submit([encoder.finish()]);
        let colour = read_staging_blocking(device, &self.colour_staging)?;
        let depth = read_staging_blocking(device, &self.depth_staging)?;
        let rgba8 = strip_row_padding(&colour, padded_row as usize, tight_row);
        let depth_bytes = strip_row_padding(&depth, padded_row as usize, tight_row);
        Ok(ApertureImage {
            width,
            height,
            rgba8,
            depth: bytemuck::pod_collect_to_vec(&depth_bytes),
        })
    }
}

#[cfg(not(target_arch = "wasm32"))]
fn staging_buffer(
    device: &wgpu::Device,
    label: &'static str,
    width: u32,
    height: u32,
) -> Result<wgpu::Buffer, RenderError> {
    let padded_row = readback_row(width)?;
    Ok(device.create_buffer(&wgpu::BufferDescriptor {
        label: Some(label),
        size: u64::from(padded_row) * u64::from(height),
        usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
        mapped_at_creation: false,
    }))
}

#[cfg(not(target_arch = "wasm32"))]
fn readback_row(width: u32) -> Result<u32, RenderError> {
    padded_bytes_per_row(width, READBACK_BYTES_PER_PIXEL).ok_or_else(|| RenderError::Readback {
        detail: format!("{width} pixel rows overflow"),
    })
}

#[cfg(not(target_arch = "wasm32"))]
fn copy_to_staging(
    encoder: &mut wgpu::CommandEncoder,
    texture: &wgpu::Texture,
    aspect: wgpu::TextureAspect,
    staging: &wgpu::Buffer,
    padded_row: u32,
) {
    encoder.copy_texture_to_buffer(
        wgpu::TexelCopyTextureInfo {
            texture,
            mip_level: 0,
            origin: wgpu::Origin3d::ZERO,
            aspect,
        },
        wgpu::TexelCopyBufferInfo {
            buffer: staging,
            layout: wgpu::TexelCopyBufferLayout {
                offset: 0,
                bytes_per_row: Some(padded_row),
                rows_per_image: None,
            },
        },
        texture.size(),
    );
}

#[cfg(not(target_arch = "wasm32"))]
fn read_staging_blocking(
    device: &wgpu::Device,
    staging: &wgpu::Buffer,
) -> Result<Vec<u8>, RenderError> {
    let failed = |detail: String| RenderError::Readback { detail };
    let (sender, receiver) = std::sync::mpsc::channel();
    staging.map_async(wgpu::MapMode::Read, .., move |result| {
        // The receiver only goes away if polling already failed.
        sender.send(result).ok();
    });
    device
        .poll(wgpu::PollType::wait_indefinitely())
        .map_err(|err| failed(err.to_string()))?;
    receiver
        .recv()
        .map_err(|err| failed(err.to_string()))?
        .map_err(|err| failed(err.to_string()))?;
    let bytes = staging
        .get_mapped_range(..)
        .map_err(|err| failed(err.to_string()))?
        .to_vec();
    staging.unmap();
    Ok(bytes)
}
