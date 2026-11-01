//! Lazy on-demand CPU RGBA8 readback for offscreen [`wgpu::Texture`] attachments.
//!
//! Keeps zero persistent `MAP_READ` staging memory on registered `ViewSlotId`s during
//! normal zero-copy operation while enabling deterministic frame capture for offline CV
//! harnesses (`Navigate` XFeat / Segformer / evaluation pipelines).

use crate::MapAdapterError;

const BYTES_PER_PIXEL: u32 = 4;

pub(crate) fn read_texture_rgba_blocking(
    device: &wgpu::Device,
    queue: &wgpu::Queue,
    texture: &wgpu::Texture,
) -> Result<Vec<u8>, MapAdapterError> {
    let size = texture.size();
    let (width, height) = (size.width, size.height);
    let tight_row =
        width
            .checked_mul(BYTES_PER_PIXEL)
            .ok_or_else(|| MapAdapterError::Readback {
                detail: format!("row width {width} overflows u32"),
            })?;
    let align = wgpu::COPY_BYTES_PER_ROW_ALIGNMENT;
    let padded_row = tight_row.div_ceil(align).saturating_mul(align);
    let buffer_bytes = u64::from(padded_row).saturating_mul(u64::from(height));

    let staging = device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("sim-maplibre lazy slot readback"),
        size: buffer_bytes,
        usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
        mapped_at_creation: false,
    });
    let mut encoder = device.create_command_encoder(&wgpu::CommandEncoderDescriptor {
        label: Some("sim-maplibre lazy slot copy"),
    });
    encoder.copy_texture_to_buffer(
        wgpu::TexelCopyTextureInfo {
            texture,
            mip_level: 0,
            origin: wgpu::Origin3d::ZERO,
            aspect: wgpu::TextureAspect::All,
        },
        wgpu::TexelCopyBufferInfo {
            buffer: &staging,
            layout: wgpu::TexelCopyBufferLayout {
                offset: 0,
                bytes_per_row: Some(padded_row),
                rows_per_image: None,
            },
        },
        size,
    );
    queue.submit([encoder.finish()]);
    let raw = map_staging_blocking(device, &staging)?;

    let is_bgra = matches!(
        texture.format(),
        wgpu::TextureFormat::Bgra8Unorm | wgpu::TextureFormat::Bgra8UnormSrgb
    );
    Ok(unpad_rows(
        &raw,
        padded_row as usize,
        tight_row as usize,
        height as usize,
        is_bgra,
    ))
}

fn map_staging_blocking(
    device: &wgpu::Device,
    staging: &wgpu::Buffer,
) -> Result<Vec<u8>, MapAdapterError> {
    let (tx, rx) = std::sync::mpsc::channel();
    staging.map_async(wgpu::MapMode::Read, .., move |res| {
        tx.send(res).ok();
    });
    device
        .poll(wgpu::PollType::wait_indefinitely())
        .map_err(|err| MapAdapterError::Readback {
            detail: err.to_string(),
        })?;
    rx.recv()
        .map_err(|err| MapAdapterError::Readback {
            detail: err.to_string(),
        })?
        .map_err(|err| MapAdapterError::Readback {
            detail: err.to_string(),
        })?;
    let raw = staging
        .get_mapped_range(..)
        .map_err(|err| MapAdapterError::Readback {
            detail: err.to_string(),
        })?
        .to_vec();
    staging.unmap();
    Ok(raw)
}

fn unpad_rows(
    raw: &[u8],
    padded_row: usize,
    tight_row: usize,
    height: usize,
    swizzle_bgra: bool,
) -> Vec<u8> {
    let mut out = Vec::with_capacity(tight_row.saturating_mul(height));
    for row in raw.chunks(padded_row).take(height) {
        let valid = row.get(..tight_row).unwrap_or(row);
        if swizzle_bgra {
            for &[b, g, r, a] in valid.as_chunks::<4>().0 {
                out.extend_from_slice(&[r, g, b, a]);
            }
        } else {
            out.extend_from_slice(valid);
        }
    }
    out
}
