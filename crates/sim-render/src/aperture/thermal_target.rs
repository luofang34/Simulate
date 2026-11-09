use super::microbolometer::SceneTemperature;
use super::target::{
    copy_to_staging, read_staging_blocking, readback_row, staging_buffer, strip_row_padding,
    target_texture,
};
use super::{APERTURE_DEPTH_FORMAT, PinholeProjection};
use crate::RenderError;

#[cfg(test)]
mod tests;

/// Format of a [`ThermalSceneTarget`]: one `f32` of brightness temperature
/// per pixel.
pub const THERMAL_KELVIN_FORMAT: wgpu::TextureFormat = wgpu::TextureFormat::R32Float;

/// What the kelvin shader writes, and the target clears to, where there is
/// no radiometric ground: open sky, or ground without radiometry.
const NO_RADIOMETRY_K: f64 = -1.0;

/// Bytes of one `R32Float` texel.
const KELVIN_BYTES_PER_PIXEL: usize = 4;

/// An offscreen target that holds the scene's brightness temperature in
/// full `f32` precision, for a [`Microbolometer`](super::Microbolometer) to
/// sense without the eight-bit quantisation of a
/// [`ThermalLwir`](super::ApertureWaveband::ThermalLwir) frame.
///
/// Only terrain is drawn: window bezels and propellers are not, so the
/// readback is the scene's truth before any airframe obstruction.
pub struct ThermalSceneTarget {
    projection: PinholeProjection,
    kelvin: wgpu::Texture,
    kelvin_view: wgpu::TextureView,
    depth_view: wgpu::TextureView,
    staging: wgpu::Buffer,
}

impl ThermalSceneTarget {
    /// Creates a target sized by `projection`'s intrinsics.
    pub fn new(device: &wgpu::Device, projection: PinholeProjection) -> Result<Self, RenderError> {
        let intrinsics = projection.intrinsics();
        let (width, height) = (intrinsics.width, intrinsics.height);
        let max = device.limits().max_texture_dimension_2d;
        if width > max || height > max {
            return Err(RenderError::TextureTooLarge {
                what: "thermal scene",
                width,
                height,
                max,
            });
        }
        let kelvin = target_texture(
            device,
            "thermal scene kelvin",
            width,
            height,
            THERMAL_KELVIN_FORMAT,
        );
        let depth = target_texture(
            device,
            "thermal scene depth",
            width,
            height,
            APERTURE_DEPTH_FORMAT,
        );
        Ok(Self {
            projection,
            kelvin_view: kelvin.create_view(&wgpu::TextureViewDescriptor::default()),
            kelvin,
            depth_view: depth.create_view(&wgpu::TextureViewDescriptor::default()),
            staging: staging_buffer(device, "thermal scene readback", width, height)?,
        })
    }

    /// The camera this target belongs to.
    pub fn projection(&self) -> &PinholeProjection {
        &self.projection
    }

    /// Copies the last rendered scene to the CPU. Pixels with no radiometric
    /// ground read `sky_k`, the clear-sky brightness temperature the caller
    /// chooses for the band and the view; this renderer has no sky model.
    pub fn read_blocking(
        &self,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        sky_k: f32,
    ) -> Result<SceneTemperature, RenderError> {
        let intrinsics = self.projection.intrinsics();
        let (width, height) = (intrinsics.width, intrinsics.height);
        let padded_row = readback_row(width)?;
        let mut encoder = device.create_command_encoder(&wgpu::CommandEncoderDescriptor {
            label: Some("thermal scene readback"),
        });
        copy_to_staging(
            &mut encoder,
            &self.kelvin,
            wgpu::TextureAspect::All,
            &self.staging,
            padded_row,
        );
        queue.submit([encoder.finish()]);
        let padded = read_staging_blocking(device, &self.staging)?;
        let tight = strip_row_padding(
            &padded,
            padded_row as usize,
            width as usize * KELVIN_BYTES_PER_PIXEL,
        );
        let raw: Vec<f32> = bytemuck::pod_collect_to_vec(&tight);
        Ok(SceneTemperature {
            width,
            height,
            kelvin: raw.into_iter().map(|k| sky_where_empty(k, sky_k)).collect(),
        })
    }

    /// Starts a pass that clears to "no radiometry" and the far plane.
    pub(super) fn begin_pass<'a>(
        &self,
        encoder: &'a mut wgpu::CommandEncoder,
    ) -> wgpu::RenderPass<'a> {
        encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
            label: Some("thermal scene"),
            color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                view: &self.kelvin_view,
                depth_slice: None,
                resolve_target: None,
                ops: wgpu::Operations {
                    load: wgpu::LoadOp::Clear(wgpu::Color {
                        r: NO_RADIOMETRY_K,
                        g: 0.0,
                        b: 0.0,
                        a: 1.0,
                    }),
                    store: wgpu::StoreOp::Store,
                },
            })],
            depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachment {
                view: &self.depth_view,
                depth_ops: Some(wgpu::Operations {
                    load: wgpu::LoadOp::Clear(0.0),
                    store: wgpu::StoreOp::Store,
                }),
                stencil_ops: None,
            }),
            timestamp_writes: None,
            occlusion_query_set: None,
            multiview_mask: None,
        })
    }
}

/// A rendered kelvin, or `sky_k` where the shader found no radiometry.
fn sky_where_empty(kelvin: f32, sky_k: f32) -> f32 {
    if kelvin.is_finite() && kelvin >= 0.0 {
        kelvin
    } else {
        sky_k
    }
}
