//! Offscreen studio "hangar" preview of one articulated OpenUSD vehicle for menus.
//!
//! Renders into a 4× MSAA colour + reversed-Z `Depth32Float` pair and resolves to a
//! single-sample `Rgba8UnormSrgb` texture an app can composite as an opaque panel. Parts
//! articulate through [`UsdAircraftAsset::evaluate_draws`], the same joint, propeller, and
//! beacon mechanism the in-world aircraft pass uses, so control surfaces deflect and
//! rotors spin exactly as they do in flight.

use sim_core::AirframeArticulation;

use crate::{MapAdapterError, UsdAircraftAsset, usd_pass::DRAW_STRIDE};

mod gpu;
mod orbit;
mod scene;
#[cfg(test)]
mod tests;

pub use orbit::{HANGAR_AUTO_ROTATE_RAD_PER_S, HangarOrbit};

/// Default preview size in physical pixels.
pub const HANGAR_DEFAULT_SIZE: [u32; 2] = [720, 440];

/// Offscreen hangar renderer: studio backdrop, floor, turntable, and one vehicle.
pub struct HangarPreviewPass {
    targets: gpu::Targets,
    pipelines: gpu::Pipelines,
    uniforms: gpu::Uniforms,
    scene: Option<scene::SceneAsset>,
}

impl HangarPreviewPass {
    /// Creates the pipelines and `size` render targets (see [`HANGAR_DEFAULT_SIZE`]).
    ///
    /// # Errors
    /// [`MapAdapterError::InvalidViewportSize`] when a dimension is zero or exceeds the
    /// device's 2D texture limit.
    pub fn new(device: &wgpu::Device, size: [u32; 2]) -> Result<Self, MapAdapterError> {
        let targets = gpu::Targets::new(device, size)?;
        let pipelines = gpu::Pipelines::new(device);
        let uniforms = gpu::Uniforms::new(device, &pipelines.layout, 0);
        Ok(Self {
            targets,
            pipelines,
            uniforms,
            scene: None,
        })
    }

    /// Recreates the render targets when `size` differs from the current size; views and
    /// textures fetched earlier then refer to the old targets and must be fetched again.
    ///
    /// # Errors
    /// [`MapAdapterError::InvalidViewportSize`] for a zero or over-limit size; the previous
    /// targets stay in place.
    pub fn ensure_size(
        &mut self,
        device: &wgpu::Device,
        size: [u32; 2],
    ) -> Result<(), MapAdapterError> {
        if self.targets.size != size {
            self.targets = gpu::Targets::new(device, size)?;
        }
        Ok(())
    }

    /// Uploads / caches GPU meshes for this asset (cheap no-op when already current).
    ///
    /// # Errors
    /// [`MapAdapterError::AircraftPass`] when the asset has no drawable part.
    pub fn set_asset(
        &mut self,
        device: &wgpu::Device,
        asset: &UsdAircraftAsset,
    ) -> Result<(), MapAdapterError> {
        let fingerprint = scene::fingerprint(asset);
        if self
            .scene
            .as_ref()
            .is_some_and(|current| current.fingerprint == fingerprint)
        {
            return Ok(());
        }
        let uploaded = scene::SceneAsset::upload(device, asset);
        if uploaded.meshes.iter().all(Option::is_none) {
            return Err(MapAdapterError::AircraftPass {
                detail: format!(
                    "hangar asset with {} parts has no drawable triangles",
                    asset.parts().len()
                ),
            });
        }
        tracing::debug!(
            parts = asset.parts().len(),
            triangles = asset.triangle_count(),
            "hangar preview asset uploaded"
        );
        self.scene = Some(uploaded);
        Ok(())
    }

    /// Records one preview frame into `encoder`: the studio backdrop and, once
    /// [`HangarPreviewPass::set_asset`] has run, the vehicle posed by `articulation`.
    ///
    /// Uniforms are written through `queue`, so render at most once per submitted encoder.
    ///
    /// # Errors
    /// [`MapAdapterError::AircraftPass`] when a draw's uniform offset overflows `u32`.
    pub fn render(
        &mut self,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        encoder: &mut wgpu::CommandEncoder,
        articulation: &AirframeArticulation,
        orbit: &HangarOrbit,
    ) -> Result<(), MapAdapterError> {
        let [width, height] = self.targets.size;
        let bounds = self.scene.as_ref().map(|s| s.bounds).unwrap_or_default();
        let rig = orbit::StudioRig::frame(orbit, bounds, f64::from(width) / f64::from(height));
        let draws = self
            .scene
            .as_ref()
            .map(|s| scene::pack_draws(&s.asset, articulation, &rig))
            .unwrap_or_default();
        if !self.uniforms.holds(draws.len()) {
            self.uniforms = gpu::Uniforms::new(device, &self.pipelines.layout, draws.len());
        }
        queue.write_buffer(&self.uniforms.frame, 0, &rig.frame_bytes());
        let stride = DRAW_STRIDE as usize;
        let mut bytes = vec![0_u8; draws.len().max(1).saturating_mul(stride)];
        for (chunk, draw) in bytes.chunks_mut(stride).zip(&draws) {
            if let Some(dst) = chunk.get_mut(..draw.uniform.len()) {
                dst.copy_from_slice(&draw.uniform);
            }
        }
        queue.write_buffer(&self.uniforms.draws, 0, &bytes);
        self.encode(encoder, &draws)
    }

    fn encode(
        &self,
        encoder: &mut wgpu::CommandEncoder,
        draws: &[scene::HangarDraw],
    ) -> Result<(), MapAdapterError> {
        let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
            label: Some("hangar preview"),
            color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                view: &self.targets.msaa,
                depth_slice: None,
                resolve_target: Some(&self.targets.resolved_view),
                ops: wgpu::Operations {
                    load: wgpu::LoadOp::Clear(wgpu::Color::BLACK),
                    store: wgpu::StoreOp::Discard,
                },
            })],
            depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachment {
                view: &self.targets.depth,
                depth_ops: Some(wgpu::Operations {
                    load: wgpu::LoadOp::Clear(0.0),
                    store: wgpu::StoreOp::Discard,
                }),
                stencil_ops: None,
            }),
            timestamp_writes: None,
            occlusion_query_set: None,
            multiview_mask: None,
        });
        pass.set_pipeline(&self.pipelines.backdrop);
        pass.set_bind_group(0, &self.uniforms.bind_group, &[0]);
        pass.draw(0..3, 0..1);
        let Some(scene) = self.scene.as_ref() else {
            return Ok(());
        };
        let mut bound = None;
        for (index, draw) in draws.iter().enumerate() {
            let Some(Some(mesh)) = scene.meshes.get(draw.part_index) else {
                continue;
            };
            let offset =
                u32::try_from((index as u64).saturating_mul(DRAW_STRIDE)).map_err(|_| {
                    MapAdapterError::AircraftPass {
                        detail: format!("hangar draw {index} uniform offset overflows u32"),
                    }
                })?;
            if bound != Some(draw.pipeline) {
                pass.set_pipeline(self.pipelines.part(draw.pipeline));
                bound = Some(draw.pipeline);
            }
            pass.set_bind_group(0, &self.uniforms.bind_group, &[offset]);
            pass.set_vertex_buffer(0, mesh.vertices.slice(..));
            pass.set_index_buffer(mesh.indices.slice(..), wgpu::IndexFormat::Uint32);
            pass.draw_indexed(0..mesh.count, 0, 0..1);
        }
        Ok(())
    }

    /// Resolved single-sample `Rgba8UnormSrgb` preview, opaque, ready to composite.
    #[must_use]
    pub fn view(&self) -> &wgpu::TextureView {
        &self.targets.resolved_view
    }

    /// Resolved preview texture (`RENDER_ATTACHMENT | TEXTURE_BINDING | COPY_SRC`), for
    /// readback and screenshots.
    #[must_use]
    pub fn texture(&self) -> &wgpu::Texture {
        &self.targets.resolved
    }

    /// Current render-target size in physical pixels.
    #[must_use]
    pub fn size(&self) -> [u32; 2] {
        self.targets.size
    }
}
