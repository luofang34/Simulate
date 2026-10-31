use glam::DVec3;

use crate::frame::FrameParams;
use crate::gpu::SurfaceGpu;
use crate::pipeline::{GlobeLayouts, globe_pipeline};
use crate::scene::GlobeScene;
use crate::{Camera, FrameOutcome, GlobeMesh, MAX_PATCHES, OrthoAtlas, RenderError};

/// Globe tessellation: 12 × 24 patches of 15° × 15°, each a 16 × 16 grid.
const GLOBE_LAT_PATCHES: u32 = 12;
const GLOBE_LON_PATCHES: u32 = 24;
const GLOBE_PATCH_GRID: u32 = 16;

/// Lighting inputs for one frame.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SceneLighting {
    /// Direction from the body toward the Sun, body-fixed (need not be
    /// normalised).
    pub sun_direction_ecef: DVec3,
}

/// Owns the GPU, the globe geometry, and the per-frame uniform block.
pub struct Renderer {
    gpu: SurfaceGpu,
    layouts: GlobeLayouts,
    pipeline: wgpu::RenderPipeline,
    scene: GlobeScene,
}

impl Renderer {
    /// Creates a renderer presenting to `target` at `width × height` pixels.
    pub async fn new(
        target: impl Into<wgpu::SurfaceTarget<'static>>,
        width: u32,
        height: u32,
    ) -> Result<Self, RenderError> {
        let gpu = SurfaceGpu::new(target.into(), width, height).await?;
        let requested = (GLOBE_LAT_PATCHES * GLOBE_LON_PATCHES) as usize;
        let mesh = GlobeMesh::build(GLOBE_LAT_PATCHES, GLOBE_LON_PATCHES, GLOBE_PATCH_GRID).ok_or(
            RenderError::TooManyPatches {
                requested,
                max: MAX_PATCHES,
            },
        )?;
        let device = &gpu.device;
        let layouts = GlobeLayouts::new(device);
        let scene = GlobeScene::new(device, &gpu.queue, &layouts, &mesh, None)?;
        let scope = device.push_error_scope(wgpu::ErrorFilter::Validation);
        let pipeline = globe_pipeline(device, &layouts, gpu.view_format, "fs_main");
        if let Some(err) = scope.pop().await {
            return Err(RenderError::PipelineValidation(err));
        }
        Ok(Self {
            gpu,
            layouts,
            pipeline,
            scene,
        })
    }

    /// Replaces the drawn terrain and its imagery.
    pub fn set_scene(
        &mut self,
        mesh: &GlobeMesh,
        atlas: Option<&OrthoAtlas>,
    ) -> Result<(), RenderError> {
        self.scene = GlobeScene::new(
            &self.gpu.device,
            &self.gpu.queue,
            &self.layouts,
            mesh,
            atlas,
        )?;
        Ok(())
    }

    /// The device, for sharing with a [`crate::SensorAperturePass`].
    pub fn device(&self) -> &wgpu::Device {
        &self.gpu.device
    }

    /// The queue, for sharing with a [`crate::SensorAperturePass`].
    pub fn queue(&self) -> &wgpu::Queue {
        &self.gpu.queue
    }

    /// Resizes the swap chain and depth buffer; zero sizes are ignored.
    pub fn resize(&mut self, width: u32, height: u32) {
        self.gpu.resize(width, height);
    }

    /// Draws and presents one frame.
    pub fn render(
        &mut self,
        camera: &Camera,
        lighting: &SceneLighting,
    ) -> Result<FrameOutcome, RenderError> {
        if let Some(message) = self.gpu.take_device_error() {
            return Err(RenderError::Device { message });
        }
        let surface_texture = match self.gpu.surface.get_current_texture() {
            wgpu::CurrentSurfaceTexture::Success(texture)
            | wgpu::CurrentSurfaceTexture::Suboptimal(texture) => texture,
            wgpu::CurrentSurfaceTexture::Timeout | wgpu::CurrentSurfaceTexture::Occluded => {
                return Ok(FrameOutcome::Skipped);
            }
            wgpu::CurrentSurfaceTexture::Outdated | wgpu::CurrentSurfaceTexture::Lost => {
                self.gpu.reconfigure();
                return Ok(FrameOutcome::Skipped);
            }
            wgpu::CurrentSurfaceTexture::Validation => return Err(RenderError::SurfaceValidation),
        };
        let params = FrameParams {
            view_proj: camera.view_projection(self.gpu.aspect()),
            sun_direction: lighting.sun_direction_ecef,
            thermal: [0.0; 4],
        };
        self.scene
            .write_frame(&self.gpu.queue, &params, camera.pose.position_ecef);
        let view = surface_texture
            .texture
            .create_view(&wgpu::TextureViewDescriptor {
                format: Some(self.gpu.view_format),
                ..Default::default()
            });
        let mut encoder = self
            .gpu
            .device
            .create_command_encoder(&wgpu::CommandEncoderDescriptor {
                label: Some("frame"),
            });
        self.encode_globe_pass(&mut encoder, &view);
        self.gpu.queue.submit([encoder.finish()]);
        self.gpu.queue.present(surface_texture);
        Ok(FrameOutcome::Presented)
    }

    fn encode_globe_pass(&self, encoder: &mut wgpu::CommandEncoder, view: &wgpu::TextureView) {
        let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
            label: Some("globe"),
            color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                view,
                depth_slice: None,
                resolve_target: None,
                ops: wgpu::Operations {
                    load: wgpu::LoadOp::Clear(wgpu::Color {
                        r: 0.01,
                        g: 0.01,
                        b: 0.03,
                        a: 1.0,
                    }),
                    store: wgpu::StoreOp::Store,
                },
            })],
            depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachment {
                view: &self.gpu.depth_view,
                depth_ops: Some(wgpu::Operations {
                    load: wgpu::LoadOp::Clear(0.0),
                    store: wgpu::StoreOp::Store,
                }),
                stencil_ops: None,
            }),
            timestamp_writes: None,
            occlusion_query_set: None,
            multiview_mask: None,
        });
        pass.set_pipeline(&self.pipeline);
        self.scene.draw(&mut pass);
    }
}
