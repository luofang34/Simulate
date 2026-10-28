use glam::DVec3;
use wgpu::util::DeviceExt;

use crate::frame::{FRAME_UNIFORM_FLOATS, write_frame_uniform};
use crate::gpu::SurfaceGpu;
use crate::pipeline::{frame_bind_group_layout, globe_pipeline};
use crate::{Camera, FrameOutcome, GlobeMesh, MAX_PATCHES, RenderError};

/// Globe tessellation: 12 × 24 patches of 15° × 15°, each a 16 × 16 grid.
const GLOBE_LAT_PATCHES: u32 = 12;
const GLOBE_LON_PATCHES: u32 = 24;
const GLOBE_PATCH_GRID: u32 = 16;

/// Lighting inputs for one frame.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SceneLighting {
    /// Direction from the Earth toward the Sun, ECEF (need not be normalised).
    pub sun_direction_ecef: DVec3,
}

/// Owns the GPU, the globe geometry, and the per-frame uniform block.
pub struct Renderer {
    gpu: SurfaceGpu,
    pipeline: wgpu::RenderPipeline,
    vertex_buffer: wgpu::Buffer,
    index_buffer: wgpu::Buffer,
    index_count: u32,
    frame_buffer: wgpu::Buffer,
    frame_bind_group: wgpu::BindGroup,
    frame_data: Box<[f32; FRAME_UNIFORM_FLOATS]>,
    patch_centres: Vec<DVec3>,
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
        let index_count =
            u32::try_from(mesh.indices.len()).map_err(|_| RenderError::TooManyPatches {
                requested,
                max: MAX_PATCHES,
            })?;
        let device = &gpu.device;
        let vertex_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("globe vertices"),
            contents: bytemuck::cast_slice(&mesh.vertices),
            usage: wgpu::BufferUsages::VERTEX,
        });
        let index_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("globe indices"),
            contents: bytemuck::cast_slice(&mesh.indices),
            usage: wgpu::BufferUsages::INDEX,
        });
        let frame_data = Box::new([0.0_f32; FRAME_UNIFORM_FLOATS]);
        let frame_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("frame uniform"),
            contents: bytemuck::cast_slice(frame_data.as_slice()),
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
        });
        let layout = frame_bind_group_layout(device);
        let frame_bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("frame"),
            layout: &layout,
            entries: &[wgpu::BindGroupEntry {
                binding: 0,
                resource: frame_buffer.as_entire_binding(),
            }],
        });
        let scope = device.push_error_scope(wgpu::ErrorFilter::Validation);
        let pipeline = globe_pipeline(device, &layout, gpu.view_format);
        if let Some(err) = scope.pop().await {
            return Err(RenderError::PipelineValidation(err));
        }
        Ok(Self {
            gpu,
            pipeline,
            vertex_buffer,
            index_buffer,
            index_count,
            frame_buffer,
            frame_bind_group,
            frame_data,
            patch_centres: mesh.patch_centres,
        })
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
        write_frame_uniform(
            &mut self.frame_data,
            camera,
            self.gpu.aspect(),
            lighting.sun_direction_ecef,
            &self.patch_centres,
        );
        self.gpu.queue.write_buffer(
            &self.frame_buffer,
            0,
            bytemuck::cast_slice(self.frame_data.as_slice()),
        );
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
        pass.set_bind_group(0, &self.frame_bind_group, &[]);
        pass.set_vertex_buffer(0, self.vertex_buffer.slice(..));
        pass.set_index_buffer(self.index_buffer.slice(..), wgpu::IndexFormat::Uint32);
        pass.draw_indexed(0..self.index_count, 0, 0..1);
    }
}
