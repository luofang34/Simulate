use glam::DVec3;
use wgpu::util::DeviceExt;

use crate::frame::{FRAME_UNIFORM_FLOATS, FrameParams, write_frame_uniform};
use crate::pipeline::GlobeLayouts;
use crate::{GlobeMesh, MAX_PATCHES, OrthoAtlas, RenderError};

/// A transparent white pixel: no imagery, so the shader's alpha test falls
/// through to the procedural surface.
const NO_IMAGERY: [u8; 4] = [255, 255, 255, 0];

/// A globe mesh, its drape, and its frame uniform on the GPU.
pub(crate) struct GlobeScene {
    vertex_buffer: wgpu::Buffer,
    index_buffer: wgpu::Buffer,
    index_count: u32,
    frame_buffer: wgpu::Buffer,
    frame_bind_group: wgpu::BindGroup,
    ortho_bind_group: wgpu::BindGroup,
    frame_data: Box<[f32; FRAME_UNIFORM_FLOATS]>,
    patch_centres: Vec<DVec3>,
}

impl GlobeScene {
    pub(crate) fn new(
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        layouts: &GlobeLayouts,
        mesh: &GlobeMesh,
        atlas: Option<&OrthoAtlas>,
    ) -> Result<Self, RenderError> {
        let index_count =
            u32::try_from(mesh.indices.len()).map_err(|_| RenderError::TooManyPatches {
                requested: mesh.patch_centres.len(),
                max: MAX_PATCHES,
            })?;
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
        let frame_bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("frame"),
            layout: &layouts.frame,
            entries: &[wgpu::BindGroupEntry {
                binding: 0,
                resource: frame_buffer.as_entire_binding(),
            }],
        });
        let ortho_bind_group = ortho_bind_group(device, queue, layouts, atlas)?;
        Ok(Self {
            vertex_buffer,
            index_buffer,
            index_count,
            frame_buffer,
            frame_bind_group,
            ortho_bind_group,
            frame_data,
            patch_centres: mesh.patch_centres.clone(),
        })
    }

    pub(crate) fn write_frame(&mut self, queue: &wgpu::Queue, params: &FrameParams, eye: DVec3) {
        write_frame_uniform(&mut self.frame_data, params, eye, &self.patch_centres);
        queue.write_buffer(
            &self.frame_buffer,
            0,
            bytemuck::cast_slice(self.frame_data.as_slice()),
        );
    }

    pub(crate) fn draw(&self, pass: &mut wgpu::RenderPass<'_>) {
        pass.set_bind_group(0, &self.frame_bind_group, &[]);
        pass.set_bind_group(1, &self.ortho_bind_group, &[]);
        pass.set_vertex_buffer(0, self.vertex_buffer.slice(..));
        pass.set_index_buffer(self.index_buffer.slice(..), wgpu::IndexFormat::Uint32);
        pass.draw_indexed(0..self.index_count, 0, 0..1);
    }
}

fn ortho_bind_group(
    device: &wgpu::Device,
    queue: &wgpu::Queue,
    layouts: &GlobeLayouts,
    atlas: Option<&OrthoAtlas>,
) -> Result<wgpu::BindGroup, RenderError> {
    let (width, height, pixels) = match atlas {
        Some(atlas) => (atlas.width, atlas.height, atlas.rgba8.as_slice()),
        None => (1, 1, NO_IMAGERY.as_slice()),
    };
    let max = device.limits().max_texture_dimension_2d;
    if width > max || height > max {
        return Err(RenderError::TextureTooLarge {
            what: "ortho atlas",
            width,
            height,
            max,
        });
    }
    let size = wgpu::Extent3d {
        width,
        height,
        depth_or_array_layers: 1,
    };
    let texture = device.create_texture(&wgpu::TextureDescriptor {
        label: Some("ortho atlas"),
        size,
        mip_level_count: 1,
        sample_count: 1,
        dimension: wgpu::TextureDimension::D2,
        format: wgpu::TextureFormat::Rgba8UnormSrgb,
        usage: wgpu::TextureUsages::TEXTURE_BINDING | wgpu::TextureUsages::COPY_DST,
        view_formats: &[],
    });
    queue.write_texture(
        texture.as_image_copy(),
        pixels,
        wgpu::TexelCopyBufferLayout {
            offset: 0,
            bytes_per_row: Some(width * 4),
            rows_per_image: Some(height),
        },
        size,
    );
    let view = texture.create_view(&wgpu::TextureViewDescriptor::default());
    let sampler = device.create_sampler(&wgpu::SamplerDescriptor {
        label: Some("ortho"),
        mag_filter: wgpu::FilterMode::Linear,
        min_filter: wgpu::FilterMode::Linear,
        ..Default::default()
    });
    Ok(device.create_bind_group(&wgpu::BindGroupDescriptor {
        label: Some("ortho"),
        layout: &layouts.ortho,
        entries: &[
            wgpu::BindGroupEntry {
                binding: 0,
                resource: wgpu::BindingResource::TextureView(&view),
            },
            wgpu::BindGroupEntry {
                binding: 1,
                resource: wgpu::BindingResource::Sampler(&sampler),
            },
        ],
    }))
}
