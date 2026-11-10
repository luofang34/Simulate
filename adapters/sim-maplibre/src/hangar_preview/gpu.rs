//! Render targets, pipelines, and buffers of the hangar preview.

use std::num::NonZeroU64;

use wgpu::util::DeviceExt;

use super::orbit::FRAME_BYTES;
use crate::{
    MapAdapterError, UsdMeshVertex,
    usd_pass::{AssetTexturesGpu, DRAW_BYTES, DRAW_STRIDE, vertex},
};

pub(crate) const SHADER: &str = concat!(
    include_str!("../usd_pass/shading.wgsl"),
    include_str!("studio.wgsl")
);
/// Multisample count of the colour and depth attachments.
pub(super) const SAMPLES: u32 = 4;
/// Resolved, compositable output format.
pub(super) const COLOR_FORMAT: wgpu::TextureFormat = wgpu::TextureFormat::Rgba8UnormSrgb;
const DEPTH_FORMAT: wgpu::TextureFormat = wgpu::TextureFormat::Depth32Float;

/// MSAA colour and depth attachments and the single-sample resolve target.
pub(super) struct Targets {
    pub(super) size: [u32; 2],
    pub(super) msaa: wgpu::TextureView,
    pub(super) depth: wgpu::TextureView,
    pub(super) resolved: wgpu::Texture,
    pub(super) resolved_view: wgpu::TextureView,
}

impl Targets {
    pub(super) fn new(device: &wgpu::Device, size: [u32; 2]) -> Result<Self, MapAdapterError> {
        let [width, height] = size;
        let max = device.limits().max_texture_dimension_2d;
        if width == 0 || height == 0 || width > max || height > max {
            return Err(MapAdapterError::InvalidViewportSize { width, height });
        }
        let texture = |label, format, samples, usage| {
            device.create_texture(&wgpu::TextureDescriptor {
                label: Some(label),
                size: wgpu::Extent3d {
                    width,
                    height,
                    depth_or_array_layers: 1,
                },
                mip_level_count: 1,
                sample_count: samples,
                dimension: wgpu::TextureDimension::D2,
                format,
                usage,
                view_formats: &[],
            })
        };
        let attachment = wgpu::TextureUsages::RENDER_ATTACHMENT;
        let msaa = texture("hangar msaa colour", COLOR_FORMAT, SAMPLES, attachment);
        let depth = texture("hangar msaa depth", DEPTH_FORMAT, SAMPLES, attachment);
        let resolved = texture(
            "hangar preview",
            COLOR_FORMAT,
            1,
            attachment | wgpu::TextureUsages::TEXTURE_BINDING | wgpu::TextureUsages::COPY_SRC,
        );
        let view = |t: &wgpu::Texture| t.create_view(&wgpu::TextureViewDescriptor::default());
        Ok(Self {
            size,
            msaa: view(&msaa),
            depth: view(&depth),
            resolved_view: view(&resolved),
            resolved,
        })
    }
}

/// Which pipeline a draw uses.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) enum PipelineKind {
    Opaque,
    OpaqueDoubleSided,
    Translucent,
}

/// Bind-group layouts and the four pipelines of the studio shader.
pub(super) struct Pipelines {
    pub(super) layout: wgpu::BindGroupLayout,
    /// The previewed asset's textures, bound as group 1.
    pub(super) textures: AssetTexturesGpu,
    pub(super) backdrop: wgpu::RenderPipeline,
    opaque: wgpu::RenderPipeline,
    opaque_double_sided: wgpu::RenderPipeline,
    translucent: wgpu::RenderPipeline,
}

impl Pipelines {
    pub(super) fn new(device: &wgpu::Device) -> Self {
        let uniform = |binding, dynamic, size: usize| wgpu::BindGroupLayoutEntry {
            binding,
            visibility: wgpu::ShaderStages::VERTEX_FRAGMENT,
            ty: wgpu::BindingType::Buffer {
                ty: wgpu::BufferBindingType::Uniform,
                has_dynamic_offset: dynamic,
                min_binding_size: NonZeroU64::new(size as u64),
            },
            count: None,
        };
        let layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("hangar studio"),
            entries: &[uniform(0, false, FRAME_BYTES), uniform(1, true, DRAW_BYTES)],
        });
        let textures = AssetTexturesGpu::new(device);
        let pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("hangar studio"),
            bind_group_layouts: &[Some(&layout), Some(&textures.layout)],
            immediate_size: 0,
        });
        let module = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("hangar studio"),
            source: wgpu::ShaderSource::Wgsl(SHADER.into()),
        });
        let build = |kind| build_part_pipeline(device, &pipeline_layout, &module, kind);
        Self {
            backdrop: build_backdrop_pipeline(device, &pipeline_layout, &module),
            opaque: build(PipelineKind::Opaque),
            opaque_double_sided: build(PipelineKind::OpaqueDoubleSided),
            translucent: build(PipelineKind::Translucent),
            layout,
            textures,
        }
    }

    pub(super) fn part(&self, kind: PipelineKind) -> &wgpu::RenderPipeline {
        match kind {
            PipelineKind::Opaque => &self.opaque,
            PipelineKind::OpaqueDoubleSided => &self.opaque_double_sided,
            PipelineKind::Translucent => &self.translucent,
        }
    }
}

fn multisample() -> wgpu::MultisampleState {
    wgpu::MultisampleState {
        count: SAMPLES,
        ..Default::default()
    }
}

fn build_backdrop_pipeline(
    device: &wgpu::Device,
    layout: &wgpu::PipelineLayout,
    module: &wgpu::ShaderModule,
) -> wgpu::RenderPipeline {
    device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
        label: Some("hangar backdrop"),
        layout: Some(layout),
        vertex: wgpu::VertexState {
            module,
            entry_point: Some("vs_backdrop"),
            compilation_options: Default::default(),
            buffers: &[],
        },
        fragment: Some(wgpu::FragmentState {
            module,
            entry_point: Some("fs_backdrop"),
            compilation_options: Default::default(),
            targets: &[Some(wgpu::ColorTargetState {
                format: COLOR_FORMAT,
                blend: None,
                write_mask: wgpu::ColorWrites::ALL,
            })],
        }),
        primitive: wgpu::PrimitiveState::default(),
        depth_stencil: Some(wgpu::DepthStencilState {
            format: DEPTH_FORMAT,
            depth_write_enabled: Some(false),
            depth_compare: Some(wgpu::CompareFunction::Always),
            stencil: wgpu::StencilState::default(),
            bias: wgpu::DepthBiasState::default(),
        }),
        multisample: multisample(),
        multiview_mask: None,
        cache: None,
    })
}

fn build_part_pipeline(
    device: &wgpu::Device,
    layout: &wgpu::PipelineLayout,
    module: &wgpu::ShaderModule,
    kind: PipelineKind,
) -> wgpu::RenderPipeline {
    let translucent = kind == PipelineKind::Translucent;
    device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
        label: Some("hangar aircraft"),
        layout: Some(layout),
        vertex: wgpu::VertexState {
            module,
            entry_point: Some("vs"),
            compilation_options: Default::default(),
            buffers: &[Some(vertex::vertex_layout())],
        },
        fragment: Some(wgpu::FragmentState {
            module,
            entry_point: Some(if translucent {
                "fs_translucent"
            } else {
                "fs_opaque"
            }),
            compilation_options: Default::default(),
            targets: &[Some(wgpu::ColorTargetState {
                format: COLOR_FORMAT,
                blend: translucent.then_some(wgpu::BlendState::PREMULTIPLIED_ALPHA_BLENDING),
                write_mask: wgpu::ColorWrites::ALL,
            })],
        }),
        primitive: wgpu::PrimitiveState {
            front_face: wgpu::FrontFace::Ccw,
            cull_mode: (kind == PipelineKind::Opaque).then_some(wgpu::Face::Back),
            ..wgpu::PrimitiveState::default()
        },
        depth_stencil: Some(wgpu::DepthStencilState {
            format: DEPTH_FORMAT,
            depth_write_enabled: Some(!translucent),
            depth_compare: Some(wgpu::CompareFunction::GreaterEqual),
            stencil: wgpu::StencilState::default(),
            bias: wgpu::DepthBiasState::default(),
        }),
        multisample: multisample(),
        multiview_mask: None,
        cache: None,
    })
}

/// The frame block and a growable, dynamically offset array of draw blocks.
pub(super) struct Uniforms {
    pub(super) frame: wgpu::Buffer,
    pub(super) draws: wgpu::Buffer,
    capacity: u64,
    pub(super) bind_group: wgpu::BindGroup,
}

impl Uniforms {
    pub(super) fn new(device: &wgpu::Device, layout: &wgpu::BindGroupLayout, draws: usize) -> Self {
        let needed = (draws as u64).max(1).saturating_mul(DRAW_STRIDE);
        let capacity = needed.next_power_of_two().max(DRAW_STRIDE * 32);
        let buffer = |label, size| {
            device.create_buffer(&wgpu::BufferDescriptor {
                label: Some(label),
                size,
                usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
                mapped_at_creation: false,
            })
        };
        let frame = buffer("hangar frame", FRAME_BYTES as u64);
        let draw_buffer = buffer("hangar draws", capacity);
        let bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("hangar studio"),
            layout,
            entries: &[
                wgpu::BindGroupEntry {
                    binding: 0,
                    resource: frame.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 1,
                    resource: wgpu::BindingResource::Buffer(wgpu::BufferBinding {
                        buffer: &draw_buffer,
                        offset: 0,
                        size: NonZeroU64::new(DRAW_BYTES as u64),
                    }),
                },
            ],
        });
        Self {
            frame,
            draws: draw_buffer,
            capacity,
            bind_group,
        }
    }

    pub(super) fn holds(&self, draws: usize) -> bool {
        (draws as u64).saturating_mul(DRAW_STRIDE) <= self.capacity
    }
}

/// One part's static vertex and index buffers.
pub(super) struct PartMesh {
    pub(super) vertices: wgpu::Buffer,
    pub(super) indices: wgpu::Buffer,
    pub(super) count: u32,
}

pub(super) fn upload_mesh(
    device: &wgpu::Device,
    vertices: &[UsdMeshVertex],
    indices: &[u32],
) -> Option<PartMesh> {
    let (index_bytes, count) = vertex::index_bytes(indices)?;
    let buffer = |label, contents: &[u8], usage| {
        device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some(label),
            contents,
            usage,
        })
    };
    Some(PartMesh {
        vertices: buffer(
            "hangar part vertices",
            &vertex::vertex_bytes(vertices),
            wgpu::BufferUsages::VERTEX,
        ),
        indices: buffer(
            "hangar part indices",
            &index_bytes,
            wgpu::BufferUsages::INDEX,
        ),
        count,
    })
}
