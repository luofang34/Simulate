//! GPU side of the ownship shadow: the shared shadow map, the `@group(1)` layouts that
//! expose it, and the depth-only caster and ground-receiver pipelines.
//!
//! Slots render one after another, so one map serves them all: each slot's node fills it
//! from that slot's light volume just before drawing the slot.

use std::num::NonZeroU64;

use super::{SHADOW_BYTES, SHADOW_MAP_SIZE};
use crate::usd_pass::{sky::SKY_BYTES, vertex};

const MAP_FORMAT: wgpu::TextureFormat = wgpu::TextureFormat::Depth32Float;

/// Colour and depth formats and sample count of the slot's main attachments.
#[derive(Clone, Copy)]
pub(crate) struct Attachments {
    pub(crate) color: wgpu::TextureFormat,
    pub(crate) depth: wgpu::TextureFormat,
    pub(crate) samples: u32,
}

/// The shadow map and everything that writes or reads it.
pub(crate) struct ShadowGpu {
    pub(crate) map: wgpu::TextureView,
    sampler: wgpu::Sampler,
    caster_layout: wgpu::BindGroupLayout,
    /// `@group(1)` of every pipeline drawing into the slot: block, map, comparison sampler.
    pub(crate) receiver_layout: wgpu::BindGroupLayout,
    pub(crate) caster: wgpu::RenderPipeline,
    pub(crate) ground: wgpu::RenderPipeline,
}

/// One slot's `Shadow` block and its two bind groups.
pub(crate) struct SlotShadow {
    pub(crate) block: wgpu::Buffer,
    /// The slot's `Sky` block (`usd_pass::sky`), read through the receiver group.
    pub(crate) sky: wgpu::Buffer,
    pub(crate) caster_group: wgpu::BindGroup,
    pub(crate) receiver_group: wgpu::BindGroup,
}

impl ShadowGpu {
    pub(crate) fn new(
        device: &wgpu::Device,
        draw_layout: &wgpu::BindGroupLayout,
        module: &wgpu::ShaderModule,
        attachments: Attachments,
    ) -> Self {
        let map = device
            .create_texture(&wgpu::TextureDescriptor {
                label: Some("usd aircraft shadow map"),
                size: wgpu::Extent3d {
                    width: SHADOW_MAP_SIZE,
                    height: SHADOW_MAP_SIZE,
                    depth_or_array_layers: 1,
                },
                mip_level_count: 1,
                sample_count: 1,
                dimension: wgpu::TextureDimension::D2,
                format: MAP_FORMAT,
                usage: wgpu::TextureUsages::RENDER_ATTACHMENT
                    | wgpu::TextureUsages::TEXTURE_BINDING,
                view_formats: &[],
            })
            .create_view(&wgpu::TextureViewDescriptor::default());
        let sampler = device.create_sampler(&wgpu::SamplerDescriptor {
            label: Some("usd aircraft shadow compare"),
            mag_filter: wgpu::FilterMode::Linear,
            min_filter: wgpu::FilterMode::Linear,
            compare: Some(wgpu::CompareFunction::LessEqual),
            ..Default::default()
        });
        let block_entry = wgpu::BindGroupLayoutEntry {
            binding: 0,
            visibility: wgpu::ShaderStages::VERTEX_FRAGMENT,
            ty: wgpu::BindingType::Buffer {
                ty: wgpu::BufferBindingType::Uniform,
                has_dynamic_offset: false,
                min_binding_size: NonZeroU64::new(SHADOW_BYTES as u64),
            },
            count: None,
        };
        let caster_layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("usd aircraft shadow caster"),
            entries: &[block_entry],
        });
        let receiver_layout = receiver_layout(device, block_entry);
        let caster = caster_pipeline(device, draw_layout, &caster_layout, module);
        let ground = ground_pipeline(device, draw_layout, &receiver_layout, module, attachments);
        Self {
            map,
            sampler,
            caster_layout,
            receiver_layout,
            caster,
            ground,
        }
    }

    pub(crate) fn slot(&self, device: &wgpu::Device) -> SlotShadow {
        let sky = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("usd aircraft slot sky"),
            size: SKY_BYTES as u64,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        let block = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("usd aircraft slot shadow"),
            size: SHADOW_BYTES as u64,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        let caster_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("usd aircraft shadow caster"),
            layout: &self.caster_layout,
            entries: &[wgpu::BindGroupEntry {
                binding: 0,
                resource: block.as_entire_binding(),
            }],
        });
        let receiver_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("usd aircraft shadow receiver"),
            layout: &self.receiver_layout,
            entries: &[
                wgpu::BindGroupEntry {
                    binding: 0,
                    resource: block.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 1,
                    resource: wgpu::BindingResource::TextureView(&self.map),
                },
                wgpu::BindGroupEntry {
                    binding: 2,
                    resource: wgpu::BindingResource::Sampler(&self.sampler),
                },
                wgpu::BindGroupEntry {
                    binding: 3,
                    resource: sky.as_entire_binding(),
                },
            ],
        });
        SlotShadow {
            block,
            sky,
            caster_group,
            receiver_group,
        }
    }
}

/// `@group(1)` of the slot's pipelines: shadow block, shadow map, comparison sampler, sky.
fn receiver_layout(
    device: &wgpu::Device,
    block_entry: wgpu::BindGroupLayoutEntry,
) -> wgpu::BindGroupLayout {
    device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
        label: Some("usd aircraft shadow receiver"),
        entries: &[
            block_entry,
            wgpu::BindGroupLayoutEntry {
                binding: 1,
                visibility: wgpu::ShaderStages::FRAGMENT,
                ty: wgpu::BindingType::Texture {
                    sample_type: wgpu::TextureSampleType::Depth,
                    view_dimension: wgpu::TextureViewDimension::D2,
                    multisampled: false,
                },
                count: None,
            },
            wgpu::BindGroupLayoutEntry {
                binding: 2,
                visibility: wgpu::ShaderStages::FRAGMENT,
                ty: wgpu::BindingType::Sampler(wgpu::SamplerBindingType::Comparison),
                count: None,
            },
            wgpu::BindGroupLayoutEntry {
                binding: 3,
                visibility: wgpu::ShaderStages::FRAGMENT,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Uniform,
                    has_dynamic_offset: false,
                    min_binding_size: NonZeroU64::new(SKY_BYTES as u64),
                },
                count: None,
            },
        ],
    })
}

/// Opens the depth-only pass that refills the shadow map.
pub(crate) fn begin_caster_pass<'e>(
    encoder: &'e mut wgpu::CommandEncoder,
    map: &'e wgpu::TextureView,
) -> wgpu::RenderPass<'e> {
    encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
        label: Some("usd aircraft shadow casters"),
        color_attachments: &[],
        depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachment {
            view: map,
            depth_ops: Some(wgpu::Operations {
                load: wgpu::LoadOp::Clear(1.0),
                store: wgpu::StoreOp::Store,
            }),
            stencil_ops: None,
        }),
        timestamp_writes: None,
        occlusion_query_set: None,
        multiview_mask: None,
    })
}

fn caster_pipeline(
    device: &wgpu::Device,
    draw_layout: &wgpu::BindGroupLayout,
    caster_layout: &wgpu::BindGroupLayout,
    module: &wgpu::ShaderModule,
) -> wgpu::RenderPipeline {
    let layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
        label: Some("usd aircraft shadow caster"),
        bind_group_layouts: &[Some(draw_layout), Some(caster_layout)],
        immediate_size: 0,
    });
    device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
        label: Some("usd aircraft shadow caster"),
        layout: Some(&layout),
        vertex: wgpu::VertexState {
            module,
            entry_point: Some("vs_shadow"),
            compilation_options: Default::default(),
            buffers: &[Some(vertex::vertex_layout())],
        },
        fragment: None,
        // Skins are open sheets, so both faces cast.
        primitive: wgpu::PrimitiveState::default(),
        depth_stencil: Some(wgpu::DepthStencilState {
            format: MAP_FORMAT,
            depth_write_enabled: Some(true),
            depth_compare: Some(wgpu::CompareFunction::Less),
            stencil: wgpu::StencilState::default(),
            bias: wgpu::DepthBiasState {
                constant: 2,
                slope_scale: 1.5,
                clamp: 0.0,
            },
        }),
        multisample: wgpu::MultisampleState::default(),
        multiview_mask: None,
        cache: None,
    })
}

fn ground_pipeline(
    device: &wgpu::Device,
    draw_layout: &wgpu::BindGroupLayout,
    receiver_layout: &wgpu::BindGroupLayout,
    module: &wgpu::ShaderModule,
    attachments: Attachments,
) -> wgpu::RenderPipeline {
    let layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
        label: Some("usd aircraft ground shadow"),
        bind_group_layouts: &[Some(draw_layout), Some(receiver_layout)],
        immediate_size: 0,
    });
    device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
        label: Some("usd aircraft ground shadow"),
        layout: Some(&layout),
        vertex: wgpu::VertexState {
            module,
            entry_point: Some("vs_ground"),
            compilation_options: Default::default(),
            buffers: &[],
        },
        fragment: Some(wgpu::FragmentState {
            module,
            entry_point: Some("fs_ground"),
            compilation_options: Default::default(),
            targets: &[Some(wgpu::ColorTargetState {
                format: attachments.color,
                blend: Some(wgpu::BlendState::PREMULTIPLIED_ALPHA_BLENDING),
                write_mask: wgpu::ColorWrites::ALL,
            })],
        }),
        primitive: wgpu::PrimitiveState {
            topology: wgpu::PrimitiveTopology::TriangleStrip,
            ..wgpu::PrimitiveState::default()
        },
        // Tested against the terrain already drawn, never written: the airframe drawn
        // next covers the shadow wherever it stands in front.
        depth_stencil: Some(wgpu::DepthStencilState {
            format: attachments.depth,
            depth_write_enabled: Some(false),
            depth_compare: Some(wgpu::CompareFunction::GreaterEqual),
            stencil: wgpu::StencilState::default(),
            bias: wgpu::DepthBiasState::default(),
        }),
        multisample: wgpu::MultisampleState {
            count: attachments.samples,
            ..Default::default()
        },
        multiview_mask: None,
        cache: None,
    })
}
