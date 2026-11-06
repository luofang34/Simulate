//! GPU state of the aircraft pass: three pipelines, static part meshes of every airframe
//! built so far (shared by every slot), and one dynamically offset uniform buffer per slot.

use std::{collections::BTreeMap, num::NonZeroU64};

use maplibre::{
    context::MapContext,
    headless::ViewSlotId,
    render::{Renderer, eye_covering::CurrentViewSlot, settings::Msaa, upload_queue::UploadQueue},
};

use super::{AircraftLibrary, AssetKey, DRAW_BYTES, DRAW_STRIDE, SharedAircraftLibrary, SlotDraws};
use crate::UsdAircraftAsset;

const SHADER: &str = include_str!("aircraft.wgsl");
const VERTEX_STRIDE: u64 = 32;

/// One part's static vertex and index buffers.
pub(super) struct PartMesh {
    pub(super) vertices: wgpu::Buffer,
    pub(super) indices: wgpu::Buffer,
    pub(super) count: u32,
}

/// One draw recorded for the node: which part, which pipeline, which uniform offset.
pub(super) struct DrawCommand {
    pub(super) asset: AssetKey,
    pub(super) part_index: usize,
    pub(super) pipeline: PipelineKind,
    pub(super) offset: u32,
}

/// Pipeline variants of the pass.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) enum PipelineKind {
    Opaque,
    OpaqueDoubleSided,
    Translucent,
}

/// A slot's own uniform buffer and the draws it holds this frame.
pub(super) struct SlotGpu {
    uniforms: wgpu::Buffer,
    capacity: u64,
    pub(super) bind_group: wgpu::BindGroup,
    pub(super) commands: Vec<DrawCommand>,
}

/// Pipelines, layouts, and meshes, built once the render target exists.
pub(super) struct UsdPassGpu {
    pub(super) opaque: wgpu::RenderPipeline,
    pub(super) opaque_double_sided: wgpu::RenderPipeline,
    pub(super) translucent: wgpu::RenderPipeline,
    layout: wgpu::BindGroupLayout,
    pub(super) meshes: BTreeMap<AssetKey, Vec<Option<PartMesh>>>,
    pub(super) slots: BTreeMap<ViewSlotId, SlotGpu>,
}

/// World resource holding the pass's GPU state once it is initialized.
#[derive(Default)]
pub(super) struct UsdPassState {
    pub(super) gpu: Option<UsdPassGpu>,
}

/// Prepare-stage system: builds the GPU state once, uploads airframes selected since the
/// last frame, then uploads the executing slot's draws.
pub(super) fn prepare(ctx: &mut MapContext, library: &SharedAircraftLibrary, draws: &SlotDraws) {
    let slot = ctx
        .world
        .resources
        .get::<CurrentViewSlot>()
        .and_then(|current| current.0);
    let Some(slot) = slot else {
        return;
    };
    let Renderer {
        device,
        queue,
        resources,
        settings,
        ..
    } = &ctx.renderer;
    let Some(state) = ctx.world.resources.get_mut::<UsdPassState>() else {
        return;
    };
    let gpu = state.gpu.get_or_insert_with(|| {
        let samples = if resources.surface.is_multisampling_supported(settings.msaa) {
            settings.msaa
        } else {
            Msaa { samples: 1 }
        };
        UsdPassGpu::new(
            device,
            resources.surface.surface_format(),
            settings.depth_texture_format,
            samples.samples,
        )
    });
    if let Ok(library) = library.try_borrow() {
        gpu.upload_new_assets(device, queue, &library);
    }
    let Ok(frames) = draws.try_borrow() else {
        return;
    };
    let slot_draws = frames.get(&slot).map_or(&[][..], Vec::as_slice);
    gpu.upload_slot(device, queue, slot, slot_draws);
}

impl UsdPassGpu {
    fn new(
        device: &wgpu::Device,
        format: wgpu::TextureFormat,
        depth_format: wgpu::TextureFormat,
        samples: u32,
    ) -> Self {
        let layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("usd aircraft draw"),
            entries: &[wgpu::BindGroupLayoutEntry {
                binding: 0,
                visibility: wgpu::ShaderStages::VERTEX_FRAGMENT,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Uniform,
                    has_dynamic_offset: true,
                    min_binding_size: NonZeroU64::new(DRAW_BYTES as u64),
                },
                count: None,
            }],
        });
        let pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("usd aircraft"),
            bind_group_layouts: &[Some(&layout)],
            immediate_size: 0,
        });
        let module = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("usd aircraft"),
            source: wgpu::ShaderSource::Wgsl(SHADER.into()),
        });
        let build = |kind| {
            build_pipeline(
                device,
                &pipeline_layout,
                &module,
                kind,
                format,
                depth_format,
                samples,
            )
        };
        Self {
            opaque: build(PipelineKind::Opaque),
            opaque_double_sided: build(PipelineKind::OpaqueDoubleSided),
            translucent: build(PipelineKind::Translucent),
            meshes: BTreeMap::new(),
            layout,
            slots: BTreeMap::new(),
        }
    }

    /// Uploads the meshes of every airframe in `library` not yet on the GPU.
    fn upload_new_assets(
        &mut self,
        device: &wgpu::Device,
        queue: &UploadQueue,
        library: &AircraftLibrary,
    ) {
        for (key, asset) in library.resident() {
            self.meshes
                .entry(key)
                .or_insert_with(|| upload_asset(device, queue, asset));
        }
    }

    fn upload_slot(
        &mut self,
        device: &wgpu::Device,
        queue: &UploadQueue,
        slot: ViewSlotId,
        draws: &[super::PackedDraw],
    ) {
        let needed = (draws.len() as u64).max(1).saturating_mul(DRAW_STRIDE);
        let stale = self.slots.get(&slot).is_none_or(|s| s.capacity < needed);
        if stale {
            let capacity = needed.next_power_of_two().max(DRAW_STRIDE * 8);
            let uniforms = device.create_buffer(&wgpu::BufferDescriptor {
                label: Some("usd aircraft slot uniforms"),
                size: capacity,
                usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
                mapped_at_creation: false,
            });
            let bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
                label: Some("usd aircraft slot"),
                layout: &self.layout,
                entries: &[wgpu::BindGroupEntry {
                    binding: 0,
                    resource: wgpu::BindingResource::Buffer(wgpu::BufferBinding {
                        buffer: &uniforms,
                        offset: 0,
                        size: NonZeroU64::new(DRAW_BYTES as u64),
                    }),
                }],
            });
            self.slots.insert(
                slot,
                SlotGpu {
                    uniforms,
                    capacity,
                    bind_group,
                    commands: Vec::new(),
                },
            );
        }
        let Some(slot_gpu) = self.slots.get_mut(&slot) else {
            return;
        };
        slot_gpu.commands.clear();
        if draws.is_empty() {
            return;
        }
        let mut bytes = vec![0_u8; draws.len().saturating_mul(DRAW_STRIDE as usize)];
        for (index, (draw, block)) in draws
            .iter()
            .zip(
                bytes
                    .as_chunks_mut::<{ DRAW_STRIDE as usize }>()
                    .0
                    .iter_mut(),
            )
            .enumerate()
        {
            if let Some(dst) = block.get_mut(..DRAW_BYTES) {
                dst.copy_from_slice(&draw.uniform);
            }
            let pipeline = match (draw.translucent, draw.double_sided) {
                (true, _) => PipelineKind::Translucent,
                (false, true) => PipelineKind::OpaqueDoubleSided,
                (false, false) => PipelineKind::Opaque,
            };
            let Ok(offset) = u32::try_from((index as u64).saturating_mul(DRAW_STRIDE)) else {
                break;
            };
            slot_gpu.commands.push(DrawCommand {
                asset: draw.asset,
                part_index: draw.part_index,
                pipeline,
                offset,
            });
        }
        queue.write_buffer(&slot_gpu.uniforms, 0, &bytes);
    }

    pub(super) fn pipeline(&self, kind: PipelineKind) -> &wgpu::RenderPipeline {
        match kind {
            PipelineKind::Opaque => &self.opaque,
            PipelineKind::OpaqueDoubleSided => &self.opaque_double_sided,
            PipelineKind::Translucent => &self.translucent,
        }
    }
}

fn upload_asset(
    device: &wgpu::Device,
    queue: &UploadQueue,
    asset: &UsdAircraftAsset,
) -> Vec<Option<PartMesh>> {
    asset
        .parts()
        .iter()
        .map(|part| upload_mesh(device, queue, &part.vertices, &part.indices))
        .collect()
}

fn upload_mesh(
    device: &wgpu::Device,
    queue: &UploadQueue,
    vertices: &[crate::UsdMeshVertex],
    indices: &[u32],
) -> Option<PartMesh> {
    let whole = indices.len() - indices.len() % 3;
    let count = u32::try_from(whole).ok().filter(|count| *count > 0)?;
    let mut vertex_bytes =
        Vec::with_capacity(vertices.len().saturating_mul(VERTEX_STRIDE as usize));
    for vertex in vertices {
        for value in vertex
            .position
            .into_iter()
            .chain(vertex.normal)
            .chain(vertex.uv)
        {
            vertex_bytes.extend_from_slice(&value.to_le_bytes());
        }
    }
    let index_bytes: Vec<u8> = indices
        .iter()
        .take(whole)
        .flat_map(|index| index.to_le_bytes())
        .collect();
    let buffer = |label, contents: &[u8], usage| {
        queue.create_buffer_init(
            device,
            &wgpu::util::BufferInitDescriptor {
                label: Some(label),
                contents,
                usage,
            },
        )
    };
    Some(PartMesh {
        vertices: buffer(
            "usd part vertices",
            &vertex_bytes,
            wgpu::BufferUsages::VERTEX,
        ),
        indices: buffer("usd part indices", &index_bytes, wgpu::BufferUsages::INDEX),
        count,
    })
}

fn build_pipeline(
    device: &wgpu::Device,
    layout: &wgpu::PipelineLayout,
    module: &wgpu::ShaderModule,
    kind: PipelineKind,
    format: wgpu::TextureFormat,
    depth_format: wgpu::TextureFormat,
    samples: u32,
) -> wgpu::RenderPipeline {
    let translucent = kind == PipelineKind::Translucent;
    device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
        label: Some("usd aircraft"),
        layout: Some(layout),
        vertex: wgpu::VertexState {
            module,
            entry_point: Some("vs"),
            compilation_options: Default::default(),
            buffers: &[Some(wgpu::VertexBufferLayout {
                array_stride: VERTEX_STRIDE,
                step_mode: wgpu::VertexStepMode::Vertex,
                attributes: &wgpu::vertex_attr_array![0 => Float32x3, 1 => Float32x3, 2 => Float32x2],
            })],
        },
        fragment: Some(wgpu::FragmentState {
            module,
            entry_point: Some(if translucent { "fs_translucent" } else { "fs_opaque" }),
            compilation_options: Default::default(),
            targets: &[Some(wgpu::ColorTargetState {
                format,
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
            format: depth_format,
            depth_write_enabled: Some(!translucent),
            depth_compare: Some(wgpu::CompareFunction::GreaterEqual),
            stencil: wgpu::StencilState::default(),
            bias: wgpu::DepthBiasState::default(),
        }),
        multisample: wgpu::MultisampleState {
            count: samples,
            ..Default::default()
        },
        multiview_mask: None,
        cache: None,
    })
}
