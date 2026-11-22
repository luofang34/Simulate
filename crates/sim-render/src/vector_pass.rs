use bytemuck::{Pod, Zeroable};

use crate::RenderError;
use crate::gpu::DEPTH_FORMAT;

mod target;
#[cfg(test)]
mod tests;

pub use target::{VECTOR_COLOUR_FORMAT, VECTOR_SAMPLE_COUNT, VectorTarget};

/// Smallest vertex buffer a [`VectorTarget`] allocates, in vertices.
pub(super) const MIN_VERTEX_CAPACITY: u64 = 4096;

/// One triangle-list vertex of an anti-aliased vector scene.
///
/// Every vertex of a triangle carries the same colour, clip rectangle, and
/// depth; the pass interpolates none of them meaningfully.
#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq, Pod, Zeroable)]
pub struct VectorVertex {
    /// Position in physical target pixels, top-left origin.
    pub pos_px: [f32; 2],
    /// Linear-light colour premultiplied by alpha.
    pub color_premul: [f32; 4],
    /// Clip rectangle `[min_x, min_y, max_x, max_y]` in physical pixels.
    pub clip_rect_px: [f32; 4],
    /// Paint-order depth in `(0.0, 1.0]`: each paint operation must use a
    /// strictly larger value than the one before it. A sample covered twice
    /// by one operation (overlapping stroke joins and caps) is blended once,
    /// as the CPU rasterizer composites each pixel once per operation.
    pub depth: f32,
}

const VERTEX_ATTRIBUTES: [wgpu::VertexAttribute; 4] = wgpu::vertex_attr_array![
    0 => Float32x2,
    1 => Float32x4,
    2 => Float32x4,
    3 => Float32,
];

/// Stateless triangle pass (`4×` MSAA by default, or `1×` on handhelds) that rasterizes
/// [`VectorVertex`] lists into any number of [`VectorTarget`]s with premultiplied blending.
pub struct VectorScenePass {
    pipeline: wgpu::RenderPipeline,
    layout: wgpu::BindGroupLayout,
    sample_count: u32,
}

impl VectorScenePass {
    /// Compiles and validates the `4×` MSAA vector pipeline.
    pub async fn new(device: &wgpu::Device) -> Result<Self, RenderError> {
        Self::with_sample_count(device, VECTOR_SAMPLE_COUNT).await
    }

    /// Compiles and validates the vector pipeline with `sample_count` (`1` or `4`).
    pub async fn with_sample_count(
        device: &wgpu::Device,
        sample_count: u32,
    ) -> Result<Self, RenderError> {
        let sample_count = target::normalized_sample_count(sample_count);
        let layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("vector scene"),
            entries: &[wgpu::BindGroupLayoutEntry {
                binding: 0,
                visibility: wgpu::ShaderStages::VERTEX,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Uniform,
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                count: None,
            }],
        });
        let scope = device.push_error_scope(wgpu::ErrorFilter::Validation);
        let pipeline = create_pipeline(device, &layout, sample_count);
        if let Some(err) = scope.pop().await {
            return Err(RenderError::PipelineValidation(err));
        }
        Ok(Self {
            pipeline,
            layout,
            sample_count,
        })
    }

    /// Configured raster sample count (`1` or `4`).
    #[must_use]
    pub const fn sample_count(&self) -> u32 {
        self.sample_count
    }

    /// Clears `target` to transparent and draws `vertices` as a triangle list,
    /// writing into [`VectorTarget::view`] (directly for `1×`, or resolved from `4×` MSAA).
    /// The pass always runs, so an empty list still leaves a cleared overlay rather than a stale one.
    ///
    /// Because each [`VectorTarget`] owns its uniform and vertex buffers,
    /// callers can encode multiple targets into the same `encoder` and submit
    /// once.
    pub fn encode(
        &self,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        encoder: &mut wgpu::CommandEncoder,
        target: &mut VectorTarget,
        vertices: &[VectorVertex],
    ) -> Result<(), RenderError> {
        let whole = vertices.len() - vertices.len() % 3;
        let drawn = vertices.get(..whole).unwrap_or_default();
        let draw_count = u32::try_from(drawn.len()).map_err(|_| too_large(device, drawn.len()))?;
        if !drawn.is_empty() {
            target.ensure_bind_group(device, &self.layout);
            target.upload_vertices(device, queue, drawn)?;
        }
        target.write_viewport(queue);

        let (color_view, resolve_target, store) = match target.msaa_view() {
            Some(msaa) => (msaa, Some(target.view()), wgpu::StoreOp::Discard),
            None => (target.view(), None, wgpu::StoreOp::Store),
        };
        let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
            label: Some("vector scene"),
            color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                view: color_view,
                depth_slice: None,
                resolve_target,
                ops: wgpu::Operations {
                    load: wgpu::LoadOp::Clear(wgpu::Color::TRANSPARENT),
                    store,
                },
            })],
            depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachment {
                view: target.depth_view(),
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
        if let (Some((bind_group, buffer)), true) = (target.draw_resources(), draw_count > 0) {
            pass.set_pipeline(&self.pipeline);
            pass.set_bind_group(0, bind_group, &[]);
            pass.set_vertex_buffer(0, buffer.slice(..));
            pass.draw(0..draw_count, 0..1);
        }
        Ok(())
    }
}

pub(super) fn too_large(device: &wgpu::Device, vertices: usize) -> RenderError {
    let stride = std::mem::size_of::<VectorVertex>() as u64;
    RenderError::VectorGeometryTooLarge {
        vertices,
        bytes: (vertices as u64).saturating_mul(stride),
        max: device.limits().max_buffer_size,
    }
}

fn create_pipeline(
    device: &wgpu::Device,
    layout: &wgpu::BindGroupLayout,
    sample_count: u32,
) -> wgpu::RenderPipeline {
    let shader = device.create_shader_module(wgpu::include_wgsl!("shaders/vector_scene.wgsl"));
    let pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
        label: Some("vector scene"),
        bind_group_layouts: &[Some(layout)],
        ..Default::default()
    });
    device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
        label: Some("vector scene"),
        layout: Some(&pipeline_layout),
        vertex: wgpu::VertexState {
            module: &shader,
            entry_point: Some("vs_vector"),
            compilation_options: Default::default(),
            buffers: &[Some(wgpu::VertexBufferLayout {
                array_stride: std::mem::size_of::<VectorVertex>() as u64,
                step_mode: wgpu::VertexStepMode::Vertex,
                attributes: &VERTEX_ATTRIBUTES,
            })],
        },
        fragment: Some(wgpu::FragmentState {
            module: &shader,
            entry_point: Some("fs_vector"),
            compilation_options: Default::default(),
            targets: &[Some(wgpu::ColorTargetState {
                format: VECTOR_COLOUR_FORMAT,
                blend: Some(wgpu::BlendState::PREMULTIPLIED_ALPHA_BLENDING),
                write_mask: wgpu::ColorWrites::ALL,
            })],
        }),
        // Tessellated geometry winds either way.
        primitive: wgpu::PrimitiveState::default(),
        // `Greater` against a per-operation depth lets later operations paint
        // over earlier ones while one operation never blends a sample twice.
        depth_stencil: Some(wgpu::DepthStencilState {
            format: DEPTH_FORMAT,
            depth_write_enabled: Some(true),
            depth_compare: Some(wgpu::CompareFunction::Greater),
            stencil: wgpu::StencilState::default(),
            bias: wgpu::DepthBiasState::default(),
        }),
        multisample: wgpu::MultisampleState {
            count: sample_count,
            mask: !0,
            alpha_to_coverage_enabled: false,
        },
        multiview_mask: None,
        cache: None,
    })
}
