use bytemuck::{Pod, Zeroable};

use crate::RenderError;

mod panel_texture;
#[cfg(test)]
mod tests;

pub use panel_texture::RgbaPanelTexture;

/// Maximum number of textured viewport/instrument quads composited in one pass.
pub const MAX_COMPOSITE_QUADS: usize = 12;

const FLAG_SRGB_UNORM: u32 = 1;
const FLAG_BEZEL: u32 = 2;
const FLAG_OPAQUE: u32 = 4;
const FLAG_PREMULTIPLIED: u32 = 8;

/// Normalized top-left rectangle `[0.0, 1.0]` within the target viewport.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ViewportRect {
    /// Left edge in `[0.0, 1.0]`.
    pub x: f32,
    /// Top edge in `[0.0, 1.0]`.
    pub y: f32,
    /// Width fraction in `[0.0, 1.0]`.
    pub width: f32,
    /// Height fraction in `[0.0, 1.0]`.
    pub height: f32,
}

impl ViewportRect {
    /// Full-window `[0.0, 0.0, 1.0, 1.0]` rectangle.
    pub const FULL: Self = Self {
        x: 0.0,
        y: 0.0,
        width: 1.0,
        height: 1.0,
    };
}

/// Colour transfer function of a sampled texture view when compositing into an sRGB target.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ColorTransfer {
    /// Texture view already decodes to linear RGB in hardware (e.g. `Rgba8UnormSrgb`).
    HardwareLinear,
    /// Texture view holds sRGB-encoded colours in a plain `Unorm` attachment (e.g. `maplibre` `Rgba8Unorm`).
    SrgbEncodedUnorm,
}

/// Texture magnification/minification filter for an [`OverlayQuad`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum QuadFilter {
    /// Bilinear filtering for scaled viewports and map textures.
    Linear,
    /// Nearest-neighbour texel sampling.
    Nearest,
}

/// One zero-copy GPU texture view positioned and blended onto the presentation target.
#[derive(Debug, Clone, Copy)]
pub struct OverlayQuad<'a> {
    /// Normalized screen rectangle.
    pub rect: ViewportRect,
    /// Source GPU texture view.
    pub view: &'a wgpu::TextureView,
    /// Colour transfer policy applied before sRGB attachment encoding.
    pub transfer: ColorTransfer,
    /// Sampling filter.
    pub filter: QuadFilter,
    /// Forces alpha to `1.0` when `true` (used for opaque map/SVS underlays).
    pub opaque: bool,
    /// Draws a crisp 2-pixel glass-cockpit bezel frame around the quad when `true`.
    pub bezel: bool,
    /// Overall alpha multiplier in `[0.0, 1.0]`.
    pub opacity: f32,
    /// Source texels already carry colour multiplied by alpha (GPU-rendered
    /// vector overlays); straight-alpha texels are premultiplied in the shader.
    pub premultiplied: bool,
}

impl<'a> OverlayQuad<'a> {
    /// Full-viewport opaque quad for a `maplibre` `Rgba8Unorm` scene or chart view.
    #[must_use]
    pub const fn full_map(view: &'a wgpu::TextureView) -> Self {
        Self {
            rect: ViewportRect::FULL,
            view,
            transfer: ColorTransfer::SrgbEncodedUnorm,
            filter: QuadFilter::Linear,
            opaque: true,
            bezel: false,
            opacity: 1.0,
            premultiplied: false,
        }
    }

    /// Inset opaque quad for a `maplibre` `Rgba8Unorm` SVS underlay or MFD moving map.
    #[must_use]
    pub const fn inset_map(rect: ViewportRect, view: &'a wgpu::TextureView, bezel: bool) -> Self {
        Self {
            rect,
            view,
            transfer: ColorTransfer::SrgbEncodedUnorm,
            filter: QuadFilter::Linear,
            opaque: true,
            bezel,
            opacity: 1.0,
            premultiplied: false,
        }
    }

    /// Inset or HUD quad for an `Indicate` `Rgba8UnormSrgb` rasterized instrument texture.
    #[must_use]
    pub const fn panel(
        rect: ViewportRect,
        view: &'a wgpu::TextureView,
        opaque: bool,
        bezel: bool,
    ) -> Self {
        Self {
            rect,
            view,
            transfer: ColorTransfer::HardwareLinear,
            filter: QuadFilter::Linear,
            opaque,
            bezel,
            opacity: 1.0,
            premultiplied: false,
        }
    }
}

impl<'a> OverlayQuad<'a> {
    /// Full-alpha quad for a premultiplied `Rgba8UnormSrgb` vector overlay
    /// such as a [`VectorTarget`](crate::VectorTarget) resolve view.
    #[must_use]
    pub const fn vector_overlay(rect: ViewportRect, view: &'a wgpu::TextureView) -> Self {
        Self {
            rect,
            view,
            transfer: ColorTransfer::HardwareLinear,
            filter: QuadFilter::Linear,
            opaque: false,
            bezel: false,
            opacity: 1.0,
            premultiplied: true,
        }
    }
}

/// Destination attachment and load operation for [`CompositorPass::encode`].
#[derive(Clone, Copy, Debug)]
pub struct CompositorTarget<'a> {
    /// Color attachment view to draw onto.
    pub view: &'a wgpu::TextureView,
    /// Optional clear color; `None` preserves existing attachment pixels (`LoadOp::Load`).
    pub clear: Option<wgpu::Color>,
    /// Physical attachment dimensions `[width_px, height_px]`.
    pub size_px: [u32; 2],
}

#[repr(C)]
#[derive(Clone, Copy, Pod, Zeroable)]
struct QuadUniform {
    rect: [f32; 4],
    pixel_size: [f32; 2],
    flags: u32,
    opacity: f32,
}

/// Alpha-blended textured-quad compositor pass for split-viewport OTW, SVS, MFD, and PFD/HSI layers.
pub struct CompositorPass {
    layout: wgpu::BindGroupLayout,
    pipeline: wgpu::RenderPipeline,
    linear_sampler: wgpu::Sampler,
    nearest_sampler: wgpu::Sampler,
    uniform_buffers: [wgpu::Buffer; MAX_COMPOSITE_QUADS],
}

impl CompositorPass {
    /// Compiles and validates the WGSL compositor pipeline targeting `target_format`.
    pub async fn new(
        device: &wgpu::Device,
        target_format: wgpu::TextureFormat,
    ) -> Result<Self, RenderError> {
        let layout = create_quad_bind_group_layout(device);
        let scope = device.push_error_scope(wgpu::ErrorFilter::Validation);
        let pipeline = create_quad_pipeline(device, &layout, target_format);
        if let Some(err) = scope.pop().await {
            return Err(RenderError::PipelineValidation(err));
        }
        let linear_sampler = create_sampler(device, "compositor linear", wgpu::FilterMode::Linear);
        let nearest_sampler =
            create_sampler(device, "compositor nearest", wgpu::FilterMode::Nearest);
        let uniform_buffers = std::array::from_fn(|_| {
            device.create_buffer(&wgpu::BufferDescriptor {
                label: Some("compositor quad uniform"),
                size: std::mem::size_of::<QuadUniform>() as u64,
                usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
                mapped_at_creation: false,
            })
        });
        Ok(Self {
            layout,
            pipeline,
            linear_sampler,
            nearest_sampler,
            uniform_buffers,
        })
    }

    /// Encodes ordered [`OverlayQuad`] draws onto `target`.
    pub fn encode(
        &self,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        encoder: &mut wgpu::CommandEncoder,
        target: CompositorTarget<'_>,
        quads: &[OverlayQuad<'_>],
    ) {
        if quads.is_empty() && target.clear.is_none() {
            return;
        }
        let [target_w, target_h] = target.size_px;
        let mut bind_groups = Vec::with_capacity(quads.len().min(MAX_COMPOSITE_QUADS));
        for (quad, buffer) in quads
            .iter()
            .take(MAX_COMPOSITE_QUADS)
            .zip(&self.uniform_buffers)
        {
            let uniform = build_quad_uniform(quad, target_w, target_h);
            queue.write_buffer(buffer, 0, bytemuck::bytes_of(&uniform));
            let sampler = match quad.filter {
                QuadFilter::Linear => &self.linear_sampler,
                QuadFilter::Nearest => &self.nearest_sampler,
            };
            bind_groups.push(device.create_bind_group(&wgpu::BindGroupDescriptor {
                label: Some("compositor quad"),
                layout: &self.layout,
                entries: &[
                    wgpu::BindGroupEntry {
                        binding: 0,
                        resource: buffer.as_entire_binding(),
                    },
                    wgpu::BindGroupEntry {
                        binding: 1,
                        resource: wgpu::BindingResource::TextureView(quad.view),
                    },
                    wgpu::BindGroupEntry {
                        binding: 2,
                        resource: wgpu::BindingResource::Sampler(sampler),
                    },
                ],
            }));
        }

        let load = target.clear.map_or(wgpu::LoadOp::Load, wgpu::LoadOp::Clear);
        let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
            label: Some("compositor"),
            color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                view: target.view,
                depth_slice: None,
                resolve_target: None,
                ops: wgpu::Operations {
                    load,
                    store: wgpu::StoreOp::Store,
                },
            })],
            depth_stencil_attachment: None,
            timestamp_writes: None,
            occlusion_query_set: None,
            multiview_mask: None,
        });
        pass.set_pipeline(&self.pipeline);
        for bg in &bind_groups {
            pass.set_bind_group(0, bg, &[]);
            pass.draw(0..6, 0..1);
        }
    }
}

fn build_quad_uniform(quad: &OverlayQuad<'_>, target_w: u32, target_h: u32) -> QuadUniform {
    let mut flags = 0_u32;
    if quad.transfer == ColorTransfer::SrgbEncodedUnorm {
        flags |= FLAG_SRGB_UNORM;
    }
    if quad.bezel {
        flags |= FLAG_BEZEL;
    }
    if quad.opaque {
        flags |= FLAG_OPAQUE;
    }
    if quad.premultiplied {
        flags |= FLAG_PREMULTIPLIED;
    }
    let width_px = (quad.rect.width * (target_w.max(1) as f32)).max(1.0);
    let height_px = (quad.rect.height * (target_h.max(1) as f32)).max(1.0);
    QuadUniform {
        rect: [quad.rect.x, quad.rect.y, quad.rect.width, quad.rect.height],
        pixel_size: [width_px, height_px],
        flags,
        opacity: quad.opacity.clamp(0.0, 1.0),
    }
}

fn create_sampler(
    device: &wgpu::Device,
    label: &'static str,
    filter: wgpu::FilterMode,
) -> wgpu::Sampler {
    device.create_sampler(&wgpu::SamplerDescriptor {
        label: Some(label),
        address_mode_u: wgpu::AddressMode::ClampToEdge,
        address_mode_v: wgpu::AddressMode::ClampToEdge,
        address_mode_w: wgpu::AddressMode::ClampToEdge,
        mag_filter: filter,
        min_filter: filter,
        mipmap_filter: wgpu::MipmapFilterMode::Nearest,
        ..Default::default()
    })
}

fn create_quad_bind_group_layout(device: &wgpu::Device) -> wgpu::BindGroupLayout {
    device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
        label: Some("compositor"),
        entries: &[
            wgpu::BindGroupLayoutEntry {
                binding: 0,
                visibility: wgpu::ShaderStages::VERTEX_FRAGMENT,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Uniform,
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                count: None,
            },
            wgpu::BindGroupLayoutEntry {
                binding: 1,
                visibility: wgpu::ShaderStages::FRAGMENT,
                ty: wgpu::BindingType::Texture {
                    sample_type: wgpu::TextureSampleType::Float { filterable: true },
                    view_dimension: wgpu::TextureViewDimension::D2,
                    multisampled: false,
                },
                count: None,
            },
            wgpu::BindGroupLayoutEntry {
                binding: 2,
                visibility: wgpu::ShaderStages::FRAGMENT,
                ty: wgpu::BindingType::Sampler(wgpu::SamplerBindingType::Filtering),
                count: None,
            },
        ],
    })
}

fn create_quad_pipeline(
    device: &wgpu::Device,
    layout: &wgpu::BindGroupLayout,
    target_format: wgpu::TextureFormat,
) -> wgpu::RenderPipeline {
    let shader = device.create_shader_module(wgpu::include_wgsl!("shaders/compositor.wgsl"));
    let pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
        label: Some("compositor"),
        bind_group_layouts: &[Some(layout)],
        ..Default::default()
    });
    device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
        label: Some("compositor"),
        layout: Some(&pipeline_layout),
        vertex: wgpu::VertexState {
            module: &shader,
            entry_point: Some("vs_quad"),
            compilation_options: Default::default(),
            buffers: &[],
        },
        fragment: Some(wgpu::FragmentState {
            module: &shader,
            entry_point: Some("fs_quad"),
            compilation_options: Default::default(),
            targets: &[Some(wgpu::ColorTargetState {
                format: target_format,
                blend: Some(wgpu::BlendState::PREMULTIPLIED_ALPHA_BLENDING),
                write_mask: wgpu::ColorWrites::ALL,
            })],
        }),
        primitive: wgpu::PrimitiveState::default(),
        depth_stencil: None,
        multisample: wgpu::MultisampleState::default(),
        multiview_mask: None,
        cache: None,
    })
}
