#[cfg(test)]
mod tests;

use super::{
    MIN_VERTEX_CAPACITY, VectorVertex, create_bind_group_layout, create_pipeline_for, too_large,
};
use crate::RenderError;

/// Maximum direct vector sub-viewports encoded into a single presentation frame.
pub const MAX_DIRECT_VECTOR_DRAWS: usize = 8;

/// Depth placed on an opaque backdrop, strictly above clear (`0.0`) and strictly below
/// `Indicate`'s first vector paint operation (`1.0 / 16_777_216.0`).
pub(super) const BACKDROP_DEPTH: f32 = 1.0 / 33_554_432.0;

/// Depths used for the outer dark and inner metallic glass-cockpit bezel rings.
pub(super) const BEZEL_OUTER_DEPTH: f32 = 0.999;
pub(super) const BEZEL_INNER_DEPTH: f32 = 0.9995;

const BEZEL_OUTER_COLOR: [f32; 4] = [0.02 * 0.96, 0.03 * 0.96, 0.05 * 0.96, 0.96];
const BEZEL_INNER_COLOR: [f32; 4] = [0.10 * 0.92, 0.16 * 0.92, 0.24 * 0.92, 0.92];

/// Swapchain attachments and surface extent targeted by [`DirectVectorPass::encode`].
#[derive(Clone, Copy)]
pub struct DirectVectorTarget<'a> {
    /// Presentation color attachment (`LoadOp::Load` preserves underlying terrain/OTW).
    pub color_view: &'a wgpu::TextureView,
    /// Single-sample reversed-Z depth attachment (cleared to `0.0` per subrect draw).
    pub depth_view: &'a wgpu::TextureView,
    /// Full swapchain surface extent `[width, height]` in physical pixels.
    pub size_px: [u32; 2],
}

/// One local vector scene drawn directly onto the presentation swapchain attachment.
#[derive(Clone, Copy, Debug)]
pub struct DirectVectorDraw<'a> {
    /// Sub-rectangle `[x, y, width, height]` in physical swapchain pixels.
    pub rect_px: [u32; 4],
    /// Panel-local vector triangles (`pos_px` and `clip_rect_px` relative to `rect_px` origin).
    pub vertices: &'a [VectorVertex],
    /// Optional linear premultiplied RGBA backdrop filled inside the rect behind `vertices`.
    pub opaque_backdrop: Option<[f32; 4]>,
    /// Whether to draw the crisp 2-pixel glass-cockpit bezel border matching `CompositorPass`.
    pub bezel: bool,
}

/// Prepared slice within the shared vertex buffer for one clamped direct draw.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) struct PreparedDraw {
    pub(super) rect_px: [u32; 4],
    pub(super) start: u32,
    pub(super) count: u32,
}

struct DirectSlot {
    uniform: wgpu::Buffer,
    bind_group: wgpu::BindGroup,
}

/// Single-sample swapchain vector pass that renders HUD, PFD, and HSI triangles directly into
/// scoped viewport/scissor rectangles without allocating intermediate color/depth textures.
pub struct DirectVectorPass {
    pipeline: wgpu::RenderPipeline,
    slots: [DirectSlot; MAX_DIRECT_VECTOR_DRAWS],
    vertex_buffer: Option<wgpu::Buffer>,
    capacity_bytes: u64,
    scratch: Vec<VectorVertex>,
}

impl DirectVectorPass {
    /// Compiles and validates the direct `1×` swapchain vector pipeline targeting `target_format`.
    pub async fn new(
        device: &wgpu::Device,
        target_format: wgpu::TextureFormat,
    ) -> Result<Self, RenderError> {
        let layout = create_bind_group_layout(device);
        let scope = device.push_error_scope(wgpu::ErrorFilter::Validation);
        let pipeline = create_pipeline_for(device, &layout, target_format, 1);
        if let Some(err) = scope.pop().await {
            return Err(RenderError::PipelineValidation(err));
        }
        let slots = std::array::from_fn(|_| {
            let uniform = device.create_buffer(&wgpu::BufferDescriptor {
                label: Some("direct vector viewport"),
                size: std::mem::size_of::<[f32; 4]>() as u64,
                usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
                mapped_at_creation: false,
            });
            let bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
                label: Some("direct vector viewport"),
                layout: &layout,
                entries: &[wgpu::BindGroupEntry {
                    binding: 0,
                    resource: uniform.as_entire_binding(),
                }],
            });
            DirectSlot {
                uniform,
                bind_group,
            }
        });
        Ok(Self {
            pipeline,
            slots,
            vertex_buffer: None,
            capacity_bytes: 0,
            scratch: Vec::with_capacity(16_384),
        })
    }

    /// Packs and encodes `draws` directly into `target` (depth cleared per draw to `0.0` and
    /// discarded afterwards).
    pub fn encode(
        &mut self,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        encoder: &mut wgpu::CommandEncoder,
        target: DirectVectorTarget<'_>,
        draws: &[DirectVectorDraw<'_>],
    ) -> Result<(), RenderError> {
        let prepared = pack_draws(draws, target.size_px, &mut self.scratch);
        if prepared.is_empty() || self.scratch.is_empty() {
            return Ok(());
        }
        self.upload_vertices(device, queue)?;
        let Some(vertex_buffer) = &self.vertex_buffer else {
            return Ok(());
        };
        for (slot_item, item) in self.slots.iter().zip(prepared) {
            let [rx, ry, rw, rh] = item.rect_px;
            let viewport = [rw as f32, rh as f32, rx as f32, ry as f32];
            queue.write_buffer(&slot_item.uniform, 0, bytemuck::bytes_of(&viewport));
            let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some("direct vector"),
                color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                    view: target.color_view,
                    depth_slice: None,
                    resolve_target: None,
                    ops: wgpu::Operations {
                        load: wgpu::LoadOp::Load,
                        store: wgpu::StoreOp::Store,
                    },
                })],
                depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachment {
                    view: target.depth_view,
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
            pass.set_viewport(rx as f32, ry as f32, rw as f32, rh as f32, 0.0, 1.0);
            pass.set_scissor_rect(rx, ry, rw, rh);
            pass.set_pipeline(&self.pipeline);
            pass.set_bind_group(0, &slot_item.bind_group, &[]);
            pass.set_vertex_buffer(0, vertex_buffer.slice(..));
            let end = item.start.saturating_add(item.count);
            pass.draw(item.start..end, 0..1);
        }
        Ok(())
    }

    fn upload_vertices(
        &mut self,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
    ) -> Result<(), RenderError> {
        let bytes: &[u8] = bytemuck::cast_slice(&self.scratch);
        let needed = bytes.len() as u64;
        if self.vertex_buffer.is_none() || needed > self.capacity_bytes {
            let max = device.limits().max_buffer_size;
            let stride = std::mem::size_of::<VectorVertex>() as u64;
            let capacity = needed
                .max(MIN_VERTEX_CAPACITY * stride)
                .checked_next_power_of_two()
                .unwrap_or(needed)
                .min(max);
            if needed > capacity {
                return Err(too_large(device, self.scratch.len()));
            }
            self.vertex_buffer = Some(device.create_buffer(&wgpu::BufferDescriptor {
                label: Some("direct vector vertices"),
                size: capacity,
                usage: wgpu::BufferUsages::VERTEX | wgpu::BufferUsages::COPY_DST,
                mapped_at_creation: false,
            }));
            self.capacity_bytes = capacity;
        }
        if let Some(buffer) = &self.vertex_buffer {
            queue.write_buffer(buffer, 0, bytes);
        }
        Ok(())
    }
}

/// Clamps `[x, y, w, h]` to `[0..sw, 0..sh]`; returns `None` if the result has zero area.
pub(super) fn clamp_rect_px([x, y, w, h]: [u32; 4], [sw, sh]: [u32; 2]) -> Option<[u32; 4]> {
    let cx = x.min(sw);
    let cy = y.min(sh);
    let cw = w.min(sw.saturating_sub(cx));
    let ch = h.min(sh.saturating_sub(cy));
    (cw > 0 && ch > 0).then_some([cx, cy, cw, ch])
}

/// Packs up to `MAX_DIRECT_VECTOR_DRAWS` into `scratch`, emitting backdrop + vertices + bezel.
pub(super) fn pack_draws(
    draws: &[DirectVectorDraw<'_>],
    surface_px: [u32; 2],
    scratch: &mut Vec<VectorVertex>,
) -> Vec<PreparedDraw> {
    scratch.clear();
    let mut prepared = Vec::with_capacity(draws.len().min(MAX_DIRECT_VECTOR_DRAWS));
    for draw in draws.iter().take(MAX_DIRECT_VECTOR_DRAWS) {
        let Some([cx, cy, cw, ch]) = clamp_rect_px(draw.rect_px, surface_px) else {
            continue;
        };
        let whole = draw.vertices.len() - (draw.vertices.len() % 3);
        let drawn = draw.vertices.get(..whole).unwrap_or_default();
        if draw.opaque_backdrop.is_none() && drawn.is_empty() && !draw.bezel {
            continue;
        }
        let Ok(start) = u32::try_from(scratch.len()) else {
            break;
        };
        let fw = cw as f32;
        let fh = ch as f32;
        let full_clip = [0.0, 0.0, fw, fh];
        if let Some(bg) = draw.opaque_backdrop {
            push_quad(scratch, [0.0, 0.0, fw, fh], bg, full_clip, BACKDROP_DEPTH);
        }
        scratch.extend_from_slice(drawn);
        if draw.bezel {
            push_bezel(scratch, fw, fh, full_clip);
        }
        let count_usize = scratch.len().saturating_sub(start as usize);
        let Ok(count) = u32::try_from(count_usize) else {
            break;
        };
        if count > 0 {
            prepared.push(PreparedDraw {
                rect_px: [cx, cy, cw, ch],
                start,
                count,
            });
        }
    }
    prepared
}

fn push_bezel(out: &mut Vec<VectorVertex>, w: f32, h: f32, clip: [f32; 4]) {
    let t0 = 1.25_f32.min(w * 0.5).min(h * 0.5);
    push_border_ring(
        out,
        [0.0, 0.0, w, h],
        t0,
        BEZEL_OUTER_COLOR,
        clip,
        BEZEL_OUTER_DEPTH,
    );
    if w > 5.0 && h > 5.0 {
        push_border_ring(
            out,
            [1.25, 1.25, w - 1.25, h - 1.25],
            1.0,
            BEZEL_INNER_COLOR,
            clip,
            BEZEL_INNER_DEPTH,
        );
    }
}

fn push_border_ring(
    out: &mut Vec<VectorVertex>,
    [x0, y0, x1, y1]: [f32; 4],
    t: f32,
    color: [f32; 4],
    clip: [f32; 4],
    depth: f32,
) {
    if t <= 0.0 || x1 <= x0 || y1 <= y0 {
        return;
    }
    push_quad(out, [x0, y0, x1, (y0 + t).min(y1)], color, clip, depth);
    push_quad(out, [x0, (y1 - t).max(y0), x1, y1], color, clip, depth);
    let inner_y0 = (y0 + t).min(y1);
    let inner_y1 = (y1 - t).max(inner_y0);
    if inner_y1 > inner_y0 {
        push_quad(
            out,
            [x0, inner_y0, (x0 + t).min(x1), inner_y1],
            color,
            clip,
            depth,
        );
        push_quad(
            out,
            [(x1 - t).max(x0), inner_y0, x1, inner_y1],
            color,
            clip,
            depth,
        );
    }
}

fn push_quad(
    out: &mut Vec<VectorVertex>,
    [x0, y0, x1, y1]: [f32; 4],
    color: [f32; 4],
    clip: [f32; 4],
    depth: f32,
) {
    if x1 <= x0 || y1 <= y0 {
        return;
    }
    let v = |pos_px| VectorVertex {
        pos_px,
        color_premul: color,
        clip_rect_px: clip,
        depth,
    };
    out.extend_from_slice(&[
        v([x0, y0]),
        v([x1, y0]),
        v([x0, y1]),
        v([x0, y1]),
        v([x1, y0]),
        v([x1, y1]),
    ]);
}
