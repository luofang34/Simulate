use std::f32::consts::TAU;

use indicate_instrument_scene::Rgba8;
use sim_render::VectorVertex;

/// Largest chord-to-arc gap, physical pixels; below MSAA's resolving power.
const ARC_TOLERANCE_PX: f32 = 0.2;

/// Depth step between paint operations; exact in `f32` and in a
/// `Depth32Float` attachment for every operation index below `2^24`.
const DEPTH_STEP: f32 = 1.0 / 16_777_216.0;

/// The paint of the operation being tessellated.
#[derive(Debug, Clone, Copy)]
struct Paint {
    color: [f32; 4],
    clip: [f32; 4],
    depth: f32,
}

/// Collects physical-pixel triangles from design-space geometry.
pub(super) struct Mesh {
    out: Vec<VectorVertex>,
    scale: [f32; 2],
    operations: u32,
    paint: Option<Paint>,
}

impl Mesh {
    pub(super) fn new(scale: [f32; 2]) -> Self {
        Self {
            out: Vec::new(),
            scale,
            operations: 0,
            paint: None,
        }
    }

    pub(super) fn into_vertices(self) -> Vec<VectorVertex> {
        self.out
    }

    /// Design-to-physical scale for lengths that must stay isotropic.
    pub(super) fn length_scale(&self) -> f32 {
        self.scale[0].min(self.scale[1])
    }

    /// Starts one paint operation. Returns `false` (and paints nothing until
    /// the next call) for a transparent colour or an empty clip, as the
    /// rasterizer skips zero-alpha composites.
    pub(super) fn begin(&mut self, color: Rgba8, clip: [f32; 4]) -> bool {
        self.paint = None;
        if color.a == 0 || clip[0] >= clip[2] || clip[1] >= clip[3] {
            return false;
        }
        self.operations = self.operations.saturating_add(1).min(1 << 24);
        self.paint = Some(Paint {
            color: premultiplied_linear(color),
            clip,
            depth: self.operations as f32 * DEPTH_STEP,
        });
        true
    }

    /// One triangle in design units.
    pub(super) fn tri(&mut self, a: [f32; 2], b: [f32; 2], c: [f32; 2]) {
        let Some(paint) = self.paint else {
            return;
        };
        let [sx, sy] = self.scale;
        let pts = [a, b, c].map(|[x, y]| [x * sx, y * sy]);
        let [x0, y0, x1, y1] = paint.clip;
        // Geometry wholly outside its clip (ladder rungs past the frame edge)
        // would only be discarded by the fragment stage.
        if pts.iter().all(|p| p[0] <= x0)
            || pts.iter().all(|p| p[0] >= x1)
            || pts.iter().all(|p| p[1] <= y0)
            || pts.iter().all(|p| p[1] >= y1)
        {
            return;
        }
        for pos_px in pts {
            self.out.push(VectorVertex {
                pos_px,
                color_premul: paint.color,
                clip_rect_px: paint.clip,
                depth: paint.depth,
            });
        }
    }

    /// Two triangles over a quad given in order around its edge.
    pub(super) fn quad(&mut self, q: [[f32; 2]; 4]) {
        let [a, b, c, d] = q;
        self.tri(a, b, c);
        self.tri(a, c, d);
    }

    /// Chord count for `sweep` radians of a design radius `r`.
    pub(super) fn arc_segments(&self, r: f32, sweep: f32, min: usize, max: usize) -> usize {
        let r_px = r * self.length_scale();
        let step = if r_px > ARC_TOLERANCE_PX {
            2.0 * (1.0 - ARC_TOLERANCE_PX / r_px).acos()
        } else {
            TAU / 4.0
        };
        let n = (sweep.abs() / step.max(1e-3)).ceil();
        (n as usize).clamp(min, max)
    }

    /// A filled disc.
    pub(super) fn disc(&mut self, c: [f32; 2], r: f32, min_segments: usize) {
        if r <= 0.0 {
            return;
        }
        let n = self.arc_segments(r, TAU, min_segments, 128);
        let mut prev = point(c, r, 0.0);
        for i in 1..=n {
            let next = point(c, r, TAU * i as f32 / n as f32);
            self.tri(c, prev, next);
            prev = next;
        }
    }

    /// A ring sector between radii `r_in` and `r_out` from angle `start`
    /// sweeping `sweep` (positive clockwise in y-down design space).
    pub(super) fn ring(
        &mut self,
        c: [f32; 2],
        radii: [f32; 2],
        start: f32,
        sweep: f32,
        min: usize,
    ) {
        let [r_in, r_out] = [radii[0].max(0.0), radii[1]];
        if r_out <= r_in {
            return;
        }
        let n = self.arc_segments(r_out, sweep, min, 256);
        let at = |i: usize| start + sweep * i as f32 / n as f32;
        for i in 0..n {
            let (a0, a1) = (at(i), at(i.wrapping_add(1)));
            let outer = [point(c, r_out, a0), point(c, r_out, a1)];
            if r_in <= 0.0 {
                self.tri(c, outer[0], outer[1]);
            } else {
                self.quad([outer[0], outer[1], point(c, r_in, a1), point(c, r_in, a0)]);
            }
        }
    }
}

/// The point at `angle` on a circle; angle `0` is `+x`, positive clockwise
/// in y-down space.
pub(super) fn point(c: [f32; 2], r: f32, angle: f32) -> [f32; 2] {
    let (s, k) = angle.sin_cos();
    [c[0] + r * k, c[1] + r * s]
}

/// sRGB-encoded straight RGBA8 to linear premultiplied colour; the
/// `Rgba8UnormSrgb` target re-encodes on write.
pub(super) fn premultiplied_linear(color: Rgba8) -> [f32; 4] {
    let a = f32::from(color.a) / 255.0;
    [
        srgb_to_linear(color.r) * a,
        srgb_to_linear(color.g) * a,
        srgb_to_linear(color.b) * a,
        a,
    ]
}

fn srgb_to_linear(c: u8) -> f32 {
    let c = f32::from(c) / 255.0;
    if c <= 0.04045 {
        c / 12.92
    } else {
        ((c + 0.055) / 1.055).powf(2.4)
    }
}
