//! Thin solid needles and hub caps extruded from Indicate's pointer outlines.
//!
//! Geometry is in the needle's own frame: origin on the pivot, `+X` right and
//! `+Z` up the dial as the pilot sees it, `-Y` out of the dial toward the
//! pilot. The sheet's back lies on `y = 0` and its front at `y = -thickness`,
//! so a part placed at a pointer's stack height stands that far off the face.

use glam::{Vec2, Vec3};

use crate::UsdMeshVertex;

/// Accumulates flat-shaded triangles.
#[derive(Default)]
pub(super) struct Solid {
    pub(super) vertices: Vec<UsdMeshVertex>,
    pub(super) indices: Vec<u32>,
}

impl Solid {
    /// One triangle facing `outward`, wound so `(b - a) x (c - a)` points out.
    fn triangle(&mut self, [a, b, c]: [Vec3; 3], outward: Vec3) {
        let normal = (b - a).cross(c - a);
        let (b, c) = if normal.dot(outward) < 0.0 {
            (c, b)
        } else {
            (b, c)
        };
        let Ok(base) = u32::try_from(self.vertices.len()) else {
            return;
        };
        for p in [a, b, c] {
            self.vertices.push(UsdMeshVertex {
                position: p.to_array(),
                normal: outward.normalize_or_zero().to_array(),
                uv: [0.0, 0.0],
                occlusion: 1.0,
                tangent: UsdMeshVertex::NO_TANGENT,
            });
        }
        self.indices.extend([base, base + 1, base + 2]);
    }

    fn quad(&mut self, [a, b, c, d]: [Vec3; 4], outward: Vec3) {
        self.triangle([a, b, c], outward);
        self.triangle([a, c, d], outward);
    }

    /// The prism of `outline` (dial plane, `x` right, `z` up) between
    /// `y = back` and `y = front`, front toward `-Y`.
    pub(super) fn prism(&mut self, outline: &[Vec2], back: f32, front: f32) {
        let at = |p: Vec2, y: f32| Vec3::new(p.x, y, p.y);
        for [a, b, c] in ear_clip(outline) {
            self.triangle([at(a, front), at(b, front), at(c, front)], Vec3::NEG_Y);
            self.triangle([at(a, back), at(b, back), at(c, back)], Vec3::Y);
        }
        let ccw = signed_area(outline) >= 0.0;
        for (i, &a) in outline.iter().enumerate() {
            let Some(&b) = outline.get((i + 1) % outline.len()) else {
                continue;
            };
            let edge = b - a;
            let side = if ccw {
                Vec3::new(edge.y, 0.0, -edge.x)
            } else {
                Vec3::new(-edge.y, 0.0, edge.x)
            };
            self.quad([at(a, front), at(b, front), at(b, back), at(a, back)], side);
        }
    }

    /// A round cap of `radius` between `y = back` and `y = front`.
    pub(super) fn cap(&mut self, radius: f32, back: f32, front: f32, segments: u32) {
        let rim: Vec<Vec2> = (0..segments)
            .map(|i| {
                let a = std::f32::consts::TAU * i as f32 / segments as f32;
                Vec2::new(a.cos(), a.sin()) * radius
            })
            .collect();
        self.prism(&rim, back, front);
    }
}

fn signed_area(outline: &[Vec2]) -> f32 {
    let n = outline.len();
    (0..n)
        .filter_map(|i| Some(outline.get(i)?.perp_dot(*outline.get((i + 1) % n)?)))
        .sum::<f32>()
        * 0.5
}

/// Triangles of a simple polygon by ear clipping; it is small (a pointer
/// outline has at most eight corners), so the quadratic search costs nothing.
pub(super) fn ear_clip(outline: &[Vec2]) -> Vec<[Vec2; 3]> {
    let mut ring: Vec<Vec2> = outline.to_vec();
    if signed_area(&ring) < 0.0 {
        ring.reverse();
    }
    let mut out = Vec::new();
    while ring.len() > 3 {
        let n = ring.len();
        let ear = (0..n).find(|&i| {
            let (Some(&a), Some(&b), Some(&c)) = (
                ring.get((i + n - 1) % n),
                ring.get(i),
                ring.get((i + 1) % n),
            ) else {
                return false;
            };
            let convex = (b - a).perp_dot(c - b) > 0.0;
            convex
                && !ring
                    .iter()
                    .any(|&p| p != a && p != b && p != c && inside([a, b, c], p))
        });
        let Some(i) = ear else {
            break;
        };
        if let (Some(&a), Some(&b), Some(&c)) = (
            ring.get((i + n - 1) % n),
            ring.get(i),
            ring.get((i + 1) % n),
        ) {
            out.push([a, b, c]);
        }
        ring.remove(i);
    }
    if let [a, b, c] = *ring.as_slice() {
        out.push([a, b, c]);
    }
    out
}

fn inside([a, b, c]: [Vec2; 3], p: Vec2) -> bool {
    let d1 = (b - a).perp_dot(p - a);
    let d2 = (c - b).perp_dot(p - b);
    let d3 = (a - c).perp_dot(p - c);
    d1 >= 0.0 && d2 >= 0.0 && d3 >= 0.0
}
