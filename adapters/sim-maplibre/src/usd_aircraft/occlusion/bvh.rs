//! Bounding-volume hierarchy over an airframe's triangles for short occlusion rays.

use glam::Vec3;

/// Triangles per leaf; small leaves keep the short rays' traversal shallow.
const LEAF_SIZE: usize = 4;
/// Traversal stack depth; median splits of `u32` triangle counts never exceed it.
const STACK_DEPTH: usize = 64;

/// One triangle in the aircraft frame.
#[derive(Clone, Copy, Debug)]
pub(super) struct Triangle {
    pub(super) a: Vec3,
    pub(super) b: Vec3,
    pub(super) c: Vec3,
}

impl Triangle {
    fn centroid(&self) -> Vec3 {
        (self.a + self.b + self.c) / 3.0
    }

    /// Two-sided Möller–Trumbore distance along `dir` to the triangle, when within `t_max`;
    /// airframe skins are open sheets, so either face occludes.
    fn hit(&self, origin: Vec3, dir: Vec3, t_max: f32) -> Option<f32> {
        let e1 = self.b - self.a;
        let e2 = self.c - self.a;
        let p = dir.cross(e2);
        let det = e1.dot(p);
        if det.abs() < 1e-12 {
            return None;
        }
        let inv = 1.0 / det;
        let s = origin - self.a;
        let u = s.dot(p) * inv;
        if !(0.0..=1.0).contains(&u) {
            return None;
        }
        let q = s.cross(e1);
        let v = dir.dot(q) * inv;
        if v < 0.0 || u + v > 1.0 {
            return None;
        }
        let t = e2.dot(q) * inv;
        (t > 0.0 && t < t_max).then_some(t)
    }
}

#[derive(Clone, Copy, Debug)]
struct Node {
    min: Vec3,
    max: Vec3,
    /// Leaf: first triangle; interior: index of the right child (the left follows).
    first: u32,
    /// Triangles in a leaf, `0` for an interior node.
    count: u32,
}

/// Flattened median-split hierarchy; triangles are reordered so each leaf is a range.
pub(super) struct Bvh {
    nodes: Vec<Node>,
    triangles: Vec<Triangle>,
}

impl Bvh {
    pub(super) fn new(mut triangles: Vec<Triangle>) -> Self {
        let mut nodes = Vec::with_capacity(triangles.len() / LEAF_SIZE * 2 + 1);
        let len = triangles.len();
        if len > 0 {
            build(&mut nodes, &mut triangles, 0, len);
        }
        Self { nodes, triangles }
    }

    /// Distance to the nearest triangle along `dir` within `t_max`, if any.
    pub(super) fn nearest_hit(&self, origin: Vec3, dir: Vec3, t_max: f32) -> Option<f32> {
        let inv_dir = dir.recip();
        let mut best = t_max;
        let mut stack = [0_u32; STACK_DEPTH];
        let mut depth = 1_usize;
        while let Some(next) = depth.checked_sub(1) {
            depth = next;
            let Some(&index) = stack.get(depth) else {
                continue;
            };
            let Some(node) = self.nodes.get(index as usize) else {
                continue;
            };
            if !slab(node.min, node.max, origin, inv_dir, best) {
                continue;
            }
            if node.count > 0 {
                let start = node.first as usize;
                let end = start.saturating_add(node.count as usize);
                for triangle in self.triangles.get(start..end).unwrap_or(&[]) {
                    if let Some(t) = triangle.hit(origin, dir, best) {
                        best = t;
                    }
                }
                continue;
            }
            for child in [node.first, index.wrapping_add(1)] {
                if let Some(slot) = stack.get_mut(depth) {
                    *slot = child;
                    depth = depth.wrapping_add(1);
                }
            }
        }
        (best < t_max).then_some(best)
    }
}

/// Builds the subtree over `triangles[start..end]` and returns its node index.
fn build(nodes: &mut Vec<Node>, triangles: &mut [Triangle], start: usize, end: usize) -> u32 {
    let index = u32::try_from(nodes.len()).unwrap_or(u32::MAX);
    let slice = triangles.get(start..end).unwrap_or(&[]);
    let (min, max) = slice.iter().fold(
        (Vec3::splat(f32::INFINITY), Vec3::splat(f32::NEG_INFINITY)),
        |(lo, hi), t| (lo.min(t.a.min(t.b).min(t.c)), hi.max(t.a.max(t.b).max(t.c))),
    );
    let count = end.saturating_sub(start);
    nodes.push(Node {
        min,
        max,
        first: u32::try_from(start).unwrap_or(u32::MAX),
        count: u32::try_from(count).unwrap_or(u32::MAX),
    });
    if count <= LEAF_SIZE {
        return index;
    }
    let extent = max - min;
    let axis: u8 = if extent.x >= extent.y && extent.x >= extent.z {
        0
    } else if extent.y >= extent.z {
        1
    } else {
        2
    };
    let mid = start + count / 2;
    if let Some(range) = triangles.get_mut(start..end) {
        range.select_nth_unstable_by(count / 2, |p, q| {
            component(p.centroid(), axis).total_cmp(&component(q.centroid(), axis))
        });
    }
    build(nodes, triangles, start, mid);
    let right = build(nodes, triangles, mid, end);
    if let Some(node) = nodes.get_mut(index as usize) {
        node.first = right;
        node.count = 0;
    }
    index
}

fn component(v: Vec3, axis: u8) -> f32 {
    match axis {
        0 => v.x,
        1 => v.y,
        _ => v.z,
    }
}

/// Whether the ray enters the box before `t_max`.
fn slab(min: Vec3, max: Vec3, origin: Vec3, inv_dir: Vec3, t_max: f32) -> bool {
    let t0 = (min - origin) * inv_dir;
    let t1 = (max - origin) * inv_dir;
    let near = t0.min(t1).max_element().max(0.0);
    let far = t0.max(t1).min_element().min(t_max);
    near <= far
}
