use std::f32::consts::TAU;

use super::mesh::{Mesh, point};

/// Fewest chords on a join or cap disc.
const JOIN_MIN_SEGMENTS: usize = 8;
/// Fewest chords on a scene circle.
const CIRCLE_MIN_SEGMENTS: usize = 48;
/// Fewest chords per radian of an arc: one every four degrees.
const ARC_SEGMENTS_PER_RAD: f32 = 45.0 / std::f32::consts::PI;

/// Strokes a path as the union of round-ended capsules over its segments,
/// the rasterizer's stroke model: every vertex gets a full disc, so joins
/// and caps are round. The pass's per-operation depth test blends samples
/// the pieces share only once.
pub(super) fn stroke_path(mesh: &mut Mesh, verts: &[[f32; 2]], closed: bool, hw: f32) {
    if hw <= 0.0 || verts.is_empty() {
        return;
    }
    let edges = verts.iter().zip(verts.iter().skip(1));
    let closing = closed
        .then(|| verts.last().zip(verts.first()))
        .flatten()
        .filter(|_| verts.len() > 1);
    for (a, b) in edges.chain(closing) {
        segment(mesh, *a, *b, hw);
    }
    for v in verts {
        mesh.disc(*v, hw, JOIN_MIN_SEGMENTS);
    }
}

fn segment(mesh: &mut Mesh, a: [f32; 2], b: [f32; 2], hw: f32) {
    let (dx, dy) = (b[0] - a[0], b[1] - a[1]);
    let len = dx.hypot(dy);
    if len <= f32::EPSILON {
        return;
    }
    let (nx, ny) = (-dy / len * hw, dx / len * hw);
    mesh.quad([
        [a[0] + nx, a[1] + ny],
        [b[0] + nx, b[1] + ny],
        [b[0] - nx, b[1] - ny],
        [a[0] - nx, a[1] - ny],
    ]);
}

/// Fills a polygon: a fan when convex, ear clipping when simple and
/// concave. A self-intersecting outline has no ear decomposition and falls
/// back to a fan, which approximates rather than reproduces the
/// rasterizer's non-zero winding there.
pub(super) fn fill_polygon(mesh: &mut Mesh, verts: &[[f32; 2]]) {
    if verts.len() < 3 {
        return;
    }
    if is_convex(verts) {
        fan(mesh, verts);
    } else {
        ear_clip(mesh, verts);
    }
}

fn fan(mesh: &mut Mesh, verts: &[[f32; 2]]) {
    let Some((first, rest)) = verts.split_first() else {
        return;
    };
    for pair in rest.windows(2) {
        if let [b, c] = pair {
            mesh.tri(*first, *b, *c);
        }
    }
}

fn cross(o: [f32; 2], a: [f32; 2], b: [f32; 2]) -> f32 {
    (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
}

fn ring_get(verts: &[[f32; 2]], i: usize) -> [f32; 2] {
    let n = verts.len().max(1);
    verts.get(i % n).copied().unwrap_or_default()
}

fn is_convex(verts: &[[f32; 2]]) -> bool {
    let n = verts.len();
    let mut sign = 0.0_f32;
    for i in 0..n {
        let c = cross(
            ring_get(verts, i),
            ring_get(verts, i.wrapping_add(1)),
            ring_get(verts, i.wrapping_add(2)),
        );
        if c.abs() <= f32::EPSILON {
            continue;
        }
        if sign == 0.0 {
            sign = c.signum();
        } else if c.signum() != sign {
            return false;
        }
    }
    true
}

fn signed_area(verts: &[[f32; 2]]) -> f32 {
    let n = verts.len();
    (0..n)
        .map(|i| {
            let (a, b) = (ring_get(verts, i), ring_get(verts, i.wrapping_add(1)));
            a[0] * b[1] - b[0] * a[1]
        })
        .sum::<f32>()
        / 2.0
}

fn inside(p: [f32; 2], a: [f32; 2], b: [f32; 2], c: [f32; 2]) -> bool {
    cross(a, b, p) >= 0.0 && cross(b, c, p) >= 0.0 && cross(c, a, p) >= 0.0
}

fn ear_clip(mesh: &mut Mesh, verts: &[[f32; 2]]) {
    let orient = signed_area(verts).signum();
    let mut ring: Vec<[f32; 2]> = verts
        .iter()
        .map(|&[x, y]| if orient < 0.0 { [x, -y] } else { [x, y] })
        .collect();
    // Clip in a counter-clockwise copy; emit from the original positions.
    let mut index: Vec<usize> = (0..verts.len()).collect();
    let original = |i: usize| verts.get(i).copied().unwrap_or_default();
    while ring.len() > 3 {
        let n = ring.len();
        let ear = (0..n).find(|&i| {
            let (a, b, c) = (
                ring_get(&ring, i + n - 1),
                ring_get(&ring, i),
                ring_get(&ring, i + 1),
            );
            cross(a, b, c) > 0.0
                && !(0..n)
                    .filter(|&j| j != i && j != (i + n - 1) % n && j != (i + 1) % n)
                    .any(|j| inside(ring_get(&ring, j), a, b, c))
        });
        let Some(i) = ear else {
            let rest: Vec<[f32; 2]> = index.iter().map(|&k| original(k)).collect();
            fan(mesh, &rest);
            return;
        };
        let at = |k: usize| index.get(k % n).copied().map(original).unwrap_or_default();
        mesh.tri(at(i + n - 1), at(i), at(i + 1));
        ring.remove(i);
        index.remove(i);
    }
    let rest: Vec<[f32; 2]> = index.iter().map(|&k| original(k)).collect();
    fan(mesh, &rest);
}

/// A filled circle.
pub(super) fn fill_circle(mesh: &mut Mesh, c: [f32; 2], r: f32) {
    mesh.disc(c, r, CIRCLE_MIN_SEGMENTS);
}

/// A circle outline centred on the ideal ring at half-width `hw`.
pub(super) fn stroke_circle(mesh: &mut Mesh, c: [f32; 2], r: f32, hw: f32) {
    if hw <= 0.0 {
        return;
    }
    mesh.ring(c, [r - hw, r + hw], 0.0, TAU, CIRCLE_MIN_SEGMENTS);
}

/// An arc from device angle `start` sweeping `sweep`, with round caps at
/// both ends; a sweep of a full turn or more is a whole ring.
pub(super) fn stroke_arc(mesh: &mut Mesh, c: [f32; 2], r: f32, start: f32, sweep: f32, hw: f32) {
    if hw <= 0.0 {
        return;
    }
    let sweep = sweep.clamp(-TAU, TAU);
    let min = (sweep.abs() * ARC_SEGMENTS_PER_RAD).ceil() as usize;
    mesh.ring(c, [r - hw, r + hw], start, sweep, min.max(1));
    mesh.disc(point(c, r, start), hw, JOIN_MIN_SEGMENTS);
    mesh.disc(point(c, r, start + sweep), hw, JOIN_MIN_SEGMENTS);
}
