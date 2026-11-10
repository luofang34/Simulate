//! The panel board: a slab cut to the measured outline whose pilot-side face
//! is cut with round instrument holes. Each hole sits in a square tile meshed
//! from the square's edge in to the circle. The rest of the face is the grid
//! the tile edges and outline corners cut the board's bounding box into, each
//! cell clipped to the outline; with every corner on a grid line, an outline
//! edge crosses a cell in one straight line, so every clipped cell is convex.

use std::f32::consts::TAU;

use glam::{Vec2, Vec3};

use crate::mesh::MeshBuilder;

/// Margin from a hole to the edge of its tile.
pub(super) const TILE_MARGIN_M: f32 = 0.002;
const HOLE_SEGMENTS: usize = 48;

/// Twice the signed area of `outline`, positive counter-clockwise.
fn signed_area2(outline: &[Vec2]) -> f32 {
    let n = outline.len();
    (0..n)
        .filter_map(|i| Some(outline.get(i)?.perp_dot(*outline.get((i + 1) % n)?)))
        .sum()
}

/// `outline` counter-clockwise.
fn counter_clockwise(outline: &[[f32; 2]]) -> Vec<Vec2> {
    let mut ring: Vec<Vec2> = outline.iter().map(|&p| Vec2::from(p)).collect();
    if signed_area2(&ring) < 0.0 {
        ring.reverse();
    }
    ring
}

/// `polygon` clipped to the half-plane `keep(p) >= 0`, `keep` linear.
fn clip(polygon: &[Vec2], keep: impl Fn(Vec2) -> f32) -> Vec<Vec2> {
    let mut out = Vec::new();
    let n = polygon.len();
    for i in 0..n {
        let (Some(&a), Some(&b)) = (polygon.get(i), polygon.get((i + 1) % n)) else {
            continue;
        };
        let (da, db) = (keep(a), keep(b));
        if da >= 0.0 {
            out.push(a);
        }
        if (da >= 0.0) != (db >= 0.0) {
            out.push(a + (b - a) * (da / (da - db)));
        }
    }
    out
}

/// The part of `outline` inside the rectangle `[x0, x1] x [z0, z1]`.
fn clip_to_cell(outline: &[Vec2], [x0, x1, z0, z1]: [f32; 4]) -> Vec<Vec2> {
    let piece = clip(outline, |p| p.x - x0);
    let piece = clip(&piece, |p| x1 - p.x);
    let piece = clip(&piece, |p| p.y - z0);
    clip(&piece, |p| z1 - p.y)
}

/// The part of `outline` inside the convex counter-clockwise `region`.
fn clip_to_convex(outline: &[Vec2], region: &[Vec2]) -> Vec<Vec2> {
    let n = region.len();
    (0..n).fold(outline.to_vec(), |piece, i| {
        match (region.get(i), region.get((i + 1) % n)) {
            (Some(&a), Some(&b)) => clip(&piece, |p| (b - a).perp_dot(p - a)),
            _ => piece,
        }
    })
}

/// Whether `p` lies inside `ring`, by crossings of a ray toward `+x`.
#[cfg(test)]
fn contains(ring: &[Vec2], p: Vec2) -> bool {
    let n = ring.len();
    (0..n)
        .filter(|&i| match (ring.get(i), ring.get((i + 1) % n)) {
            (Some(&a), Some(&b)) => {
                (a.y > p.y) != (b.y > p.y) && p.x < a.x + (p.y - a.y) / (b.y - a.y) * (b.x - a.x)
            }
            _ => false,
        })
        .count()
        % 2
        == 1
}

/// Whether the circle of `radius` about `centre` lies wholly inside `outline`.
#[cfg(test)]
pub(super) fn circle_inside(outline: &[[f32; 2]], centre: [f32; 2], radius: f32) -> bool {
    let ring = counter_clockwise(outline);
    (0..HOLE_SEGMENTS).all(|i| {
        let a = TAU * i as f32 / HOLE_SEGMENTS as f32;
        contains(
            &ring,
            Vec2::from(centre) + Vec2::new(a.sin(), a.cos()) * radius,
        )
    })
}

/// Half-size of each hole's square tile: the margin beyond the hole, or less
/// where two diagonal neighbours' tiles would overlap, split between them.
/// Holes whose circumscribed squares overlap cannot be tiled.
pub(super) fn tile_halves(holes: &[([f32; 2], f32)]) -> Vec<f32> {
    holes
        .iter()
        .enumerate()
        .map(|(i, &([xi, zi], ri))| {
            holes
                .iter()
                .enumerate()
                .filter(|&(j, _)| j != i)
                .map(|(_, &([xj, zj], rj))| {
                    let apart = (xi - xj).abs().max((zi - zj).abs());
                    ri + 0.5 * (apart - ri - rj)
                })
                .fold(ri + TILE_MARGIN_M, f32::min)
        })
        .collect()
}

/// A board `half_thickness` deep about the panel-local plane `y = 0`, cut to
/// `outline` (`[right, up]`), its `-Y` face cut with `holes`
/// (`([right, up], radius)`). Tiles must lie inside the outline and apart.
pub(super) fn board_with_holes(
    outline: &[[f32; 2]],
    half_thickness: f32,
    holes: &[([f32; 2], f32)],
) -> MeshBuilder {
    let mut mesh = MeshBuilder::new();
    let front = -half_thickness;
    let ring = counter_clockwise(outline);
    let halves = tile_halves(holes);
    let tiles: Vec<[f32; 4]> = holes
        .iter()
        .zip(&halves)
        .map(|(&([x, z], _), &t)| [x - t, x + t, z - t, z + t])
        .collect();
    let mut xs: Vec<f32> = ring.iter().map(|p| p.x).collect();
    let mut zs: Vec<f32> = ring.iter().map(|p| p.y).collect();
    for [x0, x1, z0, z1] in &tiles {
        xs.extend([x0, x1]);
        zs.extend([z0, z1]);
    }
    for axis in [&mut xs, &mut zs] {
        axis.sort_by(f32::total_cmp);
        axis.dedup_by(|a, b| (*a - *b).abs() < 1e-6);
    }
    for xw in xs.windows(2) {
        for zw in zs.windows(2) {
            let (&[x0, x1], &[z0, z1]) = (xw, zw) else {
                continue;
            };
            let (cx, cz) = (0.5 * (x0 + x1), 0.5 * (z0 + z1));
            let in_tile = tiles
                .iter()
                .any(|[a, b, c, d]| cx > *a && cx < *b && cz > *c && cz < *d);
            let piece = clip_to_cell(&ring, [x0, x1, z0, z1]);
            if !in_tile && signed_area2(&piece) > 1e-9 {
                let corners: Vec<Vec3> = piece.iter().map(|p| Vec3::new(p.x, front, p.y)).collect();
                mesh.polygon(&corners, Vec3::NEG_Y);
            }
        }
    }
    for (&(centre, r), &t) in holes.iter().zip(&halves) {
        holed_tile(&mut mesh, &ring, centre, [r, t], front);
    }
    back_and_edges(&mut mesh, &ring, half_thickness);
    mesh
}

/// The square around one hole, as quads from each square-edge point in to the
/// circle point on the same ray, each clipped to the outline where a tile's
/// corner reaches past the board's edge.
fn holed_tile(
    mesh: &mut MeshBuilder,
    ring: &[Vec2],
    [x, z]: [f32; 2],
    [r, t]: [f32; 2],
    front: f32,
) {
    let centre = Vec2::new(x, z);
    let ray = |i: usize| {
        let a = TAU * i as f32 / HOLE_SEGMENTS as f32;
        Vec2::new(a.sin(), a.cos())
    };
    for i in 0..HOLE_SEGMENTS {
        let (d0, d1) = (ray(i), ray(i + 1));
        let to_square = |d: Vec2| d * (t / d.x.abs().max(d.y.abs()));
        let mut quad = [
            centre + d0 * r,
            centre + to_square(d0),
            centre + to_square(d1),
            centre + d1 * r,
        ];
        if signed_area2(&quad) < 0.0 {
            quad.reverse();
        }
        let piece = clip_to_convex(ring, &quad);
        if signed_area2(&piece) > 1e-10 {
            let corners: Vec<Vec3> = piece.iter().map(|p| Vec3::new(p.x, front, p.y)).collect();
            mesh.polygon(&corners, Vec3::NEG_Y);
        }
    }
}

/// The back face, in strips between outline corners so the concave outline
/// needs no general triangulation, and the edge walls round the outline.
fn back_and_edges(mesh: &mut MeshBuilder, ring: &[Vec2], half_t: f32) {
    let mut xs: Vec<f32> = ring.iter().map(|p| p.x).collect();
    xs.sort_by(f32::total_cmp);
    xs.dedup_by(|a, b| (*a - *b).abs() < 1e-6);
    let (z_lo, z_hi) = ring.iter().fold((f32::MAX, f32::MIN), |(lo, hi), p| {
        (lo.min(p.y), hi.max(p.y))
    });
    for xw in xs.windows(2) {
        let &[x0, x1] = xw else {
            continue;
        };
        let piece = clip_to_cell(ring, [x0, x1, z_lo, z_hi]);
        if signed_area2(&piece) > 1e-9 {
            let corners: Vec<Vec3> = piece.iter().map(|p| Vec3::new(p.x, half_t, p.y)).collect();
            mesh.polygon(&corners, Vec3::Y);
        }
    }
    let n = ring.len();
    for i in 0..n {
        let (Some(&a), Some(&b)) = (ring.get(i), ring.get((i + 1) % n)) else {
            continue;
        };
        let edge = b - a;
        let out = Vec3::new(edge.y, 0.0, -edge.x);
        mesh.polygon(
            &[
                Vec3::new(a.x, -half_t, a.y),
                Vec3::new(b.x, -half_t, b.y),
                Vec3::new(b.x, half_t, b.y),
                Vec3::new(a.x, half_t, a.y),
            ],
            out,
        );
    }
}
