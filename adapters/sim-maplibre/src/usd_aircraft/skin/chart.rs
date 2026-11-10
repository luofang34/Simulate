//! Texture-coordinate islands of a skinned part.
//!
//! A part's triangles fall into islands: those whose builder laid out
//! texture coordinates in metres (a loft's station by girth) keep them and
//! join where they share vertices; the rest are projected onto the plane of
//! their dominant axis and join where they share a position on the same
//! axis. Every island is then a flat, non-overlapping patch of skin in
//! metres, ready to pack.

use std::collections::BTreeMap;

use glam::{Vec2, Vec3};

use crate::UsdMeshPart;

/// Below this ratio of texture to surface area a triangle has no layout of
/// its own (a cap or a hand-placed facet), and is projected instead.
const MIN_UV_AREA_RATIO: f32 = 1e-3;
/// Positions closer than this weld when projected triangles are joined.
const WELD_M: f32 = 1e-4;

/// How a triangle's texture coordinates are found.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
enum Layout {
    /// The builder's own, in metres.
    Given,
    /// Projected along axis `0..3`, onto the face whose normal has sign
    /// `positive`.
    Projected { axis: u8, positive: bool },
}

/// One flat patch of a part's skin: its corners and their coordinates in
/// metres, origin at the patch's lower corner.
#[derive(Clone, Debug)]
pub(crate) struct Island {
    /// The part's vertex each island vertex copies.
    pub(crate) source: Vec<u32>,
    /// Coordinates of each island vertex, metres from the patch's corner.
    pub(crate) uv_m: Vec<Vec2>,
    /// Triangles over island vertices.
    pub(crate) triangles: Vec<[u32; 3]>,
    /// Width and height of the patch, metres.
    pub(crate) size_m: Vec2,
}

fn position(part: &UsdMeshPart, index: u32) -> Vec3 {
    part.vertices
        .get(index as usize)
        .map_or(Vec3::ZERO, |v| Vec3::from_array(v.position))
}

fn given_uv(part: &UsdMeshPart, index: u32) -> Vec2 {
    part.vertices
        .get(index as usize)
        .map_or(Vec2::ZERO, |v| Vec2::from_array(v.uv))
}

fn layout_of(part: &UsdMeshPart, [a, b, c]: [u32; 3]) -> Layout {
    let (pa, pb, pc) = (position(part, a), position(part, b), position(part, c));
    let normal = (pb - pa).cross(pc - pa);
    let area = normal.length();
    let (ua, ub, uc) = (given_uv(part, a), given_uv(part, b), given_uv(part, c));
    let uv_area = (ub - ua).perp_dot(uc - ua).abs();
    if area > 0.0 && uv_area > MIN_UV_AREA_RATIO * area {
        return Layout::Given;
    }
    let magnitude = normal.abs();
    let axis = if magnitude.x >= magnitude.y && magnitude.x >= magnitude.z {
        0
    } else if magnitude.y >= magnitude.z {
        1
    } else {
        2
    };
    Layout::Projected {
        axis,
        positive: normal
            .to_array()
            .get(usize::from(axis))
            .copied()
            .unwrap_or(0.0)
            >= 0.0,
    }
}

/// Coordinates of `p` on the face a projected triangle lies on, laid out
/// so the face reads unmirrored from outside.
fn project(p: Vec3, axis: u8, positive: bool) -> Vec2 {
    let s = if positive { 1.0 } else { -1.0 };
    match axis {
        0 => Vec2::new(-s * p.y, -p.z),
        1 => Vec2::new(s * p.x, -p.z),
        _ => Vec2::new(p.x, -s * p.y),
    }
}

/// Union-find over the joining keys of a part's triangles.
struct Joins {
    parent: Vec<usize>,
}

impl Joins {
    fn find(&mut self, mut key: usize) -> usize {
        while let Some(&up) = self.parent.get(key) {
            if up == key {
                break;
            }
            let grand = self.parent.get(up).copied().unwrap_or(up);
            if let Some(slot) = self.parent.get_mut(key) {
                *slot = grand;
            }
            key = up;
        }
        key
    }

    fn join(&mut self, a: usize, b: usize) {
        let (ra, rb) = (self.find(a), self.find(b));
        let (low, high) = (ra.min(rb), ra.max(rb));
        if let Some(slot) = self.parent.get_mut(high) {
            *slot = low;
        }
    }
}

/// The joining key of corner `index` of a triangle laid out by `layout`:
/// its vertex for given coordinates, its welded position on the face for
/// projected ones.
fn corner_key(
    part: &UsdMeshPart,
    index: u32,
    layout: Layout,
    keys: &mut BTreeMap<(Layout, [i64; 3], u32), usize>,
) -> usize {
    let welded = match layout {
        Layout::Given => [0; 3],
        Layout::Projected { .. } => position(part, index)
            .to_array()
            .map(|c| (c / WELD_M).round() as i64),
    };
    let vertex = if layout == Layout::Given { index } else { 0 };
    let next = keys.len();
    *keys.entry((layout, welded, vertex)).or_insert(next)
}

/// The islands of `part`, in a deterministic order.
pub(crate) fn islands(part: &UsdMeshPart) -> Vec<Island> {
    let triangles: Vec<[u32; 3]> = part.indices.as_chunks::<3>().0.to_vec();
    let mut keys = BTreeMap::new();
    let corners: Vec<(Layout, [usize; 3])> = triangles
        .iter()
        .map(|&tri| {
            let layout = layout_of(part, tri);
            (layout, tri.map(|i| corner_key(part, i, layout, &mut keys)))
        })
        .collect();
    let mut joins = Joins {
        parent: (0..keys.len()).collect(),
    };
    for (_, [a, b, c]) in &corners {
        joins.join(*a, *b);
        joins.join(*a, *c);
    }
    let mut by_root: BTreeMap<usize, Island> = BTreeMap::new();
    let mut slots: BTreeMap<(usize, u32, Layout), u32> = BTreeMap::new();
    for (tri, (layout, [key, ..])) in triangles.iter().zip(&corners) {
        let root = joins.find(*key);
        let island = by_root.entry(root).or_insert_with(|| Island {
            source: Vec::new(),
            uv_m: Vec::new(),
            triangles: Vec::new(),
            size_m: Vec2::ZERO,
        });
        let local = tri.map(|index| {
            *slots.entry((root, index, *layout)).or_insert_with(|| {
                let uv = match *layout {
                    Layout::Given => given_uv(part, index),
                    Layout::Projected { axis, positive } => {
                        project(position(part, index), axis, positive)
                    }
                };
                island.source.push(index);
                island.uv_m.push(uv);
                u32::try_from(island.uv_m.len() - 1).unwrap_or(0)
            })
        });
        island.triangles.push(local);
    }
    by_root.into_values().map(normalised).collect()
}

/// `island` moved so its patch starts at the origin.
fn normalised(mut island: Island) -> Island {
    let lo = island
        .uv_m
        .iter()
        .fold(Vec2::splat(f32::INFINITY), |a, &b| a.min(b));
    let hi = island
        .uv_m
        .iter()
        .fold(Vec2::splat(f32::NEG_INFINITY), |a, &b| a.max(b));
    for uv in &mut island.uv_m {
        *uv -= lo;
    }
    island.size_m = (hi - lo).max(Vec2::ZERO);
    island
}
