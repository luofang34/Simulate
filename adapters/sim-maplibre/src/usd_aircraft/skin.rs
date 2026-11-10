//! Baked physically based skins: an airframe's albedo, tangent-space normal
//! and occlusion/roughness/metallic maps, sampled through the texture
//! coordinates its skinned parts carry.
//!
//! The bake is generic: it lays the chosen parts' islands out in one atlas
//! (see `chart` and `pack`), and asks a [`SkinSource`] what the skin looks
//! like at the point of the airframe each texel shows. The source owns the
//! paint, the markings and the wear; the layout and the maps follow the
//! geometry, so a re-shaped airframe re-bakes without touching the source.

mod chart;
mod map;
mod pack;
mod raster;
mod tangent;
#[cfg(test)]
mod tests;

use glam::{Vec2, Vec3};

pub use map::{SkinFormat, SkinMap};

use crate::{PartSurface, SurfaceDetail, UsdMeshPart, UsdMeshVertex};
use raster::{Canvas, Corner, Job};

/// The three maps of an airframe's skin, sharing one texture-coordinate
/// layout.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct SkinTextures {
    /// Base colour ([`SkinFormat::Srgba8`]); alpha is unused.
    pub albedo: SkinMap,
    /// Tangent-space normal ([`SkinFormat::Normal8`]).
    pub normal: SkinMap,
    /// Occlusion, roughness and metallic in `r`, `g`, `b`
    /// ([`SkinFormat::Rgba8`]).
    pub orm: SkinMap,
}

impl SkinTextures {
    /// Bytes of all three maps and their mips: the skin's GPU memory.
    #[must_use]
    pub fn byte_len(&self) -> usize {
        self.albedo.byte_len() + self.normal.byte_len() + self.orm.byte_len()
    }
}

/// The point of the airframe one texel shows, in the aircraft frame at rest
/// (`+X` right, `+Y` forward, `+Z` up, metres).
#[derive(Clone, Copy, Debug)]
pub(crate) struct SkinTexel {
    /// Index of the part in the asset.
    pub(crate) part: usize,
    pub(crate) position: Vec3,
    /// Unit outward surface normal.
    pub(crate) normal: Vec3,
    /// Unit tangent along increasing `u` and the bitangent along increasing
    /// `v` of the atlas, both in the surface.
    pub(crate) tangent: Vec3,
    pub(crate) bitangent: Vec3,
    /// The vertex-baked share of open sky.
    pub(crate) occlusion: f32,
    /// Width of skin one texel covers, metres.
    pub(crate) footprint_m: f32,
}

/// What the skin looks like at a texel.
#[derive(Clone, Copy, Debug)]
pub(crate) struct SkinSample {
    /// Linear base colour.
    pub(crate) albedo: Vec3,
    /// Occlusion beyond the vertex bake (seams, rivet edges), `1` open.
    pub(crate) occlusion: f32,
    pub(crate) roughness: f32,
    pub(crate) metallic: f32,
    /// Shading normal in the aircraft frame.
    pub(crate) normal: Vec3,
}

/// The paint, markings and wear of a skin, asked texel by texel.
pub(crate) trait SkinSource: Sync {
    /// The skin at `texel`.
    fn sample(&self, texel: &SkinTexel) -> SkinSample;
}

/// Size of a bake.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct SkinLayout {
    /// Side of the square albedo and normal maps, texels.
    pub(crate) side: u32,
    /// Empty texels around each island, filled from its edge.
    pub(crate) gutter: u32,
    /// Whether the occlusion/roughness/metallic map is kept at half
    /// resolution: it varies more slowly than colour and relief.
    pub(crate) half_orm: bool,
}

impl crate::UsdAircraftAsset {
    /// Bakes the skin of the parts `skinned` picks out (see [`bake`]) and
    /// keeps it for the renderer.
    pub(crate) fn bake_skin(
        &mut self,
        layout: SkinLayout,
        source: &dyn SkinSource,
        skinned: impl Fn(&UsdMeshPart) -> bool,
    ) {
        let started = web_time::Instant::now();
        let chosen: Vec<usize> = self
            .parts
            .iter()
            .enumerate()
            .filter(|(_, part)| skinned(part))
            .map(|(index, _)| index)
            .collect();
        let textures = bake(&mut self.parts, &chosen, layout, source);
        tracing::info!(
            parts = chosen.len(),
            side = layout.side,
            gpu_mib = textures.as_ref().map_or(0, SkinTextures::byte_len) >> 20,
            elapsed_ms = started.elapsed().as_secs_f64() * 1e3,
            "baked airframe skin"
        );
        self.skin = textures.map(std::sync::Arc::new);
    }
}

/// Lays `skinned` parts of `parts` out in one atlas, gives them texture
/// coordinates and tangents into it, paints it from `source`, and marks the
/// parts [`PartSurface::Skin`]. `None`, leaving the parts untouched, when
/// nothing is skinned or the islands cannot be packed.
pub(crate) fn bake(
    parts: &mut [UsdMeshPart],
    skinned: &[usize],
    layout: SkinLayout,
    source: &dyn SkinSource,
) -> Option<SkinTextures> {
    let charts: Vec<(usize, Vec<chart::Island>)> = skinned
        .iter()
        .filter_map(|&index| Some((index, chart::islands(parts.get(index)?))))
        .filter(|(_, islands)| !islands.is_empty())
        .collect();
    let sizes: Vec<Vec2> = charts
        .iter()
        .flat_map(|(_, islands)| islands.iter().map(|island| island.size_m))
        .collect();
    if sizes.is_empty() {
        return None;
    }
    let packing = pack::pack(&sizes, layout.side, layout.gutter)?;
    let mut origins = packing.placements.iter().map(|p| p.origin);
    let mut canvas = Canvas::new(layout.side);
    let mut jobs = Vec::new();
    let side = layout.side as f32;
    for (index, islands) in charts {
        let Some(part) = parts.get_mut(index) else {
            continue;
        };
        let mut vertices = Vec::new();
        let mut triangles = Vec::new();
        for island in islands {
            let origin = origins.next().unwrap_or(Vec2::ZERO);
            let base = u32::try_from(vertices.len()).unwrap_or(0);
            for (&source_index, &uv_m) in island.source.iter().zip(&island.uv_m) {
                let Some(&vertex) = part.vertices.get(source_index as usize) else {
                    continue;
                };
                let texel = origin + uv_m * packing.texels_per_m;
                vertices.push(UsdMeshVertex {
                    uv: (texel / side).to_array(),
                    ..vertex
                });
            }
            triangles.extend(island.triangles.iter().map(|t| t.map(|i| i + base)));
        }
        give_tangents(&mut vertices, &triangles);
        let footprint = 1.0 / packing.texels_per_m.max(1e-6);
        jobs.extend(triangles.iter().map(|triangle| Job {
            corners: triangle.map(|i| corner(part, &vertices, i, side)),
            part: index,
            footprint_m: footprint,
        }));
        part.indices = triangles.iter().flatten().copied().collect();
        part.vertices = vertices;
        part.surface = PartSurface::Skin;
        part.detail = SurfaceDetail::default();
    }
    canvas.fill_all(&jobs, source);
    canvas.dilate(layout.gutter.saturating_add(1));
    Some(canvas.into_textures(layout))
}

fn give_tangents(vertices: &mut [UsdMeshVertex], triangles: &[[u32; 3]]) {
    let positions: Vec<Vec3> = vertices.iter().map(|v| Vec3::from(v.position)).collect();
    let normals: Vec<Vec3> = vertices.iter().map(|v| Vec3::from(v.normal)).collect();
    let uvs: Vec<Vec2> = vertices.iter().map(|v| Vec2::from(v.uv)).collect();
    for (vertex, t) in vertices
        .iter_mut()
        .zip(tangent::tangents(&positions, &normals, &uvs, triangles))
    {
        vertex.tangent = t;
    }
}

/// The corner of `part` at vertex `i` of its new buffers, in the aircraft
/// frame and in atlas texels.
fn corner(part: &UsdMeshPart, vertices: &[UsdMeshVertex], i: u32, side: f32) -> Corner {
    let rest = part.rest_to_aircraft.as_mat4();
    let v = vertices.get(i as usize).copied().unwrap_or(UsdMeshVertex {
        position: [0.0; 3],
        normal: [0.0, 0.0, 1.0],
        uv: [0.0; 2],
        occlusion: 1.0,
        tangent: UsdMeshVertex::NO_TANGENT,
    });
    let [tx, ty, tz, tw] = v.tangent;
    Corner {
        texel: Vec2::from(v.uv) * side,
        position: rest.transform_point3(Vec3::from(v.position)),
        normal: rest
            .transform_vector3(Vec3::from(v.normal))
            .normalize_or(Vec3::Z),
        tangent: rest.transform_vector3(Vec3::new(tx, ty, tz)).extend(tw),
        occlusion: v.occlusion,
    }
}
