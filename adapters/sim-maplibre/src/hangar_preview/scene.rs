//! The previewed asset's GPU meshes, rest bounds, and per-frame articulated draws.

use std::hash::{DefaultHasher, Hash, Hasher};

use glam::{DMat4, DVec3, Vec3};
use sim_core::AirframeArticulation;

use super::{
    gpu::{PartMesh, PipelineKind, upload_mesh},
    orbit::{Bounds, StudioRig},
};
use crate::{
    PartRole, SlotCameraRole, UsdAircraftAsset, UsdMeshPart, UsdPartDraw,
    usd_pass::{DRAW_BYTES, DrawBlock},
};

/// The uploaded asset: a private copy for per-frame articulation plus its GPU meshes.
pub(super) struct SceneAsset {
    pub(super) fingerprint: u64,
    pub(super) asset: UsdAircraftAsset,
    pub(super) meshes: Vec<Option<PartMesh>>,
    pub(super) bounds: Bounds,
}

impl SceneAsset {
    pub(super) fn upload(device: &wgpu::Device, asset: &UsdAircraftAsset) -> Self {
        Self {
            fingerprint: fingerprint(asset),
            asset: asset.clone(),
            meshes: asset
                .parts()
                .iter()
                .map(|part| upload_mesh(device, &part.vertices, &part.indices))
                .collect(),
            bounds: rest_bounds(asset),
        }
    }
}

/// Content identity cheap enough to compute every frame: part paths, sizes, materials,
/// rest placement, and end vertices distinguish every built-in and loaded vehicle.
pub(super) fn fingerprint(asset: &UsdAircraftAsset) -> u64 {
    let mut hasher = DefaultHasher::new();
    asset.parts().len().hash(&mut hasher);
    for part in asset.parts() {
        part.prim_path.hash(&mut hasher);
        part.vertices.len().hash(&mut hasher);
        part.indices.len().hash(&mut hasher);
        for value in part
            .material
            .diffuse_rgb
            .iter()
            .chain(&part.material.emissive_rgb)
        {
            value.to_bits().hash(&mut hasher);
        }
        for value in part.rest_to_aircraft.w_axis.to_array() {
            value.to_bits().hash(&mut hasher);
        }
        // End vertices catch re-authored geometry that keeps the same topology and names.
        for vertex in part
            .vertices
            .first()
            .into_iter()
            .chain(part.vertices.last())
        {
            for value in vertex.position {
                value.to_bits().hash(&mut hasher);
            }
        }
    }
    hasher.finish()
}

fn rest_bounds(asset: &UsdAircraftAsset) -> Bounds {
    let mut min = DVec3::splat(f64::INFINITY);
    let mut max = DVec3::splat(f64::NEG_INFINITY);
    for part in asset.parts() {
        for vertex in &part.vertices {
            let p = part
                .rest_to_aircraft
                .transform_point3(Vec3::from_array(vertex.position).as_dvec3());
            if p.is_finite() {
                min = min.min(p);
                max = max.max(p);
            }
        }
    }
    if min.is_finite() && max.is_finite() && (max - min).max_element() > 1e-3 {
        Bounds { min, max }
    } else {
        Bounds::default()
    }
}

/// One packed part draw.
pub(super) struct HangarDraw {
    pub(super) part_index: usize,
    pub(super) pipeline: PipelineKind,
    pub(super) uniform: [u8; DRAW_BYTES],
}

/// Articulates every part through [`UsdAircraftAsset::evaluate_draws`] (the in-world
/// pass's joint and propeller mechanism) and packs opaque draws first, then translucent
/// ones back to front.
pub(super) fn pack_draws(
    asset: &UsdAircraftAsset,
    articulation: &AirframeArticulation,
    rig: &StudioRig,
) -> Vec<HangarDraw> {
    let mut packed: Vec<(bool, f64, HangarDraw)> = asset
        .evaluate_draws(articulation, SlotCameraRole::External)
        .iter()
        .filter_map(|draw| {
            let part = asset.parts().get(draw.part_index)?;
            Some(pack_part(part, draw, rig))
        })
        .collect();
    packed.sort_by(|(ta, da, _), (tb, db, _)| ta.cmp(tb).then_with(|| db.total_cmp(da)));
    packed.into_iter().map(|(_, _, draw)| draw).collect()
}

fn pack_part(part: &UsdMeshPart, draw: &UsdPartDraw, rig: &StudioRig) -> (bool, f64, HangarDraw) {
    let translucent = draw.is_prop_disc || draw.opacity < 0.999;
    let kind = if draw.is_prop_disc {
        2.0
    } else if translucent {
        1.0
    } else {
        0.0
    };
    // Glass stays see-through, matching the in-world aircraft pass.
    let opacity = if part.role == PartRole::CabinGlass {
        draw.opacity.min(0.28)
    } else {
        draw.opacity
    };
    let world_from_part: DMat4 = rig.world_from_aircraft * draw.part_to_aircraft;
    let material = part.material;
    let [r, g, b] = material.diffuse_rgb;
    let [er, eg, eb] = material.emissive_rgb.map(|c| c * draw.emissive_scale);
    let block = DrawBlock {
        clip_from_part: (rig.clip_from_world * world_from_part).as_mat4(),
        enu_from_part: world_from_part.as_mat4(),
        base_color: [r, g, b, opacity],
        emissive: [er, eg, eb, material.metallic],
        params: [
            material.roughness,
            draw.prop_phase_rad,
            draw.shutter_blur_rad,
            kind,
        ],
        sun: [rig.key.x as f32, rig.key.y as f32, rig.key.z as f32, 1.0],
    };
    let pipeline = match (translucent, part.double_sided) {
        (true, _) => PipelineKind::Translucent,
        (false, true) => PipelineKind::OpaqueDoubleSided,
        (false, false) => PipelineKind::Opaque,
    };
    (
        translucent,
        world_from_part.w_axis.truncate().length(),
        HangarDraw {
            part_index: draw.part_index,
            pipeline,
            uniform: block.to_le_bytes(),
        },
    )
}
