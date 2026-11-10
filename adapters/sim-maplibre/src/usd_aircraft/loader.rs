//! OpenUSD stage composition and `UsdGeomMesh` + `UsdPreviewSurface` extraction.

use glam::{DMat4, DVec3};
use openusd::{
    ar::AssetBuffer,
    gf, sdf,
    tf::Token,
    usd::{PrimPredicate, SchemaBase, Stage},
};
use openusd_schemas::{
    geom::{self, GprimSchema, MeshSchema, PointBasedSchema, XformCache},
    shade,
};

use super::{
    EmbeddedUsdResolver, PartRole, PbrMaterial, UsdAircraftAsset, UsdMeshPart, UsdMeshVertex,
};
use crate::MapAdapterError;

pub(super) fn load_aircraft_from_bytes(
    identifier: &str,
    bytes: AssetBuffer,
) -> Result<UsdAircraftAsset, MapAdapterError> {
    let resolver = EmbeddedUsdResolver::single(identifier, bytes);
    let stage = Stage::builder()
        .schema_registry(openusd_schemas::schema_registry())
        .resolver(resolver)
        .open(identifier)
        .map_err(|err| MapAdapterError::OpenUsd {
            detail: err.to_string(),
        })?;

    let mut mesh_paths: Vec<sdf::Path> = Vec::new();
    stage
        .traverse(PrimPredicate::DEFAULT_PROXIES, |path| {
            mesh_paths.push(path.clone());
        })
        .map_err(|err| MapAdapterError::OpenUsd {
            detail: err.to_string(),
        })?;

    let mut xform_cache = XformCache::default();
    let mut parts = Vec::new();
    for path in mesh_paths {
        if let Some(part) = decode_mesh_prim(&stage, &mut xform_cache, &path)? {
            parts.push(part);
        }
    }
    if parts.is_empty() {
        return Err(MapAdapterError::OpenUsd {
            detail: format!("OpenUSD stage '{identifier}' contained zero UsdGeomMesh prims"),
        });
    }
    Ok(UsdAircraftAsset::airframe(identifier, parts))
}

fn decode_mesh_prim(
    stage: &Stage,
    xform_cache: &mut XformCache,
    path: &sdf::Path,
) -> Result<Option<UsdMeshPart>, MapAdapterError> {
    let mesh = geom::Mesh::get(stage, path.clone()).map_err(usd_err)?;
    let Some(mesh) = mesh else {
        return Ok(None);
    };
    let prim = mesh.prim();
    let gf_mat = xform_cache
        .local_to_world_transform(prim)
        .map_err(schema_err)?;
    let rest_to_aircraft = DMat4::from_cols_array(&gf_mat.0);

    let points = mesh.points().map_err(usd_err)?.unwrap_or_default();
    let normals = mesh.normals().map_err(usd_err)?.unwrap_or_default();
    let counts = mesh
        .face_vertex_counts()
        .map_err(usd_err)?
        .unwrap_or_default();
    let raw_indices = mesh
        .face_vertex_indices()
        .map_err(usd_err)?
        .unwrap_or_default();
    let uvs = prim
        .attribute("primvars:st")
        .get::<Vec<gf::Vec2f>>()
        .ok()
        .flatten()
        .unwrap_or_default();
    let double_sided = mesh.double_sided().map_err(usd_err)?.unwrap_or(false);

    let role_tok = prim
        .attribute("sim:partRole")
        .get::<Token>()
        .ok()
        .flatten()
        .map_or_else(String::new, |t| t.as_str().to_owned());
    let mut role = PartRole::from_token(&role_tok);
    let custom_hinge_axis = resolve_hinge_axis(stage, path);
    let needle_channel = prim
        .attribute("sim:needleChannel")
        .get::<i32>()
        .ok()
        .flatten()
        .and_then(|ch| usize::try_from(ch).ok());
    let material = resolve_material(stage, path, prim)?;
    if role == PartRole::CabinExterior && material.opacity < 0.99 {
        role = PartRole::CabinGlass;
    }
    let (vertices, indices) = triangulate_usd_mesh(&points, &normals, &uvs, &counts, &raw_indices);
    if indices.is_empty() {
        return Ok(None);
    }

    Ok(Some(UsdMeshPart {
        prim_path: path.to_string(),
        role,
        material,
        double_sided,
        rest_to_aircraft,
        custom_hinge_axis,
        needle_channel,
        control_joints: Vec::new(),
        detail: crate::usd_aircraft::SurfaceDetail::default(),
        surface: crate::usd_aircraft::PartSurface::Material,
        vertices,
        indices,
    }))
}

fn resolve_hinge_axis(stage: &Stage, path: &sdf::Path) -> Option<DVec3> {
    for anc in path.ancestors_below_root() {
        let Ok(prim) = stage.prim(anc) else {
            continue;
        };
        if let Ok(Some(v)) = prim.attribute("sim:hingeAxis").get::<gf::Vec3d>() {
            return Some(DVec3::new(v.x, v.y, v.z));
        }
    }
    None
}

fn resolve_material(
    stage: &Stage,
    path: &sdf::Path,
    prim: &openusd::usd::Prim,
) -> Result<PbrMaterial, MapAdapterError> {
    let Some(mat_path) = find_bound_material_path(stage, path, prim) else {
        return Ok(PbrMaterial::default());
    };
    if let Ok(Some(surface)) = shade::read_preview_surface(stage, &mat_path) {
        let mut mat = PbrMaterial::default();
        if let Some(c) = surface.diffuse_color.value() {
            mat.diffuse_rgb = [c.x, c.y, c.z];
        }
        if let Some(e) = surface.emissive_color.value() {
            mat.emissive_rgb = [e.x, e.y, e.z];
        }
        if let Some(&m) = surface.metallic.value() {
            mat.metallic = m.clamp(0.0, 1.0);
        }
        if let Some(&r) = surface.roughness.value() {
            mat.roughness = r.clamp(0.04, 1.0);
        }
        if let Some(&o) = surface.opacity.value() {
            mat.opacity = o.clamp(0.0, 1.0);
        }
        if mat.opacity >= 0.99
            && (mat_path.to_string().ends_with("/Glass")
                || path.to_string().ends_with("/Window/Glass"))
        {
            mat.opacity = 0.32;
        }
        return Ok(mat);
    }
    let mut mat = read_fallback_pbr_shader(stage, &mat_path)?;
    if mat.opacity >= 0.99
        && (mat_path.to_string().ends_with("/Glass") || path.to_string().ends_with("/Window/Glass"))
    {
        mat.opacity = 0.32;
    }
    Ok(mat)
}

fn find_bound_material_path(
    stage: &Stage,
    path: &sdf::Path,
    prim: &openusd::usd::Prim,
) -> Option<sdf::Path> {
    if let Some(p) = direct_or_rel_material(prim) {
        return Some(p);
    }
    let ancestors: Vec<sdf::Path> = path.ancestors_below_root().collect();
    for anc in ancestors.into_iter().rev() {
        if let Ok(anc_prim) = stage.prim(anc)
            && let Some(p) = direct_or_rel_material(&anc_prim)
        {
            return Some(p);
        }
    }
    None
}

fn direct_or_rel_material(prim: &openusd::usd::Prim) -> Option<sdf::Path> {
    let binding = shade::MaterialBindingAPI::from_prim_unchecked(prim.clone());
    if let Ok(Some(p)) = binding.direct_binding(shade::tokens::ALL_PURPOSE) {
        return Some(p);
    }
    if let Ok(targets) = prim.relationship("material:binding").targets()
        && let Some(first) = targets.first()
    {
        return Some(first.clone());
    }
    None
}

fn read_fallback_pbr_shader(
    stage: &Stage,
    mat_path: &sdf::Path,
) -> Result<PbrMaterial, MapAdapterError> {
    let mut mat = PbrMaterial::default();
    let Ok(shader_path) = mat_path.append_path("PbrShader") else {
        return Ok(mat);
    };
    let Ok(shader_prim) = stage.prim(shader_path) else {
        return Ok(mat);
    };
    if let Ok(Some(c)) = shader_prim
        .attribute("inputs:diffuseColor")
        .get::<gf::Vec3f>()
    {
        mat.diffuse_rgb = [c.x, c.y, c.z];
    }
    if let Ok(Some(e)) = shader_prim
        .attribute("inputs:emissiveColor")
        .get::<gf::Vec3f>()
    {
        mat.emissive_rgb = [e.x, e.y, e.z];
    }
    if let Ok(Some(m)) = shader_prim.attribute("inputs:metallic").get::<f32>() {
        mat.metallic = m.clamp(0.0, 1.0);
    }
    if let Ok(Some(r)) = shader_prim.attribute("inputs:roughness").get::<f32>() {
        mat.roughness = r.clamp(0.04, 1.0);
    }
    if let Ok(Some(o)) = shader_prim.attribute("inputs:opacity").get::<f32>() {
        mat.opacity = o.clamp(0.0, 1.0);
    }
    Ok(mat)
}

fn triangulate_usd_mesh(
    points: &[gf::Vec3f],
    normals: &[gf::Vec3f],
    uvs: &[gf::Vec2f],
    counts: &[i32],
    face_indices: &[i32],
) -> (Vec<UsdMeshVertex>, Vec<u32>) {
    let mut vertices = Vec::with_capacity(points.len());
    for (idx, pt) in points.iter().enumerate() {
        let nrm = normals.get(idx).copied().unwrap_or(gf::Vec3f {
            x: 0.0,
            y: 0.0,
            z: 1.0,
        });
        let uv = uvs
            .get(idx)
            .copied()
            .unwrap_or(gf::Vec2f { x: 0.0, y: 0.0 });
        vertices.push(UsdMeshVertex {
            position: [pt.x, pt.y, pt.z],
            normal: [nrm.x, nrm.y, nrm.z],
            uv: [uv.x, uv.y],
            occlusion: 1.0,
            tangent: crate::UsdMeshVertex::NO_TANGENT,
        });
    }

    let mut out_indices = Vec::with_capacity(face_indices.len());
    let mut cursor = 0_usize;
    for &raw_count in counts {
        let count = usize::try_from(raw_count).unwrap_or(0);
        if count >= 3 {
            let i0 = face_indices
                .get(cursor)
                .and_then(|&v| u32::try_from(v).ok());
            for fan in 1..count.saturating_sub(1) {
                let i1 = face_indices
                    .get(cursor.saturating_add(fan))
                    .and_then(|&v| u32::try_from(v).ok());
                let i2 = face_indices
                    .get(cursor.saturating_add(fan).saturating_add(1))
                    .and_then(|&v| u32::try_from(v).ok());
                if let (Some(a), Some(b), Some(c)) = (i0, i1, i2)
                    && (a as usize) < vertices.len()
                    && (b as usize) < vertices.len()
                    && (c as usize) < vertices.len()
                {
                    out_indices.extend_from_slice(&[a, b, c]);
                }
            }
        }
        cursor = cursor.saturating_add(count);
    }
    (vertices, out_indices)
}

fn usd_err(err: openusd::Error) -> MapAdapterError {
    MapAdapterError::OpenUsd {
        detail: err.to_string(),
    }
}

fn schema_err(err: openusd_schemas::SchemaError) -> MapAdapterError {
    MapAdapterError::OpenUsd {
        detail: err.to_string(),
    }
}
