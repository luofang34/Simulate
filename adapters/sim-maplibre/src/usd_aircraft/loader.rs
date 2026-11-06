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
    Ok(UsdAircraftAsset { parts })
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
    let role = PartRole::from_token(&role_tok);
    let custom_hinge_axis = resolve_hinge_axis(stage, path);
    let material = resolve_material(stage, prim)?;
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
    prim: &openusd::usd::Prim,
) -> Result<PbrMaterial, MapAdapterError> {
    let binding = shade::MaterialBindingAPI::from_prim_unchecked(prim.clone());
    let Ok(Some(mat_path)) = binding.direct_binding(shade::tokens::ALL_PURPOSE) else {
        return Ok(PbrMaterial::default());
    };
    let preview = shade::read_preview_surface(stage, &mat_path).map_err(schema_err)?;
    let Some(surface) = preview else {
        return Ok(PbrMaterial::default());
    };
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
