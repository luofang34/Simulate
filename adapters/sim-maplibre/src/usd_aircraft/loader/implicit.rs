//! Implicit `UsdGeom` gprims (`Cube`, `Sphere`, `Cylinder`, `Cone`, `Capsule`) tessellated
//! into part meshes, so small equipment (antennas, lights, brake discs) can be authored in a
//! layer as a few attributes and an `xformOp:scale` instead of a point array.

use glam::{DMat4, Vec3};
use openusd::{tf::Token, usd::Prim};

use crate::UsdMeshVertex;
use crate::usd_aircraft::procedural::shapes::MeshBuilder;

/// Segments around a round gprim: smooth at the few-centimetre sizes equipment comes in.
const SEGMENTS: u32 = 24;

/// Triangles of an implicit gprim at its authored size, or `None` for any other prim type.
pub(super) fn geometry(prim: &Prim) -> Option<(Vec<UsdMeshVertex>, Vec<u32>)> {
    let type_name = prim.type_name().ok().flatten()?;
    let real = |name: &str, fallback: f64| -> f32 {
        prim.attribute(name)
            .get::<f64>()
            .ok()
            .flatten()
            .unwrap_or(fallback) as f32
    };
    let axis = match prim
        .attribute("axis")
        .get::<Token>()
        .ok()
        .flatten()
        .as_ref()
        .map(Token::as_str)
    {
        Some("X") => Vec3::X,
        Some("Y") => Vec3::Y,
        _ => Vec3::Z,
    };
    let mut mesh = MeshBuilder::new();
    match type_name.as_str() {
        "Cube" => {
            let half = Vec3::splat(0.5 * real("size", 2.0));
            mesh.cuboid(-half, half);
        }
        "Sphere" => {
            let r = real("radius", 1.0);
            mesh.ellipsoid(Vec3::ZERO, Vec3::splat(r), SEGMENTS, SEGMENTS / 2);
        }
        "Cylinder" | "Cone" => {
            let (r, half) = (real("radius", 1.0), 0.5 * real("height", 2.0));
            let tip = if type_name.as_str() == "Cone" { 0.0 } else { r };
            mesh.tube(-axis * half, axis * half, r, tip, SEGMENTS);
        }
        "Capsule" => {
            let (r, half) = (real("radius", 0.5), 0.5 * real("height", 1.0));
            mesh.tube(-axis * half, axis * half, r, r, SEGMENTS);
            for end in [-half, half] {
                let mut cap = MeshBuilder::new();
                cap.ellipsoid(Vec3::ZERO, Vec3::splat(r), SEGMENTS, SEGMENTS / 2);
                mesh.append(cap, glam::Mat4::from_translation(axis * end));
            }
        }
        _ => return None,
    }
    let buffers = mesh.into_buffers();
    (!buffers.1.is_empty()).then_some(buffers)
}

/// Moves a non-uniform scale out of `rest` into the vertices, scaling normals by its
/// inverse, because the aircraft shaders turn normals with the part matrix and a skewed
/// matrix would bend them. A uniform scale stays in `rest`.
///
/// The articulation joint still applies in the part's local frame, so a scaled part must
/// only be scaled symmetrically about its joint axis (equal `y` and `z` scale for a wheel
/// or hinge on `x`), which keeps the joint and the scale commuting.
pub(super) fn bake_scale(rest: &mut DMat4, vertices: &mut [UsdMeshVertex]) {
    let (scale, rotation, translation) = rest.to_scale_rotation_translation();
    let (lo, hi) = (scale.min_element(), scale.max_element());
    if lo <= 0.0 || hi / lo < 1.0 + 1e-6 {
        return;
    }
    let s = scale.as_vec3();
    for v in vertices.iter_mut() {
        v.position = (Vec3::from_array(v.position) * s).to_array();
        v.normal = (Vec3::from_array(v.normal) / s)
            .normalize_or(Vec3::Z)
            .to_array();
    }
    *rest = DMat4::from_rotation_translation(rotation, translation);
}
