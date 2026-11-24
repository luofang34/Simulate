//! Per-vertex ambient occlusion baked once per airframe from its own rest-pose triangles.
//!
//! Each vertex casts a fixed cosine-weighted ray set over its hemisphere; hits closer than
//! the occlusion radius darken the vertex, nearer hits more. Only sky light (the hemisphere
//! and environment terms) is attenuated in the shaders, so the bake darkens junctions,
//! nacelle undersides and wheel wells without dimming direct sun.

use glam::{Mat3, Vec3};

use super::{PartRole, UsdMeshPart};

mod bvh;

use bvh::{Bvh, Triangle};

/// Rays per vertex: enough that the fixed pattern, rotated per vertex, reads as smooth
/// gradients across the coarse airframe tessellation.
const RAYS: u32 = 32;
/// Occlusion radius as a fraction of the airframe's bounding radius.
const RADIUS_FRACTION: f32 = 0.12;
/// Origin lift along the normal, above the thin overlay skins (access panels, decals).
const ORIGIN_LIFT_M: f32 = 0.01;

/// Writes [`crate::UsdMeshVertex::occlusion`] for every vertex of `parts`.
pub(super) fn bake(parts: &mut [UsdMeshPart]) {
    bake_across(parts, cores());
}

/// [`bake`] with the probes shared among `workers` threads where the target has threads.
fn bake_across(parts: &mut [UsdMeshPart], workers: usize) {
    let bvh = Bvh::new(occluders(parts));
    let radius = bounding_radius(parts);
    let reach = (radius * RADIUS_FRACTION).clamp(0.25, 2.5);
    let probes: Vec<(Vec3, Vec3)> = parts
        .iter()
        .filter(|part| receives(part))
        .flat_map(|part| {
            let rest = part.rest_to_aircraft.as_mat4();
            let normals = Mat3::from_mat4(rest);
            part.vertices.iter().map(move |v| {
                let normal = (normals * Vec3::from_array(v.normal)).normalize_or(Vec3::Z);
                (rest.transform_point3(Vec3::from_array(v.position)), normal)
            })
        })
        .collect();
    let mut open = vec![1.0_f32; probes.len()];
    sample_parallel(&bvh, &probes, &mut open, reach, workers);
    let mut values = open.into_iter();
    for part in parts.iter_mut().filter(|part| receives(part)) {
        for vertex in &mut part.vertices {
            vertex.occlusion = values.next().unwrap_or(1.0);
        }
    }
}

/// Writes the occlusion of a table baked ahead of time, one little-endian `f32` per
/// receiving vertex in [`bake`]'s order. `false`, leaving `parts` untouched, when the
/// table is not this airframe's size.
pub(super) fn apply_baked(parts: &mut [UsdMeshPart], table: &[u8]) -> bool {
    let receiving: usize = parts
        .iter()
        .filter(|part| receives(part))
        .map(|part| part.vertices.len())
        .sum();
    let (values, rest) = table.as_chunks::<4>();
    if !rest.is_empty() || values.len() != receiving {
        return false;
    }
    let mut values = values.iter();
    for part in parts.iter_mut().filter(|part| receives(part)) {
        for (vertex, value) in part.vertices.iter_mut().zip(values.by_ref()) {
            vertex.occlusion = f32::from_le_bytes(*value);
        }
    }
    true
}

/// The table [`apply_baked`] reads, from `parts` whose occlusion is already baked.
#[cfg(test)]
pub(super) fn baked_table(parts: &[UsdMeshPart]) -> Vec<u8> {
    parts
        .iter()
        .filter(|part| receives(part))
        .flat_map(|part| &part.vertices)
        .flat_map(|vertex| vertex.occlusion.to_le_bytes())
        .collect()
}

/// Largest distance of any rest-pose vertex from the aircraft origin.
pub(super) fn bounding_radius(parts: &[UsdMeshPart]) -> f32 {
    parts
        .iter()
        .flat_map(|part| {
            let rest = part.rest_to_aircraft.as_mat4();
            part.vertices
                .iter()
                .map(move |v| rest.transform_point3(Vec3::from_array(v.position)).length())
        })
        .fold(0.0, f32::max)
}

/// Lowest rest-pose height on the aircraft `+Z` axis, ignoring the propeller blur disc.
pub(super) fn lowest_point_z(parts: &[UsdMeshPart]) -> f32 {
    let lowest = parts
        .iter()
        .filter(|part| receives(part))
        .flat_map(|part| {
            let rest = part.rest_to_aircraft.as_mat4();
            part.vertices
                .iter()
                .map(move |v| rest.transform_point3(Vec3::from_array(v.position)).z)
        })
        .fold(f32::INFINITY, f32::min);
    if lowest.is_finite() { lowest } else { 0.0 }
}

/// Opaque, static-looking skins occlude; glass, spinning blades, the blur disc and the
/// cockpit's moving controls (whose shadow would stay behind at their rest pose) do not.
fn occludes(part: &UsdMeshPart) -> bool {
    part.material.opacity >= 0.999
        && !matches!(
            part.role,
            PartRole::CabinGlass
                | PartRole::PropellerBlades
                | PartRole::PropellerDisc
                | PartRole::CockpitControl
        )
}

fn receives(part: &UsdMeshPart) -> bool {
    part.role != PartRole::PropellerDisc
}

fn occluders(parts: &[UsdMeshPart]) -> Vec<Triangle> {
    let mut triangles = Vec::new();
    for part in parts.iter().filter(|part| occludes(part)) {
        let rest = part.rest_to_aircraft.as_mat4();
        let corner = |index: &u32| {
            part.vertices
                .get(*index as usize)
                .map(|v| rest.transform_point3(Vec3::from_array(v.position)))
        };
        for [a, b, c] in part.indices.as_chunks::<3>().0 {
            if let (Some(a), Some(b), Some(c)) = (corner(a), corner(b), corner(c)) {
                triangles.push(Triangle { a, b, c });
            }
        }
    }
    triangles
}

/// The machine's cores.
#[cfg(not(target_arch = "wasm32"))]
fn cores() -> usize {
    std::thread::available_parallelism().map_or(1, usize::from)
}

/// Browsers give this crate no threads.
#[cfg(target_arch = "wasm32")]
fn cores() -> usize {
    1
}

/// Splits the probes across `workers` threads.
#[cfg(not(target_arch = "wasm32"))]
fn sample_parallel(
    bvh: &Bvh,
    probes: &[(Vec3, Vec3)],
    open: &mut [f32],
    reach: f32,
    workers: usize,
) {
    let chunk = probes.len().div_ceil(workers.max(1)).max(256);
    std::thread::scope(|scope| {
        let shares = probes.chunks(chunk).zip(open.chunks_mut(chunk));
        for (share, (inputs, outputs)) in shares.enumerate() {
            let first = share * chunk;
            let spawned = std::thread::Builder::new()
                .name("usd-ao-bake".to_owned())
                .spawn_scoped(scope, move || {
                    sample_all(bvh, inputs, outputs, reach, first)
                });
            if let Err(error) = spawned {
                tracing::warn!(%error, "ambient occlusion worker failed to start; chunk left open");
            }
        }
    });
}

/// Without threads the bake runs on the caller, whatever `_workers` asks for.
#[cfg(target_arch = "wasm32")]
fn sample_parallel(
    bvh: &Bvh,
    probes: &[(Vec3, Vec3)],
    open: &mut [f32],
    reach: f32,
    _workers: usize,
) {
    sample_all(bvh, probes, open, reach, 0);
}

/// Samples a share of the airframe's probes that starts at probe `first`. A probe's ray
/// pattern is seeded by its place in the airframe, not in the share, so a bake is the
/// same on any number of cores and matches a table baked elsewhere.
fn sample_all(bvh: &Bvh, probes: &[(Vec3, Vec3)], open: &mut [f32], reach: f32, first: usize) {
    for (index, ((position, normal), out)) in probes.iter().zip(open.iter_mut()).enumerate() {
        *out = sky_visibility(bvh, *position, *normal, reach, first + index);
    }
}

/// Cosine-weighted fraction of the hemisphere about `normal` left open within `reach`.
fn sky_visibility(bvh: &Bvh, position: Vec3, normal: Vec3, reach: f32, seed: usize) -> f32 {
    let (tangent, bitangent) = normal.any_orthonormal_pair();
    let origin = position + normal * ORIGIN_LIFT_M;
    // A per-vertex rotation of the shared pattern turns banding into fine noise that the
    // vertex interpolation averages out.
    let spin = hash_unit(seed) * std::f32::consts::TAU;
    let mut blocked = 0.0_f32;
    for i in 0..RAYS {
        let u = (i as f32 + 0.5) / RAYS as f32;
        let phi = radical_inverse(i) * std::f32::consts::TAU + spin;
        let r = u.sqrt();
        let local = Vec3::new(r * phi.cos(), r * phi.sin(), (1.0 - u).max(0.0).sqrt());
        let dir = (tangent * local.x + bitangent * local.y + normal * local.z).normalize_or(normal);
        if let Some(t) = bvh.nearest_hit(origin, dir, reach) {
            blocked += 1.0 - t / reach;
        }
    }
    (1.0 - blocked / RAYS as f32).clamp(0.0, 1.0)
}

/// Van der Corput radical inverse in base 2, the second Hammersley coordinate.
fn radical_inverse(i: u32) -> f32 {
    i.reverse_bits() as f32 * (1.0 / 4_294_967_296.0)
}

fn hash_unit(seed: usize) -> f32 {
    let mut x = (seed as u32).wrapping_mul(0x9E37_79B9) ^ 0x85EB_CA6B;
    x ^= x >> 16;
    x = x.wrapping_mul(0x7FEB_352D);
    x ^= x >> 15;
    (x >> 8) as f32 / 16_777_216.0
}

#[cfg(test)]
mod tests;
