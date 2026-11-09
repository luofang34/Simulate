//! Per-vertex tangents from texture coordinates, so a tangent-space normal
//! map baked in one frame is read back in the same one.

use glam::{Vec2, Vec3};

/// Unit tangents along increasing `u`, with the handedness of increasing `v`
/// in `w`, for `positions`, `normals` and `uvs` of `triangles`.
pub(crate) fn tangents(
    positions: &[Vec3],
    normals: &[Vec3],
    uvs: &[Vec2],
    triangles: &[[u32; 3]],
) -> Vec<[f32; 4]> {
    let mut along_u = vec![Vec3::ZERO; positions.len()];
    let mut along_v = vec![Vec3::ZERO; positions.len()];
    let at = |items: &[Vec3], i: u32| items.get(i as usize).copied().unwrap_or(Vec3::ZERO);
    let uv = |i: u32| uvs.get(i as usize).copied().unwrap_or(Vec2::ZERO);
    for &[a, b, c] in triangles {
        let (e1, e2) = (
            at(positions, b) - at(positions, a),
            at(positions, c) - at(positions, a),
        );
        let (d1, d2) = (uv(b) - uv(a), uv(c) - uv(a));
        let det = d1.x * d2.y - d2.x * d1.y;
        if det.abs() < 1e-12 {
            continue;
        }
        let r = 1.0 / det;
        let t = (e1 * d2.y - e2 * d1.y) * r;
        let s = (e2 * d1.x - e1 * d2.x) * r;
        for i in [a, b, c] {
            if let Some(slot) = along_u.get_mut(i as usize) {
                *slot += t;
            }
            if let Some(slot) = along_v.get_mut(i as usize) {
                *slot += s;
            }
        }
    }
    normals
        .iter()
        .zip(along_u.iter().zip(&along_v))
        .map(|(&n, (&t, &s))| {
            let t = (t - n * n.dot(t))
                .try_normalize()
                .unwrap_or_else(|| n.any_orthonormal_vector());
            let w = if n.cross(t).dot(s) < 0.0 { -1.0 } else { 1.0 };
            [t.x, t.y, t.z, w]
        })
        .collect()
}
