use glam::Vec3;

use crate::{P38PartRole, p38g_cockpit_contract, p38g_mesh_prims};

/// Möller–Trumbore: whether segment `a..b` crosses triangle `t`.
fn segment_hits(a: Vec3, b: Vec3, [p0, p1, p2]: [Vec3; 3]) -> bool {
    let dir = b - a;
    let (e1, e2) = (p1 - p0, p2 - p0);
    let h = dir.cross(e2);
    let det = e1.dot(h);
    if det.abs() < 1e-9 {
        return false;
    }
    let s = a - p0;
    let u = s.dot(h) / det;
    let q = s.cross(e1);
    let v = dir.dot(q) / det;
    let t = e2.dot(q) / det;
    (0.0..=1.0).contains(&u) && v >= 0.0 && u + v <= 1.0 && (0.0..=1.0).contains(&t)
}

fn opaque_exterior_blocks(a: Vec3, b: Vec3) -> Vec<String> {
    let mut hits = Vec::new();
    for prim in p38g_mesh_prims() {
        if prim.role != P38PartRole::CabinExterior || prim.material.opacity < 1.0 {
            continue;
        }
        let at = |i: u32| Vec3::from_array(prim.vertices[i as usize].position);
        let blocked = prim
            .indices
            .as_chunks::<3>()
            .0
            .iter()
            .any(|&[i, j, k]| segment_hits(a, b, [at(i), at(j), at(k)]));
        if blocked {
            hits.push(prim.prim_path.clone());
        }
    }
    hits
}

#[test]
fn the_seated_pilot_sees_the_instrument_panel_past_the_dorsal_skin() {
    let contract = p38g_cockpit_contract();
    let eye = Vec3::from_array(contract.design_eyepoint_m);
    let panel = &contract.main_instrument_panel;
    for dx in [-0.25, 0.0, 0.25] {
        let gauge = Vec3::from_array(panel.center_m) + Vec3::new(dx, -0.03, 0.0);
        let hits = opaque_exterior_blocks(eye, gauge);
        assert!(hits.is_empty(), "eye to panel {dx} m blocked by {hits:?}");
    }
}

/// The eye is above the sill, and the canopy clears a helmeted head: `4.4 in` from eye
/// to crown, `0.5 in` of helmet and at least `1.5 in` to spare.
#[test]
fn the_design_eyepoint_sits_under_the_canopy_with_headroom() {
    let eye = Vec3::from_array(p38g_cockpit_contract().design_eyepoint_m);
    let sill = crate::canopy_sill_z(eye.y);
    let (.., z_canopy, _) = crate::interpolate_gondola(eye.y);
    assert!(eye.z > sill + 0.08, "eye {eye} below the sill");
    assert!(
        z_canopy - eye.z > 6.4 * 0.0254,
        "eye {eye} under {z_canopy} m canopy"
    );
}
