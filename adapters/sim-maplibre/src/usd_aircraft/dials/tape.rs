//! A tape drum behind its window: a heading card printed round a cylinder
//! standing upright behind the face, turned by its slot's first drum wheel
//! channel, a tenth of a turn per unit as a digit wheel is.
//!
//! The drum's mesh is in its own frame, its axis along local `+X` so the
//! drum turns as [`super::moving_part_matrix`] turns a wheel; its rest frame
//! stands that axis up the dial.

use std::f32::consts::{FRAC_PI_2, TAU};

use glam::{DMat4, Vec3};
use sim_core::InstrumentNeedles;
use sim_indicate::{SlotDial, TapeBuild};

use super::{FaceFrame, linear_rgb};
use crate::usd_aircraft::{
    PartRole, PartSurface, PbrMaterial, SurfaceDetail, UsdMeshPart, UsdMeshVertex,
};

/// Clearance between the drum's front and the back of the face.
const DRUM_CLEARANCE_MM: f32 = 0.3;
/// Facets per row of the card round the drum.
const STEPS_PER_ROW: u32 = 24;

/// The drum's vertices and triangles: `rows` bands of the card, card
/// position `p` of period `P` at `-2π p / P` round the axis from the front,
/// `uv` into the rows of the baked card. A card rising to the left is baked
/// with `p = P - v`, which puts value `v` at `+2π v / P`, the way its drum
/// turns.
fn drum_mesh(height: f32, radius: f32, rows: u32) -> (Vec<UsdMeshVertex>, Vec<u32>) {
    let mut vertices = Vec::new();
    let mut indices = Vec::new();
    for row in 0..rows {
        let Ok(base) = u32::try_from(vertices.len()) else {
            break;
        };
        for i in 0..=STEPS_PER_ROW {
            let fraction = (row as f32 + i as f32 / STEPS_PER_ROW as f32) / rows as f32;
            let phi = -fraction * TAU;
            let normal = Vec3::new(0.0, -phi.cos(), phi.sin());
            let u = i as f32 / STEPS_PER_ROW as f32;
            for (t, x) in [(0.0, 0.5 * height), (1.0, -0.5 * height)] {
                vertices.push(UsdMeshVertex {
                    position: (Vec3::new(x, 0.0, 0.0) + normal * radius).to_array(),
                    normal: normal.to_array(),
                    uv: [u, (row as f32 + t) / rows as f32],
                    occlusion: 1.0,
                    tangent: UsdMeshVertex::NO_TANGENT,
                });
            }
        }
        for i in 0..STEPS_PER_ROW {
            let a = base + 2 * i;
            let tri = [a, a + 1, a + 3];
            // Wind each quad so its face normal agrees with the surface's.
            let at = |k: u32| {
                vertices
                    .get(k as usize)
                    .map_or(Vec3::ZERO, |v: &UsdMeshVertex| Vec3::from(v.position))
            };
            let n = vertices
                .get(a as usize)
                .map_or(Vec3::ZERO, |v| Vec3::from(v.normal));
            let [p, q, r] = tri.map(at);
            if (q - p).cross(r - p).dot(n) >= 0.0 {
                indices.extend([a, a + 1, a + 3, a, a + 3, a + 2]);
            } else {
                indices.extend([a, a + 3, a + 1, a, a + 2, a + 3]);
            }
        }
    }
    (vertices, indices)
}

/// `slot`'s tape drum behind `face`'s window, printed from the baked card in
/// `strip` of `rows` rows.
pub(super) fn tape_part(
    face: &UsdMeshPart,
    frame: &FaceFrame,
    slot: &SlotDial,
    tape: &TapeBuild<'_>,
    strip: [f32; 4],
    rows: u32,
) -> UsdMeshPart {
    let m = frame.m_per_mm;
    let [x, y, w, h] = tape.aperture_mm;
    let radius = tape.drum_radius_mm * m;
    let at = frame.centre
        + Vec3::new(
            (x + 0.5 * w) * m,
            radius + DRUM_CLEARANCE_MM * m,
            -(y + 0.5 * h) * m,
        );
    let (vertices, indices) = drum_mesh(h * m, radius, rows.max(1));
    let upright = DMat4::from_rotation_y(-f64::from(FRAC_PI_2));
    UsdMeshPart {
        prim_path: format!("{}/Tape", face.prim_path),
        role: PartRole::InstrumentDrum,
        material: PbrMaterial {
            diffuse_rgb: linear_rgb(tape.tape.drum),
            emissive_rgb: [0.0; 3],
            metallic: 0.0,
            roughness: 0.6,
            opacity: 1.0,
        },
        double_sided: false,
        rest_to_aircraft: face.rest_to_aircraft * DMat4::from_translation(at.as_dvec3()) * upright,
        custom_hinge_axis: None,
        needle_channel: InstrumentNeedles::wheel_channel(slot.slot, 0),
        control_joints: Vec::new(),
        detail: SurfaceDetail::default(),
        surface: PartSurface::Display {
            uv_rect: strip,
            glow_rgb: slot.dial.glow().map_or([0.0; 3], linear_rgb),
        },
        vertices,
        indices,
    }
}
