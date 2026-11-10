//! A drum window's digit wheels behind an aperture cut in the face, and
//! the failure flag over the face.
//!
//! Geometry is in the face part's frame: `+X` right and `+Z` up the dial as
//! the pilot sees it, `-Y` toward the pilot, origin at the face centre `c`.

use std::f32::consts::TAU;

use glam::{DMat4, Vec2, Vec3};
use sim_core::{DRUM_WHEELS_PER_SLOT, InstrumentNeedles};
use sim_indicate::SlotDial;

use super::needle::Solid;
use super::{FaceFrame, linear_rgb};
use crate::usd_aircraft::{
    PartRole, PartSurface, PbrMaterial, SurfaceDetail, UsdMeshPart, UsdMeshVertex,
};

/// Facets round a digit wheel.
const WHEEL_SEGMENTS: u32 = 40;
/// Clearance between a wheel's front and the back of the face.
const WHEEL_CLEARANCE_MM: f32 = 0.3;
/// Share of a wheel's pitch its rim fills, leaving a dark gap between wheels.
const WHEEL_FILL: f32 = 0.9;
/// Height of the failure flag over the face.
const FLAG_STANDOFF_MM: f32 = 0.25;

/// Where `face`'s drum window sits: `[left, right, top, bottom]` in the
/// face frame, m.
pub(super) fn aperture_m(frame: &FaceFrame, aperture_mm: [f32; 4]) -> [f32; 4] {
    let [x, y, w, h] = aperture_mm;
    let m = frame.m_per_mm;
    [x * m, (x + w) * m, -y * m, -(y + h) * m]
}

/// A flat vertex of the face at `p`, `uv` across its square of half-side `r`.
fn face_vertex(p: Vec2, r: f32, centre: Vec3) -> UsdMeshVertex {
    UsdMeshVertex {
        position: (centre + Vec3::new(p.x, 0.0, p.y)).to_array(),
        normal: [0.0, -1.0, 0.0],
        uv: [(p.x + r) / (2.0 * r), (r - p.y) / (2.0 * r)],
        occlusion: 1.0,
        tangent: UsdMeshVertex::NO_TANGENT,
    }
}

/// Re-meshes `face`, a square of half-side `r` about `centre`, with the
/// rectangle `[left, right, top, bottom]` cut out of it.
pub(super) fn cut_window(face: &mut UsdMeshPart, centre: Vec3, r: f32, [l, rt, t, b]: [f32; 4]) {
    let outer = [
        Vec2::new(-r, r),
        Vec2::new(r, r),
        Vec2::new(r, -r),
        Vec2::new(-r, -r),
    ];
    let inner = [
        Vec2::new(l, t),
        Vec2::new(rt, t),
        Vec2::new(rt, b),
        Vec2::new(l, b),
    ];
    let mut vertices = Vec::new();
    let mut indices = Vec::new();
    for k in 0..4 {
        let next = (k + 1) % 4;
        let (Some(&o0), Some(&o1), Some(&i0), Some(&i1)) =
            (outer.get(k), outer.get(next), inner.get(k), inner.get(next))
        else {
            continue;
        };
        let Ok(base) = u32::try_from(vertices.len()) else {
            continue;
        };
        vertices.extend([o0, o1, i1, i0].map(|p| face_vertex(p, r, centre)));
        // Clockwise in the x-z plane, which faces the pilot as the square
        // face it replaces does.
        indices.extend([base, base + 2, base + 1, base, base + 3, base + 2]);
    }
    face.vertices = vertices;
    face.indices = indices;
}

/// One wheel: a cylinder about `+X`, `width` wide and `radius` round, its
/// texture's `v` running round it with digit 0 facing the pilot.
fn wheel_mesh(width: f32, radius: f32) -> (Vec<UsdMeshVertex>, Vec<u32>) {
    let mut vertices = Vec::new();
    for i in 0..=WHEEL_SEGMENTS {
        let v = i as f32 / WHEEL_SEGMENTS as f32;
        // Digit k is centred at v = (k + 0.5) / 10, and digit 0 faces -Y.
        let phi = TAU * v - TAU / 20.0;
        let normal = Vec3::new(0.0, -phi.cos(), -phi.sin());
        for (u, x) in [(0.0, -0.5 * width), (1.0, 0.5 * width)] {
            vertices.push(UsdMeshVertex {
                position: (Vec3::new(x, 0.0, 0.0) + normal * radius).to_array(),
                normal: normal.to_array(),
                uv: [u, v],
                occlusion: 1.0,
                tangent: UsdMeshVertex::NO_TANGENT,
            });
        }
    }
    let mut indices = Vec::new();
    for i in 0..WHEEL_SEGMENTS {
        let a = 2 * i;
        // Wound so (b - a) x (c - a) points out of the rim.
        indices.extend([a, a + 3, a + 1, a, a + 2, a + 3]);
    }
    (vertices, indices)
}

fn part(path: String, role: PartRole, material: PbrMaterial, rest: DMat4) -> UsdMeshPart {
    UsdMeshPart {
        prim_path: path,
        role,
        material,
        double_sided: false,
        rest_to_aircraft: rest,
        custom_hinge_axis: None,
        needle_channel: None,
        control_joints: Vec::new(),
        detail: SurfaceDetail::default(),
        surface: PartSurface::Material,
        vertices: Vec::new(),
        indices: Vec::new(),
    }
}

const fn flat(diffuse: [f32; 3], roughness: f32) -> PbrMaterial {
    PbrMaterial {
        diffuse_rgb: diffuse,
        emissive_rgb: [0.0; 3],
        metallic: 0.0,
        roughness,
        opacity: 1.0,
    }
}

/// The digit wheels of `slot`'s drum window behind `face`'s aperture,
/// printed from `strip` in the atlas, and the separator's dot.
pub(super) fn drum_parts(
    face: &UsdMeshPart,
    frame: &FaceFrame,
    slot: &SlotDial,
    window: &sim_indicate::WindowBuild,
    strip: [f32; 4],
) -> Vec<UsdMeshPart> {
    let m = frame.m_per_mm;
    let [x, y, w, h] = window.aperture_mm;
    let wheels = usize::from(window.wheels).min(DRUM_WHEELS_PER_SLOT);
    let gap = (w - window.wheel_width_mm * wheels as f32).max(0.0);
    let separator = slot.dial.drum_separator().map(usize::from);
    let radius = window.digit_pitch_mm * 10.0 / TAU;
    let z = -(y + 0.5 * h) * m;
    let depth = (radius + WHEEL_CLEARANCE_MM) * m;
    let glow = slot.dial.glow().map_or([0.0; 3], linear_rgb);
    let mut out = Vec::new();
    for wheel in 0..wheels {
        let past = separator.is_some_and(|after| wheel >= after);
        let centre_x =
            x + w - window.wheel_width_mm * (wheel as f32 + 0.5) - if past { gap } else { 0.0 };
        let at = frame.centre + Vec3::new(centre_x * m, depth, z);
        let mut drum = part(
            format!("{}/Drum{wheel}", face.prim_path),
            PartRole::InstrumentDrum,
            flat([0.02, 0.02, 0.02], 0.6),
            face.rest_to_aircraft * DMat4::from_translation(at.as_dvec3()),
        );
        let (vertices, indices) = wheel_mesh(window.wheel_width_mm * WHEEL_FILL * m, radius * m);
        drum.vertices = vertices;
        drum.indices = indices;
        drum.needle_channel = InstrumentNeedles::wheel_channel(slot.slot, wheel);
        drum.surface = PartSurface::Display {
            uv_rect: strip,
            glow_rgb: glow,
        };
        out.push(drum);
    }
    if let Some(after) = separator.filter(|&after| after < wheels) {
        let dot_x = x + w - window.wheel_width_mm * after as f32 - 0.5 * gap;
        let dot_r = 0.08 * window.digit_height_mm;
        let at = frame.centre
            + Vec3::new(
                dot_x * m,
                WHEEL_CLEARANCE_MM * m,
                -(y + 0.5 * h + 0.4 * window.digit_height_mm) * m,
            );
        let mut dot = part(
            format!("{}/DrumSeparator", face.prim_path),
            PartRole::CabinInterior,
            flat([0.85, 0.85, 0.82], 0.5),
            face.rest_to_aircraft * DMat4::from_translation(at.as_dvec3()),
        );
        let mut solid = Solid::default();
        solid.cap(dot_r * m, 0.0, -0.2 * m, 12);
        dot.vertices = solid.vertices;
        dot.indices = solid.indices;
        dot.surface = PartSurface::Luminous { glow_rgb: glow };
        out.push(dot);
    }
    out
}

/// The failure flag of `slot`'s dial over `face`, printed from `flag` in
/// the atlas: shown only while some moving part has no value.
pub(super) fn flag_part(
    face: &UsdMeshPart,
    frame: &FaceFrame,
    slot: &SlotDial,
    flag: [f32; 4],
) -> Option<UsdMeshPart> {
    let ([ax, ay], size) = slot.dial.off_flag()?;
    let face_m = frame.m_per_mm * slot.dial.face_radius_mm();
    let (hw, hh) = (0.5 * size * 3.25 * face_m, 0.5 * size * 1.6 * face_m);
    let at = frame.centre
        + Vec3::new(
            ax * face_m,
            -FLAG_STANDOFF_MM * frame.m_per_mm,
            -ay * face_m,
        );
    let mut part = part(
        format!("{}/OffFlag", face.prim_path),
        PartRole::InstrumentFlag,
        flat([0.6, 0.05, 0.04], 0.5),
        face.rest_to_aircraft * DMat4::from_translation(at.as_dvec3()),
    );
    let corner = |x: f32, z: f32, u: f32, v: f32| UsdMeshVertex {
        position: [x, 0.0, z],
        normal: [0.0, -1.0, 0.0],
        uv: [u, v],
        occlusion: 1.0,
        tangent: UsdMeshVertex::NO_TANGENT,
    };
    part.vertices = vec![
        corner(-hw, hh, 0.0, 0.0),
        corner(hw, hh, 1.0, 0.0),
        corner(hw, -hh, 1.0, 1.0),
        corner(-hw, -hh, 0.0, 1.0),
    ];
    part.indices = vec![0, 2, 1, 0, 3, 2];
    part.needle_channel = Some(slot.slot);
    part.surface = PartSurface::Display {
        uv_rect: flag,
        glow_rgb: [0.0; 3],
    };
    Some(part)
}
