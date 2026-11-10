//! A dial's cards and fixed overlays: Indicate's marks extruded into thin
//! plates at their stack heights, a card on its own channel of
//! [`sim_core::InstrumentNeedles`] so a frame turns and slides it, an overlay
//! fixed in front of every moving part.

use std::f32::consts::TAU;

use glam::{DMat4, Vec2, Vec3};
use indicate_instrument_scene::Rgba8;
use sim_core::InstrumentNeedles;
use sim_indicate::{Mark, Shape, SlotDial};

use super::needle::Solid;
use super::{FaceFrame, linear_rgb};
use crate::usd_aircraft::{PartRole, PartSurface, PbrMaterial, SurfaceDetail, UsdMeshPart};

/// Thickness of a card's or overlay's plate.
const PLATE_THICKNESS_MM: f32 = 0.4;
/// Segments of a full circle in an arc or disc outline.
const CIRCLE_SEGMENTS: f32 = 48.0;

/// The outline of `shape` in face units, `+x` right and `+y` up the dial.
fn outline(shape: &Shape<'_>) -> Vec<Vec2> {
    // Indicate's face units run `y` down the dial.
    let up = |[x, y]: [f32; 2]| Vec2::new(x, -y);
    match *shape {
        Shape::Line { from, to, width } => {
            let (a, b) = (up(from), up(to));
            let across = (b - a).perp().normalize_or_zero() * (0.5 * width);
            vec![a - across, b - across, b + across, a + across]
        }
        Shape::Arc {
            center,
            radius,
            from_deg,
            to_deg,
            width,
        } => {
            let c = up(center);
            let span = (to_deg - from_deg).to_radians();
            let steps = ((span / TAU * CIRCLE_SEGMENTS).ceil() as usize).max(2);
            // Angles run clockwise from twelve as the pilot sees the dial.
            let at = |i: usize, r: f32| {
                let a = from_deg.to_radians() + span * i as f32 / steps as f32;
                c + Vec2::new(a.sin(), a.cos()) * r
            };
            let (outer, inner) = (radius + 0.5 * width, radius - 0.5 * width);
            (0..=steps)
                .map(|i| at(i, outer))
                .chain((0..=steps).rev().map(|i| at(i, inner)))
                .collect()
        }
        Shape::Disc { center, radius } => (0..CIRCLE_SEGMENTS as usize)
            .map(|i| {
                let a = TAU * i as f32 / CIRCLE_SEGMENTS;
                up(center) + Vec2::new(a.cos(), a.sin()) * radius
            })
            .collect(),
        Shape::Rect([x, y, w, h]) => {
            vec![
                up([x, y]),
                up([x + w, y]),
                up([x + w, y + h]),
                up([x, y + h]),
            ]
        }
        Shape::Polygon(points) => points.iter().map(|p| up(*p)).collect(),
    }
}

/// `marks`' plates, `m` metres per face unit, one solid per paint so each
/// part has one material.
fn plates(marks: &[Mark<'_>], m: f32, thickness_m: f32) -> Vec<(Rgba8, Solid)> {
    let mut out: Vec<(Rgba8, Solid)> = Vec::new();
    for mark in marks {
        let colour = mark.paint.day;
        let points: Vec<Vec2> = outline(&mark.shape).into_iter().map(|p| p * m).collect();
        let index = match out.iter().position(|(c, _)| *c == colour) {
            Some(index) => index,
            None => {
                out.push((colour, Solid::default()));
                out.len() - 1
            }
        };
        if let Some((_, solid)) = out.get_mut(index) {
            solid.prism(&points, 0.0, -thickness_m);
        }
    }
    out
}

fn plate_part(
    path: String,
    role: PartRole,
    colour: Rgba8,
    solid: Solid,
    rest: DMat4,
) -> UsdMeshPart {
    UsdMeshPart {
        prim_path: path,
        role,
        material: PbrMaterial {
            diffuse_rgb: linear_rgb(colour),
            emissive_rgb: [0.0; 3],
            metallic: 0.0,
            roughness: 0.45,
            opacity: f32::from(colour.a) / 255.0,
        },
        double_sided: false,
        rest_to_aircraft: rest,
        custom_hinge_axis: None,
        needle_channel: None,
        control_joints: Vec::new(),
        detail: SurfaceDetail::default(),
        surface: PartSurface::Material,
        vertices: solid.vertices,
        indices: solid.indices,
    }
}

/// `slot`'s cards and overlays in front of `face`, framed by `frame`.
pub(super) fn card_and_overlay_parts(
    face: &UsdMeshPart,
    frame: &FaceFrame,
    slot: &SlotDial,
) -> Vec<UsdMeshPart> {
    let Ok(build) = slot.dial.physical_build() else {
        return Vec::new();
    };
    let m_per_unit = frame.m_per_mm * build.face_radius_mm;
    let thickness = PLATE_THICKNESS_MM * frame.m_per_mm;
    let place = |pivot_mm: [f32; 2], height_mm: f32| {
        let at = frame.centre
            + Vec3::new(
                pivot_mm[0] * frame.m_per_mm,
                -height_mm * frame.m_per_mm,
                -pivot_mm[1] * frame.m_per_mm,
            );
        face.rest_to_aircraft * DMat4::from_translation(at.as_dvec3())
    };
    let mut out = Vec::new();
    for (index, card) in build.cards.iter().enumerate() {
        let Some(card) = card else {
            continue;
        };
        let rest = place(card.pivot_mm, card.stack_height_mm);
        for (k, (colour, solid)) in plates(card.shapes, m_per_unit, thickness)
            .into_iter()
            .enumerate()
        {
            let mut part = plate_part(
                format!("{}/Card{index}/{k}", face.prim_path),
                PartRole::InstrumentCard,
                colour,
                solid,
                rest,
            );
            part.needle_channel = InstrumentNeedles::card_channel(slot.slot, index);
            out.push(part);
        }
    }
    for (index, overlay) in build.overlays.iter().enumerate() {
        let rest = place([0.0, 0.0], overlay.stack_height_mm);
        for (k, (colour, solid)) in plates(overlay.shapes, m_per_unit, thickness)
            .into_iter()
            .enumerate()
        {
            out.push(plate_part(
                format!("{}/Overlay{index}/{k}", face.prim_path),
                PartRole::CabinInterior,
                colour,
                solid,
                rest,
            ));
        }
    }
    out
}
