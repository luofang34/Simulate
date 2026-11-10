//! Indicate's analog dials on an airframe's instrument faces.
//!
//! Every [`PartRole::DisplaySurface`] named `.../<slot>/<instrument>` after
//! an instrument-pack layout slot gets that slot's dial: its face rasterised
//! once by Indicate into a shared [`DisplayAtlas`] the face samples, and one
//! thin solid needle per pointer extruded from Indicate's outline at its
//! stack height above the face. A needle is a [`PartRole::InstrumentNeedle`]
//! on its `(slot, pointer)` channel of [`sim_core::InstrumentNeedles`], so a
//! frame costs one rotation per needle and nothing is drawn or decoded.
//!
//! The face is printed once, under daylight, and lit like any painted part.
//! At night printed paint, the red limit lines included, keeps its hue and
//! shows only by the light that reaches it; luminous paint alone glows, by
//! the strength the face's alpha records, under the cockpit's ultraviolet
//! lamps. Nothing is re-printed when the light changes.

mod atlas;
mod card;
mod drum;
mod needle;
mod placard;
mod tape;
#[cfg(test)]
mod tests;

pub use atlas::DisplayAtlas;
pub(crate) use placard::Placards;

use glam::{DMat4, Vec2, Vec3};
use indicate_instrument_scene::Rgba8;
use sim_core::InstrumentNeedles;
use sim_indicate::{
    FaceBake, FaceTable, PanelDials, PointerBuild, SlotDial, bake_drum_strip, bake_face,
    bake_off_flag, bake_tape_strip,
};
use tracing::warn;

use self::atlas::AtlasBuilder;
use self::needle::Solid;
use super::{PartRole, PartSurface, PbrMaterial, SurfaceDetail, UsdMeshPart};

/// Side of one baked face, pixels: about 10 px per millimetre of a
/// 3-1/8 in case, finer than a close look from the seat resolves, with a
/// panel's sixteen dials in a 3072 px square atlas.
pub(crate) const FACE_PX: u32 = 768;
/// Thickness of a needle's painted sheet.
const NEEDLE_THICKNESS_MM: f32 = 0.4;
/// Clearance of the hub cap over the topmost needle.
const HUB_CLEARANCE_MM: f32 = 0.3;
const HUB_SEGMENTS: u32 = 24;
/// Blackened brass of a hub cap.
const HUB: PbrMaterial = PbrMaterial {
    diffuse_rgb: [0.025, 0.025, 0.027],
    emissive_rgb: [0.0, 0.0, 0.0],
    metallic: 0.3,
    roughness: 0.4,
    opacity: 1.0,
};

fn linear(channel: u8) -> f32 {
    let c = f32::from(channel) / 255.0;
    if c <= 0.04045 {
        c / 12.92
    } else {
        ((c + 0.055) / 1.055).powf(2.4)
    }
}

fn linear_rgb(colour: Rgba8) -> [f32; 3] {
    [linear(colour.r), linear(colour.g), linear(colour.b)]
}

/// Centre and radius of a square face quad, in its part's frame.
fn face_geometry(face: &UsdMeshPart) -> Option<(Vec3, f32)> {
    let first = face.vertices.first()?;
    let (mut lo, mut hi) = (Vec3::from(first.position), Vec3::from(first.position));
    for v in &face.vertices {
        lo = lo.min(Vec3::from(v.position));
        hi = hi.max(Vec3::from(v.position));
    }
    Some(((lo + hi) * 0.5, (hi.x - lo.x) * 0.5))
}

/// The slot dial a face part is named after.
fn dial_for<'a>(part: &UsdMeshPart, dials: &'a PanelDials) -> Option<&'a SlotDial> {
    let mut segments = part.prim_path.rsplit('/');
    let instrument = segments.next()?;
    let slot = segments.next()?;
    dials
        .slots()
        .iter()
        .find(|d| d.name == slot && d.instrument == instrument)
}

/// A face part's frame: its centre, half-side and metres per millimetre
/// of the dial printed on it.
pub(super) struct FaceFrame {
    pub(super) centre: Vec3,
    pub(super) m_per_mm: f32,
}

/// The red flag's bake, pixels: Indicate's proportions, 3.25 by 1.6.
const FLAG_PX: [u32; 2] = [260, 128];

/// Faces `parts` with `dials`, lettered in the faces `lettering` binds, and appends their
/// needles, drums and flags; the atlas the faces sample, or `None` when no
/// face matched a dial.
pub(super) fn attach(
    parts: &mut Vec<UsdMeshPart>,
    dials: &PanelDials,
    lettering: &FaceTable<'_>,
    placards: &Placards,
) -> Option<DisplayAtlas> {
    let faces: Vec<(usize, &SlotDial)> = parts
        .iter()
        .enumerate()
        .filter(|(_, part)| part.role == PartRole::DisplaySurface)
        .filter_map(|(index, part)| Some((index, dial_for(part, dials)?)))
        .collect();
    if faces.is_empty() {
        return None;
    }
    let started = web_time::Instant::now();
    // A drum window's digit strip, or a tape drum's card and its rows.
    let strips: Vec<Option<(FaceBake, Option<u32>)>> = faces
        .iter()
        .map(|(_, slot)| {
            let drum = bake_drum_strip(&slot.dial, FACE_PX, lettering)
                .ok()
                .flatten();
            drum.map(|bake| (bake, None)).or_else(|| {
                let tape = bake_tape_strip(&slot.dial, FACE_PX, lettering)
                    .ok()
                    .flatten()?;
                Some((tape.image, Some(tape.rows)))
            })
        })
        .collect();
    let lettered = placard::bake(parts, placards, lettering);
    let cells = faces.len() + strips.iter().flatten().count() + lettered.len() + 1;
    let mut atlas = AtlasBuilder::new(u32::try_from(cells).ok()?, FACE_PX);
    let mut cell = 0_u32..;
    let flag = bake_off_flag(FLAG_PX[0], FLAG_PX[1])
        .ok()
        .and_then(|bake| Some(atlas.place(cell.next()?, [bake.width, bake.height], &bake.rgba)));
    let mut moving = Vec::new();
    for ((index, slot), strip) in faces.iter().zip(&strips) {
        let face = match bake_face(&slot.dial, FACE_PX, lettering) {
            Ok(face) => face,
            Err(err) => {
                warn!(slot = %slot.name, error = %err, "dial face did not bake");
                continue;
            }
        };
        let [u0, v0, du, dv] = atlas.place(cell.next()?, [face.width, face.height], &face.rgba);
        let strip = strip.as_ref().and_then(|(bake, rows)| {
            let rect = atlas.place(cell.next()?, [bake.width, bake.height], &bake.rgba);
            Some((rect, *rows))
        });
        // The bake spans the case; the quad spans the visible face.
        let share = slot.dial.face_radius_mm() / (0.5 * slot.dial.case_diameter_mm());
        let inset = 0.5 * (1.0 - share);
        let Some(part) = parts.get_mut(*index) else {
            continue;
        };
        part.surface = PartSurface::Display {
            uv_rect: [u0 + du * inset, v0 + dv * inset, du * share, dv * share],
            glow_rgb: slot.dial.glow().map_or([0.0; 3], linear_rgb),
        };
        moving.extend(needle_parts(part, slot));
        moving.extend(window_and_flag(part, slot, strip, flag));
    }
    placard::apply(parts, &mut atlas, &mut cell, lettered);
    parts.extend(moving);
    let atlas = atlas.finish();
    tracing::info!(
        dials = faces.len(),
        elapsed_ms = started.elapsed().as_secs_f64() * 1e3,
        "baked instrument dial faces"
    );
    Some(atlas)
}

/// Cuts `face`'s drum or tape window and returns its wheels or tape drum,
/// its cards and overlays, and its failure flag. `strip` is the window's
/// baked digit strip, or a tape's card with its rows.
fn window_and_flag(
    face: &mut UsdMeshPart,
    slot: &SlotDial,
    strip: Option<([f32; 4], Option<u32>)>,
    flag: Option<[f32; 4]>,
) -> Vec<UsdMeshPart> {
    let (Some((centre, radius_m)), Ok(build)) = (face_geometry(face), slot.dial.physical_build())
    else {
        return Vec::new();
    };
    let frame = FaceFrame {
        centre,
        m_per_mm: radius_m / build.face_radius_mm,
    };
    let mut out = Vec::new();
    let window = build.windows.first().copied().flatten();
    let tape = build.tapes.first().copied().flatten();
    match (window, tape, strip) {
        (Some(window), _, Some((strip, None))) => {
            let aperture = drum::aperture_m(&frame, window.aperture_mm);
            drum::cut_window(face, centre, radius_m, aperture);
            out.extend(drum::drum_parts(face, &frame, slot, &window, strip));
        }
        (None, Some(tape), Some((strip, Some(rows)))) => {
            let aperture = drum::aperture_m(&frame, tape.aperture_mm);
            drum::cut_window(face, centre, radius_m, aperture);
            out.push(tape::tape_part(face, &frame, slot, &tape, strip, rows));
        }
        _ => {}
    }
    out.extend(card::card_and_overlay_parts(face, &frame, slot));
    out.extend(flag.and_then(|flag| drum::flag_part(face, &frame, slot, flag)));
    out
}

/// The needle of `pointer`, hand `hand` of `slot`'s dial, at `rest`; `m`
/// metres per millimetre of the face.
fn pointer_part(
    face: &UsdMeshPart,
    slot: &SlotDial,
    hand: usize,
    pointer: &PointerBuild<'_>,
    rest: DMat4,
    m: f32,
) -> UsdMeshPart {
    let outline: Vec<Vec2> = pointer
        .outline_mm
        .points()
        .iter()
        .map(|&[x, y]| Vec2::new(x * m, -y * m))
        .collect();
    let mut solid = Solid::default();
    solid.prism(&outline, 0.0, -NEEDLE_THICKNESS_MM * m);
    let paint = pointer.paint;
    UsdMeshPart {
        prim_path: format!("{}/Pointer{hand}", face.prim_path),
        role: PartRole::InstrumentNeedle,
        material: PbrMaterial {
            diffuse_rgb: linear_rgb(paint.day),
            emissive_rgb: [0.0; 3],
            metallic: 0.0,
            roughness: 0.45,
            opacity: 1.0,
        },
        double_sided: false,
        rest_to_aircraft: rest,
        custom_hinge_axis: None,
        needle_channel: InstrumentNeedles::channel(slot.slot, hand),
        control_joints: Vec::new(),
        detail: SurfaceDetail::default(),
        surface: paint
            .luminous
            .map_or(PartSurface::Material, |glow| PartSurface::Luminous {
                glow_rgb: linear_rgb(glow),
            }),
        vertices: solid.vertices,
        indices: solid.indices,
    }
}

/// One needle part per pointer of `slot`'s dial, and a hub cap over them.
fn needle_parts(face: &UsdMeshPart, slot: &SlotDial) -> Vec<UsdMeshPart> {
    let Some((centre, radius_m)) = face_geometry(face) else {
        return Vec::new();
    };
    let build = match slot.dial.physical_build() {
        Ok(build) => build,
        Err(err) => {
            warn!(slot = %slot.name, error = %err, "dial has no physical build");
            return Vec::new();
        }
    };
    let m = radius_m / build.face_radius_mm;
    let place = |pivot: [f32; 2], height_mm: f32| {
        let at = centre + Vec3::new(pivot[0] * m, -height_mm * m, -pivot[1] * m);
        face.rest_to_aircraft * DMat4::from_translation(at.as_dvec3())
    };
    let mut out = Vec::new();
    let (mut top_mm, mut hub_mm, mut hub_pivot) = (0.0_f32, 0.0_f32, [0.0; 2]);
    for (hand, pointer) in build.pointers.iter().enumerate() {
        let Some(pointer) = pointer else {
            continue;
        };
        let rest = place(pointer.pivot_mm, pointer.stack_height_mm);
        out.push(pointer_part(face, slot, hand, pointer, rest, m));
        top_mm = top_mm.max(pointer.stack_height_mm + NEEDLE_THICKNESS_MM);
        if pointer.hub_radius_mm > hub_mm {
            (hub_mm, hub_pivot) = (pointer.hub_radius_mm, pointer.pivot_mm);
        }
    }
    if hub_mm > 0.0 {
        let mut solid = Solid::default();
        solid.cap(
            hub_mm * m,
            0.0,
            -(top_mm + HUB_CLEARANCE_MM) * m,
            HUB_SEGMENTS,
        );
        out.push(UsdMeshPart {
            prim_path: format!("{}/Hub", face.prim_path),
            role: PartRole::CabinInterior,
            material: HUB,
            double_sided: false,
            rest_to_aircraft: place(hub_pivot, 0.0),
            custom_hinge_axis: None,
            needle_channel: None,
            control_joints: Vec::new(),
            detail: SurfaceDetail::default(),
            surface: PartSurface::Material,
            vertices: solid.vertices,
            indices: solid.indices,
        });
    }
    out
}

/// The rotation (and a card's travel) of an instrument's moving part: a
/// needle about its local `+Y`, the dial normal, clockwise as the pilot sees
/// it; a card the same way, then down its turned vertical; a drum wheel back
/// about its local `+X` by a tenth of a turn per digit, which brings the
/// next digit, printed below the last, up into the window.
pub(super) fn moving_part_matrix(part: &UsdMeshPart, needles: &InstrumentNeedles) -> DMat4 {
    let Some(channel) = part.needle_channel else {
        return DMat4::IDENTITY;
    };
    match part.role {
        PartRole::InstrumentNeedle => DMat4::from_rotation_y(needles.angle_rad(channel)),
        PartRole::InstrumentDrum => {
            DMat4::from_rotation_x(-needles.wheel(channel) * std::f64::consts::TAU / 10.0)
        }
        PartRole::InstrumentCard => {
            let (turn, travel) = needles.card(channel);
            DMat4::from_rotation_y(turn)
                * DMat4::from_translation(glam::DVec3::new(0.0, 0.0, -travel))
        }
        _ => DMat4::IDENTITY,
    }
}

/// Whether an instrument's moving part shows: a hidden pointer, a drum with
/// no value and a flag down are not drawn.
pub(super) fn instrument_part_shown(
    part: &UsdMeshPart,
    needles: &sim_core::InstrumentNeedles,
) -> bool {
    let Some(channel) = part.needle_channel else {
        return true;
    };
    match part.role {
        PartRole::InstrumentNeedle => needles.visible(channel),
        PartRole::InstrumentDrum => needles.drum_live(channel / sim_core::DRUM_WHEELS_PER_SLOT),
        PartRole::InstrumentFlag => needles.flagged(channel),
        PartRole::InstrumentCard => needles.card_visible(channel),
        _ => true,
    }
}
