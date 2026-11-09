//! The black main instrument panel and the gauges of the pinned instrument pack
//! (`usaaf-1943-fighter`) at its layout slots: cases, faces, graduations and
//! numerals per dial scale, coloured markings, and needles the panel needle
//! channels turn.
//!
//! Panel positions not in the pack carry uninstrumented cases so the board reads
//! as a full 1943 fighter panel; they have no needles because nothing drives them.

use glam::{DMat4, DVec3, Vec3};
use syspack::InstrumentPackDocument;

use crate::cockpit_reservation::p38g_cockpit_contract;
use crate::mesh::{
    MeshBuilder, NEEDLE_HANDS_PER_SLOT, P38Material, P38MeshPrim, P38PartRole, mat, part,
};
use crate::stations::PANEL_BLACK;

mod dial;
mod glyphs;
#[cfg(test)]
mod tests;

use dial::{DialFace, NEEDLE_M};
use glyphs::stroke;

/// The instrument pack the P-38G's manifest pins.
const INSTRUMENT_PACK_JSON: &[u8] =
    include_bytes!("../../../instruments/usaaf-1943-fighter/instruments.json");

/// Half the board's thickness; dials mount on its front face.
const BOARD_HALF_M: f32 = 0.006;

/// Uninstrumented cases as `([right, up] from the panel origin, diameter)`: gyro
/// horizon, directional gyro, turn and bank, coolant and oil temperatures,
/// clock, and fuel quantity.
const UNINSTRUMENTED: [([f32; 2], f32); 9] = [
    ([-0.20, 0.05], 0.0794),
    ([-0.20, -0.05], 0.0794),
    ([0.0, -0.05], 0.0794),
    ([0.26, 0.06], 0.0572),
    ([0.26, -0.01], 0.0572),
    ([-0.29, -0.08], 0.0572),
    ([0.12, -0.08], 0.0572),
    ([0.19, -0.08], 0.0572),
    ([0.26, -0.08], 0.0572),
];

const DIAL_BLACK: P38Material = mat([0.035, 0.036, 0.04], 0.05, 0.62);
const BEZEL: P38Material = mat([0.10, 0.10, 0.11], 0.55, 0.42);
const NEEDLE_WHITE: P38Material = mat([0.90, 0.90, 0.84], 0.0, 0.50);
const MARKING_MATERIALS: [(&str, P38Material); 5] = [
    ("White", mat([0.86, 0.86, 0.80], 0.0, 0.55)),
    ("Green", mat([0.12, 0.62, 0.20], 0.0, 0.55)),
    ("Yellow", mat([0.90, 0.72, 0.10], 0.0, 0.55)),
    ("Red", mat([0.80, 0.10, 0.08], 0.0, 0.55)),
    ("Blue", mat([0.14, 0.32, 0.80], 0.0, 0.55)),
];

/// The pinned pack's instruments document, or `None` if it does not parse.
pub(crate) fn instrument_pack() -> Option<InstrumentPackDocument> {
    InstrumentPackDocument::parse(INSTRUMENT_PACK_JSON).ok()
}

/// Panel plane to aircraft: origin at the contract's panel centre, `+X` right,
/// `+Z` up the tilted face, `+Y` into the board.
fn panel_frame() -> DMat4 {
    let panel = p38g_cockpit_contract().main_instrument_panel;
    let [x, y, z] = panel.center_m.map(f64::from);
    DMat4::from_translation(DVec3::new(x, y, z))
        * DMat4::from_rotation_x(-f64::from(panel.tilt_deg).to_radians())
}

fn on_board([right, up]: [f32; 2]) -> Vec3 {
    Vec3::new(right, -BOARD_HALF_M, up)
}

struct PanelMeshes {
    board: MeshBuilder,
    cases: MeshBuilder,
    faces: MeshBuilder,
    colours: [MeshBuilder; 5],
}

pub(crate) fn instrument_panel_prims() -> Vec<P38MeshPrim> {
    let frame = panel_frame();
    let panel = p38g_cockpit_contract().main_instrument_panel;
    let (w, h) = (0.5 * panel.width_m, 0.5 * panel.height_m);
    let mut m = PanelMeshes {
        board: MeshBuilder::new(),
        cases: MeshBuilder::new(),
        faces: MeshBuilder::new(),
        colours: Default::default(),
    };
    m.board.cuboid(
        Vec3::new(-w, -BOARD_HALF_M, -h),
        Vec3::new(w, BOARD_HALF_M, h),
    );
    let mut needles = Vec::new();
    if let Some(pack) = instrument_pack() {
        for (slot, place) in pack.layout.iter().enumerate() {
            let Some(face) = pack
                .instrument(&place.instrument)
                .and_then(DialFace::from_spec)
            else {
                continue;
            };
            let centre = on_board(place.offset_m.map(|v| v as f32));
            face.case(&mut m.cases, centre);
            face.face(&mut m.faces, centre);
            let [white, ..] = &mut m.colours;
            face.paint(white, centre);
            face.markings(&mut m.colours, centre);
            needles.extend(needle_prims(frame, slot, &place.instrument, &face, centre));
        }
    }
    for (offset, diameter) in UNINSTRUMENTED {
        uninstrumented_case(&mut m, on_board(offset), 0.5 * diameter);
    }
    let path = |leaf: &str| format!("/P38g/CockpitInterior/InstrumentPanel/{leaf}");
    let interior = P38PartRole::CabinInterior;
    let mut prims = vec![
        part(&path("Board"), interior, PANEL_BLACK, frame, m.board),
        part(&path("Cases"), interior, BEZEL, frame, m.cases),
        part(&path("Faces"), interior, DIAL_BLACK, frame, m.faces),
    ];
    for ((name, material), mesh) in MARKING_MATERIALS.into_iter().zip(m.colours) {
        let paint = part(
            &path(&format!("Paint{name}")),
            interior,
            material,
            frame,
            mesh,
        );
        if !paint.indices.is_empty() {
            prims.push(paint);
        }
    }
    prims.extend(needles);
    prims
}

/// The needles of the dial at `centre` on layout `slot`, each resting at the
/// dial's zero graduation and turned by its channel.
fn needle_prims(
    frame: DMat4,
    slot: usize,
    instrument: &str,
    face: &DialFace,
    centre: Vec3,
) -> Vec<P38MeshPrim> {
    let pivot = centre - Vec3::Y * NEEDLE_M;
    (0..face.hands.min(NEEDLE_HANDS_PER_SLOT))
        .map(|hand| {
            // Successive hands stack towards the pilot so they never z-fight.
            let lifted = pivot - Vec3::Y * (0.0006 * hand as f32);
            let rest = frame
                * DMat4::from_translation(lifted.as_dvec3())
                * DMat4::from_rotation_y(face.zero_offset_rad());
            let mut prim = part(
                &format!("/P38g/CockpitInterior/InstrumentPanel/Needles/{instrument}_{hand}"),
                P38PartRole::InstrumentNeedle,
                NEEDLE_WHITE,
                rest,
                face.needle(hand),
            );
            prim.needle_channel = Some(slot * NEEDLE_HANDS_PER_SLOT + hand);
            prim
        })
        .collect()
}

/// A case with a plain graduated face and no needle.
fn uninstrumented_case(m: &mut PanelMeshes, centre: Vec3, radius: f32) {
    m.cases.tube(
        centre,
        centre - Vec3::Y * dial::CASE_FRONT_M,
        radius,
        radius,
        36,
    );
    let face_r = radius * 0.86;
    let paint = centre - Vec3::Y * 0.0102;
    let [white, ..] = &mut m.colours;
    for i in 0..12 {
        let a = std::f32::consts::TAU * i as f32 / 12.0;
        let d = Vec3::new(a.sin(), 0.0, a.cos());
        stroke(
            white,
            paint + d * (face_r * 0.82),
            paint + d * (face_r * 0.97),
            face_r * 0.04,
        );
    }
    let rim: Vec<Vec3> = (0..24)
        .map(|i| {
            let a = std::f32::consts::TAU * i as f32 / 24.0;
            centre - Vec3::Y * 0.0095 + Vec3::new(a.sin(), 0.0, a.cos()) * face_r
        })
        .collect();
    m.faces.polygon(&rim, Vec3::NEG_Y);
}
