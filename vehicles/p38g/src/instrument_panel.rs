//! The black main instrument panel, cut to the measured P-38G-13 outline, and,
//! at every layout slot of the pinned instrument pack (`usaaf-1943-fighter`)
//! and every empty case the pack cannot fill yet, an instrument's physical
//! depth stack
//! sized from the pack's bezel: the hole in the board, the bezel ring standing
//! proud of it and bored back to the dial, the dial face recessed in the bore,
//! and the cover glass just behind the bezel front. The gap between dial and
//! glass is left for needles.
//!
//! The dial face is a [`P38PartRole::DisplaySurface`] quad named
//! `/P38g/CockpitInterior/Instruments/<slot>/<instrument>`, whose `uv` span
//! `0..1` across the dial (`u` right, `v` down) so a rendered dial maps straight
//! on. Its own material is a matte black blank, which shows wherever no dial
//! is drawn; an empty case's face is named `.../<slot>/empty`. The board is
//! satin black: TM 1-413 (1942) gives instrument panels "a dull or black
//! satin finish", and no source shows crackle on a P-38G.

use std::f32::consts::TAU;

use glam::{DMat4, DVec3, Vec3};
use syspack::InstrumentPackDocument;

use crate::cockpit_reservation::p38g_cockpit_contract;
use crate::mesh::{
    Inside, MeshBuilder, P38Material, P38MeshPrim, P38MeshVertex, P38PartRole, mat, part,
};
use crate::stations::PANEL_BLACK;

mod board;
mod cases;
#[cfg(test)]
mod tests;

/// The instrument pack the P-38G's manifest pins.
const INSTRUMENT_PACK_JSON: &[u8] =
    include_bytes!("../../../instruments/usaaf-1943-fighter/instruments.json");

/// Half the board's thickness; the front face is at `-BOARD_HALF_M` (pilot side).
const BOARD_HALF_M: f32 = 0.006;
/// Bezel front, in front of the board face.
pub(crate) const BEZEL_PROUD_M: f32 = 0.004;
/// Dial face, behind the bezel front.
pub(crate) const FACE_RECESS_M: f32 = 0.009;
/// Cover glass, behind the bezel front.
pub(crate) const GLASS_RECESS_M: f32 = 0.002;
/// Visible dial diameter as a fraction of the case (bezel size) diameter,
/// where the pack does not print the face's own diameter.
pub(crate) const DIAL_FRACTION: f32 = 0.86;
/// Bezel flange beyond the case, overlapping the board around the hole.
const FLANGE_M: f32 = 0.004;

/// A dark red glass jewel, unlit.
const LAMP_JEWEL: P38Material = mat([0.35, 0.03, 0.02], 0.0, 0.15);
const DIAL_BLANK: P38Material = mat([0.03, 0.03, 0.032], 0.0, 0.92);
const BEZEL_BLACK: P38Material = mat([0.07, 0.07, 0.075], 0.35, 0.48);
const COVER_GLASS: P38Material = P38Material {
    diffuse_rgb: [0.80, 0.84, 0.86],
    emissive_rgb: [0.0, 0.0, 0.0],
    metallic: 0.0,
    roughness: 0.05,
    opacity: 0.10,
};

/// The instruments document of the pack the P-38G's panel is built for, or
/// `None` if it does not parse.
pub fn p38g_instrument_pack() -> Option<InstrumentPackDocument> {
    InstrumentPackDocument::parse(INSTRUMENT_PACK_JSON).ok()
}

/// One instrument opening: its layout slot, instrument, centre on the board
/// plane (`[right, up]` from the panel origin) and case diameter.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct Opening {
    pub(crate) slot: String,
    pub(crate) instrument: String,
    pub(crate) centre: [f32; 2],
    pub(crate) case_diameter_m: f32,
    pub(crate) dial_diameter_m: f32,
}

impl Opening {
    pub(crate) fn dial_radius(&self) -> f32 {
        0.5 * self.dial_diameter_m
    }

    fn hole_radius(&self) -> f32 {
        0.5 * self.case_diameter_m
    }

    /// The opening's centre `depth` in front of the board face, panel-local.
    pub(crate) fn at(&self, depth: f32) -> Vec3 {
        let [x, z] = self.centre;
        Vec3::new(x, -BOARD_HALF_M - depth, z)
    }
}

/// Instrument name of an empty case's opening.
pub(crate) const EMPTY_INSTRUMENT: &str = "empty";

/// Every layout slot of the pinned pack as an opening, then every empty case.
pub(crate) fn openings() -> Vec<Opening> {
    let empty = cases::EMPTY_CASES.iter().map(|c| Opening {
        slot: c.slot.to_owned(),
        instrument: EMPTY_INSTRUMENT.to_owned(),
        centre: c.centre,
        case_diameter_m: c.diameter_m,
        dial_diameter_m: c.diameter_m * DIAL_FRACTION,
    });
    let Some(pack) = p38g_instrument_pack() else {
        return empty.collect();
    };
    pack.layout
        .iter()
        .filter_map(|place| {
            let spec = pack.instrument(&place.instrument)?;
            let case_diameter_m = spec.bezel.diameter_m() as f32;
            let dial_diameter_m = spec
                .dial
                .as_ref()
                .and_then(|dial| dial.face_diameter_m)
                .map_or(case_diameter_m * DIAL_FRACTION, |d| d as f32);
            Some(Opening {
                slot: place.slot.clone(),
                instrument: place.instrument.clone(),
                centre: place.offset_m.map(|v| v as f32),
                case_diameter_m,
                dial_diameter_m,
            })
        })
        .chain(empty)
        .collect()
}

/// Panel plane to aircraft: origin at the contract's panel centre, `+X` right,
/// `+Z` up the tilted face, `+Y` into the board.
pub(crate) fn panel_frame() -> DMat4 {
    let panel = p38g_cockpit_contract().main_instrument_panel;
    let [x, y, z] = panel.center_m.map(f64::from);
    DMat4::from_translation(DVec3::new(x, y, z))
        * DMat4::from_rotation_x(-f64::from(panel.tilt_deg).to_radians())
}

pub(crate) fn instrument_panel_prims() -> Vec<P38MeshPrim> {
    let frame = panel_frame();
    let openings = openings();
    let lamps = cases::WARNING_LAMPS;
    let holes: Vec<([f32; 2], f32)> = openings
        .iter()
        .map(|o| (o.centre, o.hole_radius()))
        .chain(lamps.iter().map(|l| (l.centre, 0.5 * l.diameter_m)))
        .collect();
    let board = board::board_with_holes(
        p38g_cockpit_contract().main_instrument_panel_outline_m,
        BOARD_HALF_M,
        &holes,
    );
    let mut bezels = MeshBuilder::new();
    let mut glass = MeshBuilder::new();
    for opening in &openings {
        bezel(&mut bezels, opening);
        let glass_at = opening.at(BEZEL_PROUD_M - GLASS_RECESS_M);
        disc(&mut glass, glass_at, opening.dial_radius());
    }
    let mut bezel_prim = part(
        "/P38g/CockpitInterior/InstrumentPanel/Bezels",
        P38PartRole::CabinInterior,
        BEZEL_BLACK,
        frame,
        bezels,
    );
    bezel_prim.double_sided = true;
    let mut glass_prim = part(
        "/P38g/CockpitInterior/InstrumentPanel/CoverGlass",
        P38PartRole::CabinGlass,
        COVER_GLASS,
        frame,
        glass,
    );
    glass_prim.double_sided = true;
    let mut prims = vec![
        part(
            "/P38g/CockpitInterior/InstrumentPanel/Board",
            P38PartRole::CabinInterior,
            PANEL_BLACK,
            frame,
            board,
        ),
        bezel_prim,
        glass_prim,
    ];
    prims.extend(openings.iter().map(|o| dial_face(frame, o)));
    prims.push(warning_lamps(frame, &lamps));
    prims
}

/// The lamps' red jewels, domed proud of the board: unlit, since nothing
/// in the flight model drives them.
fn warning_lamps(frame: DMat4, lamps: &[cases::WarningLamp]) -> P38MeshPrim {
    let mut jewels = MeshBuilder::new();
    for lamp in lamps {
        let r = 0.5 * lamp.diameter_m;
        let [x, z] = lamp.centre;
        jewels.ellipsoid(
            Vec3::new(x, -BOARD_HALF_M, z),
            Vec3::new(r, 0.6 * r, r),
            16,
            8,
        );
    }
    part(
        "/P38g/CockpitInterior/InstrumentPanel/CoolantWarningLamps",
        P38PartRole::CabinInterior,
        LAMP_JEWEL,
        frame,
        jewels,
    )
}

fn ring(centre: Vec3, radius: f32) -> Vec<Vec3> {
    (0..=48)
        .map(|i| {
            let a = TAU * i as f32 / 48.0;
            centre + Vec3::new(a.sin(), 0.0, a.cos()) * radius
        })
        .collect()
}

/// Outer wall from the board to the bezel front, the front annulus, and the
/// bore back to the dial face.
fn bezel(mesh: &mut MeshBuilder, o: &Opening) {
    let outer = o.hole_radius() + FLANGE_M;
    let inner = o.dial_radius();
    let rows = [
        ring(o.at(0.0), outer),
        ring(o.at(BEZEL_PROUD_M), outer),
        ring(o.at(BEZEL_PROUD_M), inner),
        ring(o.at(BEZEL_PROUD_M - FACE_RECESS_M), inner),
    ];
    mesh.loft_open(&rows, Inside::Point(o.at(-0.05)));
}

fn disc(mesh: &mut MeshBuilder, centre: Vec3, radius: f32) {
    let mut rim = ring(centre, radius);
    rim.pop();
    mesh.polygon(&rim, Vec3::NEG_Y);
}

/// The dial face: a square spanning the visible dial, facing the pilot, with
/// `uv` `(0, 0)` at its top left. Its corners lie behind the board face, so only
/// the circle inside the bore shows.
fn dial_face(frame: DMat4, o: &Opening) -> P38MeshPrim {
    let r = o.dial_radius();
    let c = o.at(BEZEL_PROUD_M - FACE_RECESS_M);
    let corner = |x: f32, z: f32, u: f32, v: f32| P38MeshVertex {
        position: (c + Vec3::new(x * r, 0.0, z * r)).to_array(),
        normal: [0.0, -1.0, 0.0],
        uv: [u, v],
    };
    let mut prim = part(
        &format!(
            "/P38g/CockpitInterior/Instruments/{}/{}",
            o.slot, o.instrument
        ),
        P38PartRole::DisplaySurface,
        DIAL_BLANK,
        frame,
        MeshBuilder::new(),
    );
    prim.vertices = vec![
        corner(-1.0, 1.0, 0.0, 0.0),
        corner(1.0, 1.0, 1.0, 0.0),
        corner(1.0, -1.0, 1.0, 1.0),
        corner(-1.0, -1.0, 0.0, 1.0),
    ];
    // Counter-clockwise seen from the pilot (`-Y`).
    prim.indices = vec![0, 2, 1, 0, 3, 2];
    prim
}
