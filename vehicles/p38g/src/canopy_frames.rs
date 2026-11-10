//! Canopy framing seen from the seat: the windscreen quarter posts, the top hatch's
//! bows and side rails, the side-window sills, the spine of the fixed rear glazing,
//! and the frame around the internal armoured glass.
//!
//! Members are solid tubes set just inside the glazing loft so they read from the
//! cockpit as well as from outside; flush exterior straps vanish from inside.

use std::f32::consts::FRAC_PI_2;

use glam::{DMat4, DVec3, Vec3};

use crate::canopy::{HATCH_RAIL_A, armor_glass_corners};
use crate::drawing::{
    CANOPY_TAIL_FS_IN, HATCH_AFT_FRAME_FS_IN, WINDSCREEN_BASE_FS_IN, WINDSCREEN_TOP_FRAME_FS_IN,
    fs_to_y,
};
use crate::gondola::{canopy_point, canopy_sill_z};
use crate::mesh::{MeshBuilder, P38MeshPrim, P38PartRole, fixed, part};
use crate::stations::{CANOPY_FRAME, GUNMETAL};

/// Arc angle of the windscreen quarter posts either side of the centre panel.
const QUARTER_POST_A: f32 = 0.46;
/// Frame member radius: a 20 mm wide aluminium extrusion.
const MEMBER_R: f32 = 0.010;
/// How far inside the glass the member centreline sits.
const INSET_M: f32 = 0.016;
/// Stations short of the canopy's pointed ends where the lengthwise members stop.
const END_MARGIN_IN: f32 = 4.0;

/// A point on the glazing loft at station `y`, arc angle `a` (`-π/2` left sill,
/// `0` crown, `+π/2` right sill), pulled `INSET_M` towards the cockpit.
fn glazing_point(y: f32, a: f32) -> Vec3 {
    let p = canopy_point(y, a);
    let inward = (Vec3::new(0.0, y, canopy_sill_z(y)) - p).normalize_or(Vec3::NEG_Z);
    p + inward * INSET_M
}

fn polyline(mesh: &mut MeshBuilder, points: &[Vec3]) {
    for pair in points.windows(2) {
        if let [a, b] = pair {
            mesh.tube(*a, *b, MEMBER_R, MEMBER_R, 6);
        }
    }
}

/// A frame member across the canopy at station `y`, from arc angle `a_from` to `a_to`,
/// authored about `origin`.
fn bow(mesh: &mut MeshBuilder, y: f32, (a_from, a_to): (f32, f32), origin: Vec3) {
    let points: Vec<Vec3> = (0..=16)
        .map(|i| glazing_point(y, a_from + (a_to - a_from) * i as f32 / 16.0) - origin)
        .collect();
    polyline(mesh, &points);
}

/// A frame member along the canopy at arc angle `a`, authored about `origin`.
fn rail(mesh: &mut MeshBuilder, a: f32, (y_from, y_to): (f32, f32), origin: Vec3) {
    let points: Vec<Vec3> = (0..=12)
        .map(|i| glazing_point(y_from + (y_to - y_from) * i as f32 / 12.0, a) - origin)
        .collect();
    polyline(mesh, &points);
}

/// How far inside the fixed bows the hatch's own end members sit, metres.
const HATCH_END_INSET_M: f32 = 0.025;

pub(crate) fn canopy_frame_prims() -> Vec<P38MeshPrim> {
    let windscreen_top = fs_to_y(WINDSCREEN_TOP_FRAME_FS_IN);
    let hatch_aft = fs_to_y(HATCH_AFT_FRAME_FS_IN);
    let front = fs_to_y(WINDSCREEN_BASE_FS_IN + END_MARGIN_IN);
    let tail = fs_to_y(CANOPY_TAIL_FS_IN - END_MARGIN_IN);
    // Fixed structure: the two bows, the windscreen's quarter posts, the sills and the
    // rear glazing's spine.
    let mut frames = MeshBuilder::new();
    for y in [windscreen_top, hatch_aft] {
        bow(&mut frames, y, (-FRAC_PI_2, FRAC_PI_2), Vec3::ZERO);
    }
    for side in [-1.0_f32, 1.0] {
        rail(
            &mut frames,
            side * QUARTER_POST_A,
            (front, windscreen_top),
            Vec3::ZERO,
        );
        rail(&mut frames, side * FRAC_PI_2, (front, tail), Vec3::ZERO);
    }
    rail(&mut frames, 0.0, (hatch_aft, tail), Vec3::ZERO);
    // The hatch's own frame, which rises with it: its side rails and an end member
    // inside each fixed bow, authored about the hatch pane's hinge.
    let hinge = canopy_point(hatch_aft, 0.0);
    let ends = (
        windscreen_top - HATCH_END_INSET_M,
        hatch_aft + HATCH_END_INSET_M,
    );
    let mut hatch = MeshBuilder::new();
    for side in [-1.0_f32, 1.0] {
        rail(&mut hatch, side * HATCH_RAIL_A, ends, hinge);
    }
    for y in [ends.0, ends.1] {
        bow(&mut hatch, y, (-HATCH_RAIL_A, HATCH_RAIL_A), hinge);
    }
    let mut hatch = part(
        "/P38g/Canopy/HatchFrames",
        P38PartRole::CabinExterior,
        CANOPY_FRAME,
        DMat4::from_translation(hinge.as_dvec3()),
        hatch,
    );
    hatch.custom_hinge_axis = Some(DVec3::X);
    let mut prims = vec![
        fixed(
            "/P38g/Canopy/InteriorFrames",
            P38PartRole::CabinExterior,
            CANOPY_FRAME,
            frames,
        ),
        hatch,
    ];

    // Steel retainer around the internal armoured glass, matching its quad corners.
    let mut armor = MeshBuilder::new();
    let corners = armor_glass_corners();
    for (i, a) in corners.iter().enumerate() {
        if let Some(b) = corners.get((i + 1) % corners.len()) {
            armor.tube(*a, *b, 0.012, 0.012, 6);
        }
    }
    prims.push(fixed(
        "/P38g/Canopy/ArmorGlassFrame",
        P38PartRole::CabinInterior,
        GUNMETAL,
        armor,
    ));
    prims
}
