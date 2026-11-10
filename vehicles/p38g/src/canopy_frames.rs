//! Canopy framing seen from the seat: the windscreen quarter posts, the top hatch's
//! bows and side rails, the side-window sills, the spine of the fixed rear glazing,
//! and the frame around the internal armoured glass.
//!
//! Members are solid tubes set just inside the glazing loft so they read from the
//! cockpit as well as from outside; flush exterior straps vanish from inside.

use std::f32::consts::FRAC_PI_2;

use glam::Vec3;

use crate::canopy::{HATCH_RAIL_A, armor_glass_corners};
use crate::drawing::{
    CANOPY_TAIL_FS_IN, HATCH_AFT_FRAME_FS_IN, WINDSCREEN_BASE_FS_IN, WINDSCREEN_TOP_FRAME_FS_IN,
    fs_to_y,
};
use crate::gondola::{canopy_point, canopy_sill_z};
use crate::mesh::{MeshBuilder, P38MeshPrim, P38PartRole, fixed};
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

fn bow(mesh: &mut MeshBuilder, y: f32) {
    let points: Vec<Vec3> = (0..=16)
        .map(|i| glazing_point(y, -FRAC_PI_2 + std::f32::consts::PI * i as f32 / 16.0))
        .collect();
    polyline(mesh, &points);
}

fn rail(mesh: &mut MeshBuilder, a: f32, y_from: f32, y_to: f32) {
    let points: Vec<Vec3> = (0..=12)
        .map(|i| glazing_point(y_from + (y_to - y_from) * i as f32 / 12.0, a))
        .collect();
    polyline(mesh, &points);
}

pub(crate) fn canopy_frame_prims() -> Vec<P38MeshPrim> {
    let windscreen_top = fs_to_y(WINDSCREEN_TOP_FRAME_FS_IN);
    let hatch_aft = fs_to_y(HATCH_AFT_FRAME_FS_IN);
    let front = fs_to_y(WINDSCREEN_BASE_FS_IN + END_MARGIN_IN);
    let tail = fs_to_y(CANOPY_TAIL_FS_IN - END_MARGIN_IN);
    let mut frames = MeshBuilder::new();
    for y in [windscreen_top, hatch_aft] {
        bow(&mut frames, y);
    }
    for side in [-1.0_f32, 1.0] {
        rail(&mut frames, side * QUARTER_POST_A, front, windscreen_top);
        rail(&mut frames, side * HATCH_RAIL_A, windscreen_top, hatch_aft);
        rail(&mut frames, side * FRAC_PI_2, front, tail);
    }
    rail(&mut frames, 0.0, hatch_aft, tail);
    let mut prims = vec![fixed(
        "/P38g/Canopy/InteriorFrames",
        P38PartRole::CabinExterior,
        CANOPY_FRAME,
        frames,
    )];

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
