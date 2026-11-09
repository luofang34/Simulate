//! Canopy framing seen from the seat: the windscreen bows and quarter posts, the
//! top hatch's side rails and aft hinge bow, the side-window sills, the fixed rear
//! glazing bows, and the frame around the internal armoured glass.
//!
//! Members are solid tubes set just inside the glazing loft so they read from the
//! cockpit as well as from outside; flush exterior straps vanish from inside.

use std::f32::consts::FRAC_PI_2;

use glam::Vec3;

use crate::cockpit_opening::{GLAZING_BASE_DROP_M, OPENING_AFT_Y, OPENING_FWD_Y};
use crate::mesh::{MeshBuilder, P38MeshPrim, P38PartRole, fixed};
use crate::stations::{CANOPY_FRAME, GUNMETAL, interpolate_gondola};

/// Windscreen top bow, where the hinged top hatch begins (`FS 94`).
const WINDSCREEN_TOP_Y: f32 = 1.16;
/// Aft hatch bow carrying the hatch hinge, behind the pilot's head.
const HATCH_HINGE_Y: f32 = -0.10;
/// Forward bow of the fixed rear glazing.
const REAR_GLAZING_Y: f32 = -0.42;
/// Arc angle of the windscreen quarter posts either side of the centre panel.
const QUARTER_POST_A: f32 = 0.46;
/// Arc angle of the hatch side rails above the wind-down side windows.
const HATCH_RAIL_A: f32 = 1.0;
/// Frame member radius: a 20 mm wide aluminium extrusion.
const MEMBER_R: f32 = 0.010;
/// How far inside the glass the member centreline sits.
const INSET_M: f32 = 0.016;

/// A point on the glazing loft at station `y`, arc angle `a` (`-π/2` left sill,
/// `0` crown, `+π/2` right sill), pulled `INSET_M` towards the cockpit.
fn glazing_point(y: f32, a: f32) -> Vec3 {
    let (_, _, z_top_hull, _, z_canopy, w_canopy) = interpolate_gondola(y);
    let z_base = z_top_hull - GLAZING_BASE_DROP_M;
    let dz = (z_canopy - z_base).max(0.02);
    let (sin_a, cos_a) = a.sin_cos();
    let x = sin_a.signum() * sin_a.abs().powf(0.86) * w_canopy;
    let z = z_base + cos_a.max(0.0).powf(0.84) * dz;
    let p = Vec3::new(x, y, z);
    let inward = (Vec3::new(0.0, y, z_base) - p).normalize_or(Vec3::NEG_Z);
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
    let mut frames = MeshBuilder::new();
    for y in [
        OPENING_FWD_Y,
        WINDSCREEN_TOP_Y,
        HATCH_HINGE_Y,
        REAR_GLAZING_Y,
    ] {
        bow(&mut frames, y);
    }
    for side in [-1.0_f32, 1.0] {
        rail(
            &mut frames,
            side * QUARTER_POST_A,
            OPENING_FWD_Y,
            WINDSCREEN_TOP_Y,
        );
        rail(
            &mut frames,
            side * HATCH_RAIL_A,
            WINDSCREEN_TOP_Y,
            OPENING_AFT_Y,
        );
        rail(&mut frames, side * FRAC_PI_2, OPENING_FWD_Y, OPENING_AFT_Y);
    }
    rail(&mut frames, 0.0, HATCH_HINGE_Y, OPENING_AFT_Y);
    let mut prims = vec![fixed(
        "/P38g/Canopy/InteriorFrames",
        P38PartRole::CabinExterior,
        CANOPY_FRAME,
        frames,
    )];

    // Steel retainer around the internal armoured glass, matching its quad corners.
    let mut armor = MeshBuilder::new();
    let corners = [
        Vec3::new(-0.18, 1.54, 0.52),
        Vec3::new(0.18, 1.54, 0.52),
        Vec3::new(0.16, 1.18, 0.87),
        Vec3::new(-0.16, 1.18, 0.87),
        Vec3::new(-0.18, 1.54, 0.52),
    ];
    for pair in corners.windows(2) {
        if let [a, b] = pair {
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
