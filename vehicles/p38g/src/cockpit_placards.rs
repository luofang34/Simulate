//! Placards whose wording a P-38G photograph shows, each a flat plate the
//! renderer letters with the panel's own typeface: `GUN HEAT` and `COCKPIT
//! HEAT` by their knobs on the windscreen's aft frame, and `SURFACE CONTROL
//! LOCK` along the stowed lock on the right sill
//! (<https://commons.wikimedia.org/wiki/File:Lockheed_P-38G_cockpit_looking_in_from_left_wing_061019-F-1234P-004.jpg>,
//! <https://commons.wikimedia.org/wiki/File:Lockheed_P-38G_cockpit_looking_in_from_right_wing_061019-F-1234P-003.jpg>).
//! A placard whose wording no source shows is not drawn, nor one whose box is
//! not modelled.

use glam::Vec3;

use crate::drawing::{HATCH_AFT_FRAME_FS_IN, WINDSCREEN_TOP_FRAME_FS_IN, fs_to_y};
use crate::gondola::{canopy_sill_z, gondola_half_width_at};
use crate::mesh::{MeshBuilder, P38Material, P38MeshPrim, P38PartRole, fixed, mat};

/// Data and instruction placards: black-painted plates, lettered by the renderer.
const PLACARD_BLACK: P38Material = mat([0.03, 0.03, 0.03], 0.3, 0.5);
/// Depth of the frame and skin a plate stands inboard of the mould line.
const FRAME_DEPTH_M: f32 = 0.03;
/// The heat knobs' height on the windscreen's aft frame: where the rays to
/// them in the two photographs pass each other, good to about 0.05 m.
const HEAT_KNOB_Z_M: f32 = 0.66;

/// A placard: its wording and where its plate is, aircraft frame.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct P38Placard {
    /// Name of its prim under `/P38g/CockpitInterior/Placards/`.
    pub name: &'static str,
    /// The wording, as the photographs show it.
    pub text: &'static str,
    /// Centre of the plate, m.
    pub centre_m: [f32; 3],
    /// Half the plate's width along the text's direction, m.
    pub right_m: [f32; 3],
    /// Half the plate's height, up the letters, m.
    pub up_m: [f32; 3],
}

/// A plate on a wall facing inboard, read by someone looking at that wall.
fn on_wall(name: &'static str, text: &'static str, centre: Vec3, half: [f32; 2]) -> P38Placard {
    // Facing the left wall a reader looks outboard (-x) with forward on the
    // right; facing the right wall, aft is on the right.
    let along = if centre.x < 0.0 { Vec3::Y } else { Vec3::NEG_Y };
    P38Placard {
        name,
        text,
        centre_m: centre.to_array(),
        right_m: (along * half[0]).to_array(),
        up_m: (Vec3::Z * half[1]).to_array(),
    }
}

/// The P-38G's sourced placards.
#[must_use]
pub fn p38g_placards() -> Vec<P38Placard> {
    // On the aft face of the windscreen's aft frame, just under each knob.
    let frame_y = fs_to_y(WINDSCREEN_TOP_FRAME_FS_IN) - FRAME_DEPTH_M;
    let z = HEAT_KNOB_Z_M - 0.03;
    let heat = |side: f32| {
        Vec3::new(
            side * (gondola_half_width_at(frame_y, z) - FRAME_DEPTH_M),
            frame_y,
            z,
        )
    };
    let lock_y = 0.5 * (fs_to_y(WINDSCREEN_TOP_FRAME_FS_IN) + fs_to_y(HATCH_AFT_FRAME_FS_IN));
    let lock_z = canopy_sill_z(lock_y) - 0.035;
    let lock_x = gondola_half_width_at(lock_y, lock_z) - FRAME_DEPTH_M - 0.02;
    vec![
        on_wall("GunHeat", "GUN HEAT", heat(-1.0), [0.035, 0.009]),
        on_wall("CockpitHeat", "COCKPIT HEAT", heat(1.0), [0.042, 0.009]),
        on_wall(
            "SurfaceControlLock",
            "SURFACE CONTROL LOCK",
            Vec3::new(lock_x, lock_y, lock_z),
            [0.11, 0.009],
        ),
    ]
}

/// One flat black plate per placard, `uv` spanning it for the lettering.
pub(crate) fn prims() -> Vec<P38MeshPrim> {
    p38g_placards()
        .into_iter()
        .map(|placard| {
            let mut mesh = MeshBuilder::new();
            mesh.label_quad(
                Vec3::from_array(placard.centre_m),
                Vec3::from_array(placard.right_m),
                Vec3::from_array(placard.up_m),
            );
            fixed(
                &format!("/P38g/CockpitInterior/Placards/{}", placard.name),
                P38PartRole::DisplaySurface,
                PLACARD_BLACK,
                mesh,
            )
        })
        .collect()
}
