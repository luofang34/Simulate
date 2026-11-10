//! Fittings round the instrument board placed from Lockheed's frontal photograph
//! Z-4986 ("P-38 #3474 INSTRUMENT PANEL 12-23-42", a view forward over the control
//! wheel): the main switch box hanging under the board's right half, the row of
//! switches under it, and the reflector gunsight in its frame over the coaming.
//!
//! Each outline is read off the photograph through the camera solved on the
//! station 126.2 frame and the board, which places a point across and up the
//! ship to about 3 cm once its station is known. The stations are not seen in a
//! view from behind: the gunsight's is the manual's arm, the boxes' are assumed,
//! and each is stated at its constant.

use glam::Vec3;

use crate::cockpit_reservation::{CockpitInteriorContract, p38g_cockpit_contract};
use crate::drawing::CANOPY_SILL_ABOVE_FRL_IN;
use crate::drawing::frl_to_z;
use crate::mesh::{MeshBuilder, P38Material, P38MeshPrim, P38PartRole, fixed, mat};
use crate::stations::ARMOR_GLASS;

/// Black wrinkle-finished electrical boxes.
const BOX_BLACK: P38Material = mat([0.035, 0.035, 0.037], 0.05, 0.85);
/// Bright toggle levers and screw heads.
const TOGGLE_BRIGHT: P38Material = mat([0.62, 0.62, 0.60], 0.9, 0.35);
/// The sight frame's unpainted sheet.
const FRAME_ALUMINIUM: P38Material = mat([0.55, 0.56, 0.57], 0.85, 0.45);
/// The sight head's black body.
const SIGHT_BLACK: P38Material = mat([0.03, 0.03, 0.032], 0.2, 0.6);

/// The main switch box: its span across the ship, and its top and bottom below
/// the centre of the board, as photographed.
const SWITCH_BOX_X: (f32, f32) = (-0.155, 0.390);
const SWITCH_BOX_BELOW_BOARD_CENTRE: (f32, f32) = (0.194, 0.332);
/// How far the box's face stands aft of the board's plane. Assumed.
const SWITCH_BOX_AFT_M: f32 = 0.058;
/// The six toggles along the top of the box's right half, as photographed.
const SWITCH_BOX_TOGGLES_X: (f32, f32) = (0.07, 0.21);

/// The lower switch row: its span across the ship, and its top and bottom below
/// the centre of the board, as photographed.
const SWITCH_ROW_X: (f32, f32) = (-0.134, 0.396);
const SWITCH_ROW_BELOW_BOARD_CENTRE: (f32, f32) = (0.332, 0.394);
/// How far the row's face stands aft of the board's plane. Assumed.
const SWITCH_ROW_AFT_M: f32 = 0.078;

/// The gunsight frame: half its width, and its bottom and top above the canopy
/// sill, as photographed with the frame taken at the gunsight's station.
const SIGHT_FRAME_HALF_WIDTH_M: f32 = 0.18;
const SIGHT_FRAME_ABOVE_SILL: (f32, f32) = (0.034, 0.172);
/// Height of the frame's lower member, the bar with seven lightening holes.
const SIGHT_BAR_HEIGHT_M: f32 = 0.035;
const SIGHT_BAR_HOLES: usize = 7;
/// The sight head's ring: its centre above the sill and its radius.
const SIGHT_RING_ABOVE_SILL_M: f32 = 0.147;
const SIGHT_RING_RADIUS_M: f32 = 0.043;

/// The middle of the main switch box's face, the side the pilot sees.
pub(crate) fn main_switch_box_face() -> Vec3 {
    let [_, board_y, board_z] = p38g_cockpit_contract().main_instrument_panel.center_m;
    let (top, bottom) = SWITCH_BOX_BELOW_BOARD_CENTRE;
    Vec3::new(
        0.5 * (SWITCH_BOX_X.0 + SWITCH_BOX_X.1),
        board_y - SWITCH_BOX_AFT_M,
        board_z - 0.5 * (top + bottom),
    )
}

pub(crate) fn fitting_prims() -> Vec<P38MeshPrim> {
    let contract = p38g_cockpit_contract();
    let mut prims = switch_prims(&contract);
    prims.extend(gunsight_prims(&contract));
    prims
}

/// A row of `count` toggle levers from `from` to `to`, each standing out of
/// the face toward the pilot and a little up.
fn toggles(mesh: &mut MeshBuilder, from: Vec3, to: Vec3, count: usize) {
    for i in 0..count {
        let t = if count > 1 {
            i as f32 / (count - 1) as f32
        } else {
            0.5
        };
        let base = from + (to - from) * t;
        let tip = base + Vec3::new(0.0, -0.022, 0.008);
        mesh.tube(base, tip, 0.0035, 0.0028, 8);
        mesh.ellipsoid(tip, Vec3::splat(0.0042), 8, 6);
    }
}

fn switch_prims(contract: &CockpitInteriorContract) -> Vec<P38MeshPrim> {
    let [_, board_y, board_z] = contract.main_instrument_panel.center_m;
    let mut boxes = MeshBuilder::new();
    let mut levers = MeshBuilder::new();
    let (box_top, box_bottom) = SWITCH_BOX_BELOW_BOARD_CENTRE;
    let box_face = board_y - SWITCH_BOX_AFT_M;
    boxes.cuboid(
        Vec3::new(SWITCH_BOX_X.0, box_face, board_z - box_bottom),
        Vec3::new(SWITCH_BOX_X.1, board_y, board_z - box_top),
    );
    let toggle_z = board_z - box_top - 0.030;
    toggles(
        &mut levers,
        Vec3::new(SWITCH_BOX_TOGGLES_X.0, box_face, toggle_z),
        Vec3::new(SWITCH_BOX_TOGGLES_X.1, box_face, toggle_z),
        6,
    );
    let (row_top, row_bottom) = SWITCH_ROW_BELOW_BOARD_CENTRE;
    let row_face = board_y - SWITCH_ROW_AFT_M;
    boxes.cuboid(
        Vec3::new(SWITCH_ROW_X.0, row_face, board_z - row_bottom),
        Vec3::new(SWITCH_ROW_X.1, board_y, board_z - row_top),
    );
    let row_z = board_z - 0.5 * (row_top + row_bottom);
    toggles(
        &mut levers,
        Vec3::new(SWITCH_ROW_X.0 + 0.16, row_face, row_z),
        Vec3::new(SWITCH_ROW_X.0 + 0.36, row_face, row_z),
        8,
    );
    vec![
        fixed(
            "/P38g/CockpitInterior/SwitchBoxes",
            P38PartRole::CabinInterior,
            BOX_BLACK,
            boxes,
        ),
        fixed(
            "/P38g/CockpitInterior/SwitchBoxes/Toggles",
            P38PartRole::CabinInterior,
            TOGGLE_BRIGHT,
            levers,
        ),
    ]
}

fn gunsight_prims(contract: &CockpitInteriorContract) -> Vec<P38MeshPrim> {
    let mount = contract.reflector_gunsight_mount;
    let y = mount.center_m[1];
    let sill = frl_to_z(CANOPY_SILL_ABOVE_FRL_IN);
    let (bottom, top) = (
        sill + SIGHT_FRAME_ABOVE_SILL.0,
        sill + SIGHT_FRAME_ABOVE_SILL.1,
    );
    let half = SIGHT_FRAME_HALF_WIDTH_M;
    let sheet = 0.004;
    let mut frame = MeshBuilder::new();
    // Side posts and the top member.
    for side in [-1.0_f32, 1.0] {
        frame.cuboid(
            Vec3::new(side * half - 0.007, y - sheet, bottom),
            Vec3::new(side * half + 0.007, y + sheet, top),
        );
    }
    frame.cuboid(
        Vec3::new(-half, y - sheet, top - 0.012),
        Vec3::new(half, y + sheet, top),
    );
    // The lower member: two flanges and the webs left between its holes.
    let bar_top = bottom + SIGHT_BAR_HEIGHT_M;
    for z in [bottom, bar_top - 0.006] {
        frame.cuboid(
            Vec3::new(-half, y - 0.012, z),
            Vec3::new(half, y + sheet, z + 0.006),
        );
    }
    let pitch = 2.0 * half / SIGHT_BAR_HOLES as f32;
    for i in 0..=SIGHT_BAR_HOLES {
        let x = -half + pitch * i as f32;
        frame.cuboid(
            Vec3::new(x - 0.008, y - sheet, bottom),
            Vec3::new(x + 0.008, y + sheet, bar_top),
        );
    }
    // The sight head, its lens ring up toward the pilot's line of sight.
    let ring = Vec3::new(0.0, y, sill + SIGHT_RING_ABOVE_SILL_M);
    let mut head = MeshBuilder::new();
    head.tube(
        Vec3::new(0.0, y + 0.03, bar_top),
        ring,
        SIGHT_RING_RADIUS_M * 0.8,
        SIGHT_RING_RADIUS_M,
        20,
    );
    let mut glass = MeshBuilder::new();
    glass.cuboid(
        ring + Vec3::new(-0.04, -0.002, 0.0),
        ring + Vec3::new(0.04, 0.002, 0.075),
    );
    vec![
        fixed(
            "/P38g/CockpitInterior/GunsightFrame",
            P38PartRole::CabinInterior,
            FRAME_ALUMINIUM,
            frame,
        ),
        fixed(
            "/P38g/CockpitInterior/GunsightHead",
            P38PartRole::CabinInterior,
            SIGHT_BLACK,
            head,
        ),
        fixed(
            mount.prim_path,
            P38PartRole::CabinInterior,
            ARMOR_GLASS,
            glass,
        ),
    ]
}

#[cfg(test)]
mod tests;
