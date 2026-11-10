//! Cockpit interior stand-in geometry bound to the station contract in
//! [`crate::cockpit_reservation`]: the tub (floor and side walls inside the skin), the
//! glare shield, gunsight and radio boxes, the seat-back armour, the seat and the pilot
//! figure.

use glam::Vec3;

use crate::canopy_frames::canopy_frame_prims;
use crate::cockpit_controls::cockpit_control_prims;
use crate::cockpit_reservation::{
    CockpitInteriorContract, MAIN_BEAM_FS_IN, PANEL_TOP_ABOVE_FRL_IN, RUDDER_PEDAL_FS_IN,
    SEAT_ARMOUR_FS_IN, p38g_cockpit_contract,
};
use crate::drawing::{CANOPY_SILL_ABOVE_FRL_IN, WINDSCREEN_BASE_FS_IN, frl_to_z, fs_to_y};
use crate::gondola::{gondola_half_width_at, gondola_section};
use crate::instrument_panel::instrument_panel_prims;
use crate::mesh::{Inside, MeshBuilder, P38MeshPrim, P38PartRole, fixed};
use crate::stations::{
    ARMOR_GLASS, CHROME_OLEO, COCKPIT_GREEN, GUNMETAL, INSIGNIA_RED, LIFE_VEST, PANEL_BLACK,
    PILOT_HELMET, PILOT_SUIT,
};

/// Clearance between interior parts and the mould line: skin and frame depth.
const SKIN_GAP_M: f32 = 0.025;

pub(crate) fn cockpit_interior_prims() -> Vec<P38MeshPrim> {
    let contract = p38g_cockpit_contract();
    let mut prims = Vec::new();
    build_cockpit_tub(&mut prims, &contract);
    build_glare_shield_gunsight_and_radios(&mut prims, &contract);
    build_armour_and_switches(&mut prims, &contract);
    build_seat_and_pilot_bust(&mut prims, &contract);
    prims.extend(canopy_frame_prims());
    prims.extend(instrument_panel_prims());
    prims.extend(cockpit_control_prims());
    prims
}

/// Half-width available inside the skin at station `y` and height `z`.
fn inner_half_width(y: f32, z: f32) -> f32 {
    (gondola_half_width_at(y, z) - SKIN_GAP_M).max(0.02)
}

/// Floor height at station `y`: two levels joined by a ramp over the nose-wheel well.
fn floor_z_at(contract: &CockpitInteriorContract, y: f32) -> f32 {
    let [y0, y1] = contract.floor_step_y_m;
    let [z0, z1] = contract.floor_z_m;
    z0 + (z1 - z0) * ((y0 - y) / (y0 - y1)).clamp(0.0, 1.0)
}

/// A flat plate across the cabin at station `y` from `z_lo` to `z_hi`, following the
/// inside of the skin.
fn section_plate(mesh: &mut MeshBuilder, y: f32, z_lo: f32, z_hi: f32, facing: Vec3) {
    const STEPS: usize = 8;
    let z_at = |i: usize| z_lo + (z_hi - z_lo) * i as f32 / STEPS as f32;
    let port = (0..=STEPS).map(|i| Vec3::new(-inner_half_width(y, z_at(i)), y, z_at(i)));
    let starboard = (0..=STEPS)
        .rev()
        .map(|i| Vec3::new(inner_half_width(y, z_at(i)), y, z_at(i)));
    let outline: Vec<Vec3> = port.chain(starboard).collect();
    mesh.polygon(&outline, facing);
}

fn build_cockpit_tub(prims: &mut Vec<P38MeshPrim>, contract: &CockpitInteriorContract) {
    const WALL_STEPS: usize = 6;
    const LENGTH_STEPS: usize = 12;
    let sill = frl_to_z(CANOPY_SILL_ABOVE_FRL_IN);
    let [eye_x, eye_y, _] = contract.design_eyepoint_m;
    let mut ys: Vec<f32> = (0..=LENGTH_STEPS)
        .map(|i| {
            let t = i as f32 / LENGTH_STEPS as f32;
            fs_to_y(RUDDER_PEDAL_FS_IN + (SEAT_ARMOUR_FS_IN - RUDDER_PEDAL_FS_IN) * t)
        })
        .chain(contract.floor_step_y_m)
        .collect();
    ys.sort_by(|a, b| b.total_cmp(a));
    let mut tub = MeshBuilder::new();
    // Floor and side walls up to the sill, inside the skin.
    let floor: Vec<Vec<Vec3>> = ys
        .iter()
        .map(|&y| {
            let z = floor_z_at(contract, y);
            let half = inner_half_width(y, z);
            vec![Vec3::new(-half, y, z), Vec3::new(half, y, z)]
        })
        .collect();
    tub.loft_open(&floor, Inside::Point(Vec3::new(eye_x, eye_y, -2.0)));
    for side in [-1.0_f32, 1.0] {
        let wall: Vec<Vec<Vec3>> = ys
            .iter()
            .map(|&y| {
                let z0 = floor_z_at(contract, y);
                // Under the closed deck ahead of the windscreen the wall stops short of it.
                let top = sill.min(frl_to_z(gondola_section(y).deck_h_in - 2.0)) - 0.01;
                (0..=WALL_STEPS)
                    .map(|i| {
                        let z = z0 + (top - z0) * i as f32 / WALL_STEPS as f32;
                        Vec3::new(side * inner_half_width(y, z), y, z)
                    })
                    .collect()
            })
            .collect();
        tub.loft_open(&wall, Inside::Point(Vec3::new(side * 2.0, eye_y, 0.0)));
    }
    // Pedal-bay frame ahead of the panel and the frame behind the seat armour.
    let [floor_fwd, floor_aft] = contract.floor_z_m;
    let deck = frl_to_z(CANOPY_SILL_ABOVE_FRL_IN - 2.0);
    section_plate(
        &mut tub,
        contract.rudder_pedal_pivot_y_m,
        floor_fwd,
        deck,
        Vec3::NEG_Y,
    );
    section_plate(
        &mut tub,
        contract.seat_armour_y_m - 0.03,
        floor_aft,
        sill,
        Vec3::Y,
    );
    // Side console shelves
    for console in [
        contract.left_throttle_quadrant,
        contract.right_flap_cooling_console,
    ] {
        let top = Vec3::from_array(console.center_m);
        let half = Vec3::new(0.5 * console.width_m, 0.5 * console.height_m, 0.0);
        tub.cuboid(top - half - Vec3::new(0.0, 0.0, 0.06), top + half);
    }
    let mut tub = fixed(
        "/P38g/CockpitInterior/TubAndBulkheads",
        P38PartRole::CabinInterior,
        COCKPIT_GREEN,
        tub,
    );
    tub.double_sided = true;
    prims.push(tub);
}

fn build_glare_shield_gunsight_and_radios(
    prims: &mut Vec<P38MeshPrim>,
    contract: &CockpitInteriorContract,
) {
    let panel_y = contract.main_instrument_panel.center_m[1];
    let mut panel = MeshBuilder::new();
    // Glare shield from the windscreen base back to the top of the panel
    let top = frl_to_z(PANEL_TOP_ABOVE_FRL_IN) + 0.01;
    let fwd_y = fs_to_y(WINDSCREEN_BASE_FS_IN + 1.0);
    let fwd_z = frl_to_z(CANOPY_SILL_ABOVE_FRL_IN - 1.5);
    let half_aft = inner_half_width(panel_y, top);
    let half_fwd = inner_half_width(fwd_y, fwd_z).min(half_aft);
    panel.polygon(
        &[
            Vec3::new(-half_aft, panel_y - 0.02, top),
            Vec3::new(half_aft, panel_y - 0.02, top),
            Vec3::new(half_fwd, fwd_y, fwd_z),
            Vec3::new(-half_fwd, fwd_y, fwd_z),
        ],
        Vec3::Z,
    );
    // Reflector gunsight body on its arm from the panel
    let sight = Vec3::from_array(contract.reflector_gunsight_mount.center_m);
    panel.tube(
        Vec3::new(0.0, panel_y, top),
        sight - Vec3::new(0.0, 0.0, 0.06),
        0.012,
        0.012,
        8,
    );
    panel.tube(
        sight + Vec3::new(0.0, 0.05, -0.08),
        sight + Vec3::new(0.0, -0.04, -0.01),
        0.034,
        0.028,
        10,
    );
    // SCR-274-N command set boxes behind the main beam, under the rear glazing
    let (radio_fwd, radio_aft) = (
        fs_to_y(MAIN_BEAM_FS_IN + 2.0),
        fs_to_y(MAIN_BEAM_FS_IN + 15.0),
    );
    for rx in [-0.16_f32, 0.0, 0.16] {
        panel.cuboid(
            Vec3::new(rx - 0.07, radio_aft, frl_to_z(15.0)),
            Vec3::new(rx + 0.07, radio_fwd, frl_to_z(22.0)),
        );
    }
    let mut shield = fixed(
        "/P38g/CockpitInterior/CoamingGunsightAndRadios",
        P38PartRole::CabinInterior,
        PANEL_BLACK,
        panel,
    );
    shield.double_sided = true;
    prims.push(shield);

    // Tilted glass reflector plate
    let mut sight_glass = MeshBuilder::new();
    sight_glass.cuboid(
        sight + Vec3::new(-0.042, -0.02, 0.0),
        sight + Vec3::new(0.042, 0.02, 0.08),
    );
    prims.push(fixed(
        contract.reflector_gunsight_mount.prim_path,
        P38PartRole::CabinInterior,
        ARMOR_GLASS,
        sight_glass,
    ));
}

fn build_armour_and_switches(prims: &mut Vec<P38MeshPrim>, contract: &CockpitInteriorContract) {
    // Seat-back armour plate with its narrower head plate, directly behind the pilot
    let y = contract.seat_armour_y_m;
    let mut armour = MeshBuilder::new();
    armour.cuboid(
        Vec3::new(-0.20, y - 0.012, frl_to_z(13.0)),
        Vec3::new(0.20, y, frl_to_z(28.0)),
    );
    armour.cuboid(
        Vec3::new(-0.11, y - 0.012, frl_to_z(28.0)),
        Vec3::new(0.11, y, frl_to_z(37.0)),
    );
    prims.push(fixed(
        "/P38g/CockpitInterior/SeatArmor",
        P38PartRole::CabinInterior,
        GUNMETAL,
        armour,
    ));

    // Red Curtiss Electric propeller feathering switch guards on the left console
    let mut red_knobs = MeshBuilder::new();
    let [cx, cy, cz] = contract.left_throttle_quadrant.center_m;
    for dy in [0.30_f32, 0.26] {
        red_knobs.cuboid(
            Vec3::new(cx - 0.02, cy + dy - 0.015, cz),
            Vec3::new(cx + 0.02, cy + dy + 0.015, cz + 0.03),
        );
    }
    prims.push(fixed(
        "/P38g/CockpitInterior/FeatherSwitches",
        P38PartRole::CabinInterior,
        INSIGNIA_RED,
        red_knobs,
    ));
}

/// Metal bucket seat on the seat reference point, and the pilot figure with the eyes on
/// the design eyepoint.
fn build_seat_and_pilot_bust(prims: &mut Vec<P38MeshPrim>, contract: &CockpitInteriorContract) {
    let srp = Vec3::from_array(contract.seat_reference_point_m);
    let eye = Vec3::from_array(contract.design_eyepoint_m);
    let lean = contract.seat_back_recline_deg.to_radians().tan();
    let mut seat = MeshBuilder::new();
    seat.cuboid(
        srp + Vec3::new(-0.22, 0.0, -0.06),
        srp + Vec3::new(0.22, 0.42, 0.0),
    );
    let back: Vec<Vec<Vec3>> = [0.0_f32, 0.55]
        .into_iter()
        .map(|up| {
            let y = srp.y - up * lean;
            vec![
                Vec3::new(-0.22, y, srp.z + up),
                Vec3::new(0.22, y, srp.z + up),
            ]
        })
        .collect();
    seat.loft_open(&back, Inside::Point(srp - Vec3::new(0.0, 2.0, 0.0)));
    let mut seat = fixed(
        "/P38g/CockpitInterior/BucketSeat",
        P38PartRole::CabinInterior,
        CHROME_OLEO,
        seat,
    );
    seat.double_sided = true;
    prims.push(seat);

    let mut pilot = MeshBuilder::new();
    pilot.ellipsoid(
        eye + Vec3::new(0.0, -0.06, -0.46),
        Vec3::new(0.21, 0.15, 0.28),
        14,
        8,
    );
    prims.push(fixed(
        "/P38g/CockpitInterior/PilotTorso",
        P38PartRole::PilotFigure,
        PILOT_SUIT,
        pilot,
    ));

    let mut head = MeshBuilder::new();
    head.ellipsoid(
        eye + Vec3::new(0.0, -0.08, -0.06),
        Vec3::new(0.11, 0.12, 0.13),
        14,
        8,
    );
    // Headrest pad on the armour plate
    let armour_y = contract.seat_armour_y_m;
    head.cuboid(
        Vec3::new(-0.10, armour_y, eye.z - 0.12),
        Vec3::new(0.10, armour_y + 0.075, eye.z + 0.04),
    );
    prims.push(fixed(
        "/P38g/CockpitInterior/PilotHelmet",
        P38PartRole::PilotFigure,
        PILOT_HELMET,
        head,
    ));

    let mut vest = MeshBuilder::new();
    vest.ellipsoid(
        eye + Vec3::new(0.0, -0.02, -0.40),
        Vec3::new(0.17, 0.13, 0.18),
        12,
        6,
    );
    prims.push(fixed(
        "/P38g/CockpitInterior/MaeWestVest",
        P38PartRole::PilotFigure,
        LIFE_VEST,
        vest,
    ));
}
