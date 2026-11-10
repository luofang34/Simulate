use glam::{DVec3, Vec3};

use super::board::{TILE_MARGIN_M, circle_inside};
use super::{
    BEZEL_PROUD_M, DIAL_FRACTION, FACE_RECESS_M, GLASS_RECESS_M, instrument_panel_prims, openings,
    panel_frame,
};
use crate::{P38PartRole, p38g_cockpit_contract};

#[test]
fn every_layout_slot_gets_one_display_surface_sized_from_its_bezel() {
    let openings = openings();
    // Sixteen pack slots and eleven empty cases.
    assert_eq!(openings.len(), 27);
    let prims = instrument_panel_prims();
    let faces: Vec<_> = prims
        .iter()
        .filter(|p| p.role == P38PartRole::DisplaySurface)
        .collect();
    assert_eq!(faces.len(), openings.len());
    for o in &openings {
        let path = format!(
            "/P38g/CockpitInterior/Instruments/{}/{}",
            o.slot, o.instrument
        );
        let face = faces.iter().find(|p| p.prim_path == path).expect(&path);
        let xs: Vec<f32> = face.vertices.iter().map(|v| v.position[0]).collect();
        let width = xs.iter().copied().fold(f32::MIN, f32::max)
            - xs.iter().copied().fold(f32::MAX, f32::min);
        assert!((width - o.dial_diameter_m).abs() < 1e-5);
        // The pack prints the face at the bezel bore's share of the case.
        assert!((o.dial_diameter_m - o.case_diameter_m * DIAL_FRACTION).abs() < 5e-4);
        // u runs right, v runs down the dial.
        let top_left = face.vertices.iter().find(|v| v.uv == [0.0, 0.0]).unwrap();
        let bottom_right = face.vertices.iter().find(|v| v.uv == [1.0, 1.0]).unwrap();
        assert!(top_left.position[0] < bottom_right.position[0]);
        assert!(top_left.position[2] > bottom_right.position[2]);
    }
}

#[test]
fn dial_faces_face_the_pilot_recessed_behind_bezel_and_glass() {
    const {
        assert!(GLASS_RECESS_M < FACE_RECESS_M);
        assert!(FACE_RECESS_M >= 0.008 && FACE_RECESS_M <= 0.010);
        assert!(
            BEZEL_PROUD_M > GLASS_RECESS_M,
            "glass sits inside the bezel"
        );
    }
    let eye = DVec3::from_array(p38g_cockpit_contract().design_eyepoint_m.map(f64::from));
    for prim in instrument_panel_prims()
        .iter()
        .filter(|p| p.role == P38PartRole::DisplaySurface)
    {
        let at = |i: usize| {
            let p = Vec3::from_array(prim.vertices[i].position).as_dvec3();
            prim.rest_to_aircraft.transform_point3(p)
        };
        let [a, b, c] = [0, 1, 2].map(|k| at(prim.indices[k] as usize));
        let normal = (b - a).cross(c - a).normalize();
        assert!(
            normal.dot((eye - a).normalize()) > 0.5,
            "{}",
            prim.prim_path
        );
    }
}

#[test]
fn every_case_sits_inside_the_measured_board_and_clear_of_its_neighbours() {
    let outline = p38g_cockpit_contract().main_instrument_panel_outline_m;
    let openings = openings();
    for lamp in super::cases::WARNING_LAMPS {
        let r = 0.5 * lamp.diameter_m;
        assert!(
            circle_inside(outline, lamp.centre, r + TILE_MARGIN_M),
            "{}",
            lamp.name
        );
        for o in &openings {
            let [x, z] = o.centre;
            let gap = (x - lamp.centre[0]).abs().max((z - lamp.centre[1]).abs())
                - r
                - 0.5 * o.case_diameter_m;
            assert!(gap > 0.0002, "{} and {} overlap", lamp.name, o.slot);
        }
    }
    for (i, o) in openings.iter().enumerate() {
        let t = 0.5 * o.case_diameter_m + TILE_MARGIN_M;
        let [x, z] = o.centre;
        assert!(
            circle_inside(outline, o.centre, t),
            "{} leaves the board",
            o.slot
        );
        // Square tiles about two holes may shrink to share a diagonal, but
        // never below the squares circumscribing them.
        for other in openings.iter().skip(i + 1) {
            let [ox, oz] = other.centre;
            let r = 0.5 * o.case_diameter_m;
            let gap = (ox - x).abs().max((oz - z).abs()) - r - 0.5 * other.case_diameter_m;
            assert!(gap > 0.0002, "{} and {} overlap", o.slot, other.slot);
        }
    }
}

/// The highest point of the board's crown, aircraft frame.
fn crown_top_m() -> DVec3 {
    let outline = p38g_cockpit_contract().main_instrument_panel_outline_m;
    let top = outline.iter().max_by(|a, b| a[1].total_cmp(&b[1])).unwrap();
    panel_frame().transform_point3(DVec3::new(f64::from(top[0]), 0.0, f64::from(top[1])))
}

#[test]
fn the_crown_clears_the_gunsight_mount_and_the_board_fits_the_gondola() {
    let contract = p38g_cockpit_contract();
    let crown = crown_top_m();
    let sight = &contract.reflector_gunsight_mount;
    // The mount is a plate tilted back about its centre; its lowest edge.
    let [_, sy, sz] = sight.center_m.map(f64::from);
    let half = 0.5 * f64::from(sight.height_m);
    let tilt = f64::from(sight.tilt_deg).to_radians();
    let sight_bottom = DVec3::new(0.0, sy + half * tilt.sin(), sz - half * tilt.cos());
    assert!(
        crown.z < sight_bottom.z || (crown.y - sight_bottom.y).abs() > 0.02,
        "crown {crown} meets the gunsight mount's lower edge {sight_bottom}"
    );
    // Every outline corner lies inside the gondola's cross-section at its
    // station, below the hull top the windscreen frame sits on.
    for &[x, z] in contract.main_instrument_panel_outline_m {
        let p = panel_frame().transform_point3(DVec3::new(f64::from(x), 0.0, f64::from(z)));
        let (z_mid, half_w, z_top, z_bot, _, _) = crate::interpolate_gondola(p.y as f32);
        assert!(
            (p.x.abs() as f32) < half_w,
            "corner {x},{z} at {p} beyond the {half_w} m half-width"
        );
        assert!(
            (p.z as f32) < z_top + 0.25 && (p.z as f32) > z_bot,
            "corner {x},{z} at {p} outside {z_bot}..{z_top} (mid {z_mid})"
        );
    }
}
