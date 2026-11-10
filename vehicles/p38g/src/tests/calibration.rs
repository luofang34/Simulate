//! The built mesh, measured, against the AN 01-75F-2 callouts in [`crate::drawing`].

use std::fmt::Write as _;

use serde_json::Value;

use crate::drawing as d;
use crate::{WING_AREA_M2, WING_DIHEDRAL_DEG, verify_p38g_three_view_and_stations};

/// Fin top above the fuselage reference line: the highest point of the digitised side view.
fn drawn_fin_top_frl_in() -> f32 {
    let doc: Value = serde_json::from_str(d::GENERAL_ARRANGEMENT_JSON).expect("drawing json");
    doc["views"]["side"]["outline_rings"]
        .as_array()
        .expect("rings")
        .iter()
        .flat_map(|ring| ring.as_array().expect("ring").iter())
        .filter_map(|p| p[1].as_f64())
        .fold(f64::NEG_INFINITY, f64::max) as f32
}

/// Every dimensioned quantity the mesh can be measured for, within the reading
/// tolerance of the drawing (about one stroke width, 0.5–1.5 in).
#[test]
fn built_mesh_measures_to_the_an01_75f_2_callouts() {
    let r = verify_p38g_three_view_and_stations();
    let ground = -(d::MAIN_AXLE_BELOW_FRL_STATIC_IN + d::MAIN_ROLLING_RADIUS_IN);
    let checks = [
        ("span", r.span_in, d::SPAN_IN, 1.0),
        ("length", r.length_in, d::OVERALL_LENGTH_IN, 1.5),
        ("boom centreline", r.boom_bl_in, d::BOOM_BL_IN, 0.5),
        (
            "propeller plane",
            r.prop_plane_fs_in,
            d::PROP_PLANE_FS_IN,
            1.0,
        ),
        ("root leading edge", r.root_le_fs_in, d::ROOT_LE_FS_IN, 1.0),
        ("root chord", r.root_chord_in, d::ROOT_CHORD_IN, 1.0),
        ("main tread", r.main_tread_in, d::MAIN_TREAD_IN, 1.0),
        ("nose axle", r.nose_axle_fs_in, d::NOSE_AXLE_FS_IN, 1.5),
        ("main axle", r.main_axle_fs_in, d::MAIN_AXLE_FS_IN, 1.5),
        (
            "main axle height",
            r.main_axle_frl_in,
            -d::MAIN_AXLE_BELOW_FRL_STATIC_IN,
            0.5,
        ),
        // The round 36 in tyre reaches 2.7 in below the static ground: the deflection
        // the drawing's 15.3 in rolling radius implies.
        (
            "tyre static deflection",
            ground - r.main_tyre_bottom_frl_in,
            18.0 - d::MAIN_ROLLING_RADIUS_IN,
            0.5,
        ),
        ("fin top", r.fin_top_frl_in, drawn_fin_top_frl_in(), 1.5),
        ("tailplane span", r.stab_span_in, d::STAB_SPAN_IN, 1.0),
        (
            "tailplane leading edge",
            r.stab_le_fs_in,
            d::STAB_LE_FS_IN,
            1.0,
        ),
        ("wing area m2", r.wing_area_m2, WING_AREA_M2, 0.8),
        ("dihedral deg", r.dihedral_deg, WING_DIHEDRAL_DEG, 0.15),
    ];
    let mut report = String::new();
    let mut failures = 0_usize;
    for (name, measured, drawn, tolerance) in checks {
        let bad = (measured - drawn).abs() > tolerance;
        failures += usize::from(bad);
        writeln!(
            report,
            "{} {name:<24} mesh {measured:8.2}  drawing {drawn:8.2}  error {:+7.2}",
            if bad { "FAIL" } else { "ok  " },
            measured - drawn
        )
        .expect("report");
    }
    assert!(
        failures == 0,
        "{failures} dimensions off the drawing:\n{report}"
    );
    assert!(r.prim_count >= 50 && r.total_triangles >= 8_000);
}

/// The airframe stands nose-up on the drawing's static ground line: the nose axle at its
/// static callout, the ground under both tyres at `5°33'46"`, and the propeller tips the
/// dimensioned `16.726 in` clear of it.
#[test]
fn built_mesh_stands_on_the_drawn_static_ground_line() {
    let r = verify_p38g_three_view_and_stations();
    for (name, measured, drawn, tolerance) in [
        (
            "nose axle height",
            r.nose_axle_frl_in,
            -d::NOSE_AXLE_BELOW_FRL_STATIC_IN,
            0.5,
        ),
        (
            "static ground angle deg",
            r.static_ground_angle_deg,
            d::STATIC_GROUND_ANGLE_DEG,
            0.1,
        ),
        (
            "propeller ground clearance",
            r.prop_tip_ground_clearance_in,
            d::PROP_TIP_GROUND_CLEARANCE_STATIC_IN,
            0.5,
        ),
    ] {
        assert!(
            (measured - drawn).abs() <= tolerance,
            "{name}: mesh {measured:.2}, drawing {drawn:.2}"
        );
    }
}

/// The tail's height callouts: the tailplane's chord line `21.099 in` above the reference
/// line, and the fin `82.156 + 2.828 in` from its top to its lowest point. With both met
/// and the airframe on the static ground line, the fin top stands `121 in` above the
/// ground against the sheet's `9'-10 3/8"`: that dimension disagrees with the sheet's
/// own gear and fin callouts by `2.7 in`, and its drawn length is `122.7 in`.
#[test]
fn the_tail_stands_at_its_drawn_heights() {
    let r = verify_p38g_three_view_and_stations();
    for (name, measured, drawn, tolerance) in [
        (
            "tailplane height",
            r.tailplane_frl_in,
            d::TAILPLANE_ABOVE_FRL_IN,
            0.3,
        ),
        (
            "fin height",
            r.fin_top_frl_in - r.fin_bottom_frl_in,
            d::FIN_HEIGHT_IN + d::FIN_LOWER_TIP_IN,
            0.5,
        ),
    ] {
        assert!(
            (measured - drawn).abs() <= tolerance,
            "{name}: mesh {measured:.2}, drawing {drawn:.2}"
        );
    }
}
