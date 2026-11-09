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
        ("fin top", r.fin_top_frl_in, drawn_fin_top_frl_in(), 1.5),
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
