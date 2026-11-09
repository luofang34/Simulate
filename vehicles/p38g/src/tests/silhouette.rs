//! Silhouette overlap between the shipped mesh and the digitised AN 01-75F-2 general
//! arrangement, per view: intersection over union and boundary deviation in inches.

use std::fmt::Write as _;
use std::path::PathBuf;

use glam::{DVec3, Vec2, Vec3};
use serde_json::Value;

use crate::drawing::{GENERAL_ARRANGEMENT_JSON, INCH_M, y_to_fs, z_to_frl};
use crate::{P38PartRole, p38g_mesh_prims};

mod raster;

use raster::{Grid, Metrics, compare, diff_image, fill_rings, fill_triangles};

/// Raster cell size, inches.
const RES_IN: f32 = 0.5;

#[derive(Clone, Copy, Debug)]
enum View {
    Side,
    Plan,
    Front,
}

impl View {
    const ALL: [Self; 3] = [Self::Side, Self::Plan, Self::Front];

    fn key(self) -> &'static str {
        match self {
            Self::Side => "side",
            Self::Plan => "plan",
            Self::Front => "front",
        }
    }

    /// Model point to the view plane as drawn on the sheet (`right`, `up`), inches. The
    /// side view looks at the port side, nose left; the plan view looks down, nose at the
    /// bottom; the front view looks aft, so starboard is on the left in both.
    fn project(self, p: Vec3) -> Vec2 {
        let (bl, fs, h) = (p.x / INCH_M, y_to_fs(p.y), z_to_frl(p.z));
        match self {
            Self::Side => Vec2::new(fs, h),
            Self::Plan => Vec2::new(-bl, fs),
            Self::Front => Vec2::new(-bl, h),
        }
    }

    /// A ring point stored in the view's own axes, to (`right`, `up`).
    fn ring_point(self, a: f32, b: f32) -> Vec2 {
        match self {
            Self::Side => Vec2::new(a, b),
            Self::Plan | Self::Front => Vec2::new(-a, b),
        }
    }

    /// Parts the drawing does not show in this view, so they must not count against it.
    fn excludes(self, path: &str, role: P38PartRole) -> bool {
        // Propellers are lines or dashed discs, the landing gear is phantom, and the gun
        // barrels, pitot mast and drop-tank pylons are not drawn as outline.
        let hidden_everywhere = matches!(
            role,
            P38PartRole::PropellerBlades
                | P38PartRole::PropellerDisc
                | P38PartRole::GearStruts
                | P38PartRole::NoseWheel
                | P38PartRole::WheelLeft
                | P38PartRole::WheelRight
        ) || [
            "/Gear/",
            "NoseArmament",
            "GunBlastTroughs",
            "PitotMast",
            "InnerPylons",
        ]
        .iter()
        .any(|s| path.contains(s));
        // The side view leaves the wing out; only its root section is dashed in.
        let side_wing = matches!(self, Self::Side)
            && (path.starts_with("/P38g/Wing/") || path.contains("Intercooler"));
        hidden_everywhere || side_wing
    }

    /// Acceptance limits (minimum IoU, maximum boundary deviation in, 95th percentile in);
    /// each sits just outside the geometry's present agreement with the drawing, so a
    /// regression fails while the remaining gaps are closed.
    fn limits(self) -> (f32, f32, f32) {
        match self {
            Self::Side => (0.84, 15.5, 11.5),
            Self::Plan => (0.92, 12.5, 5.0),
            Self::Front => (0.80, 13.5, 4.0),
        }
    }
}

fn reference_rings(doc: &Value, view: View) -> Vec<Vec<Vec2>> {
    doc["views"][view.key()]["outline_rings"]
        .as_array()
        .expect("outline_rings")
        .iter()
        .map(|ring| {
            ring.as_array()
                .expect("ring")
                .iter()
                .map(|p| {
                    let a = p[0].as_f64().expect("x") as f32;
                    let b = p[1].as_f64().expect("y") as f32;
                    view.ring_point(a, b)
                })
                .collect()
        })
        .collect()
}

fn model_triangles(view: View) -> Vec<[Vec2; 3]> {
    let mut tris = Vec::new();
    for prim in p38g_mesh_prims() {
        if view.excludes(&prim.prim_path, prim.role) {
            continue;
        }
        let pts: Vec<Vec2> = prim
            .vertices
            .iter()
            .map(|v| {
                let p = prim
                    .rest_to_aircraft
                    .transform_point3(Vec3::from(v.position).as_dvec3());
                view.project(DVec3::as_vec3(p))
            })
            .collect();
        for tri in prim.indices.as_chunks::<3>().0 {
            tris.push([
                pts[tri[0] as usize],
                pts[tri[1] as usize],
                pts[tri[2] as usize],
            ]);
        }
    }
    tris
}

fn bounds(points: impl Iterator<Item = Vec2>) -> (Vec2, Vec2) {
    points.fold(
        (Vec2::splat(f32::INFINITY), Vec2::splat(f32::NEG_INFINITY)),
        |(lo, hi), p| (lo.min(p), hi.max(p)),
    )
}

fn measure(doc: &Value, view: View, out: Option<&PathBuf>) -> Metrics {
    let rings = reference_rings(doc, view);
    let tris = model_triangles(view);
    let (rlo, rhi) = bounds(rings.iter().flatten().copied());
    let (mlo, mhi) = bounds(tris.iter().flatten().copied());
    let grid = Grid::covering(rlo.min(mlo), rhi.max(mhi), RES_IN, 4.0);
    let reference = fill_rings(&grid, &rings);
    let model = fill_triangles(&grid, &tris);
    if let Some(dir) = out {
        diff_image(&grid, &reference, &model)
            .save(dir.join(format!("p38g_silhouette_{}.png", view.key())))
            .expect("write silhouette diff");
    }
    compare(&grid, &reference, &model)
}

/// The mesh, seen orthographically from the side, above and ahead, must cover the
/// digitised AN 01-75F-2 outline of each view. Set `SIMULATE_VERIFY_OUT` to also write
/// `p38g_silhouette_<view>.png` (drawing only red, model only blue, both grey) and
/// `p38g_silhouette_metrics.txt`.
#[test]
fn mesh_silhouettes_match_the_an01_75f_2_general_arrangement() {
    let doc: Value = serde_json::from_str(GENERAL_ARRANGEMENT_JSON).expect("drawing json");
    let out = std::env::var_os("SIMULATE_VERIFY_OUT").map(PathBuf::from);
    let mut report = String::new();
    let mut failures = Vec::new();
    for view in View::ALL {
        let m = measure(&doc, view, out.as_ref());
        let (iou, max_dev, p95) = view.limits();
        writeln!(
            report,
            "{:<5} IoU {:.4}  max deviation {:5.1} in  p95 {:5.2} in",
            view.key(),
            m.iou,
            m.max_dev_in,
            m.p95_dev_in
        )
        .expect("report");
        if m.iou < iou || m.max_dev_in > max_dev || m.p95_dev_in > p95 {
            failures.push(view.key());
        }
    }
    if let Some(dir) = out {
        std::fs::write(dir.join("p38g_silhouette_metrics.txt"), &report).expect("metrics");
    }
    assert!(
        failures.is_empty(),
        "{failures:?} outside limits:\n{report}"
    );
}

/// Every station constant in [`crate::drawing`] repeats a callout in the JSON.
#[test]
fn drawing_constants_repeat_the_digitised_callouts() {
    use crate::drawing as d;
    let doc: Value = serde_json::from_str(GENERAL_ARRANGEMENT_JSON).expect("drawing json");
    let value = |name: &str| {
        doc["stations"]
            .as_array()
            .expect("stations")
            .iter()
            .find(|s| s["name"] == name)
            .and_then(|s| s["value"].as_f64())
            .unwrap_or_else(|| panic!("no station {name}")) as f32
    };
    for (name, constant) in [
        ("overall_length", d::OVERALL_LENGTH_IN),
        ("span", d::SPAN_IN),
        ("prop_plane_fs", d::PROP_PLANE_FS_IN),
        ("root_le_fs", d::ROOT_LE_FS_IN),
        ("root_chord", d::ROOT_CHORD_IN),
        ("mac_le_fs", d::MAC_LE_FS_IN),
        ("mac", d::MAC_IN),
        ("tip_chord_ws289", d::TIP_CHORD_WS289_IN),
        ("boom_bl", d::BOOM_BL_IN),
        ("stab_le_fs", d::STAB_LE_FS_IN),
        ("stab_span", d::STAB_SPAN_IN),
        ("stab_chord", d::STAB_CHORD_IN),
        ("prop_diameter", d::PROP_DIAMETER_IN),
        ("main_tread", d::MAIN_TREAD_IN),
        (
            "main_axle_below_frl_static",
            d::MAIN_AXLE_BELOW_FRL_STATIC_IN,
        ),
        ("main_rolling_radius_static", d::MAIN_ROLLING_RADIUS_IN),
        (
            "frl_above_ground_nose_static",
            d::FRL_ABOVE_GROUND_NOSE_STATIC_IN,
        ),
        ("nose_rolling_radius_static", d::NOSE_ROLLING_RADIUS_IN),
        ("nose_axle_fs_drawn", d::NOSE_AXLE_FS_IN),
        ("main_axle_fs_drawn", d::MAIN_AXLE_FS_IN),
        ("overall_height_static", d::OVERALL_HEIGHT_IN),
        ("canopy_top_above_frl", d::CANOPY_TOP_ABOVE_FRL_IN),
        ("gondola_bottom_below_frl", d::GONDOLA_BOTTOM_BELOW_FRL_IN),
        ("thrust_line_above_frl", d::THRUST_LINE_ABOVE_FRL_IN),
    ] {
        assert!(
            (value(name) - constant).abs() < 1e-3,
            "{name}: json {} vs constant {constant}",
            value(name)
        );
    }
    assert!((value("tailplane_chord") - d::STAB_CHORD_IN - d::ELEVATOR_CHORD_IN).abs() < 1e-3);
}
