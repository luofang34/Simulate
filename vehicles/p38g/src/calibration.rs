//! Dimensions measured on the built mesh, in the drawing's units, so they can be checked
//! against the AN 01-75F-2 general arrangement in [`crate::drawing`].

use glam::{DVec3, Vec3};

use crate::drawing::{INCH_M, y_to_fs, z_to_frl};
use crate::mesh::{P38MeshPrim, P38PartRole};
use crate::package::p38g_mesh_prims;
use crate::stations::{SEMI_SPAN, TIP_START_X, wing_station};

/// Dimensions read off the built mesh (`p38g_mesh_prims`), in inches unless named
/// otherwise. Stations are fuselage stations (inches aft of the gondola nose) and
/// heights are above the fuselage reference line, as on the drawing.
#[derive(Clone, Debug, PartialEq)]
pub struct P38BlueprintCalibrationReport {
    /// Wingtip to wingtip.
    pub span_in: f32,
    /// Gondola nose to rudder trailing edge, gun barrels excluded.
    pub length_in: f32,
    /// Ship centreline to the boom centreline (mid-thickness of the starboard fin).
    pub boom_bl_in: f32,
    /// Station of the propeller blade plane (mean of the blade vertices).
    pub prop_plane_fs_in: f32,
    /// Station of the spinner tip.
    pub spinner_tip_fs_in: f32,
    /// Leading-edge station of the wing section on the ship centreline.
    pub root_le_fs_in: f32,
    /// Chord of the wing section on the ship centreline.
    pub root_chord_in: f32,
    /// Main-wheel axle to main-wheel axle.
    pub main_tread_in: f32,
    /// Nose-wheel axle station.
    pub nose_axle_fs_in: f32,
    /// Main-wheel axle station.
    pub main_axle_fs_in: f32,
    /// Main axle height above the fuselage reference line (negative: below).
    pub main_axle_frl_in: f32,
    /// Lowest tyre point above the fuselage reference line (negative: below).
    pub tyre_bottom_frl_in: f32,
    /// Fin top above the fuselage reference line.
    pub fin_top_frl_in: f32,
    /// Horizontal-tail tip to tip.
    pub stab_span_in: f32,
    /// Horizontal-tail leading-edge station between the booms.
    pub stab_le_fs_in: f32,
    /// Planform wing area from the wing station function, square metres.
    pub wing_area_m2: f32,
    /// Dihedral of the chord plane from the wing station function, degrees.
    pub dihedral_deg: f32,
    /// Number of authored mesh prims.
    pub prim_count: usize,
    /// Total vertex count across all prims.
    pub total_vertices: usize,
    /// Total triangle count across all prims.
    pub total_triangles: usize,
}

/// Aircraft-frame vertex positions of one prim.
fn points(prim: &P38MeshPrim) -> impl Iterator<Item = Vec3> + '_ {
    prim.vertices.iter().map(|v| {
        let p = prim
            .rest_to_aircraft
            .transform_point3(Vec3::from(v.position).as_dvec3());
        p.as_vec3()
    })
}

/// Every vertex of the prims `select` accepts.
fn gather(prims: &[P38MeshPrim], select: impl Fn(&P38MeshPrim) -> bool) -> Vec<Vec3> {
    prims
        .iter()
        .filter(|p| select(p))
        .flat_map(points)
        .collect()
}

fn extent(pts: &[Vec3], axis: fn(Vec3) -> f32) -> (f32, f32) {
    pts.iter()
        .fold((f32::INFINITY, f32::NEG_INFINITY), |(lo, hi), &p| {
            (lo.min(axis(p)), hi.max(axis(p)))
        })
}

const X: fn(Vec3) -> f32 = |p| p.x;
const Y: fn(Vec3) -> f32 = |p| p.y;
const Z: fn(Vec3) -> f32 = |p| p.z;

fn translation(prims: &[P38MeshPrim], role: P38PartRole) -> DVec3 {
    prims
        .iter()
        .find(|p| p.role == role && !p.prim_path.ends_with("Hub"))
        .map_or(DVec3::ZERO, |p| p.rest_to_aircraft.w_axis.truncate())
}

/// Measures the built mesh against the quantities the drawing dimensions.
#[must_use]
pub fn verify_p38g_three_view_and_stations() -> P38BlueprintCalibrationReport {
    let prims = p38g_mesh_prims();
    let all = gather(&prims, |_| true);
    let hull = gather(&prims, |p| {
        !p.prim_path.contains("NoseArmament")
            && !matches!(
                p.role,
                P38PartRole::PropellerBlades | P38PartRole::PropellerDisc
            )
    });
    let fins = gather(&prims, |p| {
        p.prim_path.ends_with("VerticalFins") || p.role == P38PartRole::Rudder
    });
    let starboard_fin: Vec<Vec3> = fins.iter().copied().filter(|p| p.x > 0.0).collect();
    let blades = gather(&prims, |p| p.role == P38PartRole::PropellerBlades);
    let spinners = gather(&prims, |p| p.role == P38PartRole::PropellerSpinner);
    let root: Vec<Vec3> = gather(&prims, |p| p.prim_path == "/P38g/Wing/UpperOlive")
        .into_iter()
        .filter(|p| p.x.abs() < 1e-4)
        .collect();
    let tyres = gather(&prims, |p| {
        matches!(
            p.role,
            P38PartRole::WheelLeft | P38PartRole::WheelRight | P38PartRole::NoseWheel
        )
    });
    let stab = gather(&prims, |p| p.prim_path.ends_with("HorizontalTail"));
    let stab_inner: Vec<Vec3> = stab.iter().copied().filter(|p| p.x.abs() < 1.0).collect();
    let (left, right) = (
        translation(&prims, P38PartRole::WheelLeft),
        translation(&prims, P38PartRole::WheelRight),
    );
    let nose = translation(&prims, P38PartRole::NoseWheel);
    let blade_y = blades.iter().map(|p| p.y).sum::<f32>() / blades.len().max(1) as f32;
    let (root_te, root_le) = extent(&root, Y);
    let (span_lo, span_hi) = extent(&all, X);
    let (tail_y, nose_y) = extent(&hull, Y);
    let (fin_lo, fin_hi) = extent(&starboard_fin, X);
    let stab_x = extent(&stab, X);
    let (_, _, z_root, ..) = wing_station(0.0);
    let (_, _, z_tip, ..) = wing_station(TIP_START_X);
    P38BlueprintCalibrationReport {
        span_in: (span_hi - span_lo) / INCH_M,
        length_in: (nose_y - tail_y) / INCH_M,
        boom_bl_in: 0.5 * (fin_lo + fin_hi) / INCH_M,
        prop_plane_fs_in: y_to_fs(blade_y),
        spinner_tip_fs_in: y_to_fs(extent(&spinners, Y).1),
        root_le_fs_in: y_to_fs(root_le),
        root_chord_in: (root_le - root_te) / INCH_M,
        main_tread_in: (right.x - left.x) as f32 / INCH_M,
        nose_axle_fs_in: y_to_fs(nose.y as f32),
        main_axle_fs_in: y_to_fs(right.y as f32),
        main_axle_frl_in: z_to_frl(right.z as f32),
        tyre_bottom_frl_in: z_to_frl(extent(&tyres, Z).0),
        fin_top_frl_in: z_to_frl(extent(&fins, Z).1),
        stab_span_in: (stab_x.1 - stab_x.0) / INCH_M,
        stab_le_fs_in: y_to_fs(extent(&stab_inner, Y).1),
        wing_area_m2: planform_wing_area(),
        dihedral_deg: (z_tip - z_root).atan2(TIP_START_X).to_degrees(),
        prim_count: prims.len(),
        total_vertices: prims.iter().map(|p| p.vertices.len()).sum(),
        total_triangles: prims.iter().map(|p| p.indices.len() / 3).sum(),
    }
}

fn planform_wing_area() -> f32 {
    let steps = 400_u16;
    let dx = 2.0 * SEMI_SPAN / f32::from(steps);
    (0..steps)
        .map(|i| {
            let xa = -SEMI_SPAN + f32::from(i) * dx;
            let chord = |x: f32| {
                let (lead, trail, ..) = wing_station(x);
                lead - trail
            };
            0.5 * (chord(xa) + chord(xa + dx)) * dx
        })
        .sum()
}
