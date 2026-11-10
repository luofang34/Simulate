//! The open landing-gear bays and their doors, measured on the built mesh.

use glam::Vec3;

use crate::bays::{NOSE_BAY, main_bay};
use crate::drawing::{INCH_M, y_to_fs};
use crate::{P38MeshPrim, P38PartRole, p38g_mesh_prims};

fn points(prim: &P38MeshPrim) -> Vec<Vec3> {
    prim.vertices
        .iter()
        .map(|v| {
            prim.rest_to_aircraft
                .transform_point3(Vec3::from(v.position).as_dvec3())
                .as_vec3()
        })
        .collect()
}

fn span(points: &[Vec3], key: impl Fn(&Vec3) -> f32) -> (f32, f32) {
    points
        .iter()
        .map(key)
        .fold((f32::INFINITY, f32::NEG_INFINITY), |(lo, hi), v| {
            (lo.min(v), hi.max(v))
        })
}

/// Each bay has an interior and two doors that, closed, span the bay's stations and its
/// width, and hinge on opposite edges along the airframe's length.
#[test]
fn each_bay_has_two_doors_that_close_its_opening() {
    let prims = p38g_mesh_prims();
    for (path, bay) in [
        ("/P38g/Gondola/NoseBay", NOSE_BAY),
        ("/P38g/Booms/MainBayLeft", main_bay(-1.0)),
        ("/P38g/Booms/MainBayRight", main_bay(1.0)),
    ] {
        assert!(
            prims
                .iter()
                .any(|p| p.prim_path == format!("{path}/BayInterior")),
            "{path} has no interior"
        );
        let doors: Vec<&P38MeshPrim> = prims
            .iter()
            .filter(|p| p.prim_path.starts_with(&format!("{path}/GearDoor")))
            .collect();
        assert_eq!(doors.len(), 2, "{path}");
        let closed: Vec<Vec3> = doors.iter().flat_map(|d| points(d)).collect();
        let (fwd, aft) = span(&closed, |p| y_to_fs(p.y));
        assert!(
            (fwd - bay.fs.0).abs() < 0.1 && (aft - bay.fs.1).abs() < 0.1,
            "{path} doors span FS {fwd:.1}..{aft:.1}"
        );
        let (left, right) = span(&closed, |p| p.x);
        let width_in = (right - left) / INCH_M;
        let opening_in = 2.0 * bay.half_width / INCH_M;
        assert!(
            (width_in - opening_in).abs() < 3.0,
            "{path} doors {width_in:.1} in wide, opening {opening_in:.1} in"
        );
        let hinges: Vec<f64> = doors
            .iter()
            .map(|d| d.custom_hinge_axis.expect("door hinge").y)
            .collect();
        assert!(hinges[0] * hinges[1] < 0.0, "{path} hinges {hinges:?}");
    }
}

/// The openings pass the wheels they house: each is wider than its tyre.
#[test]
fn the_wheels_fit_through_their_bays() {
    let prims = p38g_mesh_prims();
    for (role, bay) in [
        (P38PartRole::NoseWheel, NOSE_BAY),
        (P38PartRole::WheelLeft, main_bay(-1.0)),
        (P38PartRole::WheelRight, main_bay(1.0)),
    ] {
        let tyre: Vec<Vec3> = prims
            .iter()
            .filter(|p| p.role == role)
            .flat_map(points)
            .collect();
        let (left, right) = span(&tyre, |p| p.x);
        assert!(
            left > bay.centre_x - bay.half_width && right < bay.centre_x + bay.half_width,
            "{role:?} tyre {left:.3}..{right:.3} m against a {:.3} m opening at {:.3} m",
            2.0 * bay.half_width,
            bay.centre_x
        );
    }
}

/// The bays lie where the manuals put the gear: the main bay spans the main leg's pivot
/// (station `179 7/8`) and ends inside the forward boom (station 265), with room aft of
/// the pivot for the leg and the 36 in wheel to lie stowed; the nose bay spans the
/// hanging nose wheel's axle.
#[test]
fn the_bays_span_their_gear() {
    use crate::drawing::{MAIN_AXLE_BELOW_FRL_STATIC_IN, NOSE_AXLE_FS_IN};
    const MAIN_PIVOT_FS_IN: f32 = 179.875;
    const FORWARD_BOOM_END_FS_IN: f32 = 265.0;
    let main = main_bay(1.0);
    assert!(main.fs.0 < MAIN_PIVOT_FS_IN && main.fs.1 <= FORWARD_BOOM_END_FS_IN);
    // The stowed wheel's aft edge: the leg no longer than the static axle's depth below
    // the reference line, swung aft about the pivot, plus the tyre's 18 in radius.
    let stowed_aft = MAIN_PIVOT_FS_IN + MAIN_AXLE_BELOW_FRL_STATIC_IN + 18.0;
    assert!(
        main.fs.1 >= stowed_aft - 3.0,
        "{} < {stowed_aft}",
        main.fs.1
    );
    assert!((NOSE_BAY.fs.0..NOSE_BAY.fs.1).contains(&NOSE_AXLE_FS_IN));
}

/// Each leg reaches from inside its bay to its wheel's axle, the main legs from the
/// stated pivot station `179 7/8`, and no leg part is thinner than its photograph shows
/// the oleo (about `4 in` across at the nose, `5 in` at the mains).
#[test]
fn the_legs_run_from_their_bays_to_their_axles() {
    use crate::drawing::{MAIN_AXLE_BELOW_FRL_STATIC_IN, NOSE_AXLE_BELOW_FRL_STATIC_IN, z_to_frl};
    let prims = p38g_mesh_prims();
    for (strut, axle_in, min_width_m) in [
        ("/P38g/Gear/NoseStrut", NOSE_AXLE_BELOW_FRL_STATIC_IN, 0.10),
        (
            "/P38g/Gear/MainLeftStrut",
            MAIN_AXLE_BELOW_FRL_STATIC_IN,
            0.125,
        ),
        (
            "/P38g/Gear/MainRightStrut",
            MAIN_AXLE_BELOW_FRL_STATIC_IN,
            0.125,
        ),
    ] {
        let prim = prims.iter().find(|p| p.prim_path == strut).expect(strut);
        let pts = points(prim);
        let (low, high) = span(&pts, |p| z_to_frl(p.z));
        assert!((low + axle_in).abs() < 2.0, "{strut} ends at {low:.1} in");
        // The cylinder's upper end, at the pivot (the drag strut ends well aft of it).
        let pivot = prim.rest_to_aircraft.w_axis.truncate().as_vec3();
        assert!(
            z_to_frl(pivot.z) <= high + 0.01,
            "{strut} pivot above its parts"
        );
        let top: Vec<Vec3> = pts
            .iter()
            .copied()
            .filter(|p| (p.z - pivot.z).abs() < 0.01 && (p.y - pivot.y).abs() < 0.1)
            .collect();
        let (left, right) = span(&top, |p| p.x);
        assert!(
            right - left >= min_width_m,
            "{strut} {:.3} m across",
            right - left
        );
        if strut.contains("Main") {
            let pivot_fs = y_to_fs(prim.rest_to_aircraft.w_axis.y as f32);
            assert!(
                (pivot_fs - 179.875).abs() < 0.01,
                "{strut} pivots at FS {pivot_fs}"
            );
        }
    }
}
