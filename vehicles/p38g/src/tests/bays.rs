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
