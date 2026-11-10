//! Tricycle landing gear (`T.O. 01-75F-1`): nose and main oleo struts, wheels and wells,
//! placed at the AN 01-75F-2 axle stations.

use std::f32::consts::TAU;

use glam::{DMat4, DVec3, Vec3};

use crate::airframe::keel_z_at;
use crate::drawing::{
    FRL_Z_M, INCH_M, MAIN_AXLE_BELOW_FRL_STATIC_IN, MAIN_AXLE_FS_IN, MAIN_TREAD_IN,
    NOSE_AXLE_BELOW_FRL_STATIC_IN, NOSE_AXLE_FS_IN, NOSE_Y_M,
};
use crate::mesh::{Inside, MeshBuilder, P38MeshPrim, P38PartRole, circle_segments, part};
use crate::stations::{CHROME_OLEO, GEAR_ALUMINIUM, TIRE_RUBBER};

/// Nose axle: centre of the dashed nose tyre on the AN 01-75F-2 side view.
const NOSE_AXLE_Y: f32 = NOSE_Y_M - NOSE_AXLE_FS_IN * INCH_M;
/// Main axle: centre of the dashed main tyre on the AN 01-75F-2 side view.
const MAIN_AXLE_Y: f32 = NOSE_Y_M - MAIN_AXLE_FS_IN * INCH_M;
/// Main axle height at the static oleo: `54.575 in` below the fuselage reference line
/// (AN 01-75F-2 front view).
const MAIN_AXLE_Z: f32 = FRL_Z_M - MAIN_AXLE_BELOW_FRL_STATIC_IN * INCH_M;
/// Nose axle height at the static oleo: `69.970 in` below the fuselage reference line
/// (AN 01-75F-2 side view). With the `11.6 in` and `15.3 in` static rolling radii the
/// nose tyre stands `11.7 in` lower than the mains, so the airframe sits nose-up on the
/// drawing's `5°33'46"` static ground line.
const NOSE_AXLE_Z: f32 = FRL_Z_M - NOSE_AXLE_BELOW_FRL_STATIC_IN * INCH_M;
/// Main and nose tyres: `36 in` and `27 in` (AN 01-75F-2).
const MAIN_TYRE_DIAMETER_IN: f32 = 36.0;
const NOSE_TYRE_DIAMETER_IN: f32 = 27.0;
/// Nose tyre section width: the 27 in Type I (smooth contour) tyre is `9.75 in` wide
/// inflated (Goodyear Aviation data book, Type I table, which also gives the `11.6 in`
/// loaded radius the drawing calls out).
const NOSE_TYRE_HALF_WIDTH_M: f32 = 0.5 * 9.75 * INCH_M;
/// Main tyre section width. No table found lists the 36 in Type I tyre; it is taken at
/// the 27 in tyre's proportions (`9.75 x 36 / 27 = 13.0 in`). The 339th FS photograph
/// of 43-2239 fits `13.2 in`.
const MAIN_TYRE_HALF_WIDTH_M: f32 = 0.5 * 13.0 * INCH_M;
/// Where the main oleo struts leave the boom gear wells.
const STRUT_TOP_Z: f32 = -0.36;
/// Main tyre centre off the ship centreline: half the `198 in` tread.
const MAIN_HALF_TREAD_M: f32 = 0.5 * MAIN_TREAD_IN * INCH_M;

fn polyline(mesh: &mut MeshBuilder, points: &[Vec3], radius: f32) {
    for pair in points.windows(2) {
        if let [a, b] = pair {
            mesh.tube(*a, *b, radius, radius, 10);
        }
    }
}

/// Scissor (torque) links on the `aft` side of a leg between its cylinder's lower end
/// and its piston's lower end.
fn torque_links(mesh: &mut MeshBuilder, cylinder_end: Vec3, piston_end: Vec3, aft: f32) {
    let off = Vec3::new(0.0, -0.07 * aft.signum(), 0.0);
    let apex = 0.5 * (cylinder_end + piston_end) + Vec3::new(0.0, -aft, 0.0);
    polyline(mesh, &[cylinder_end + off, apex, piston_end + off], 0.016);
}

/// One leg's two prims (painted structure, chromed piston), both retracting with the
/// gear about `pivot`.
fn push_leg(
    prims: &mut Vec<P38MeshPrim>,
    name: &str,
    pivot: Vec3,
    painted: MeshBuilder,
    piston: MeshBuilder,
) {
    let rest = DMat4::from_translation(pivot.as_dvec3());
    for (leaf, material, mesh) in [
        ("Strut", GEAR_ALUMINIUM, painted),
        ("Piston", CHROME_OLEO, piston),
    ] {
        prims.push(part(
            &format!("/P38g/Gear/{name}{leaf}"),
            P38PartRole::GearStruts,
            material,
            rest,
            mesh,
        ));
    }
}

/// Nose gear, its parts as AN 01-75FF-2 figure 88 names them and its proportions as the
/// 339th FS photographs show: a thick oleo cylinder, a chromed piston, a one-sided fork
/// sweeping round the port side of the wheel to the axle, torque arms, the shimmy damper
/// across the leg, and a pair of drag struts running up and aft into the bay.
pub(crate) fn build_nose_gear(prims: &mut Vec<P38MeshPrim>) {
    let radius = 0.5 * NOSE_TYRE_DIAMETER_IN * INCH_M;
    // The leg's trunnion is inside the bay, above the keel.
    let top = keel_z_at(0.0, NOSE_AXLE_Y) + 0.25;
    let pivot = Vec3::new(0.0, NOSE_AXLE_Y + 0.05, top);
    let axle = Vec3::new(0.0, NOSE_AXLE_Y, NOSE_AXLE_Z) - pivot;
    // AN 01-75FF-2 figure 88 (nose landing gear): the cylinder is a little over half the
    // leg, the piston short, and the fork takes the rest.
    let cylinder_end = Vec3::new(0.0, -0.01, 0.55 * axle.z);
    let piston_end = Vec3::new(0.0, -0.02, 0.66 * axle.z);
    let fork_x = -(NOSE_TYRE_HALF_WIDTH_M + 0.055);
    let mut painted = MeshBuilder::new();
    painted.tube(Vec3::ZERO, cylinder_end, 0.055, 0.055, 14);
    painted.tube(
        cylinder_end + Vec3::new(0.0, 0.0, 0.04),
        cylinder_end,
        0.063,
        0.063,
        14,
    );
    polyline(
        &mut painted,
        &[
            piston_end,
            Vec3::new(0.6 * fork_x, piston_end.y, piston_end.z - 0.02),
            Vec3::new(fork_x, axle.y + 0.02, axle.z + 0.75 * radius),
            Vec3::new(fork_x, axle.y, axle.z),
        ],
        0.034,
    );
    painted.tube(
        Vec3::new(fork_x - 0.02, axle.y, axle.z),
        Vec3::new(0.02, axle.y, axle.z),
        0.028,
        0.028,
        10,
    );
    torque_links(&mut painted, cylinder_end, piston_end, 0.16);
    // Shimmy damper: a cylinder lying across the leg at the oleo cylinder's lower end
    painted.tube(
        cylinder_end + Vec3::new(-0.03, 0.07, 0.03),
        cylinder_end + Vec3::new(0.17, 0.07, 0.03),
        0.026,
        0.026,
        10,
    );
    // The pair of drag struts running up and aft to the actuating cylinder
    for x in [-0.05_f32, 0.05] {
        painted.tube(
            Vec3::new(x, -0.04, 0.38 * axle.z),
            Vec3::new(x, -0.62, 0.02),
            0.018,
            0.018,
            10,
        );
    }
    let mut piston = MeshBuilder::new();
    piston.tube(cylinder_end, piston_end, 0.036, 0.036, 14);
    push_leg(prims, "Nose", pivot, painted, piston);

    append_wheel_and_hub(
        prims,
        "/P38g/Gear/NoseWheel",
        P38PartRole::NoseWheel,
        DVec3::new(0.0, f64::from(NOSE_AXLE_Y), f64::from(NOSE_AXLE_Z)),
        radius,
        NOSE_TYRE_HALF_WIDTH_M,
    );
}

/// Main legs: each pivots at station `179 7/8` (T.O. 01-75FG-2) inside its boom and
/// rakes forward to the axle; the wheel sits outboard of the leg on a stub axle, as the
/// photographs show, with the side strut, torque arms and drag strut of AN 01-75FF-2
/// figure 79.
pub(crate) fn build_main_gear(prims: &mut Vec<P38MeshPrim>) {
    const PIVOT_FS_IN: f32 = 179.875;
    for (name, w_name, w_role, side) in [
        ("MainLeft", "WheelLeft", P38PartRole::WheelLeft, -1.0_f32),
        ("MainRight", "WheelRight", P38PartRole::WheelRight, 1.0_f32),
    ] {
        let wheel = Vec3::new(side * MAIN_HALF_TREAD_M, MAIN_AXLE_Y, MAIN_AXLE_Z);
        let inboard = MAIN_TYRE_HALF_WIDTH_M + 0.085;
        let pivot = Vec3::new(
            wheel.x - side * inboard,
            NOSE_Y_M - PIVOT_FS_IN * INCH_M,
            STRUT_TOP_Z,
        );
        let axle = Vec3::new(0.0, wheel.y - pivot.y, wheel.z - pivot.z);
        // AN 01-75FF-2 figure 79 (main landing gear): the cylinder runs most of the leg,
        // braced at its top by a side strut from the fulcrum's inboard end.
        let cylinder_end = 0.62 * axle;
        let mut painted = MeshBuilder::new();
        painted.tube(Vec3::ZERO, cylinder_end, 0.066, 0.066, 14);
        painted.tube(
            Vec3::new(-side * 0.20, 0.0, 0.0),
            0.40 * axle,
            0.024,
            0.024,
            10,
        );
        painted.tube(cylinder_end * 0.96, cylinder_end, 0.075, 0.075, 14);
        painted.tube(
            axle,
            axle + Vec3::new(side * (inboard + 0.02), 0.0, 0.0),
            0.040,
            0.040,
            12,
        );
        torque_links(&mut painted, cylinder_end, axle, 0.20);
        // Drag strut
        painted.tube(0.45 * axle, Vec3::new(0.0, -0.62, 0.0), 0.028, 0.028, 10);
        let mut piston = MeshBuilder::new();
        piston.tube(cylinder_end, axle, 0.045, 0.045, 14);
        push_leg(prims, name, pivot, painted, piston);
        append_wheel_and_hub(
            prims,
            &format!("/P38g/Gear/{w_name}"),
            w_role,
            wheel.as_dvec3(),
            0.5 * MAIN_TYRE_DIAMETER_IN * INCH_M,
            MAIN_TYRE_HALF_WIDTH_M,
        );
    }
}

fn append_wheel_and_hub(
    prims: &mut Vec<P38MeshPrim>,
    path: &str,
    role: P38PartRole,
    center: DVec3,
    radius: f32,
    half_width: f32,
) {
    let mut tire = MeshBuilder::new();
    let segs = circle_segments(radius);
    let rings: Vec<Vec<Vec3>> = [
        (-half_width, radius * 0.42),
        (-half_width * 0.92, radius * 0.86),
        (-half_width * 0.48, radius),
        (half_width * 0.48, radius),
        (half_width * 0.92, radius * 0.86),
        (half_width, radius * 0.42),
    ]
    .into_iter()
    .map(|(x, r)| {
        (0..segs)
            .map(|i| {
                let (s, c) = (TAU * i as f32 / segs as f32).sin_cos();
                Vec3::new(x, c * r, s * r)
            })
            .collect()
    })
    .collect();
    tire.loft(&rings, true, Inside::Axis);
    let rest = DMat4::from_translation(center);
    prims.push(part(path, role, TIRE_RUBBER, rest, tire));

    let mut hub = MeshBuilder::new();
    hub.tube(
        Vec3::new(-half_width * 1.04, 0.0, 0.0),
        Vec3::new(half_width * 1.04, 0.0, 0.0),
        radius * 0.44,
        radius * 0.44,
        16,
    );
    prims.push(part(&format!("{path}Hub"), role, CHROME_OLEO, rest, hub));
}
