//! Flight and engine controls in reach of the seat, each moving with the control it
//! is: the open-top control wheel on its arm from the starboard column, the left
//! console's throttle quadrant, elevator tab wheel and gear lever, the right side's
//! flap lever and aileron and rudder tab controls, the rudder pedals and the fuel
//! selector valves on the floor.
//!
//! Placement and shapes follow two period Lockheed photographs of a P-38G cockpit
//! (<https://commons.wikimedia.org/wiki/File:Lockheed_P-38G_cockpit_looking_in_from_left_wing_061019-F-1234P-004.jpg>,
//! <https://commons.wikimedia.org/wiki/File:Lockheed_P-38G_cockpit_looking_in_from_right_wing_061019-F-1234P-003.jpg>)
//! and the P-38H/J/L pilot's flight operating instructions
//! (<http://www.airwar.ru/other/bibl/p-38pilot.pdf>, figures 4, 7 and 9). No source
//! found gives a control's travel, so every travel constant is an assumption that
//! maps the flight model's full command range (−1..1 or 0..1) onto it.

use glam::{DMat4, Vec3};

use crate::cockpit_reservation::p38g_cockpit_contract;

use crate::mesh::{
    Inside, MeshBuilder, P38ControlJoint, P38Material, P38MeshPrim, P38PartRole, fixed, mat,
};

mod column;
mod floor;
mod left_console;
mod right_side;

pub use column::p38g_uv_flood_lamp_m;

/// Black phenolic or hard-rubber grips and small knobs.
const GRIP_BLACK: P38Material = mat([0.042, 0.04, 0.038], 0.0, 0.45);
/// The column, arm and hub boss: polished steel, as Z-4972's highlights show.
const POLISHED: P38Material = mat([0.45, 0.45, 0.44], 1.0, 0.25);
/// Hard rubber worn brown-grey and polished where the hands grip.
const GRIP_WORN: P38Material = mat([0.085, 0.075, 0.065], 0.0, 0.30);
/// The throttles' light ball knobs; the photographs show them pale, read as ivory phenolic.
const KNOB_IVORY: P38Material = mat([0.62, 0.58, 0.48], 0.0, 0.35);
/// Black enamel on the column, arm and levers, worn semi-gloss.
const ENAMEL_BLACK: P38Material = mat([0.05, 0.05, 0.052], 0.2, 0.5);
/// Steel where hands and boots have worn the paint through.
const WORN_STEEL: P38Material = mat([0.50, 0.50, 0.49], 0.9, 0.36);
/// Cadmium-plated fittings: the hub boss, pivots and pedal frames.
const CADMIUM: P38Material = mat([0.30, 0.30, 0.29], 0.8, 0.5);
/// The canvas boot where the column passes through the floor.
const CANVAS: P38Material = mat([0.68, 0.66, 0.6], 0.0, 0.9);

pub(crate) fn polyline(mesh: &mut MeshBuilder, points: &[Vec3], radius: f32, sides: u32) {
    for pair in points.windows(2) {
        if let [a, b] = pair {
            mesh.tube(*a, *b, radius, radius, sides);
        }
    }
    // Spheres at the bends close the tubes' open joints.
    for p in points.iter().skip(1).take(points.len().saturating_sub(2)) {
        mesh.ellipsoid(*p, Vec3::splat(radius), sides, sides / 2);
    }
}

/// A smooth tube of `radius` swept along `points`, which lie in a plane whose normal
/// is `normal` (a wheel rim's axis): one shaded surface, without the facets of
/// joined straight tubes.
pub(crate) fn sweep(
    mesh: &mut MeshBuilder,
    points: &[Vec3],
    radius: f32,
    normal: Vec3,
    sides: u32,
) {
    let tangent = |i: usize| {
        let before = points.get(i.saturating_sub(1)).or(points.get(i));
        let after = points.get(i + 1).or(points.get(i));
        match (before, after) {
            (Some(a), Some(b)) => (*b - *a).normalize_or(Vec3::X),
            _ => Vec3::X,
        }
    };
    let rings: Vec<Vec<Vec3>> = points
        .iter()
        .enumerate()
        .map(|(i, centre)| {
            let side = tangent(i).cross(normal).normalize_or(Vec3::Z);
            (0..sides)
                .map(|k| {
                    let a = std::f32::consts::TAU * k as f32 / sides as f32;
                    *centre + (normal * a.cos() + side * a.sin()) * radius
                })
                .collect()
        })
        .collect();
    mesh.loft(&rings, true, Inside::Axis);
}

/// A joint turning about `axis` through `origin`, in aircraft axes: the controls
/// are modelled in place, so their part-local frame is the aircraft's.
const fn turn(
    channel: &'static str,
    origin: Vec3,
    axis: Vec3,
    per_unit: f32,
    rest: f32,
) -> P38ControlJoint {
    P38ControlJoint {
        channel,
        origin_m: [origin.x, origin.y, origin.z],
        axis: [axis.x, axis.y, axis.z],
        per_unit,
        rest,
        slides: false,
    }
}

/// A moving control part of one material on `joints`, outermost first.
fn control(
    path: &str,
    material: P38Material,
    mesh: MeshBuilder,
    joints: &[P38ControlJoint],
) -> P38MeshPrim {
    let mut prim = fixed(path, P38PartRole::CockpitControl, material, mesh);
    prim.control_joints = joints.to_vec();
    prim
}

/// Where the side and floor controls were laid out, before the cockpit's
/// stations were drawn: the top of each console shelf at its middle, and the
/// rudder pedals' faces on the floor. Each group keeps its own arrangement and
/// moves as one onto the contract's station for it; where in that station the
/// photographs put each control is not yet measured.
const LAID_OUT_CONSOLE_TOP: Vec3 = Vec3::new(0.42, 0.72, left_console::CONSOLE_TOP_Z);
const LAID_OUT_PEDALS: Vec3 = Vec3::new(0.0, 1.10, -0.22);
/// The pedals' faces stand this far aft of their pivots' station.
const PEDAL_FACE_AFT_M: f32 = 0.085;

/// `prims` moved so the point they were laid out about lands on `to`, joints
/// and all.
fn restation(mut prims: Vec<P38MeshPrim>, laid_out: Vec3, to: Vec3) -> Vec<P38MeshPrim> {
    let shift = DMat4::from_translation((to - laid_out).as_dvec3());
    for prim in &mut prims {
        prim.rest_to_aircraft = shift * prim.rest_to_aircraft;
    }
    prims
}

pub(crate) fn cockpit_control_prims() -> Vec<P38MeshPrim> {
    let contract = p38g_cockpit_contract();
    let left = Vec3::from_array(contract.left_throttle_quadrant.center_m);
    let right = Vec3::from_array(contract.right_flap_cooling_console.center_m);
    let pedals = Vec3::new(
        0.0,
        contract.rudder_pedal_pivot_y_m - PEDAL_FACE_AFT_M,
        contract.floor_z_m[0],
    );
    let mirrored = LAID_OUT_CONSOLE_TOP * Vec3::new(-1.0, 1.0, 1.0);
    let mut prims = column::prims();
    prims.extend(restation(left_console::prims(), mirrored, left));
    prims.extend(restation(right_side::prims(), LAID_OUT_CONSOLE_TOP, right));
    prims.extend(restation(floor::prims(), LAID_OUT_PEDALS, pedals));
    prims
}

#[cfg(test)]
mod tests;
