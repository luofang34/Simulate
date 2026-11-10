//! Flight and engine controls in reach of the seat: the control wheel on the arm of
//! its column, which stands on the starboard floor, the left-console quadrant with a
//! throttle, propeller and mixture lever per engine, the right-console flap lever,
//! and the rudder pedals.

use glam::Vec3;

use crate::cockpit_reservation::p38g_cockpit_contract;
use crate::drawing::{frl_to_z, fs_to_y};
use crate::mesh::{MeshBuilder, P38Material, P38MeshPrim, P38PartRole, fixed, mat};
use crate::stations::{GUNMETAL, INSIGNIA_RED};

/// Centre of the control wheel's hub: `3 in` aft of the column's station, within a
/// `29 in` grip reach of a shoulder `4 in` aft of the design eyepoint, at chest height.
/// Assumed until the hub is placed from the cockpit photographs.
const WHEEL_HUB: Vec3 = Vec3::new(0.0, fs_to_y(119.0), frl_to_z(20.0));
/// Black phenolic grips and knobs.
const GRIP_BLACK: P38Material = mat([0.05, 0.05, 0.055], 0.05, 0.55);

fn polyline(mesh: &mut MeshBuilder, points: &[Vec3], radius: f32) {
    for pair in points.windows(2) {
        if let [a, b] = pair {
            mesh.tube(*a, *b, radius, radius, 8);
        }
    }
}

pub(crate) fn cockpit_control_prims() -> Vec<P38MeshPrim> {
    let mut steel = MeshBuilder::new();
    let mut grips = MeshBuilder::new();
    control_wheel(&mut steel, &mut grips);
    let mut knobs_red = MeshBuilder::new();
    engine_quadrant(&mut steel, &mut grips, &mut knobs_red);
    flap_lever_and_pedals(&mut steel, &mut grips);
    let interior = P38PartRole::CabinInterior;
    vec![
        fixed(
            "/P38g/CockpitInterior/ControlWheel",
            interior,
            GUNMETAL,
            steel,
        ),
        fixed(
            "/P38g/CockpitInterior/GripsAndKnobs",
            interior,
            GRIP_BLACK,
            grips,
        ),
        fixed(
            "/P38g/CockpitInterior/MixtureKnobs",
            interior,
            INSIGNIA_RED,
            knobs_red,
        ),
    ]
}

/// The column rises from its starboard floor attachment, turns inboard above the
/// pilot's knee and carries the wheel: a grip loop either side of the hub.
fn control_wheel(steel: &mut MeshBuilder, grips: &mut MeshBuilder) {
    let pivot = Vec3::from_array(p38g_cockpit_contract().control_column_base_m);
    let elbow = Vec3::new(pivot.x, WHEEL_HUB.y + 0.02, WHEEL_HUB.z - 0.06);
    polyline(
        steel,
        &[pivot, elbow, WHEEL_HUB + Vec3::new(0.05, 0.02, 0.0)],
        0.02,
    );
    steel.tube(
        WHEEL_HUB + Vec3::new(0.0, 0.03, 0.0),
        WHEEL_HUB - Vec3::new(0.0, 0.02, 0.0),
        0.03,
        0.03,
        12,
    );
    for side in [-1.0_f32, 1.0] {
        let p = |x: f32, z: f32| WHEEL_HUB + Vec3::new(side * x, 0.0, z);
        polyline(steel, &[p(0.0, 0.0), p(0.06, 0.045), p(0.14, 0.06)], 0.011);
        polyline(
            steel,
            &[p(0.0, 0.0), p(0.06, -0.045), p(0.14, -0.06)],
            0.011,
        );
        polyline(
            grips,
            &[p(0.14, 0.06), p(0.175, 0.0), p(0.14, -0.06)],
            0.017,
        );
    }
}

/// Six levers in the quadrant on the left console: throttles inboard, then
/// propeller rpm, then mixture, a pair (left and right engine) of each.
fn engine_quadrant(steel: &mut MeshBuilder, grips: &mut MeshBuilder, red: &mut MeshBuilder) {
    let [cx, cy, cz] = p38g_cockpit_contract().left_throttle_quadrant.center_m;
    let box_min = Vec3::new(cx - 0.07, cy - 0.02, cz);
    let box_max = Vec3::new(cx + 0.07, cy + 0.20, cz + 0.06);
    steel.cuboid(box_min, box_max);
    for i in 0..6_u8 {
        // Inboard to outboard: throttles, propellers, mixtures, a pair of each.
        let pair_gap = 0.026 * f32::from(i / 2);
        let x = cx + 0.055 - 0.017 * f32::from(i) - pair_gap * 0.35;
        let root = Vec3::new(x, cy + 0.08, box_max.z);
        let top = root + Vec3::new(0.0, 0.05, 0.09);
        steel.tube(root, top, 0.005, 0.005, 6);
        let radius = if i < 2 { 0.016 } else { 0.011 };
        let knob = if i >= 4 { &mut *red } else { &mut *grips };
        knob.ellipsoid(top, Vec3::splat(radius), 10, 6);
    }
}

fn flap_lever_and_pedals(steel: &mut MeshBuilder, grips: &mut MeshBuilder) {
    let contract = p38g_cockpit_contract();
    let [cx, cy, cz] = contract.right_flap_cooling_console.center_m;
    let root = Vec3::new(cx, cy + 0.10, cz);
    let top = root + Vec3::new(-0.02, 0.04, 0.10);
    steel.tube(root, top, 0.008, 0.008, 6);
    grips.ellipsoid(top, Vec3::splat(0.014), 10, 6);
    let pivot_y = contract.rudder_pedal_pivot_y_m;
    let [floor, _] = contract.floor_z_m;
    for side in [-1.0_f32, 1.0] {
        let x = side * 0.16;
        steel.tube(
            Vec3::new(x, pivot_y, floor + 0.02),
            Vec3::new(x, pivot_y - 0.08, floor + 0.22),
            0.012,
            0.012,
            6,
        );
        steel.cuboid(
            Vec3::new(x - 0.05, pivot_y - 0.10, floor + 0.18),
            Vec3::new(x + 0.05, pivot_y - 0.07, floor + 0.28),
        );
    }
}
