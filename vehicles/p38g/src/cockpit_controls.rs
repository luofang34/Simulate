//! Flight and engine controls in reach of the seat: the bow-tie control wheel on
//! its girder pivoting from the starboard wall, the left-console quadrant with a
//! throttle, propeller and mixture lever per engine, the right-console flap lever,
//! and the rudder pedals.

use glam::Vec3;

use crate::cockpit_reservation::p38g_cockpit_contract;
use crate::mesh::{MeshBuilder, P38Material, P38MeshPrim, P38PartRole, fixed, mat};
use crate::stations::{GUNMETAL, INSIGNIA_RED};

/// Centre of the control wheel's hub, ahead of the pilot's chest.
const WHEEL_HUB: Vec3 = Vec3::new(0.0, 0.80, 0.30);
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

/// The girder rises from its starboard pivot, turns inboard above the pilot's
/// knee and carries the bow-tie wheel: a grip loop either side of the hub.
fn control_wheel(steel: &mut MeshBuilder, grips: &mut MeshBuilder) {
    let pivot = Vec3::from_array(p38g_cockpit_contract().control_column_floor_socket_m);
    let elbow = Vec3::new(pivot.x - 0.04, WHEEL_HUB.y + 0.02, WHEEL_HUB.z - 0.06);
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
    let box_min = Vec3::new(-0.43, 0.60, 0.18);
    let box_max = Vec3::new(-0.29, 0.82, 0.24);
    steel.cuboid(box_min, box_max);
    let xs = [-0.305, -0.322, -0.348, -0.365, -0.392, -0.409];
    for (i, x) in xs.into_iter().enumerate() {
        let root = Vec3::new(x, 0.70, box_max.z);
        let top = root + Vec3::new(0.0, 0.05, 0.09);
        steel.tube(root, top, 0.005, 0.005, 6);
        let radius = if i < 2 { 0.016 } else { 0.011 };
        let knob = if i >= 4 { &mut *red } else { &mut *grips };
        knob.ellipsoid(top, Vec3::splat(radius), 10, 6);
    }
}

fn flap_lever_and_pedals(steel: &mut MeshBuilder, grips: &mut MeshBuilder) {
    let root = Vec3::new(0.37, 0.74, 0.18);
    let top = Vec3::new(0.35, 0.78, 0.28);
    steel.tube(root, top, 0.008, 0.008, 6);
    grips.ellipsoid(top, Vec3::splat(0.014), 10, 6);
    for side in [-1.0_f32, 1.0] {
        let x = side * 0.16;
        steel.tube(
            Vec3::new(x, 1.16, -0.24),
            Vec3::new(x, 1.08, -0.04),
            0.012,
            0.012,
            6,
        );
        steel.cuboid(
            Vec3::new(x - 0.05, 1.06, -0.08),
            Vec3::new(x + 0.05, 1.09, 0.02),
        );
    }
}
