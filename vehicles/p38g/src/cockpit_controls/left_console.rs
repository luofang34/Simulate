//! The left console: the engine quadrant's six levers, the elevator tab wheel on the
//! console's inboard face and the landing-gear lever ahead of the quadrant.

use std::f32::consts::TAU;

use glam::Vec3;

use super::{CADMIUM, ENAMEL_BLACK, GRIP_BLACK, KNOB_IVORY, WORN_STEEL, control, sweep, turn};
use crate::mesh::{MeshBuilder, P38MeshPrim, P38PartRole, fixed};

/// Height of the console tops above the cockpit datum, m.
pub(crate) const CONSOLE_TOP_Z: f32 = 0.18;
/// The quadrant's lever pivots: fore and aft, and height (inside its housing).
const QUADRANT_PIVOT_Y: f32 = 0.73;
const QUADRANT_PIVOT_Z: f32 = 0.205;
/// Lever length from pivot to knob centre, m.
const LEVER_LENGTH: f32 = 0.13;
/// Throttle travel per unit of the command, rad, forward (open) negative about `+X`.
/// Assumed: no source gives the quadrant's arc; 0.87 rad is a 50° sweep.
const THROTTLE_PER_UNIT: f32 = -0.87;
/// Propeller and mixture travel per unit, rad. Assumed, as for the throttles.
const SMALL_LEVER_PER_UNIT: f32 = -0.70;

/// The six levers inboard to outboard: throttles (the biggest and innermost knobs,
/// light balls), then propeller rpm, then mixture, with small dark knobs; each pair
/// left engine outboard of right.
const LEVERS: [(&str, f32, f32); 6] = [
    ("throttle_right", -0.300, THROTTLE_PER_UNIT),
    ("throttle_left", -0.319, THROTTLE_PER_UNIT),
    ("propeller_right", -0.343, SMALL_LEVER_PER_UNIT),
    ("propeller_left", -0.358, SMALL_LEVER_PER_UNIT),
    ("mixture_right", -0.381, SMALL_LEVER_PER_UNIT),
    ("mixture_left", -0.396, SMALL_LEVER_PER_UNIT),
];

/// Centre and radius of the elevator tab wheel, standing fore and aft against the
/// console's inboard face (01-75FF-1 figure 4 item 35; P-38G photograph from the
/// right wing).
const TAB_WHEEL_CENTRE: Vec3 = Vec3::new(-0.282, 0.50, 0.085);
const TAB_WHEEL_RADIUS: f32 = 0.072;
/// Tab wheel turn per unit of the elevator trim command, rad. JSBSim's positive
/// pitch trim is nose down, which rolls the wheel's top forward (negative about
/// `+X`). Assumed: one and a half turns either way.
const TAB_WHEEL_PER_UNIT: f32 = -1.5 * TAU;

/// The gear lever's pivot, low and forward of the quadrant (01-75FF-1 figure 4
/// items 32 and 33).
const GEAR_PIVOT: Vec3 = Vec3::new(-0.372, 1.00, CONSOLE_TOP_Z + 0.01);
/// Lever swing from down (forward) to up (aft), rad. Assumed.
const GEAR_SWING: f32 = -1.2;

pub(super) fn prims() -> Vec<P38MeshPrim> {
    let mut prims = quadrant();
    prims.extend(tab_wheel());
    prims.extend(gear_lever());
    prims
}

fn quadrant() -> Vec<P38MeshPrim> {
    let mut housing = MeshBuilder::new();
    housing.cuboid(
        Vec3::new(-0.415, QUADRANT_PIVOT_Y - 0.11, CONSOLE_TOP_Z),
        Vec3::new(-0.290, QUADRANT_PIVOT_Y + 0.11, CONSOLE_TOP_Z + 0.05),
    );
    // Friction and vernier knobs on the housing's outboard side (figure 4 items 18, 19).
    let mut knobs = MeshBuilder::new();
    for y in [QUADRANT_PIVOT_Y - 0.07, QUADRANT_PIVOT_Y + 0.06] {
        knobs.tube(
            Vec3::new(-0.415, y, CONSOLE_TOP_Z + 0.03),
            Vec3::new(-0.43, y, CONSOLE_TOP_Z + 0.03),
            0.011,
            0.011,
            10,
        );
    }
    let mut prims = vec![
        fixed(
            "/P38g/CockpitInterior/EngineQuadrant",
            P38PartRole::CabinInterior,
            ENAMEL_BLACK,
            housing,
        ),
        fixed(
            "/P38g/CockpitInterior/EngineQuadrant/Knobs",
            P38PartRole::CabinInterior,
            GRIP_BLACK,
            knobs,
        ),
    ];
    for (channel, x, per_unit) in LEVERS {
        let pivot = Vec3::new(x, QUADRANT_PIVOT_Y, QUADRANT_PIVOT_Z);
        let joint = [turn(channel, pivot, Vec3::X, per_unit, 0.5)];
        let tip = pivot + Vec3::new(0.0, 0.0, LEVER_LENGTH);
        let mut arm = MeshBuilder::new();
        arm.tube(pivot, tip, 0.0045, 0.0045, 8);
        let throttle = channel.starts_with("throttle");
        let (radius, material) = if throttle {
            (0.019, KNOB_IVORY)
        } else {
            (0.011, GRIP_BLACK)
        };
        let mut knob = MeshBuilder::new();
        knob.ellipsoid(
            tip + Vec3::new(0.0, 0.0, radius * 0.7),
            Vec3::splat(radius),
            14,
            8,
        );
        let path = format!("/P38g/CockpitInterior/EngineQuadrant/{channel}");
        // Hands rub the paint off the lever just under the knob.
        let mut wear = MeshBuilder::new();
        wear.tube(tip - Vec3::new(0.0, 0.0, 0.025), tip, 0.0048, 0.0048, 8);
        prims.push(control(&path, ENAMEL_BLACK, arm, &joint));
        prims.push(control(&format!("{path}/Knob"), material, knob, &joint));
        prims.push(control(&format!("{path}/Wear"), WORN_STEEL, wear, &joint));
    }
    prims
}

fn tab_wheel() -> Vec<P38MeshPrim> {
    let joint = [turn(
        "trim_elevator",
        TAB_WHEEL_CENTRE,
        Vec3::X,
        TAB_WHEEL_PER_UNIT,
        0.0,
    )];
    let at =
        |angle: f32, r: f32| TAB_WHEEL_CENTRE + Vec3::new(0.0, angle.sin() * r, angle.cos() * r);
    let mut rim = MeshBuilder::new();
    let ring: Vec<Vec3> = (0..=24)
        .map(|i| at(i as f32 / 24.0 * TAU, TAB_WHEEL_RADIUS))
        .collect();
    sweep(&mut rim, &ring, 0.0065, Vec3::X, 10);
    for spoke in 0..5 {
        let angle = spoke as f32 / 5.0 * TAU;
        rim.tube(
            at(angle, 0.015),
            at(angle, TAB_WHEEL_RADIUS),
            0.004,
            0.004,
            6,
        );
    }
    let mut hub = MeshBuilder::new();
    hub.tube(
        TAB_WHEEL_CENTRE - Vec3::new(0.012, 0.0, 0.0),
        TAB_WHEEL_CENTRE + Vec3::new(0.008, 0.0, 0.0),
        0.017,
        0.017,
        12,
    );
    vec![
        control(
            "/P38g/CockpitInterior/ElevatorTabWheel",
            ENAMEL_BLACK,
            rim,
            &joint,
        ),
        control(
            "/P38g/CockpitInterior/ElevatorTabWheel/Hub",
            CADMIUM,
            hub,
            &joint,
        ),
    ]
}

fn gear_lever() -> Vec<P38MeshPrim> {
    let joint = [turn("gear_lever", GEAR_PIVOT, Vec3::X, GEAR_SWING, 1.0)];
    // Modelled down: forward and low.
    let tip = GEAR_PIVOT + Vec3::new(0.0, 0.075, 0.07);
    let mut arm = MeshBuilder::new();
    arm.tube(GEAR_PIVOT, tip, 0.006, 0.006, 8);
    arm.cuboid(
        GEAR_PIVOT - Vec3::new(0.012, 0.02, 0.012),
        GEAR_PIVOT + Vec3::new(0.012, 0.02, 0.012),
    );
    let mut knob = MeshBuilder::new();
    knob.ellipsoid(tip, Vec3::new(0.022, 0.016, 0.016), 14, 8);
    vec![
        control("/P38g/CockpitInterior/GearLever", CADMIUM, arm, &joint),
        control(
            "/P38g/CockpitInterior/GearLever/Knob",
            GRIP_BLACK,
            knob,
            &joint,
        ),
    ]
}
