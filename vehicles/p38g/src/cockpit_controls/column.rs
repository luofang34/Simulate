//! The control column standing on the floor ahead of the seat, right of the
//! pilot's right knee, its arm reaching down and inboard to the hub, and the
//! ring wheel the arm carries in front of the pilot, open only where the arm
//! joins it. Lockheed's negative Z-4972 is the primary evidence for every
//! shape here
//! (<https://commons.wikimedia.org/wiki/File:Lockheed_P-38G_cockpit_looking_in_from_left_wing_061019-F-1234P-004.jpg>):
//! a polished column with a large white canvas boot at the floor, a black
//! three-spoke ring, a polished hub boss, and a large black cylindrical housing
//! beside the hub. Sizes are scaled from that photograph against the 3-1/8 in
//! instrument cases, about 15 % uncertain.

use std::f32::consts::PI;

use glam::Vec3;

use super::{CANVAS, GRIP_BLACK, GRIP_WORN, POLISHED, control, polyline, sweep, turn};
use crate::cockpit_reservation::p38g_cockpit_contract;
use crate::drawing::{frl_to_z, fs_to_y};
use crate::mesh::{Inside, MeshBuilder, P38ControlJoint, P38MeshPrim};

/// Centre of the wheel's hub: `3 in` aft of the column's station, within a
/// `29 in` grip reach of a shoulder `4 in` aft of the design eyepoint, at chest
/// height. Assumed until the hub is placed from the cockpit photographs.
pub(super) const HUB: Vec3 = Vec3::new(0.0, fs_to_y(119.0), frl_to_z(20.0));
/// Radius of the rim's centreline: the rim's outside diameter scales to 0.38 m
/// against the 3-1/8 in gauges.
const RIM_RADIUS: f32 = 0.176;
/// Where the arm meets the ring, clockwise from twelve as the pilot sees it,
/// and half the opening there, rad: the ring is whole but for the arm's gap
/// at its upper right.
/// Radius of the ring's section: a thick moulded rim, about 38 mm across against
/// the hub's 60 mm in Z-4986.
const RIM_TUBE_RADIUS: f32 = 0.019;
/// Where the hands hold the ring, clockwise from twelve, each with moulded
/// finger ridges on its inside.
const GRIPS: [(f32, f32); 2] = [(1.75, 2.2), (4.1, 4.55)];
const GRIP_RIDGES: usize = 4;
const ARM_ANGLE: f32 = 1.35;
const ARM_GAP: f32 = 0.24;
/// The other two spokes, clockwise from twelve: up-left and down.
const SPOKES: [f32; 2] = [5.32, 3.40];
/// Where the arm leaves the top of the column, from the hub: forward of the
/// ring a little above the hub, over the column's foot.
const COLUMN_TOP_FROM_HUB: Vec3 = Vec3::new(0.0, 0.03, 0.05);
/// Radius of the column tube: about two thirds as wide as a 3-1/8 in bezel
/// beside it in Z-4972, so about 50 mm across. The two are at different
/// ranges from a camera that is not solved, so this is good to a third.
const COLUMN_RADIUS: f32 = 0.025;
/// Column pitch per unit of the pitch command, rad, back positive. Assumed: no
/// source gives the travel; ±0.17 rad moves the hub about ±0.11 m fore and aft.
const PITCH_PER_UNIT: f32 = 0.17;
/// Wheel roll per unit of the roll command, rad, clockwise (right) positive as the
/// pilot sees it. Assumed: no source gives the travel.
const ROLL_PER_UNIT: f32 = 1.05;

/// The large black cylindrical housing beside the hub, its axis across the
/// aircraft: centre, half-length and radius. Read as the ultraviolet instrument
/// flood lamp the P-38H/J/L pilot's instructions (01-75FF-1) put on the column;
/// its function on the G is inferred.
const LAMP_CENTRE: Vec3 = Vec3::new(HUB.x - 0.095, HUB.y + 0.02, HUB.z + 0.01);
const LAMP_HALF_LENGTH: f32 = 0.075;
const LAMP_RADIUS: f32 = 0.045;

/// Where the ultraviolet flood lamp's lens is with the column centred, aircraft
/// frame, m: on top of the housing, aimed up at the panel.
#[must_use]
pub fn p38g_uv_flood_lamp_m() -> [f32; 3] {
    (LAMP_CENTRE + Vec3::new(0.0, 0.02, LAMP_RADIUS)).to_array()
}

fn pivot() -> Vec3 {
    Vec3::from_array(p38g_cockpit_contract().control_column_base_m)
}

/// The top of the column, straight above its foot's buttock line.
fn column_top() -> Vec3 {
    Vec3::new(pivot().x, HUB.y, HUB.z) + COLUMN_TOP_FROM_HUB
}

/// A point on the rim's centreline `angle` clockwise from twelve o'clock as the
/// pilot sees it, `inset` inside the centreline.
fn rim_point(angle: f32, inset: f32) -> Vec3 {
    let r = RIM_RADIUS - inset;
    HUB + Vec3::new(angle.sin() * r, 0.0, angle.cos() * r)
}

fn rim_arc(from: f32, to: f32) -> Vec<Vec3> {
    let steps = (((to - from) / 0.08).ceil() as usize).max(1);
    (0..=steps)
        .map(|i| rim_point(from + (to - from) * i as f32 / steps as f32, 0.0))
        .collect()
}

pub(super) fn prims() -> Vec<P38MeshPrim> {
    let pitch = turn("pitch", pivot(), Vec3::X, PITCH_PER_UNIT, 0.0);
    let roll = turn("roll", HUB, Vec3::Y, ROLL_PER_UNIT, 0.0);
    let mut prims = column_prims(pitch);
    prims.extend(wheel_prims([pitch, roll]));
    prims
}

/// The column, its arm to the hub, the lamp housing and the boot, pitching
/// about the pivot at the floor.
fn column_prims(pitch: P38ControlJoint) -> Vec<P38MeshPrim> {
    let path = "/P38g/CockpitInterior/ControlColumn";
    let p = pivot();
    let mut column = MeshBuilder::new();
    polyline(&mut column, &[p, column_top()], COLUMN_RADIUS, 16);
    column.ellipsoid(column_top(), Vec3::splat(COLUMN_RADIUS * 1.1), 14, 8);
    // The polished hub boss, about 60 mm across.
    column.tube(HUB - Vec3::Y * 0.015, HUB + Vec3::Y * 0.04, 0.03, 0.026, 18);
    column.ellipsoid(HUB - Vec3::Y * 0.015, Vec3::new(0.03, 0.01, 0.03), 18, 6);
    let mut housing = MeshBuilder::new();
    let along = Vec3::X * LAMP_HALF_LENGTH;
    housing.tube(
        LAMP_CENTRE - along,
        LAMP_CENTRE + along,
        LAMP_RADIUS,
        LAMP_RADIUS,
        20,
    );
    // The heavy box-section arm from the column's head inboard to the hub,
    // black like the housing, and the switch plate on its aft face: Z-4986
    // shows both from behind the wheel.
    let (head, hub) = (column_top(), HUB + Vec3::new(0.0, 0.02, 0.0));
    let reach = 0.5 * (head - hub);
    let across = reach.cross(Vec3::Y).normalize_or(Vec3::Z);
    housing.slab(hub + reach, reach, Vec3::Y * 0.022, across * 0.032);
    housing.slab(
        hub + reach * 1.1 - Vec3::Y * 0.024,
        reach.normalize_or(Vec3::X) * 0.045,
        Vec3::Y * 0.002,
        across * 0.022,
    );
    // The large canvas boot over the floor opening, 0.18 m across at the floor.
    let mut boot = MeshBuilder::new();
    boot_folds(&mut boot, p);
    vec![
        control(path, POLISHED, column, &[pitch]),
        control(
            &format!("{path}/LampHousing"),
            GRIP_BLACK,
            housing,
            &[pitch],
        ),
        control(&format!("{path}/Boot"), CANVAS, boot, &[pitch]),
    ]
}

/// The canvas boot over the floor opening: about 0.18 m across at the floor,
/// gathered to the column, slack canvas hanging in folds.
fn boot_folds(mesh: &mut MeshBuilder, pivot: Vec3) {
    const FOLDS: f32 = 7.0;
    const SIDES: usize = 42;
    let rings: Vec<Vec<Vec3>> = (0..=6)
        .map(|k| {
            let t = k as f32 / 6.0;
            let z = -0.02 + t * 0.24;
            // Wide and deeply folded at the floor, tight at the collar.
            let radius = 0.09 + (COLUMN_RADIUS + 0.006 - 0.09) * t.powf(0.7);
            let fold = 0.018 * (1.0 - t) * (1.0 + 0.4 * (t * 9.0).sin());
            (0..SIDES)
                .map(|i| {
                    let a = std::f32::consts::TAU * i as f32 / SIDES as f32;
                    let r = radius + fold * (FOLDS * a + 1.3 * t).sin();
                    pivot + Vec3::new(a.cos() * r, a.sin() * r, z)
                })
                .collect()
        })
        .collect();
    mesh.loft(&rings, true, Inside::Axis);
}

/// The ring, its two spokes and the switch housing on the right, rolling about
/// the hub on the pitching column.
fn wheel_prims(both: [P38ControlJoint; 2]) -> Vec<P38MeshPrim> {
    let path = "/P38g/CockpitInterior/ControlWheel";
    let mut rim = MeshBuilder::new();
    sweep(
        &mut rim,
        &rim_arc(ARM_ANGLE + ARM_GAP, ARM_ANGLE - ARM_GAP + 2.0 * PI),
        RIM_TUBE_RADIUS,
        Vec3::Y,
        14,
    );
    for end in [ARM_ANGLE + ARM_GAP, ARM_ANGLE - ARM_GAP] {
        rim.ellipsoid(rim_point(end, 0.0), Vec3::splat(RIM_TUBE_RADIUS), 12, 6);
    }
    // Finger grips moulded into the inside of the ring where each hand holds it.
    for (from, to) in GRIPS {
        for k in 0..GRIP_RIDGES {
            let angle = from + (to - from) * (k as f32 + 0.5) / GRIP_RIDGES as f32;
            rim.ellipsoid(
                rim_point(angle, RIM_TUBE_RADIUS * 0.55),
                Vec3::splat(RIM_TUBE_RADIUS * 0.75),
                10,
                6,
            );
        }
    }
    let mut spokes = MeshBuilder::new();
    for angle in SPOKES.into_iter().chain([ARM_ANGLE]) {
        spokes.tube(
            rim_point(angle, RIM_RADIUS - 0.028),
            rim_point(angle, 0.008),
            0.008,
            0.0065,
            8,
        );
    }
    // The firing switch housing on the wheel's right, beside the arm's gap
    // (01-75FF-1 figure 4 item 12; the G photographs show a rectangular box).
    let switch = rim_point(ARM_ANGLE + ARM_GAP + 0.25, 0.0);
    spokes.cuboid(
        switch - Vec3::new(0.016, 0.02, 0.02),
        switch + Vec3::new(0.016, 0.012, 0.02),
    );
    // Hands polish the black where they hold the ring, either side.
    let mut grip_wear = MeshBuilder::new();
    for (a, b) in GRIPS {
        sweep(
            &mut grip_wear,
            &rim_arc(a, b),
            RIM_TUBE_RADIUS + 0.0003,
            Vec3::Y,
            14,
        );
    }
    vec![
        control(path, GRIP_BLACK, rim, &both),
        control(&format!("{path}/Spokes"), GRIP_BLACK, spokes, &both),
        control(&format!("{path}/GripWear"), GRIP_WORN, grip_wear, &both),
    ]
}
