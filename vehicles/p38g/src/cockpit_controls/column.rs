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
const ARM_ANGLE: f32 = 1.35;
const ARM_GAP: f32 = 0.24;
/// The other two spokes, clockwise from twelve: up-left and down.
const SPOKES: [f32; 2] = [5.32, 3.40];
/// Where the arm leaves the top of the column, from the hub: forward of the
/// ring a little above the hub, over the column's foot.
const COLUMN_TOP_FROM_HUB: Vec3 = Vec3::new(0.0, 0.03, 0.05);
/// Radius of the column tube: 23 px across in Z-4972 at about 1.2 m from its
/// resected camera (`70°` field, 561 px focal length), so about 50 mm across.
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
const LAMP_CENTRE: Vec3 = Vec3::new(HUB.x - 0.085, HUB.y + 0.05, HUB.z + 0.06);
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
    // The arm runs down and inboard to the hub through the ring's gap.
    let arm_end = HUB + Vec3::new(0.0, 0.02, 0.0);
    // The column carries the hub on a short stub of its own tube, not a
    // separate arm.
    polyline(
        &mut column,
        &[column_top(), arm_end],
        COLUMN_RADIUS * 0.85,
        14,
    );
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
    housing.tube(LAMP_CENTRE + along, HUB, 0.012, 0.012, 8);
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
        0.0145,
        Vec3::Y,
        14,
    );
    for end in [ARM_ANGLE + ARM_GAP, ARM_ANGLE - ARM_GAP] {
        rim.ellipsoid(rim_point(end, 0.0), Vec3::splat(0.0145), 12, 6);
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
    for (a, b) in [(1.75_f32, 2.2_f32), (4.1, 4.55)] {
        sweep(&mut grip_wear, &rim_arc(a, b), 0.0148, Vec3::Y, 14);
    }
    vec![
        control(path, GRIP_BLACK, rim, &both),
        control(&format!("{path}/Spokes"), GRIP_BLACK, spokes, &both),
        control(&format!("{path}/GripWear"), GRIP_WORN, grip_wear, &both),
    ]
}
