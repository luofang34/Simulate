//! Tricycle landing gear (`T.O. 01-75F-1`): nose and main oleo struts, wheels and wells,
//! placed at the AN 01-75F-2 axle stations.

use std::f32::consts::TAU;

use glam::{DMat4, DVec3, Vec3};

use crate::drawing::{INCH_M, MAIN_AXLE_FS_IN, MAIN_TREAD_IN, NOSE_AXLE_FS_IN, NOSE_Y_M};
use crate::mesh::{Inside, MeshBuilder, P38MeshPrim, P38PartRole, fixed, part};
use crate::stations::{BOOM_X, CHROME_OLEO, NEUTRAL_GRAY, TIRE_RUBBER, WHEEL_WELL_ZINC};

/// Nose axle: centre of the dashed nose tyre on the AN 01-75F-2 side view.
const NOSE_AXLE_Y: f32 = NOSE_Y_M - NOSE_AXLE_FS_IN * INCH_M;
/// Main axle: centre of the dashed main tyre on the AN 01-75F-2 side view.
const MAIN_AXLE_Y: f32 = NOSE_Y_M - MAIN_AXLE_FS_IN * INCH_M;
/// Main tyre centre off the ship centreline: half the `198 in` tread.
const MAIN_HALF_TREAD_M: f32 = 0.5 * MAIN_TREAD_IN * INCH_M;

pub(crate) fn build_nose_gear(prims: &mut Vec<P38MeshPrim>) {
    let mut wells = MeshBuilder::new();
    wells.cuboid(
        Vec3::new(-0.20, NOSE_AXLE_Y - 0.79, -0.44),
        Vec3::new(0.20, NOSE_AXLE_Y - 0.03, -0.34),
    );
    for side in [-1.0_f32, 1.0] {
        let bx = side * BOOM_X;
        wells.cuboid(
            Vec3::new(bx - 0.24, MAIN_AXLE_Y - 1.03, -0.42),
            Vec3::new(bx + 0.24, MAIN_AXLE_Y + 0.13, -0.32),
        );
    }
    prims.push(fixed(
        "/P38g/Gear/WheelWells",
        P38PartRole::CabinExterior,
        WHEEL_WELL_ZINC,
        wells,
    ));

    let nose_rest = DMat4::from_translation(DVec3::new(0.0, f64::from(NOSE_AXLE_Y - 0.08), -0.36));
    let mut n_strut = MeshBuilder::new();
    n_strut.tube(Vec3::ZERO, Vec3::new(0.0, 0.06, -0.82), 0.044, 0.036, 12);
    n_strut.tube(
        Vec3::new(0.0, 0.06, -0.82),
        Vec3::new(0.06, 0.08, -1.36),
        0.032,
        0.028,
        12,
    );
    prims.push(part(
        "/P38g/Gear/NoseStrut",
        P38PartRole::GearStruts,
        CHROME_OLEO,
        nose_rest,
        n_strut,
    ));

    let mut n_door = MeshBuilder::new();
    n_door.cuboid(Vec3::new(0.20, -0.68, -0.26), Vec3::new(0.23, 0.08, -0.02));
    prims.push(part(
        "/P38g/Gear/NoseDoor",
        P38PartRole::GearStruts,
        NEUTRAL_GRAY,
        nose_rest,
        n_door,
    ));
    append_wheel_and_hub(
        prims,
        "/P38g/Gear/NoseWheel",
        P38PartRole::NoseWheel,
        DVec3::new(0.0, f64::from(NOSE_AXLE_Y), -1.72),
        0.34,
        0.10,
    );
}

pub(crate) fn build_main_gear(prims: &mut Vec<P38MeshPrim>) {
    for (s_name, w_name, w_role, side) in [
        (
            "MainStrutLeft",
            "WheelLeft",
            P38PartRole::WheelLeft,
            -1.0_f32,
        ),
        (
            "MainStrutRight",
            "WheelRight",
            P38PartRole::WheelRight,
            1.0_f32,
        ),
    ] {
        let bx = side * MAIN_HALF_TREAD_M;
        let m_rest = DMat4::from_translation(DVec3::new(
            f64::from(bx),
            f64::from(MAIN_AXLE_Y + 0.04),
            -0.36,
        ));
        let mut m_strut = MeshBuilder::new();
        m_strut.tube(Vec3::ZERO, Vec3::new(0.0, -0.04, -1.24), 0.056, 0.044, 14);
        m_strut.tube(
            Vec3::new(0.0, -0.58, -0.06),
            Vec3::new(0.0, -0.02, -0.78),
            0.030,
            0.024,
            10,
        );
        m_strut.cuboid(
            Vec3::new(side * 0.36, -0.92, -0.28),
            Vec3::new(side * 0.39, 0.12, -0.02),
        );
        prims.push(part(
            &format!("/P38g/Gear/{s_name}"),
            P38PartRole::GearStruts,
            CHROME_OLEO,
            m_rest,
            m_strut,
        ));
        append_wheel_and_hub(
            prims,
            &format!("/P38g/Gear/{w_name}"),
            w_role,
            DVec3::new(f64::from(bx), f64::from(MAIN_AXLE_Y), -1.60),
            0.46,
            0.15,
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
        (0..24)
            .map(|i| {
                let (s, c) = (TAU * i as f32 / 24.0).sin_cos();
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
