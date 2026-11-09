//! Allison V-1710-51/-55 `60°` V-12 powerplant, exhaust collectors to the dorsal GE
//! Type B-13 turbosuperchargers, wing leading-edge intercooler
//! louvers, chin oil-cooler scoops, and aft-boom glycol radiator fairings (`T.O. 01-75F-1`).

use std::f32::consts::{PI, TAU};

use glam::{DMat4, DVec3, Vec3};

use crate::drawing::{INCH_M, fs_to_y};
use crate::mesh::{Inside, MeshBuilder, P38MeshPrim, P38PartRole, fixed, glowing, part};
use crate::radiators::radiator_prims;
use crate::stations::{
    BOOM_X, EXHAUST_RUST, GUNMETAL, NEUTRAL_GRAY, OLIVE_DRAB, PANEL_BLACK, TURBO_STEEL, boom_point,
    wing_surface_point,
};

pub(crate) fn powerplant_prims() -> Vec<P38MeshPrim> {
    let mut prims = Vec::new();
    build_allison_exhaust_and_gearboxes(&mut prims);
    build_ge_b13_turbosuperchargers(&mut prims);
    build_intercooler_louvers(&mut prims);
    build_chin_oil_coolers(&mut prims);
    build_oil_shutters(&mut prims);
    prims.extend(radiator_prims());
    build_exterior_lights(&mut prims);
    prims
}

fn build_allison_exhaust_and_gearboxes(prims: &mut Vec<P38MeshPrim>) {
    let mut shrouds = MeshBuilder::new();
    let mut collectors = MeshBuilder::new();
    let mut gearboxes = MeshBuilder::new();

    for side in [-1.0_f32, 1.0] {
        let bx = side * BOOM_X;
        // Allison V-1710 external 2:1 spur reduction gear casing collar & breather dome
        gearboxes.tube(
            Vec3::new(bx, 1.747, 0.04),
            Vec3::new(bx, 1.559, 0.03),
            0.29,
            0.34,
            18,
        );
        gearboxes.ellipsoid(
            Vec3::new(bx, 1.508, 0.36),
            Vec3::new(0.11, 0.18, 0.07),
            12,
            6,
        );

        // Inboard & outboard 60-deg V-12 cylinder-bank stainless exhaust manifold shrouds
        for bank_sign in [-1.0_f32, 1.0] {
            let theta = bank_sign * 0.66;
            let out = Vec3::new(bank_sign * 0.74, 0.0, 0.67).normalize_or(Vec3::Z);
            let shroud_ys = [1.264_f32, 0.756, 0.248, -0.260];
            for pair in shroud_ys.windows(2) {
                if let &[y0, y1] = pair {
                    let p0 = boom_point(side, y0, theta) - out * 0.008;
                    let p1 = boom_point(side, y1, theta) - out * 0.008;
                    shrouds.tube(p0, p1, 0.036, 0.038, 10);
                }
            }

            // No ejector stubs on the turbocharged G: each bank's manifold collects the
            // exhaust and a trunk carries it aft along the boom top to the turbo's nozzle box.
            let p_aft = boom_point(side, -0.26, theta) - out * 0.012;
            let p_mid = boom_point(side, -0.56, bank_sign * 0.36) - Vec3::new(0.0, 0.0, 0.010);
            let p_trb =
                boom_point(side, TURBO_BOX_FRONT_Y, bank_sign * 0.18) - Vec3::new(0.0, 0.0, 0.008);
            collectors.tube(p_aft, p_mid, 0.030, 0.028, 10);
            collectors.tube(p_mid, p_trb, 0.028, 0.025, 10);
        }
    }

    prims.push(fixed(
        "/P38g/Powerplant/ReductionGearCastings",
        P38PartRole::CabinExterior,
        GUNMETAL,
        gearboxes,
    ));
    prims.push(fixed(
        "/P38g/Powerplant/ExhaustManifoldShrouds",
        P38PartRole::CabinExterior,
        TURBO_STEEL,
        shrouds,
    ));
    prims.push(fixed(
        "/P38g/Powerplant/ExhaustCollectors",
        P38PartRole::CabinExterior,
        EXHAUST_RUST,
        collectors,
    ));
}

/// AN 01-75F-2 plan view: an oval housing on each boom top from `FS 182` to `FS 231`,
/// `24 in` wide, with the turbine bucket wheel (`9 in` radius) at `FS 205.6` and the
/// exhaust nozzle box ahead of it, which the side view shows rising `3 in` above the
/// boom at its front.
const TURBO_HOUSING_FS: (f32, f32) = (182.0, 231.0);
const TURBO_WHEEL_FS: f32 = 205.6;
const TURBO_BOX_FRONT_Y: f32 = 3.55 - 182.5 * 0.0254;

fn build_ge_b13_turbosuperchargers(prims: &mut Vec<P38MeshPrim>) {
    let mut housings = MeshBuilder::new();
    let mut boxes = MeshBuilder::new();
    let mut wheels = MeshBuilder::new();
    let (h0, h1) = TURBO_HOUSING_FS;
    let wy = fs_to_y(TURBO_WHEEL_FS);

    for side in [-1.0_f32, 1.0] {
        let bx = side * BOOM_X;
        let top = |y: f32| boom_point(side, y, 0.0).z;
        // Low cooling-shroud cap over the turbine, faired into the boom top.
        housings.ellipsoid(
            Vec3::new(bx, fs_to_y(0.5 * (h0 + h1)), top(wy) - 0.02),
            Vec3::new(12.0 * INCH_M, 0.5 * (h1 - h0) * INCH_M, 0.075),
            24,
            8,
        );
        // Nozzle box: a ramp from the collector trunks up to the wheel's rim.
        let box_rows: Vec<Vec<Vec3>> = [(182.5_f32, 0.080_f32), (190.0, 0.075), (197.0, 0.055)]
            .into_iter()
            .map(|(fs, rise)| {
                let y = fs_to_y(fs);
                [(-1.0_f32, 0.0_f32), (-1.0, rise), (1.0, rise), (1.0, 0.0)]
                    .into_iter()
                    .map(|(sx, dz)| Vec3::new(bx + sx * 6.0 * INCH_M, y, top(y) - 0.02 + dz))
                    .collect()
            })
            .collect();
        boxes.loft(&box_rows, true, Inside::Axis);
        // Exposed bucket wheel and its hub in the cap.
        let tz = top(wy) + 0.05;
        wheels.tube(
            Vec3::new(bx, wy, tz - 0.01),
            Vec3::new(bx, wy, tz + 0.012),
            9.0 * INCH_M,
            9.0 * INCH_M,
            28,
        );
        wheels.tube(
            Vec3::new(bx, wy, tz + 0.010),
            Vec3::new(bx, wy, tz + 0.030),
            0.045,
            0.032,
            12,
        );
        for vane in 0..24_u32 {
            let (s, c) = (TAU * vane as f32 / 24.0).sin_cos();
            let p_in = Vec3::new(bx + c * 0.05, wy + s * 0.05, tz + 0.014);
            let p_out = Vec3::new(bx + c * 0.21, wy + s * 0.21, tz + 0.014);
            wheels.tube(p_in, p_out, 0.008, 0.006, 6);
        }
    }

    prims.push(fixed(
        "/P38g/Powerplant/TurboHousings",
        P38PartRole::CabinExterior,
        OLIVE_DRAB,
        housings,
    ));
    prims.push(fixed(
        "/P38g/Powerplant/TurboNozzleBoxes",
        P38PartRole::CabinExterior,
        EXHAUST_RUST,
        boxes,
    ));
    prims.push(fixed(
        "/P38g/Powerplant/TurboBucketWheels",
        P38PartRole::CabinExterior,
        TURBO_STEEL,
        wheels,
    ));
}

fn build_intercooler_louvers(prims: &mut Vec<P38MeshPrim>) {
    let mut olive_ducts = MeshBuilder::new();
    let mut vanes = MeshBuilder::new();
    let mut cores = MeshBuilder::new();

    for side in [-1.0_f32, 1.0] {
        for (x_in, x_out) in [(1.14_f32, 1.88_f32), (2.84_f32, 3.56_f32)] {
            let x0 = (side * x_in).min(side * x_out);
            let x1 = (side * x_in).max(side * x_out);
            let xm = 0.5 * (x0 + x1);
            let p_up = wing_surface_point(xm, 0.03, true);
            let p_lo = wing_surface_point(xm, 0.05, false);
            olive_ducts.cuboid(
                Vec3::new(x0, p_up.y - 0.06, p_lo.z + 0.01),
                Vec3::new(x1, p_up.y + 0.03, p_up.z - 0.01),
            );
            cores.cuboid(
                Vec3::new(x0 + 0.03, p_up.y - 0.03, p_lo.z + 0.025),
                Vec3::new(x1 - 0.03, p_up.y + 0.032, p_up.z - 0.025),
            );
            for vane in 1..4_u32 {
                let xv = x0 + (x1 - x0) * (vane as f32 / 4.0);
                vanes.cuboid(
                    Vec3::new(xv - 0.006, p_up.y - 0.02, p_lo.z + 0.02),
                    Vec3::new(xv + 0.006, p_up.y + 0.034, p_up.z - 0.02),
                );
            }
        }
    }

    prims.push(fixed(
        "/P38g/Powerplant/IntercoolerLouvers",
        P38PartRole::WingStrut,
        OLIVE_DRAB,
        olive_ducts,
    ));
    prims.push(fixed(
        "/P38g/Powerplant/IntercoolerVanes",
        P38PartRole::WingStrut,
        NEUTRAL_GRAY,
        vanes,
    ));
    prims.push(fixed(
        "/P38g/Powerplant/IntercoolerCores",
        P38PartRole::WingStrut,
        PANEL_BLACK,
        cores,
    ));
}

fn build_chin_oil_coolers(prims: &mut Vec<P38MeshPrim>) {
    let mut gray_scoops = MeshBuilder::new();
    let mut cores = MeshBuilder::new();

    for side in [-1.0_f32, 1.0] {
        let bx = side * BOOM_X;
        let chin_sections: Vec<Vec<Vec3>> = [
            (1.670_f32, 0.17_f32, -0.42_f32, -0.60_f32),
            (1.416, 0.21, -0.44, -0.66),
            (1.010, 0.22, -0.46, -0.66),
            (0.629, 0.18, -0.46, -0.58),
        ]
        .into_iter()
        .map(|(y, hw, z_top, z_bot)| {
            (0..=10)
                .map(|i| {
                    let (s, c) = (-0.5 * PI + (i as f32 / 10.0) * PI).sin_cos();
                    Vec3::new(
                        bx + s * hw,
                        y,
                        z_top - c.max(0.0).powf(0.8) * (z_top - z_bot),
                    )
                })
                .collect()
        })
        .collect();
        gray_scoops.loft_open(&chin_sections, Inside::Point(Vec3::new(bx, 1.15, -0.35)));
        cores.cuboid(
            Vec3::new(bx - 0.14, 1.467, -0.59),
            Vec3::new(bx + 0.14, 1.594, -0.45),
        );
    }

    prims.push(fixed(
        "/P38g/Powerplant/ChinOilCoolerScoops",
        P38PartRole::CabinExterior,
        NEUTRAL_GRAY,
        gray_scoops,
    ));
    prims.push(fixed(
        "/P38g/Powerplant/CoolerMatrixCores",
        P38PartRole::CabinExterior,
        PANEL_BLACK,
        cores,
    ));
}

fn build_oil_shutters(prims: &mut Vec<P38MeshPrim>) {
    for (label, side) in [("Left", -1.0_f32), ("Right", 1.0_f32)] {
        let bx = side * BOOM_X;
        let mut oil = MeshBuilder::new();
        oil.cuboid(Vec3::new(-0.18, -0.22, -0.014), Vec3::new(0.18, 0.0, 0.014));
        let rest = DMat4::from_translation(DVec3::new(f64::from(bx), 0.578, -0.62));
        let mut oil_prim = part(
            &format!("/P38g/Cooling/OilShutter{label}"),
            P38PartRole::CabinExterior,
            NEUTRAL_GRAY,
            rest,
            oil,
        );
        oil_prim.custom_hinge_axis = Some(DVec3::X);
        prims.push(oil_prim);
    }
}

fn build_exterior_lights(prims: &mut Vec<P38MeshPrim>) {
    let mut l_light = MeshBuilder::new();
    l_light.ellipsoid(
        wing_surface_point(-4.15, 0.06, false) + Vec3::new(0.0, 0.0, -0.02),
        Vec3::new(0.09, 0.05, 0.07),
        12,
        8,
    );
    prims.push(fixed(
        "/P38g/Lights/LandingLight",
        P38PartRole::LandingLight,
        glowing([0.98, 0.95, 0.84], [2.4, 2.2, 1.8]),
        l_light,
    ));

    let mut beacon = MeshBuilder::new();
    for side in [-1.0_f32, 1.0] {
        beacon.ellipsoid(
            Vec3::new(side * BOOM_X, -7.24, 1.865),
            Vec3::new(0.028, 0.055, 0.028),
            10,
            6,
        );
    }
    prims.push(fixed(
        "/P38g/Lights/FinBeacons",
        P38PartRole::BeaconLight,
        glowing([0.95, 0.15, 0.10], [2.2, 0.16, 0.08]),
        beacon,
    ));
}
