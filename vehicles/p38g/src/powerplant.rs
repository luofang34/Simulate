//! Allison V-1710-51/-55 `60°` V-12 powerplant, 24 individual exhaust ejector stacks,
//! recessed dorsal GE Type B-13 turbosuperchargers, wing leading-edge intercooler
//! louvers, chin oil-cooler scoops, and aft-boom glycol radiator fairings (`T.O. 01-75F-1`).

use std::f32::consts::{PI, TAU};

use glam::{DMat4, DVec3, Vec3};

use crate::mesh::{Inside, MeshBuilder, P38MeshPrim, P38PartRole, fixed, glowing, part};
use crate::stations::{
    BOOM_X, CHROME_OLEO, EXHAUST_RUST, GUNMETAL, NEUTRAL_GRAY, OLIVE_DRAB, PANEL_BLACK,
    TURBO_NOZZLE_Y, TURBO_STEEL, boom_point, wing_surface_point,
};

pub(crate) fn powerplant_prims() -> Vec<P38MeshPrim> {
    let mut prims = Vec::new();
    build_allison_exhaust_and_gearboxes(&mut prims);
    build_ge_b13_turbosuperchargers(&mut prims);
    build_intercooler_louvers(&mut prims);
    build_chin_and_radiator_ducts(&mut prims);
    build_cooling_flaps(&mut prims);
    build_exterior_lights(&mut prims);
    prims
}

fn build_allison_exhaust_and_gearboxes(prims: &mut Vec<P38MeshPrim>) {
    let mut shrouds = MeshBuilder::new();
    let mut stacks = MeshBuilder::new();
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

            // 6 individual exhaust ejector stubs per V-12 bank (12 per engine, 24 total)
            for cyl in 0..6_u32 {
                let y = 1.137 - cyl as f32 * 0.244;
                let base = boom_point(side, y, theta) + out * 0.004;
                let tip = base + Vec3::new(bank_sign * 0.044, -0.042, 0.030);
                stacks.tube(base, tip, 0.020, 0.016, 8);
            }

            // Recessed dorsal collector trunk routing hot exhaust aft to the GE B-13 nozzle box
            let p_aft = boom_point(side, -0.26, theta) - out * 0.012;
            let p_mid = boom_point(side, -0.56, bank_sign * 0.36) - Vec3::new(0.0, 0.0, 0.010);
            let p_trb = boom_point(side, -0.82, bank_sign * 0.18) - Vec3::new(0.0, 0.0, 0.008);
            stacks.tube(p_aft, p_mid, 0.030, 0.028, 10);
            stacks.tube(p_mid, p_trb, 0.028, 0.025, 10);
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
        "/P38g/Powerplant/ExhaustEjectorStacks24",
        P38PartRole::CabinExterior,
        EXHAUST_RUST,
        stacks,
    ));
}

fn build_ge_b13_turbosuperchargers(prims: &mut Vec<P38MeshPrim>) {
    let mut wells = MeshBuilder::new();
    let mut scrolls = MeshBuilder::new();
    let mut impellers = MeshBuilder::new();

    for side in [-1.0_f32, 1.0] {
        let bx = side * BOOM_X;
        let tz = boom_point(side, -1.12, 0.0).z;
        wells.tube(
            Vec3::new(bx, -1.12, tz - 0.06),
            Vec3::new(bx, -1.12, tz + 0.012),
            0.25,
            0.26,
            20,
        );
        wells.cuboid(
            Vec3::new(bx - 0.14, -0.78, tz - 0.03),
            Vec3::new(bx + 0.14, -0.52, tz + 0.025),
        );
        scrolls.tube(
            Vec3::new(bx, -1.12, tz - 0.03),
            Vec3::new(bx, -1.12, tz + 0.024),
            0.21,
            0.19,
            18,
        );
        scrolls.tube(
            Vec3::new(bx, -1.32, tz - 0.01),
            Vec3::new(bx, TURBO_NOZZLE_Y, tz + 0.035),
            0.055,
            0.048,
            12,
        );
        impellers.tube(
            Vec3::new(bx, -1.12, tz + 0.010),
            Vec3::new(bx, -1.12, tz + 0.034),
            0.145,
            0.130,
            18,
        );
        impellers.tube(
            Vec3::new(bx, -1.12, tz + 0.028),
            Vec3::new(bx, -1.12, tz + 0.048),
            0.045,
            0.032,
            12,
        );
        for vane in 0..18_u32 {
            let (s, c) = (TAU * vane as f32 / 18.0).sin_cos();
            let p_in = Vec3::new(bx + c * 0.045, -1.12 + s * 0.045, tz + 0.032);
            let p_out = Vec3::new(bx + c * 0.140, -1.12 + s * 0.140, tz + 0.024);
            impellers.tube(p_in, p_out, 0.009, 0.006, 6);
        }
    }

    prims.push(fixed(
        "/P38g/Powerplant/TurboWells",
        P38PartRole::CabinExterior,
        GUNMETAL,
        wells,
    ));
    prims.push(fixed(
        "/P38g/Powerplant/TurboNozzleScrolls",
        P38PartRole::CabinExterior,
        EXHAUST_RUST,
        scrolls,
    ));
    prims.push(fixed(
        "/P38g/Powerplant/TurboImpellerWheels",
        P38PartRole::CabinExterior,
        CHROME_OLEO,
        impellers,
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

fn build_chin_and_radiator_ducts(prims: &mut Vec<P38MeshPrim>) {
    let mut olive_ducts = MeshBuilder::new();
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

        for out_sign in [-1.0_f32, 1.0] {
            let rx = bx + out_sign * 0.42;
            olive_ducts.cuboid(
                Vec3::new(rx - 0.008, -2.95, -0.16),
                Vec3::new(rx + 0.008, -2.54, 0.26),
            );
            let cx0 = (rx + out_sign * 0.02).min(rx + out_sign * 0.16);
            let cx1 = (rx + out_sign * 0.02).max(rx + out_sign * 0.16);
            cores.cuboid(Vec3::new(cx0, -2.82, -0.12), Vec3::new(cx1, -2.58, 0.22));
        }
    }

    prims.push(fixed(
        "/P38g/Powerplant/IntercoolerAndRadiatorDucts",
        P38PartRole::WingStrut,
        OLIVE_DRAB,
        olive_ducts,
    ));
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

fn build_cooling_flaps(prims: &mut Vec<P38MeshPrim>) {
    for (label, side) in [("Left", -1.0_f32), ("Right", 1.0_f32)] {
        let bx = side * BOOM_X;
        for (sub, out_sign) in [("In", -1.0_f32), ("Out", 1.0_f32)] {
            let mut flap = MeshBuilder::new();
            flap.cuboid(Vec3::new(-0.012, -0.34, -0.20), Vec3::new(0.012, 0.0, 0.20));
            let rest =
                DMat4::from_translation(DVec3::new(f64::from(bx + out_sign * 0.56), -3.72, 0.06));
            let mut prim = part(
                &format!("/P38g/Cooling/CoolantFlap{label}{sub}"),
                P38PartRole::CabinExterior,
                OLIVE_DRAB,
                rest,
                flap,
            );
            prim.custom_hinge_axis = Some(DVec3::new(0.0, 0.0, f64::from(-out_sign)));
            prims.push(prim);
        }

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
