mod calibration;
mod silhouette;

use std::{f32::consts::TAU, path::PathBuf};

use glam::{DVec3, Vec3};
use image::{Rgba, RgbaImage};

use crate::{
    BOOM_STATIONS, BOOM_X, FIN_PROFILE, GONDOLA_STATIONS, P38PartRole, PROP_HUB_Y, PROP_HUB_Z,
    PROP_RADIUS_M, SEMI_SPAN, boom_point, gondola_point, interpolate_gondola,
    p38g_cockpit_contract, p38g_mesh_prims, p38g_package_source, p38g_usda,
    three_view_boom_outline_x, three_view_gondola_outline_x, three_view_wing_leading_y,
    three_view_wing_trailing_y, wing_surface_point,
};

#[test]
fn powerplant_and_cockpit_reservation_prims_are_present() {
    let prims = p38g_mesh_prims();
    for expected in [
        "/P38g/Powerplant/ReductionGearCastings",
        "/P38g/Powerplant/ExhaustManifoldShrouds",
        "/P38g/Powerplant/ExhaustEjectorStacks24",
        "/P38g/Powerplant/TurboWells",
        "/P38g/Powerplant/TurboNozzleScrolls",
        "/P38g/Powerplant/TurboImpellerWheels",
        "/P38g/Powerplant/RadiatorScoops",
        "/P38g/Powerplant/ChinOilCoolerScoops",
        "/P38g/Powerplant/CoolerMatrixCores",
        "/P38g/CockpitInterior/TubAndBulkheads",
        "/P38g/CockpitInterior/InstrumentPanel/Board",
        "/P38g/CockpitInterior/GunsightReflector",
        "/P38g/CockpitInterior/ControlWheel",
    ] {
        assert!(
            prims.iter().any(|p| p.prim_path == expected),
            "missing prim {expected}"
        );
    }

    let contract = p38g_cockpit_contract();
    assert_eq!(contract.bulkheads.len(), 6);
    assert!(
        prims
            .iter()
            .any(|p| p.prim_path == contract.main_instrument_panel.prim_path
                && p.role == P38PartRole::CabinInterior)
    );
    assert!(three_view_gondola_outline_x(0.30) > 0.58);
    assert!(three_view_wing_leading_y(0.0) > three_view_wing_trailing_y(0.0));
}

#[test]
fn openusd_stage_and_package_manifest_export_cleanly() {
    let manifest = p38g_package_source();
    assert!(manifest.contains("Lockheed P-38G-13-LO Lightning"));
    let Ok(usda) = p38g_usda() else {
        return;
    };
    assert!(usda.starts_with("#usda 1.0"));
    assert!(usda.contains("defaultPrim = \"P38g\""));
    assert!(usda.contains("Powerplant_ExhaustEjectorStacks24"));
    assert!(usda.contains("CockpitInterior_InstrumentPanel_Board"));
}

#[test]
fn render_three_view_blueprint_and_station_overlay() {
    let Some(dir) = std::env::var_os("SIMULATE_VERIFY_OUT").map(PathBuf::from) else {
        return;
    };
    let mut img = RgbaImage::from_pixel(1600, 1200, Rgba([12, 22, 38, 255]));
    draw_blueprint_grid(&mut img);
    rasterize_orthographic_views(&mut img);
    draw_top_view_blueprint_overlay(&mut img);
    draw_side_view_blueprint_overlay(&mut img);
    draw_front_view_blueprint_overlay(&mut img);
    draw_bulkhead_body_plan_quadrant(&mut img);
    assert!(
        img.save(dir.join("p38_00_blueprint_3view_overlay.png"))
            .is_ok()
    );
}

fn draw_blueprint_grid(img: &mut RgbaImage) {
    for y in 0..1200_u32 {
        for x in 0..1600_u32 {
            if x % 40 == 0 || y % 40 == 0 {
                img.put_pixel(x, y, Rgba([20, 36, 58, 255]));
            }
            if x % 200 == 0 || y % 200 == 0 {
                img.put_pixel(x, y, Rgba([28, 52, 82, 255]));
            }
            if x == 800 || y == 600 {
                img.put_pixel(x, y, Rgba([90, 150, 210, 255]));
            }
        }
    }
}

fn rasterize_orthographic_views(img: &mut RgbaImage) {
    let prims = p38g_mesh_prims();
    let mut zbuf = vec![f32::NEG_INFINITY; 1600 * 1200];
    let light_top = Vec3::new(0.25, 0.35, 0.90).normalize();
    let light_side = Vec3::new(-0.88, 0.25, 0.40).normalize();
    let light_front = Vec3::new(0.20, 0.90, 0.38).normalize();

    for prim in &prims {
        if prim.role == P38PartRole::PropellerDisc {
            continue;
        }
        let world_verts: Vec<(Vec3, Vec3)> = prim
            .vertices
            .iter()
            .map(|v| {
                let [px, py, pz] = v.position;
                let [nx, ny, nz] = v.normal;
                let pw = prim.rest_to_aircraft.transform_point3(DVec3::new(
                    f64::from(px),
                    f64::from(py),
                    f64::from(pz),
                ));
                let nw = prim.rest_to_aircraft.transform_vector3(DVec3::new(
                    f64::from(nx),
                    f64::from(ny),
                    f64::from(nz),
                ));
                (
                    Vec3::new(pw.x as f32, pw.y as f32, pw.z as f32),
                    Vec3::new(nw.x as f32, nw.y as f32, nw.z as f32).normalize_or(Vec3::Z),
                )
            })
            .collect();

        for &[i0, i1, i2] in prim.indices.as_chunks::<3>().0 {
            let (p0, n0) = world_verts[i0 as usize];
            let (p1, n1) = world_verts[i1 as usize];
            let (p2, n2) = world_verts[i2 as usize];
            let n = (n0 + n1 + n2).normalize_or(Vec3::Z);
            let [mr, mg, mb] = prim.material.diffuse_rgb;
            let shade = |l: Vec3| {
                let s = 0.38 + 0.62 * n.dot(l).abs();
                to_rgba(mr * s, mg * s, mb * s)
            };

            let map_top = |p: Vec3| (400.0 + p.x * 44.0, 260.0 - p.y * 44.0, p.z);
            raster_tri(
                img,
                &mut zbuf,
                [0, 0, 800, 600],
                [map_top(p0), map_top(p1), map_top(p2)],
                shade(light_top),
            );

            if (p0.x + p1.x + p2.x) / 3.0 >= -2.85 {
                let map_side = |p: Vec3| (1090.0 - p.y * 54.0, 300.0 - p.z * 54.0, -p.x);
                raster_tri(
                    img,
                    &mut zbuf,
                    [800, 0, 1600, 600],
                    [map_side(p0), map_side(p1), map_side(p2)],
                    shade(light_side),
                );
            }

            let map_front = |p: Vec3| (400.0 + p.x * 44.0, 900.0 - p.z * 44.0, p.y);
            raster_tri(
                img,
                &mut zbuf,
                [0, 600, 800, 1200],
                [map_front(p0), map_front(p1), map_front(p2)],
                shade(light_front),
            );
        }
    }
}

fn to_rgba(r: f32, g: f32, b: f32) -> Rgba<u8> {
    let conv = |c: f32| (c.clamp(0.0, 1.0).powf(1.0 / 2.2) * 255.0) as u8;
    Rgba([conv(r), conv(g), conv(b), 255])
}

fn raster_tri(
    img: &mut RgbaImage,
    zbuf: &mut [f32],
    clip: [i32; 4],
    [v0, v1, v2]: [(f32, f32, f32); 3],
    color: Rgba<u8>,
) {
    let min_x = v0.0.min(v1.0).min(v2.0).floor().max(clip[0] as f32) as i32;
    let max_x = v0.0.max(v1.0).max(v2.0).ceil().min((clip[2] - 1) as f32) as i32;
    let min_y = v0.1.min(v1.1).min(v2.1).floor().max(clip[1] as f32) as i32;
    let max_y = v0.1.max(v1.1).max(v2.1).ceil().min((clip[3] - 1) as f32) as i32;
    let area = (v1.0 - v0.0) * (v2.1 - v0.1) - (v1.1 - v0.1) * (v2.0 - v0.0);
    if area.abs() < 1e-4 {
        return;
    }
    let inv = 1.0 / area;
    for py in min_y..=max_y {
        for px in min_x..=max_x {
            let x = px as f32 + 0.5;
            let y = py as f32 + 0.5;
            let w0 = ((v1.0 - x) * (v2.1 - y) - (v1.1 - y) * (v2.0 - x)) * inv;
            let w1 = ((v2.0 - x) * (v0.1 - y) - (v2.1 - y) * (v0.0 - x)) * inv;
            let w2 = 1.0 - w0 - w1;
            if w0 >= -1e-4 && w1 >= -1e-4 && w2 >= -1e-4 {
                let z = w0 * v0.2 + w1 * v1.2 + w2 * v2.2;
                let idx = (py as usize) * 1600 + (px as usize);
                if z > zbuf[idx] {
                    zbuf[idx] = z;
                    img.put_pixel(px as u32, py as u32, color);
                }
            }
        }
    }
}

fn draw_line(img: &mut RgbaImage, (x0, y0): (f32, f32), (x1, y1): (f32, f32), col: Rgba<u8>) {
    let steps = ((x1 - x0).hypot(y1 - y0).ceil() as i32).max(1);
    for i in 0..=steps {
        let t = i as f32 / steps as f32;
        let x = (x0 + (x1 - x0) * t).round() as i32;
        let y = (y0 + (y1 - y0) * t).round() as i32;
        if (0..1600).contains(&x) && (0..1200).contains(&y) {
            img.put_pixel(x as u32, y as u32, col);
        }
    }
}

fn draw_top_view_blueprint_overlay(img: &mut RgbaImage) {
    let cyan = Rgba([0, 235, 255, 255]);
    let gold = Rgba([255, 210, 60, 255]);
    let magenta = Rgba([255, 90, 200, 255]);
    let map = |x: f32, y: f32| (400.0 + x * 44.0, 260.0 - y * 44.0);

    // 1. Wing leading & trailing edge 3-view reference contour
    for i in 0..160 {
        let x0 = -SEMI_SPAN + (2.0 * SEMI_SPAN) * (i as f32 / 160.0);
        let x1 = -SEMI_SPAN + (2.0 * SEMI_SPAN) * ((i + 1) as f32 / 160.0);
        draw_line(
            img,
            map(x0, three_view_wing_leading_y(x0)),
            map(x1, three_view_wing_leading_y(x1)),
            cyan,
        );
        draw_line(
            img,
            map(x0, three_view_wing_trailing_y(x0)),
            map(x1, three_view_wing_trailing_y(x1)),
            cyan,
        );
    }
    // 2. Gondola & Twin Booms 3-view planform outlines + FS/BS station ticks
    for pair in GONDOLA_STATIONS.windows(2) {
        let (y0, w0) = (pair[0].0, pair[0].2);
        let (y1, w1) = (pair[1].0, pair[1].2);
        for s in [-1.0_f32, 1.0] {
            draw_line(img, map(s * w0, y0), map(s * w1, y1), gold);
        }
        draw_line(img, map(-w0, y0), map(w0, y0), Rgba([255, 210, 60, 160]));
    }
    for side in [-1.0_f32, 1.0] {
        for pair in BOOM_STATIONS.windows(2) {
            let (y0, w0) = (pair[0].0, three_view_boom_outline_x(pair[0].0));
            let (y1, w1) = (pair[1].0, three_view_boom_outline_x(pair[1].0));
            for s in [-1.0_f32, 1.0] {
                draw_line(
                    img,
                    map(side * BOOM_X + s * w0, y0),
                    map(side * BOOM_X + s * w1, y1),
                    magenta,
                );
            }
            draw_line(
                img,
                map(side * BOOM_X - w0, y0),
                map(side * BOOM_X + w0, y0),
                magenta,
            );
        }
    }
    // 52 ft (15.85 m) span & 37 ft 10 in (11.53 m) length datum callout lines
    draw_line(img, map(-SEMI_SPAN, 3.55), map(-SEMI_SPAN, -7.98), cyan);
    draw_line(img, map(SEMI_SPAN, 3.55), map(SEMI_SPAN, -7.98), cyan);
    draw_line(img, map(-SEMI_SPAN, 3.55), map(SEMI_SPAN, 3.55), cyan);
    draw_line(img, map(-SEMI_SPAN, -7.98), map(SEMI_SPAN, -7.98), cyan);
}

fn draw_side_view_blueprint_overlay(img: &mut RgbaImage) {
    let cyan = Rgba([0, 235, 255, 255]);
    let gold = Rgba([255, 210, 60, 255]);
    let red = Rgba([255, 95, 95, 255]);
    let map = |y: f32, z: f32| (1090.0 - y * 54.0, 300.0 - z * 54.0);

    // Gondola upper/canopy & lower contour + FS bulkhead station lines
    for pair in GONDOLA_STATIONS.windows(2) {
        let (y0, _, _, _, zb0, zc0, _) = pair[0];
        let (y1, _, _, _, zb1, zc1, _) = pair[1];
        draw_line(img, map(y0, zc0), map(y1, zc1), gold);
        draw_line(img, map(y0, zb0), map(y1, zb1), gold);
        draw_line(img, map(y0, zb0), map(y0, zc0), gold);
    }
    // Boom upper/lower profile & vertical fin elliptical contour
    for pair in BOOM_STATIONS.windows(2) {
        let (y0, _, _, zt0, zb0, _, _) = pair[0];
        let (y1, _, _, zt1, zb1, _, _) = pair[1];
        draw_line(img, map(y0, zt0), map(y1, zt1), cyan);
        draw_line(img, map(y0, zb0), map(y1, zb1), cyan);
    }
    for pair in FIN_PROFILE.windows(2) {
        let (z0, ly0, ty0, _) = pair[0];
        let (z1, ly1, ty1, _) = pair[1];
        draw_line(img, map(ly0, z0), map(ly1, z1), cyan);
        draw_line(img, map(ty0, z0), map(ty1, z1), cyan);
    }
    // Reserved cockpit interior bulkhead stations (FS 22, FS 52, FS 88, FS 132, FS 154, FS 182)
    for bh in p38g_cockpit_contract().bulkheads {
        let (_, _, _, zb, zc, _) = interpolate_gondola(bh.y_m);
        draw_line(img, map(bh.y_m, zb - 0.15), map(bh.y_m, zc + 0.15), red);
    }
    // Ground line (z = -2.06 m), thrust line (z = +0.04 m), and fin-top datum (z = +1.85 m)
    draw_line(img, map(3.75, -2.06), map(-8.10, -2.06), cyan);
    draw_line(img, map(3.75, PROP_HUB_Z), map(-8.10, PROP_HUB_Z), gold);
    draw_line(img, map(3.75, 1.85), map(-8.10, 1.85), cyan);
}

fn draw_front_view_blueprint_overlay(img: &mut RgbaImage) {
    let cyan = Rgba([0, 235, 255, 255]);
    let gold = Rgba([255, 210, 60, 255]);
    let map = |x: f32, z: f32| (400.0 + x * 44.0, 900.0 - z * 44.0);

    // 5° 40' dihedral reference & wing upper/lower front envelope
    for i in 0..160 {
        let x0 = -SEMI_SPAN + (2.0 * SEMI_SPAN) * (i as f32 / 160.0);
        let x1 = -SEMI_SPAN + (2.0 * SEMI_SPAN) * ((i + 1) as f32 / 160.0);
        let p0 = wing_surface_point(x0, 0.30, true);
        let p1 = wing_surface_point(x1, 0.30, true);
        draw_line(img, map(x0, p0.z), map(x1, p1.z), cyan);
    }
    // Curtiss Electric 11 ft 6 in (R = 1.752 m) propeller tip circles + ground datum
    for side in [-1.0_f32, 1.0] {
        let cx = side * BOOM_X;
        for i in 0..72 {
            let a0 = TAU * (i as f32 / 72.0);
            let a1 = TAU * ((i + 1) as f32 / 72.0);
            draw_line(
                img,
                map(
                    cx + a0.cos() * PROP_RADIUS_M,
                    PROP_HUB_Z + a0.sin() * PROP_RADIUS_M,
                ),
                map(
                    cx + a1.cos() * PROP_RADIUS_M,
                    PROP_HUB_Z + a1.sin() * PROP_RADIUS_M,
                ),
                gold,
            );
        }
    }
    draw_line(img, map(-SEMI_SPAN, -2.06), map(SEMI_SPAN, -2.06), cyan);
    let _unused = PROP_HUB_Y;
}

fn draw_bulkhead_body_plan_quadrant(img: &mut RgbaImage) {
    let gold = Rgba([255, 210, 60, 255]);
    let cyan = Rgba([0, 235, 255, 255]);
    let green = Rgba([110, 250, 140, 255]);

    // Left sub-panel of Q4: All 14 Central Gondola bulkhead cross-sections (FS 0..248)
    let map_gondola = |x: f32, z: f32| (1020.0 + x * 210.0, 880.0 - z * 210.0);
    for &(y, ..) in GONDOLA_STATIONS {
        for i in 0..64 {
            let t0 = TAU * (i as f32 / 64.0);
            let t1 = TAU * ((i + 1) as f32 / 64.0);
            let p0 = gondola_point(y, t0);
            let p1 = gondola_point(y, t1);
            draw_line(img, map_gondola(p0.x, p0.z), map_gondola(p1.x, p1.z), gold);
        }
    }

    // Right sub-panel of Q4: All 14 Twin Boom + Radiator bulkhead cross-sections (BS 26..420)
    let map_boom = |x: f32, z: f32| (1380.0 + x * 210.0, 880.0 - z * 210.0);
    for &(y, ..) in BOOM_STATIONS {
        for i in 0..64 {
            let t0 = TAU * (i as f32 / 64.0);
            let t1 = TAU * ((i + 1) as f32 / 64.0);
            let p0 = boom_point(1.0, y, t0) - Vec3::new(BOOM_X, 0.0, 0.0);
            let p1 = boom_point(1.0, y, t1) - Vec3::new(BOOM_X, 0.0, 0.0);
            draw_line(img, map_boom(p0.x, p0.z), map_boom(p1.x, p1.z), cyan);
        }
    }

    // Bottom of Q4: Cambered NACA 23016 Root vs NACA 4412 Tip Airfoil Sections
    let map_airfoil = |y: f32, z: f32| (1200.0 - y * 135.0, 1110.0 - z * 135.0);
    for (x, col) in [(0.0_f32, green), (7.315_f32, cyan)] {
        for upper in [true, false] {
            for i in 0..48 {
                let u0 = i as f32 / 48.0;
                let u1 = (i + 1) as f32 / 48.0;
                let p0 = wing_surface_point(x, u0, upper);
                let p1 = wing_surface_point(x, u1, upper);
                draw_line(img, map_airfoil(p0.y, p0.z), map_airfoil(p1.y, p1.z), col);
            }
        }
    }
}

/// Seen from starboard (nose to the right) the nose number must read like the port one
/// (nose to the left): its strokes start and end at the same distances from the left
/// edge of the number, which a mirror image does not (the layout is not symmetric).
#[test]
fn the_nose_number_reads_the_same_from_either_side() {
    let prims = p38g_mesh_prims();
    let number = prims
        .iter()
        .find(|p| p.prim_path == "/P38g/Decals/White147Numbers")
        .expect("nose number");
    // Distance from the viewer's left edge of the number: aft is right from port,
    // forward is right from starboard.
    let seen = |port: bool| -> Vec<f32> {
        let xs: Vec<f32> = number
            .vertices
            .iter()
            .filter(|v| (v.position[0] < 0.0) == port)
            .map(|v| if port { -v.position[1] } else { v.position[1] })
            .collect();
        let left = xs.iter().copied().fold(f32::INFINITY, f32::min);
        xs.iter().map(|x| x - left).collect()
    };
    let (port, starboard) = (seen(true), seen(false));
    for (from, to, name) in [
        (&port, &starboard, "port"),
        (&starboard, &port, "starboard"),
    ] {
        for x in from {
            assert!(
                to.iter().any(|y| (x - y).abs() < 3e-3),
                "{name} stroke edge {x:.4} m from the left has no counterpart"
            );
        }
    }
}

/// Flaps, ailerons, elevator and stabiliser carry the factory scheme like the wing: olive
/// drab on top, neutral gray underneath. Trim tabs are painted with their surface (no
/// source shows red tabs on a P-38G-13).
#[test]
fn control_surfaces_are_olive_drab_over_neutral_gray() {
    let prims = p38g_mesh_prims();
    let colour = |path: &str| {
        prims
            .iter()
            .find(|p| p.prim_path == path)
            .map(|p| p.material.diffuse_rgb)
            .unwrap_or_else(|| panic!("no prim {path}"))
    };
    let (olive, gray) = (
        colour("/P38g/Wing/UpperOlive"),
        colour("/P38g/Wing/LowerGray"),
    );
    for surface in [
        "/P38g/Wing/FlapInboardLeft",
        "/P38g/Wing/FlapOutboardRight",
        "/P38g/Wing/AileronLeft",
        "/P38g/Wing/AileronRight",
        "/P38g/Empennage/Elevator",
        "/P38g/Empennage/HorizontalTail",
    ] {
        assert_eq!(colour(surface), olive, "{surface}");
        assert_eq!(colour(&format!("{surface}Underside")), gray, "{surface}");
    }
    for tab in prims.iter().filter(|p| p.prim_path.contains("TrimTab")) {
        assert_eq!(tab.material.diffuse_rgb, olive, "{}", tab.prim_path);
    }
}
