//! Zero-thickness surface-conformal markings for P-38G-13-LO `"Miss Virginia"`
//! (`s/n 43-2264`, White `147`, 339th FS / 347th FG, 13th Air Force, Guadalcanal 1943).

use std::f32::consts::{FRAC_PI_2, PI, TAU};

use glam::Vec3;

use crate::mesh::{Inside, MeshBuilder, P38MeshPrim, P38PartRole, fixed};
use crate::stations::{
    ANTI_GLARE, INSIGNIA_BLUE, INSIGNIA_WHITE, NOSE_ART_RED, STENCIL_YELLOW, boom_point, fin_point,
    gondola_point, wing_surface_point,
};

pub(crate) fn conformal_decal_prims() -> Vec<P38MeshPrim> {
    let mut prims = Vec::new();
    build_antiglare_panels(&mut prims);
    build_usaaf_star_roundels(&mut prims);
    build_nose_147_and_miss_virginia(&mut prims);
    build_tail_serial_32264(&mut prims);
    prims
}

fn build_antiglare_panels(prims: &mut Vec<P38MeshPrim>) {
    let mut glare = MeshBuilder::new();
    let nose_rows: Vec<Vec<Vec3>> = [3.48_f32, 3.15, 2.76, 2.20, 1.68]
        .into_iter()
        .map(|y| {
            (0..=10)
                .map(|i| {
                    let theta = -0.28 * PI + (i as f32 / 10.0) * (0.56 * PI);
                    gondola_point(y, theta) + Vec3::new(0.0, 0.0, 0.0022)
                })
                .collect()
        })
        .collect();
    glare.loft_open(&nose_rows, Inside::Point(Vec3::new(0.0, 2.5, 0.0)));

    for side in [-1.0_f32, 1.0] {
        let boom_rows: Vec<Vec<Vec3>> = [2.46_f32, 2.15, 1.82, 1.10, 0.45]
            .into_iter()
            .map(|y| {
                (0..=8)
                    .map(|i| {
                        let frac = i as f32 / 8.0;
                        let theta = if side < 0.0 {
                            0.04 * PI + frac * (0.42 * PI)
                        } else {
                            -0.46 * PI + frac * (0.42 * PI)
                        };
                        let p = boom_point(side, y, theta);
                        let (s, c) = theta.sin_cos();
                        p + Vec3::new(s * 0.0024, 0.0, c * 0.0024)
                    })
                    .collect()
            })
            .collect();
        glare.loft_open(&boom_rows, Inside::Axis);
    }

    prims.push(fixed(
        "/P38g/Decals/AntiGlarePanels",
        P38PartRole::CabinExterior,
        ANTI_GLARE,
        glare,
    ));
}

fn build_usaaf_star_roundels(prims: &mut Vec<P38MeshPrim>) {
    let mut blue_discs = MeshBuilder::new();
    let mut white_stars = MeshBuilder::new();

    // 1. Left upper wing & right lower wing 45-in (radius 0.57 m) roundels at WS 205 (x = ±5.20 m)
    for (xc, yc, upper) in [(-5.20_f32, -0.18_f32, true), (5.20, -0.18, false)] {
        project_wing_disc(&mut blue_discs, xc, yc, 0.56, upper, 0.0022);
        project_wing_star(&mut white_stars, xc, yc, 0.54, upper, 0.0046);
    }

    // 2. Outboard left & right boom roundels aft of wing trailing edge (y = -2.02 m)
    for side in [-1.0_f32, 1.0] {
        project_boom_disc(&mut blue_discs, side, -2.02, 0.27, 0.0024);
        project_boom_star(&mut white_stars, side, -2.02, 0.255, 0.0048);
    }

    prims.push(fixed(
        "/P38g/Decals/RoundelBlueDiscs",
        P38PartRole::WingStructure,
        INSIGNIA_BLUE,
        blue_discs,
    ));
    prims.push(fixed(
        "/P38g/Decals/RoundelWhiteStars",
        P38PartRole::WingStructure,
        INSIGNIA_WHITE,
        white_stars,
    ));
}

fn project_wing_pt(x: f32, y_target: f32, upper: bool, lift: f32) -> (Vec3, Vec3) {
    let p_le = wing_surface_point(x, 0.0, upper);
    let p_te = wing_surface_point(x, 1.0, upper);
    let chord = (p_le.y - p_te.y).max(0.05);
    let u = ((p_le.y - y_target) / chord).clamp(0.02, 0.98);
    let p = wing_surface_point(x, u, upper);
    let px = wing_surface_point(x + 0.02, u, upper);
    let pu = wing_surface_point(x, (u + 0.02).min(0.99), upper);
    let mut n = (px - p).cross(pu - p).normalize_or(Vec3::Z);
    let out = if upper { Vec3::Z } else { Vec3::NEG_Z };
    if n.dot(out) < 0.0 {
        n = -n;
    }
    (p + n * lift, n)
}

fn project_wing_disc(
    mesh: &mut MeshBuilder,
    xc: f32,
    yc: f32,
    radius: f32,
    upper: bool,
    lift: f32,
) {
    const SEGS: u32 = 32;
    const RINGS: u32 = 4;
    let out = if upper { Vec3::Z } else { Vec3::NEG_Z };
    let (p0, n0) = project_wing_pt(xc, yc, upper, lift);
    for r in 0..RINGS {
        let r0 = radius * (r as f32 / RINGS as f32);
        let r1 = radius * ((r + 1) as f32 / RINGS as f32);
        for i in 0..SEGS {
            let a0 = TAU * i as f32 / SEGS as f32;
            let a1 = TAU * ((i + 1) % SEGS) as f32 / SEGS as f32;
            let (s0, c0) = a0.sin_cos();
            let (s1, c1) = a1.sin_cos();
            let (q10, n10) = project_wing_pt(xc + c0 * r1, yc + s0 * r1, upper, lift);
            let (q11, n11) = project_wing_pt(xc + c1 * r1, yc + s1 * r1, upper, lift);
            if r == 0 {
                mesh.surface_tri([p0, q10, q11], [n0, n10, n11], out);
            } else {
                let (q00, n00) = project_wing_pt(xc + c0 * r0, yc + s0 * r0, upper, lift);
                let (q01, n01) = project_wing_pt(xc + c1 * r0, yc + s1 * r0, upper, lift);
                mesh.surface_tri([q00, q10, q11], [n00, n10, n11], out);
                mesh.surface_tri([q00, q11, q01], [n00, n11, n01], out);
            }
        }
    }
}

fn project_wing_star(mesh: &mut MeshBuilder, xc: f32, yc: f32, r_out: f32, upper: bool, lift: f32) {
    const N: u32 = 5;
    let r_in = r_out * 0.381_966;
    let out = if upper { Vec3::Z } else { Vec3::NEG_Z };
    for k in 0..5_u32 {
        let a_tip = FRAC_PI_2 + TAU * (k as f32) / 5.0;
        let a_l = a_tip - PI / 5.0;
        let a_r = a_tip + PI / 5.0;
        let v_tip = Vec3::new(a_tip.cos() * r_out, a_tip.sin() * r_out, 0.0);
        let v_l = Vec3::new(a_l.cos() * r_in, a_l.sin() * r_in, 0.0);
        let v_r = Vec3::new(a_r.cos() * r_in, a_r.sin() * r_in, 0.0);
        for (v_a, v_b) in [(v_l, v_tip), (v_tip, v_r)] {
            let map = |r: u32, u: u32| {
                let v = (v_a * ((r - u) as f32) + v_b * (u as f32)) / (N as f32);
                project_wing_pt(xc + v.x, yc + v.y, upper, lift)
            };
            for r in 0..N {
                for u in 0..=r {
                    let (p00, n00) = map(r, u);
                    let (p10, n10) = map(r + 1, u);
                    let (p11, n11) = map(r + 1, u + 1);
                    mesh.surface_tri([p00, p10, p11], [n00, n10, n11], out);
                    if u < r {
                        let (p01, n01) = map(r, u + 1);
                        mesh.surface_tri([p00, p11, p01], [n00, n11, n01], out);
                    }
                }
            }
        }
    }
}

fn project_boom_pt(side: f32, y: f32, dtheta: f32, lift: f32) -> (Vec3, Vec3) {
    let theta = side * FRAC_PI_2 + dtheta;
    let p = boom_point(side, y, theta);
    let py = boom_point(side, y + 0.02, theta);
    let pt = boom_point(side, y, theta + 0.03);
    let mut n = (py - p)
        .cross(pt - p)
        .normalize_or(Vec3::new(side, 0.0, 0.0));
    let out = Vec3::new(side, 0.0, 0.0);
    if n.dot(out) < 0.0 {
        n = -n;
    }
    (p + n * lift, n)
}

fn project_boom_disc(mesh: &mut MeshBuilder, side: f32, yc: f32, radius: f32, lift: f32) {
    const SEGS: u32 = 28;
    const RINGS: u32 = 4;
    let out = Vec3::new(side, 0.0, 0.0);
    let (p0, n0) = project_boom_pt(side, yc, 0.0, lift);
    for r in 0..RINGS {
        let r0 = radius * (r as f32 / RINGS as f32);
        let r1 = radius * ((r + 1) as f32 / RINGS as f32);
        for i in 0..SEGS {
            let a0 = TAU * i as f32 / SEGS as f32;
            let a1 = TAU * ((i + 1) % SEGS) as f32 / SEGS as f32;
            let (s0, c0) = a0.sin_cos();
            let (s1, c1) = a1.sin_cos();
            let (q10, n10) = project_boom_pt(side, yc + c0 * r1, -side * (s0 * r1 / 0.44), lift);
            let (q11, n11) = project_boom_pt(side, yc + c1 * r1, -side * (s1 * r1 / 0.44), lift);
            if r == 0 {
                mesh.surface_tri([p0, q10, q11], [n0, n10, n11], out);
            } else {
                let (q00, n00) =
                    project_boom_pt(side, yc + c0 * r0, -side * (s0 * r0 / 0.44), lift);
                let (q01, n01) =
                    project_boom_pt(side, yc + c1 * r0, -side * (s1 * r0 / 0.44), lift);
                mesh.surface_tri([q00, q10, q11], [n00, n10, n11], out);
                mesh.surface_tri([q00, q11, q01], [n00, n11, n01], out);
            }
        }
    }
}

fn project_boom_star(mesh: &mut MeshBuilder, side: f32, yc: f32, r_out: f32, lift: f32) {
    const N: u32 = 5;
    let r_in = r_out * 0.381_966;
    let out = Vec3::new(side, 0.0, 0.0);
    for k in 0..5_u32 {
        let a_tip = FRAC_PI_2 + TAU * (k as f32) / 5.0;
        let a_l = a_tip - PI / 5.0;
        let a_r = a_tip + PI / 5.0;
        let v_tip = Vec3::new(a_tip.cos() * r_out, a_tip.sin() * r_out, 0.0);
        let v_l = Vec3::new(a_l.cos() * r_in, a_l.sin() * r_in, 0.0);
        let v_r = Vec3::new(a_r.cos() * r_in, a_r.sin() * r_in, 0.0);
        for (v_a, v_b) in [(v_l, v_tip), (v_tip, v_r)] {
            let map = |r: u32, u: u32| {
                let v = (v_a * ((r - u) as f32) + v_b * (u as f32)) / (N as f32);
                project_boom_pt(side, yc + v.x, -side * (v.y / 0.44), lift)
            };
            for r in 0..N {
                for u in 0..=r {
                    let (p00, n00) = map(r, u);
                    let (p10, n10) = map(r + 1, u);
                    let (p11, n11) = map(r + 1, u + 1);
                    mesh.surface_tri([p00, p10, p11], [n00, n10, n11], out);
                    if u < r {
                        let (p01, n01) = map(r, u + 1);
                        mesh.surface_tri([p00, p11, p01], [n00, n11, n01], out);
                    }
                }
            }
        }
    }
}

fn project_gondola_quad(
    mesh: &mut MeshBuilder,
    side: f32,
    y0: f32,
    y1: f32,
    th0: f32,
    th1: f32,
    lift: f32,
) {
    const NY: u32 = 4;
    const NT: u32 = 4;
    let out = Vec3::new(side, 0.0, 0.0);
    let eval = |y: f32, th: f32| -> (Vec3, Vec3) {
        let theta = side * th;
        let p = gondola_point(y, theta);
        let py = gondola_point(y + 0.015, theta);
        let pt = gondola_point(y, theta + 0.02);
        let mut n = (py - p).cross(pt - p).normalize_or(out);
        if n.dot(out) < 0.0 {
            n = -n;
        }
        (p + n * lift, n)
    };
    for iy in 0..NY {
        let ya = y0 + (y1 - y0) * (iy as f32 / NY as f32);
        let yb = y0 + (y1 - y0) * ((iy + 1) as f32 / NY as f32);
        for it in 0..NT {
            let ta = th0 + (th1 - th0) * (it as f32 / NT as f32);
            let tb = th0 + (th1 - th0) * ((it + 1) as f32 / NT as f32);
            let (p00, n00) = eval(ya, ta);
            let (p10, n10) = eval(yb, ta);
            let (p11, n11) = eval(yb, tb);
            let (p01, n01) = eval(ya, tb);
            mesh.surface_tri([p00, p10, p11], [n00, n10, n11], out);
            mesh.surface_tri([p00, p11, p01], [n00, n11, n01], out);
        }
    }
}

fn build_nose_147_and_miss_virginia(prims: &mut Vec<P38MeshPrim>) {
    let mut white_num = MeshBuilder::new();
    let mut nose_red = MeshBuilder::new();
    let mut nose_yellow = MeshBuilder::new();

    for side in [-1.0_f32, 1.0] {
        // White "147" tactical fighter number on forward gondola cheeks (y = 2.04..2.64),
        // laid out as read from port (nose to the left). Seen from starboard the nose is
        // to the right, so that side reflects the layout about the number's centre to
        // keep "1" first and every glyph unmirrored.
        const CENTRE_Y: f32 = 2.34;
        for (y0, y1, t0, t1) in [
            (2.58_f32, 2.64_f32, 0.34 * PI, 0.56 * PI),
            (2.42, 2.47, 0.34 * PI, 0.46 * PI),
            (2.28, 2.47, 0.44 * PI, 0.49 * PI),
            (2.28, 2.34, 0.34 * PI, 0.56 * PI),
            (2.04, 2.22, 0.34 * PI, 0.39 * PI),
            (2.04, 2.10, 0.34 * PI, 0.56 * PI),
        ] {
            let (y0, y1) = if side > 0.0 {
                (2.0 * CENTRE_Y - y1, 2.0 * CENTRE_Y - y0)
            } else {
                (y0, y1)
            };
            project_gondola_quad(&mut white_num, side, y0, y1, t0, t1, 0.0026);
        }

        // "Miss Virginia" cursive script banner & emblem on forward nose (y = 1.50..1.94)
        project_gondola_quad(
            &mut nose_red,
            side,
            1.54,
            1.92,
            0.38 * PI,
            0.44 * PI,
            0.0026,
        );
        project_gondola_quad(
            &mut nose_yellow,
            side,
            1.50,
            1.94,
            0.45 * PI,
            0.50 * PI,
            0.0028,
        );
        project_gondola_quad(
            &mut nose_red,
            side,
            1.62,
            1.82,
            0.27 * PI,
            0.35 * PI,
            0.0028,
        );
    }

    prims.push(fixed(
        "/P38g/Decals/White147Numbers",
        P38PartRole::CabinExterior,
        INSIGNIA_WHITE,
        white_num,
    ));
    prims.push(fixed(
        "/P38g/Decals/MissVirginiaScript",
        P38PartRole::CabinExterior,
        NOSE_ART_RED,
        nose_red,
    ));
    prims.push(fixed(
        "/P38g/Decals/MissVirginiaTrim",
        P38PartRole::CabinExterior,
        STENCIL_YELLOW,
        nose_yellow,
    ));
}

fn project_fin_quad(
    mesh: &mut MeshBuilder,
    side: f32,
    z0: f32,
    z1: f32,
    u0: f32,
    u1: f32,
    lift: f32,
) {
    const NU: u32 = 4;
    let out = Vec3::new(side, 0.0, 0.0);
    let eval = |z: f32, u: f32| -> (Vec3, Vec3) {
        let p = fin_point(side, z, u, side);
        let pz = fin_point(side, z + 0.02, u, side);
        let pu = fin_point(side, z, (u + 0.02).min(0.98), side);
        let mut n = (pz - p).cross(pu - p).normalize_or(out);
        if n.dot(out) < 0.0 {
            n = -n;
        }
        (p + n * lift, n)
    };
    for iu in 0..NU {
        let ua = u0 + (u1 - u0) * (iu as f32 / NU as f32);
        let ub = u0 + (u1 - u0) * ((iu + 1) as f32 / NU as f32);
        let (p00, n00) = eval(z0, ua);
        let (p10, n10) = eval(z1, ua);
        let (p11, n11) = eval(z1, ub);
        let (p01, n01) = eval(z0, ub);
        mesh.surface_tri([p00, p10, p11], [n00, n10, n11], out);
        mesh.surface_tri([p00, p11, p01], [n00, n11, n01], out);
    }
}

fn build_tail_serial_32264(prims: &mut Vec<P38MeshPrim>) {
    // 7-segment bitmask: bit0=top, bit1=tl, bit2=tr, bit3=mid, bit4=bl, bit5=br, bit6=bot
    // 3 = 0b110_1101, 2 = 0b101_1101, 6 = 0b111_1011, 4 = 0b010_1110
    const DIGITS: [u8; 5] = [0b110_1101, 0b101_1101, 0b101_1101, 0b111_1011, 0b010_1110];
    let mut serial = MeshBuilder::new();

    for side in [-1.0_f32, 1.0] {
        for (idx, mask) in DIGITS.into_iter().enumerate() {
            // Port side (side < 0, viewed from -X): forward (small u) is on the left.
            // Starboard side (side > 0, viewed from +X): aft (large u) is on the left.
            let u_base = if side < 0.0 {
                0.20 + idx as f32 * 0.072
            } else {
                0.49 - idx as f32 * 0.072
            };
            let u_l = u_base;
            let u_r = u_base + (if side < 0.0 { 0.048 } else { -0.048 });
            let (u0, u1) = (u_l.min(u_r), u_l.max(u_r));
            let du = 0.012;
            let z_bot = 1.02_f32;
            let z_mid = 1.12_f32;
            let z_top = 1.22_f32;
            let dz = 0.026;
            let lift = 0.0032;

            if mask & 1 != 0 {
                project_fin_quad(&mut serial, side, z_top - dz, z_top, u0, u1, lift);
            }
            if mask & 8 != 0 {
                project_fin_quad(
                    &mut serial,
                    side,
                    z_mid - 0.5 * dz,
                    z_mid + 0.5 * dz,
                    u0,
                    u1,
                    lift,
                );
            }
            if mask & 64 != 0 {
                project_fin_quad(&mut serial, side, z_bot, z_bot + dz, u0, u1, lift);
            }
            let (ul_a, ul_b) = if side < 0.0 {
                (u0, u0 + du)
            } else {
                (u1 - du, u1)
            };
            let (ur_a, ur_b) = if side < 0.0 {
                (u1 - du, u1)
            } else {
                (u0, u0 + du)
            };
            if mask & 2 != 0 {
                project_fin_quad(&mut serial, side, z_mid, z_top, ul_a, ul_b, lift);
            }
            if mask & 4 != 0 {
                project_fin_quad(&mut serial, side, z_mid, z_top, ur_a, ur_b, lift);
            }
            if mask & 16 != 0 {
                project_fin_quad(&mut serial, side, z_bot, z_mid, ul_a, ul_b, lift);
            }
            if mask & 32 != 0 {
                project_fin_quad(&mut serial, side, z_bot, z_mid, ur_a, ur_b, lift);
            }
        }
    }

    prims.push(fixed(
        "/P38g/Decals/Serial32264",
        P38PartRole::CabinExterior,
        STENCIL_YELLOW,
        serial,
    ));
}
