//! Two-stage crewed lunar lander; the origin lies on the footpad contact plane.

use std::f32::consts::{FRAC_PI_2, FRAC_PI_8, TAU};

use glam::{Vec2, Vec3};

use super::{OPTICAL_GLASS, fixed, material, shapes::MeshBuilder};
use crate::{PartRole, PbrMaterial, UsdMeshPart};

/// Descent-stage underside height.
const DESCENT_BOTTOM_Z: f32 = 1.25;
/// Descent-stage top deck height.
const DESCENT_TOP_Z: f32 = 2.95;
/// Descent-stage octagon circumradius (`4.2 m` across flats).
const DESCENT_RADIUS: f32 = 2.27;
/// Radial distance of each footpad centre.
const FOOTPAD_RADIUS: f32 = 4.1;

const GOLD_KAPTON: PbrMaterial = material([1.0, 0.72, 0.26], 0.92, 0.34);
const ALUMINIUM: PbrMaterial = material([0.7, 0.71, 0.7], 0.75, 0.36);
const BLACK_PANEL: PbrMaterial = material([0.05, 0.05, 0.055], 0.1, 0.6);
const ENGINE: PbrMaterial = material([0.28, 0.27, 0.26], 0.85, 0.5);

pub(super) fn parts() -> Vec<UsdMeshPart> {
    let mut windows = MeshBuilder::new();
    for side in [-1.0_f32, 1.0] {
        let x = |t: f32| side * t;
        windows.polygon(
            &[
                Vec3::new(x(0.15), 1.012, 4.0),
                Vec3::new(x(0.78), 1.012, 4.0),
                Vec3::new(x(0.15), 1.012, 4.5),
            ],
            Vec3::Y,
        );
    }
    let mut window_part = fixed(
        "/LunarLander/AscentStage/Windows",
        PartRole::CabinGlass,
        PbrMaterial {
            opacity: 0.6,
            ..OPTICAL_GLASS
        },
        windows,
    );
    window_part.double_sided = true;
    vec![
        fixed(
            "/LunarLander/DescentStage/Insulation",
            PartRole::CabinExterior,
            GOLD_KAPTON,
            descent_stage(),
        ),
        fixed(
            "/LunarLander/DescentStage/Engine",
            PartRole::CabinExterior,
            ENGINE,
            engine(),
        ),
        fixed(
            "/LunarLander/AscentStage/Cabin",
            PartRole::CabinExterior,
            ALUMINIUM,
            ascent_cabin(),
        ),
        fixed(
            "/LunarLander/AscentStage/Details",
            PartRole::CabinExterior,
            BLACK_PANEL,
            ascent_details(),
        ),
        fixed(
            "/LunarLander/AscentStage/Antenna",
            PartRole::CabinExterior,
            ALUMINIUM,
            antenna(),
        ),
        window_part,
        fixed(
            "/LunarLander/LandingGear/Struts",
            PartRole::GearStruts,
            GOLD_KAPTON,
            struts(),
        ),
        fixed(
            "/LunarLander/LandingGear/Footpads",
            PartRole::CabinExterior,
            ALUMINIUM,
            footpads(),
        ),
        fixed(
            "/LunarLander/LandingGear/Ladder",
            PartRole::GearStruts,
            ALUMINIUM,
            ladder(),
        ),
    ]
}

/// Deterministic crinkle offset in `[-1, 1]` for one blanket facet vertex.
fn crinkle(face: u32, i: u32, j: u32) -> f32 {
    let mut h = face
        .wrapping_mul(0x9E37_79B9)
        .wrapping_add(i.wrapping_mul(0x85EB_CA6B))
        .wrapping_add(j.wrapping_mul(0xC2B2_AE35));
    h ^= h >> 15;
    h = h.wrapping_mul(0x2C1B_3C6D);
    h ^= h >> 12;
    (h % 2001) as f32 / 1000.0 - 1.0
}

fn octagon_corner(k: u32, z: f32) -> Vec3 {
    let angle = FRAC_PI_8 + TAU * k as f32 / 8.0;
    Vec3::new(
        angle.cos() * DESCENT_RADIUS,
        angle.sin() * DESCENT_RADIUS,
        z,
    )
}

/// Octagonal box wrapped in crinkled gold kapton: every side is a jittered 4×3 grid of
/// flat triangles whose interior vertices bulge along the face normal.
fn descent_stage() -> MeshBuilder {
    const COLUMNS: u32 = 4;
    const ROWS: u32 = 3;
    let mut mesh = MeshBuilder::new();
    for face in 0..8 {
        let a = octagon_corner(face, DESCENT_BOTTOM_Z);
        let b = octagon_corner(face + 1, DESCENT_BOTTOM_Z);
        let outward = ((a + b) * 0.5).with_z(0.0).normalize_or(Vec3::X);
        let height = DESCENT_TOP_Z - DESCENT_BOTTOM_Z;
        let point = |i: u32, j: u32| {
            let s = i as f32 / COLUMNS as f32;
            let t = j as f32 / ROWS as f32;
            let interior = i > 0 && i < COLUMNS && j > 0 && j < ROWS;
            let bulge = if interior {
                0.06 * crinkle(face, i, j) + 0.03
            } else {
                0.0
            };
            a.lerp(b, s) + Vec3::Z * (height * t) + outward * bulge
        };
        for i in 0..COLUMNS {
            for j in 0..ROWS {
                let (p00, p10) = (point(i, j), point(i + 1, j));
                let (p01, p11) = (point(i, j + 1), point(i + 1, j + 1));
                mesh.polygon(&[p00, p10, p11], outward);
                mesh.polygon(&[p00, p11, p01], outward);
            }
        }
    }
    let deck = |z: f32| (0..8).map(|k| octagon_corner(k, z)).collect::<Vec<_>>();
    mesh.polygon(&deck(DESCENT_TOP_Z), Vec3::Z);
    mesh.polygon(&deck(DESCENT_BOTTOM_Z), Vec3::NEG_Z);
    mesh
}

fn engine() -> MeshBuilder {
    let mut mesh = MeshBuilder::new();
    mesh.tube(
        Vec3::new(0.0, 0.0, DESCENT_BOTTOM_Z + 0.02),
        Vec3::new(0.0, 0.0, 0.62),
        0.34,
        0.76,
        32,
    );
    mesh
}

/// Irregular faceted cabin: broad flat front face, chamfered sides, narrower top.
fn ascent_cabin() -> MeshBuilder {
    const OUTLINE: [Vec2; 8] = [
        Vec2::new(-0.9, 1.0),
        Vec2::new(0.9, 1.0),
        Vec2::new(1.35, 0.4),
        Vec2::new(1.35, -0.6),
        Vec2::new(0.8, -1.25),
        Vec2::new(-0.8, -1.25),
        Vec2::new(-1.35, -0.6),
        Vec2::new(-1.35, 0.4),
    ];
    let levels = [(DESCENT_TOP_Z, 0.82), (3.3, 1.0), (4.65, 1.0), (5.05, 0.72)];
    let rings: Vec<Vec<Vec3>> = levels
        .iter()
        .map(|&(z, scale)| OUTLINE.iter().map(|p| (*p * scale).extend(z)).collect())
        .collect();
    let mut mesh = MeshBuilder::new();
    mesh.loft_faceted(&rings);
    mesh.tube(
        Vec3::new(0.0, -0.15, 5.0),
        Vec3::new(0.0, -0.15, 5.4),
        0.46,
        0.44,
        24,
    );
    mesh
}

/// Front hatch, RCS thruster quads, and the aft equipment bay.
fn ascent_details() -> MeshBuilder {
    let mut mesh = MeshBuilder::new();
    mesh.cuboid(Vec3::new(-0.4, 0.98, 3.35), Vec3::new(0.4, 1.03, 3.95));
    for (x, y) in [(-1.45, 0.75), (1.45, 0.75), (-1.45, -0.95), (1.45, -0.95)] {
        let centre = Vec3::new(x, y, 4.35);
        mesh.cuboid(centre - Vec3::splat(0.14), centre + Vec3::splat(0.14));
        for direction in [Vec3::Z, Vec3::NEG_Z, Vec3::new(x.signum(), 0.0, 0.0)] {
            mesh.tube(
                centre + direction * 0.12,
                centre + direction * 0.3,
                0.035,
                0.07,
                12,
            );
        }
    }
    mesh.cuboid(Vec3::new(-0.75, -1.6, 3.4), Vec3::new(0.75, -1.2, 4.4));
    mesh
}

fn antenna() -> MeshBuilder {
    let mut mesh = MeshBuilder::new();
    let foot = Vec3::new(0.9, -0.7, 4.85);
    let top = foot + Vec3::new(0.25, -0.1, 0.75);
    mesh.tube(foot, top, 0.03, 0.03, 10);
    mesh.ellipsoid(top + Vec3::Z * 0.05, Vec3::new(0.38, 0.38, 0.07), 24, 8);
    mesh
}

/// Radial unit vector and tangent of the leg at quadrant `k` (front, right, back, left).
fn leg_axes(k: u32) -> (Vec3, Vec3) {
    let angle = FRAC_PI_2 - FRAC_PI_2 * k as f32;
    let radial = Vec3::new(angle.cos(), angle.sin(), 0.0);
    (radial, Vec3::Z.cross(radial))
}

fn footpad_centre(radial: Vec3) -> Vec3 {
    radial * FOOTPAD_RADIUS + Vec3::Z * 0.22
}

fn struts() -> MeshBuilder {
    let mut mesh = MeshBuilder::new();
    for k in 0..4 {
        let (radial, tangent) = leg_axes(k);
        let hip = radial * 2.05 + Vec3::Z * 2.7;
        let foot = footpad_centre(radial);
        mesh.tube(hip, foot, 0.11, 0.08, 16);
        let knee = hip.lerp(foot, 0.62);
        for side in [-1.0_f32, 1.0] {
            let root = radial * 1.95 + tangent * (side * 0.85) + Vec3::Z * 1.35;
            mesh.tube(root, knee, 0.05, 0.05, 10);
        }
        let deck_root = radial * 2.05 + Vec3::Z * 1.3;
        mesh.tube(deck_root, knee, 0.045, 0.045, 10);
    }
    mesh
}

fn footpads() -> MeshBuilder {
    let mut mesh = MeshBuilder::new();
    for k in 0..4 {
        let (radial, _) = leg_axes(k);
        let centre = footpad_centre(radial).with_z(0.0);
        mesh.tube(centre, centre + Vec3::Z * 0.2, 0.4, 0.47, 28);
        mesh.ellipsoid(centre + Vec3::Z * 0.2, Vec3::new(0.12, 0.12, 0.06), 12, 6);
    }
    mesh
}

/// Egress ladder on the forward leg.
fn ladder() -> MeshBuilder {
    let (radial, tangent) = leg_axes(0);
    let top = radial * 2.25 + Vec3::Z * 2.45;
    let bottom = footpad_centre(radial) + radial * -0.45 + Vec3::Z * 0.35;
    let mut mesh = MeshBuilder::new();
    for side in [-1.0_f32, 1.0] {
        let offset = tangent * (side * 0.28) + radial * 0.12;
        mesh.tube(top + offset, bottom + offset, 0.025, 0.025, 8);
    }
    for rung in 1..8 {
        let at = top.lerp(bottom, rung as f32 / 8.0) + radial * 0.12;
        mesh.tube(at - tangent * 0.28, at + tangent * 0.28, 0.018, 0.018, 8);
    }
    mesh
}
