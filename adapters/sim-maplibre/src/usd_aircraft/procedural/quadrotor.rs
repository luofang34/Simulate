//! Holybro X500-class carbon X-frame quadrotor (500 mm motor-to-motor diagonal).

use glam::Vec3;

use super::{
    ANODIZED, CARBON, GLOSS_WHITE, GRAPHITE, OPTICAL_GLASS, fixed, glowing, rotor,
    shapes::{Inside, MeshBuilder},
    vertical_spin_rest,
};
use crate::{PartRole, UsdMeshPart};

/// Motor offset along each body axis (`0.25 m` arm on the 45° diagonal).
const ARM_XY: f32 = 0.177;
/// Arm tube centreline height above the body origin.
const ARM_Z: f32 = 0.01;
/// Rotor hub plane height.
const HUB_Z: f32 = 0.048;
/// 13-inch propeller radius.
const PROP_RADIUS: f32 = 0.165;

/// Motor positions with spin sense: front-right and rear-left turn counter-clockwise
/// seen from above, the other diagonal clockwise, so yaw torques cancel.
const MOTORS: [(&str, f32, f32, bool); 4] = [
    ("FrontRight", 1.0, 1.0, true),
    ("RearLeft", -1.0, -1.0, true),
    ("FrontLeft", -1.0, 1.0, false),
    ("RearRight", 1.0, -1.0, false),
];

pub(super) fn parts() -> Vec<UsdMeshPart> {
    let mut parts = vec![
        fixed(
            "/X500/Frame/Plates",
            PartRole::CabinExterior,
            CARBON,
            plates(),
        ),
        fixed("/X500/Frame/Arms", PartRole::CabinExterior, CARBON, arms()),
        fixed(
            "/X500/Frame/Motors",
            PartRole::CabinExterior,
            ANODIZED,
            motors(),
        ),
        fixed(
            "/X500/Avionics/Pod",
            PartRole::CabinExterior,
            GLOSS_WHITE,
            pod(),
        ),
        fixed(
            "/X500/Avionics/GpsMast",
            PartRole::CabinExterior,
            GRAPHITE,
            gps(),
        ),
        fixed(
            "/X500/Gimbal/Housing",
            PartRole::CabinExterior,
            GRAPHITE,
            gimbal(),
        ),
        fixed("/X500/Gear/Skids", PartRole::GearStruts, CARBON, skids()),
        fixed(
            "/X500/Lights/FrontLeds",
            PartRole::CabinExterior,
            glowing([0.3, 0.9, 1.0], [0.6, 2.6, 3.4]),
            leds(1.0),
        ),
        fixed(
            "/X500/Lights/RearStrobe",
            PartRole::BeaconLight,
            glowing([0.9, 0.12, 0.08], [2.2, 0.25, 0.12]),
            leds(-1.0),
        ),
    ];
    let mut dome = MeshBuilder::new();
    dome.ellipsoid(Vec3::new(0.0, 0.072, -0.07), Vec3::splat(0.03), 24, 12);
    let mut dome_part = fixed(
        "/X500/Gimbal/Dome",
        PartRole::CabinGlass,
        OPTICAL_GLASS,
        dome,
    );
    dome_part.double_sided = true;
    parts.push(dome_part);
    for (name, sx, sy, ccw) in MOTORS {
        let hub = Vec3::new(sx * ARM_XY, sy * ARM_XY, HUB_Z);
        parts.extend(rotor(
            &format!("/X500/Rotors/{name}"),
            vertical_spin_rest(hub, ccw),
            PROP_RADIUS,
            2,
            CARBON,
        ));
    }
    parts
}

fn plates() -> MeshBuilder {
    let mut mesh = MeshBuilder::new();
    for (bottom, top) in [(-0.004, 0.0), (0.02, 0.024)] {
        mesh.tube(
            Vec3::new(0.0, 0.0, bottom),
            Vec3::new(0.0, 0.0, top),
            0.085,
            0.085,
            8,
        );
    }
    mesh
}

fn arms() -> MeshBuilder {
    let mut mesh = MeshBuilder::new();
    for (_, sx, sy, _) in MOTORS {
        let tip = Vec3::new(sx * ARM_XY, sy * ARM_XY, ARM_Z);
        mesh.tube(tip * Vec3::new(0.2, 0.2, 1.0), tip, 0.0085, 0.0085, 12);
    }
    mesh
}

fn motors() -> MeshBuilder {
    let mut mesh = MeshBuilder::new();
    for (_, sx, sy, _) in MOTORS {
        let x = sx * ARM_XY;
        let y = sy * ARM_XY;
        mesh.tube(
            Vec3::new(x, y, ARM_Z - 0.012),
            Vec3::new(x, y, HUB_Z - 0.006),
            0.0185,
            0.017,
            20,
        );
    }
    mesh
}

/// Rounded avionics/battery canopy with a tapered nose.
fn pod() -> MeshBuilder {
    const SECTIONS: [(f32, f32, f32); 6] = [
        // (y, half width, half height)
        (-0.10, 0.035, 0.018),
        (-0.085, 0.05, 0.03),
        (-0.02, 0.056, 0.034),
        (0.05, 0.052, 0.031),
        (0.095, 0.035, 0.022),
        (0.112, 0.012, 0.008),
    ];
    let rings: Vec<Vec<Vec3>> = SECTIONS
        .iter()
        .map(|&(y, w, h)| rounded_section(Vec3::new(0.0, y, 0.058), w, h, 24))
        .collect();
    let mut mesh = MeshBuilder::new();
    mesh.loft(&rings, true, Inside::Axis);
    mesh
}

fn gps() -> MeshBuilder {
    let mut mesh = MeshBuilder::new();
    let base = Vec3::new(0.0, -0.06, 0.085);
    mesh.tube(base, base + Vec3::Z * 0.07, 0.004, 0.004, 10);
    mesh.tube(
        base + Vec3::Z * 0.07,
        base + Vec3::Z * 0.084,
        0.032,
        0.03,
        24,
    );
    mesh
}

fn gimbal() -> MeshBuilder {
    let mut mesh = MeshBuilder::new();
    mesh.cuboid(
        Vec3::new(-0.022, 0.03, -0.03),
        Vec3::new(0.022, 0.06, -0.004),
    );
    mesh.ellipsoid(Vec3::new(0.0, 0.06, -0.06), Vec3::splat(0.034), 24, 12);
    mesh
}

fn skids() -> MeshBuilder {
    let mut mesh = MeshBuilder::new();
    for side in [-1.0_f32, 1.0] {
        for fore in [-1.0_f32, 1.0] {
            mesh.tube(
                Vec3::new(side * 0.05, fore * 0.06, -0.004),
                Vec3::new(side * 0.12, fore * 0.075, -0.2),
                0.006,
                0.006,
                10,
            );
        }
        mesh.tube(
            Vec3::new(side * 0.12, -0.13, -0.2),
            Vec3::new(side * 0.12, 0.13, -0.2),
            0.008,
            0.008,
            12,
        );
    }
    mesh
}

/// LED lenses under the two arms on the `fore` (`+1` front, `-1` rear) side.
fn leds(fore: f32) -> MeshBuilder {
    let mut mesh = MeshBuilder::new();
    for side in [-1.0_f32, 1.0] {
        mesh.ellipsoid(
            Vec3::new(side * ARM_XY * 0.82, fore * ARM_XY * 0.82, ARM_Z - 0.01),
            Vec3::new(0.008, 0.008, 0.005),
            12,
            6,
        );
    }
    mesh
}

/// Superellipse section in the `XZ` plane: flat sides with rounded corners.
fn rounded_section(centre: Vec3, half_width: f32, half_height: f32, segments: u32) -> Vec<Vec3> {
    (0..segments)
        .map(|k| {
            let angle = std::f32::consts::TAU * k as f32 / segments as f32;
            let (s, c) = angle.sin_cos();
            let x = c.signum() * c.abs().sqrt() * half_width;
            let z = s.signum() * s.abs().sqrt() * half_height;
            centre + Vec3::new(x, 0.0, z)
        })
        .collect()
}
