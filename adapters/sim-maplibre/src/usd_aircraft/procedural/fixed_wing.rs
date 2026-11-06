//! Twin-boom, high-aspect corridor-mapping UAV with an inverted-V (`Λ`) tail.
//!
//! The wing has a straight trailing edge so flaps and ailerons share one hinge line along
//! aircraft `+X`; the outboard taper is taken on the leading edge instead.

use glam::{DMat4, Mat4, Vec3};

use super::{
    CARBON, GLOSS_WHITE, GRAPHITE, OPTICAL_GLASS, fixed, glowing, material, part, rotor,
    shapes::{Inside, MeshBuilder, aerofoil, ring},
};
use crate::{PartRole, PbrMaterial, UsdMeshPart};

/// Wing chord-plane height above the body origin.
const WING_Z: f32 = 0.085;
/// Straight wing trailing edge.
const WING_TE_Y: f32 = -0.16;
/// Chord of the flaps and ailerons aft of their hinge line.
const SURFACE_CHORD: f32 = 0.075;
/// Wing semi-span.
const SEMI_SPAN: f32 = 1.6;
/// Lateral boom offset.
const BOOM_X: f32 = 0.45;
/// Boom centreline height.
const BOOM_Z: f32 = 0.06;
/// Tail apex height above the boom centreline.
const TAIL_RISE: f32 = 0.38;
/// Ruddervator hinge line.
const TAIL_HINGE_Y: f32 = -1.33;
/// Fixed tail-panel chord ahead of the hinge.
const TAIL_STAB_CHORD: f32 = 0.19;
/// Ruddervator chord aft of the hinge.
const RUDDERVATOR_CHORD: f32 = 0.075;

const SURFACE_WHITE: PbrMaterial = material([0.72, 0.74, 0.77], 0.0, 0.3);

pub(super) fn parts() -> Vec<UsdMeshPart> {
    let mut parts = vec![
        fixed(
            "/FixedWingUav/Fuselage/Pod",
            PartRole::CabinExterior,
            GLOSS_WHITE,
            fuselage(),
        ),
        fixed(
            "/FixedWingUav/Wing/Main",
            PartRole::WingStructure,
            GLOSS_WHITE,
            wing(),
        ),
        fixed(
            "/FixedWingUav/Booms",
            PartRole::CabinExterior,
            CARBON,
            booms(),
        ),
        fixed(
            "/FixedWingUav/Tail/Stabilisers",
            PartRole::CabinExterior,
            GLOSS_WHITE,
            tail_fixed(),
        ),
        fixed(
            "/FixedWingUav/Payload/Housing",
            PartRole::CabinExterior,
            GRAPHITE,
            payload(),
        ),
        fixed(
            "/FixedWingUav/Lights/NavLeft",
            PartRole::CabinExterior,
            glowing([0.9, 0.1, 0.08], [2.4, 0.2, 0.12]),
            wingtip_light(-1.0),
        ),
        fixed(
            "/FixedWingUav/Lights/NavRight",
            PartRole::CabinExterior,
            glowing([0.1, 0.9, 0.2], [0.2, 2.4, 0.4]),
            wingtip_light(1.0),
        ),
        fixed(
            "/FixedWingUav/Lights/Strobe",
            PartRole::BeaconLight,
            glowing([0.95, 0.95, 0.95], [2.5, 2.5, 2.5]),
            strobe(),
        ),
    ];
    let mut window = MeshBuilder::new();
    window.ellipsoid(
        Vec3::new(0.0, 0.1, -0.135),
        Vec3::new(0.045, 0.045, 0.03),
        24,
        12,
    );
    let mut window_part = fixed(
        "/FixedWingUav/Payload/NadirWindow",
        PartRole::CabinGlass,
        OPTICAL_GLASS,
        window,
    );
    window_part.double_sided = true;
    parts.push(window_part);
    parts.extend(trailing_edge_surfaces());
    parts.extend(ruddervators());
    // Pusher: local +Y (spin axis, spinner tip) points aft.
    let pusher = DMat4::from_translation(glam::DVec3::new(0.0, -0.585, 0.0))
        * DMat4::from_rotation_z(std::f64::consts::PI);
    parts.extend(rotor("/FixedWingUav/Pusher", pusher, 0.2, 2, GRAPHITE));
    parts
}

fn fuselage() -> MeshBuilder {
    const SECTIONS: [(f32, f32, f32, f32); 8] = [
        // (y, centre z, half width, half height)
        (0.64, -0.005, 0.006, 0.006),
        (0.58, -0.005, 0.045, 0.04),
        (0.44, 0.0, 0.08, 0.072),
        (0.18, 0.01, 0.095, 0.09),
        (-0.12, 0.01, 0.09, 0.085),
        (-0.38, 0.005, 0.07, 0.062),
        (-0.52, 0.0, 0.045, 0.04),
        (-0.56, 0.0, 0.03, 0.028),
    ];
    let rings: Vec<Vec<Vec3>> = SECTIONS
        .iter()
        .map(|&(y, z, w, h)| ring(Vec3::new(0.0, y, z), Vec3::X * w, Vec3::Z * h, 28))
        .collect();
    let mut mesh = MeshBuilder::new();
    mesh.loft(&rings, true, Inside::Axis);
    mesh
}

/// Total chord at span station `x` (constant inboard, tapered outboard).
fn chord_at(x: f32) -> f32 {
    let outboard = ((x.abs() - 0.9) / (SEMI_SPAN - 0.9)).clamp(0.0, 1.0);
    0.30 + (0.18 - 0.30) * outboard
}

fn wing() -> MeshBuilder {
    let hinge_y = WING_TE_Y + SURFACE_CHORD;
    let rings: Vec<Vec<Vec3>> = [-SEMI_SPAN, -0.9, 0.0, 0.9, SEMI_SPAN]
        .iter()
        .map(|&x| {
            let chord = chord_at(x);
            let leading = Vec3::new(x, WING_TE_Y + chord, WING_Z);
            aerofoil(
                leading,
                Vec3::new(0.0, hinge_y - leading.y, 0.0),
                Vec3::Z * chord * 0.13,
            )
        })
        .collect();
    let mut mesh = MeshBuilder::new();
    mesh.loft(&rings, true, Inside::Axis);
    mesh
}

/// Flaps and ailerons, each authored with its hinge line on local `+X` through the origin.
fn trailing_edge_surfaces() -> Vec<UsdMeshPart> {
    let surfaces = [
        ("FlapLeft", PartRole::FlapLeft, -0.85, -0.12),
        ("FlapRight", PartRole::FlapRight, 0.12, 0.85),
        ("AileronLeft", PartRole::AileronLeft, -1.5, -0.9),
        ("AileronRight", PartRole::AileronRight, 0.9, 1.5),
    ];
    surfaces
        .into_iter()
        .map(|(name, role, x0, x1)| {
            let hinge = Vec3::new(x0, WING_TE_Y + SURFACE_CHORD, WING_Z);
            let mut mesh = MeshBuilder::new();
            let rings: Vec<Vec<Vec3>> = [0.0, x1 - x0]
                .iter()
                .map(|&x| {
                    aerofoil(
                        Vec3::new(x, 0.0, 0.0),
                        Vec3::NEG_Y * SURFACE_CHORD,
                        Vec3::Z * 0.016,
                    )
                })
                .collect();
            mesh.loft(&rings, true, Inside::Axis);
            let rest = DMat4::from_translation(hinge.as_dvec3());
            part(
                &format!("/FixedWingUav/Wing/{name}"),
                role,
                SURFACE_WHITE,
                rest,
                mesh,
            )
        })
        .collect()
}

fn booms() -> MeshBuilder {
    let mut mesh = MeshBuilder::new();
    for side in [-1.0_f32, 1.0] {
        let front = Vec3::new(side * BOOM_X, 0.32, BOOM_Z);
        mesh.tube(
            front,
            Vec3::new(side * BOOM_X, -1.42, BOOM_Z),
            0.02,
            0.016,
            16,
        );
        mesh.ellipsoid(front, Vec3::new(0.02, 0.06, 0.02), 16, 8);
    }
    mesh
}

/// Rest frame of one tail panel: origin on the boom at the hinge line, local `+X` along the
/// hinge, local `+Z` the panel normal (up and outboard). Geometry spans local `x` from 0
/// to `span_sign * panel_length`.
fn tail_panel_frame(side: f32) -> (Mat4, f32) {
    let dihedral = TAIL_RISE.atan2(BOOM_X);
    let rotation = Mat4::from_rotation_y(side * dihedral);
    let origin = Vec3::new(side * BOOM_X, TAIL_HINGE_Y, BOOM_Z);
    (Mat4::from_translation(origin) * rotation, -side)
}

fn tail_panel(span_sign: f32, leading_y: f32, chord: f32, thickness: f32) -> MeshBuilder {
    let length = (BOOM_X * BOOM_X + TAIL_RISE * TAIL_RISE).sqrt();
    let rings: Vec<Vec<Vec3>> = [0.0, span_sign * length]
        .iter()
        .map(|&x| {
            aerofoil(
                Vec3::new(x, leading_y, 0.0),
                Vec3::NEG_Y * chord,
                Vec3::Z * thickness,
            )
        })
        .collect();
    let mut mesh = MeshBuilder::new();
    mesh.loft(&rings, true, Inside::Axis);
    mesh
}

fn tail_fixed() -> MeshBuilder {
    let mut mesh = MeshBuilder::new();
    for side in [-1.0_f32, 1.0] {
        let (frame, span_sign) = tail_panel_frame(side);
        mesh.append(
            tail_panel(span_sign, TAIL_STAB_CHORD, TAIL_STAB_CHORD, 0.022),
            frame,
        );
    }
    mesh
}

/// Left panel deflects by `elevator - rudder`, right by `elevator + rudder`; see
/// [`PartRole::RuddervatorLeft`].
fn ruddervators() -> Vec<UsdMeshPart> {
    [
        (-1.0_f32, "RuddervatorLeft", PartRole::RuddervatorLeft),
        (1.0, "RuddervatorRight", PartRole::RuddervatorRight),
    ]
    .into_iter()
    .map(|(side, name, role)| {
        let (frame, span_sign) = tail_panel_frame(side);
        let mesh = tail_panel(span_sign, 0.0, RUDDERVATOR_CHORD, 0.012);
        part(
            &format!("/FixedWingUav/Tail/{name}"),
            role,
            SURFACE_WHITE,
            frame.as_dmat4(),
            mesh,
        )
    })
    .collect()
}

fn payload() -> MeshBuilder {
    let mut mesh = MeshBuilder::new();
    mesh.ellipsoid(
        Vec3::new(0.0, 0.08, -0.095),
        Vec3::new(0.065, 0.12, 0.05),
        24,
        12,
    );
    mesh
}

fn wingtip_light(side: f32) -> MeshBuilder {
    let mut mesh = MeshBuilder::new();
    let tip_chord = chord_at(SEMI_SPAN);
    mesh.ellipsoid(
        Vec3::new(
            side * (SEMI_SPAN + 0.01),
            WING_TE_Y + tip_chord * 0.7,
            WING_Z,
        ),
        Vec3::new(0.012, 0.025, 0.01),
        12,
        6,
    );
    mesh
}

fn strobe() -> MeshBuilder {
    let mut mesh = MeshBuilder::new();
    mesh.ellipsoid(
        Vec3::new(0.0, 0.0, 0.11),
        Vec3::new(0.014, 0.02, 0.012),
        12,
        6,
    );
    mesh
}
