//! Procedurally built-in vehicles composed directly as [`UsdMeshPart`]s.
//!
//! Every vehicle uses the aircraft root axes the USD pass expects (`+X` right, `+Y`
//! forward, `+Z` up, metres). Articulated parts follow the joint contract of
//! [`UsdAircraftAsset::evaluate_draws`]: geometry is authored with its hinge line along
//! local `+X` (control surfaces) or its spin axis along local `+Y` (propellers and rotors)
//! through the local origin, and `rest_to_aircraft` places that origin on the airframe.

use std::f32::consts::TAU;

use glam::{DMat4, Mat4, Vec3};

use super::{PartRole, PbrMaterial, SurfaceDetail, UsdAircraftAsset, UsdMeshPart, UsdMeshVertex};
use crate::MapAdapterError;

mod fixed_wing;
mod lunar_lander;
mod p38_finish;
mod quadrotor;
mod shapes;
#[cfg(test)]
mod tests;

use shapes::{Inside, MeshBuilder};

impl UsdAircraftAsset {
    /// Builds a Holybro X500-class carbon X-frame quadrotor with four counter-rotating,
    /// articulated rotors (`PropellerSpinner` / `PropellerBlades` / `PropellerDisc`).
    pub fn x500_quadrotor() -> Result<Self, MapAdapterError> {
        assemble("x500_quadrotor", quadrotor::parts())
    }

    /// Builds a twin-boom, high-aspect corridor-mapping UAV with ailerons, inverted-V
    /// ruddervators, a belly nadir optical payload pod, and an articulated pusher propeller.
    pub fn fixed_wing_uav() -> Result<Self, MapAdapterError> {
        assemble("fixed_wing_uav", fixed_wing::parts())
    }

    /// Builds a Lockheed P-38G-13-LO Lightning (`"Miss Virginia"`, `s/n 43-2264`) from the
    /// standalone `sim-p38g` vehicle crate (`vehicles/p38g`).
    pub fn p38g() -> Result<Self, MapAdapterError> {
        let prims = sim_p38g::p38g_mesh_prims();
        let skin = sim_p38g::P38Skin::new(&prims).map_err(|err| MapAdapterError::OpenUsd {
            detail: format!("P-38G skin: {err}"),
        })?;
        let parts = prims
            .into_iter()
            .map(|prim| UsdMeshPart {
                detail: p38_finish::detail(&prim.prim_path),
                surface: crate::usd_aircraft::PartSurface::Material,
                prim_path: prim.prim_path,
                role: PartRole::from_token(prim.role.as_token()),
                material: PbrMaterial {
                    diffuse_rgb: prim.material.diffuse_rgb,
                    emissive_rgb: prim.material.emissive_rgb,
                    metallic: prim.material.metallic,
                    roughness: prim.material.roughness,
                    opacity: prim.material.opacity,
                },
                double_sided: prim.double_sided,
                rest_to_aircraft: prim.rest_to_aircraft,
                custom_hinge_axis: prim.custom_hinge_axis,
                needle_channel: prim.needle_channel,
                vertices: prim
                    .vertices
                    .into_iter()
                    .map(|v| UsdMeshVertex {
                        position: v.position,
                        normal: v.normal,
                        uv: v.uv,
                        occlusion: 1.0,
                        tangent: crate::UsdMeshVertex::NO_TANGENT,
                    })
                    .collect(),
                indices: prim.indices,
            })
            .collect();
        let mut asset = assemble("p38g", parts)?;
        p38_finish::finish(&mut asset, skin);
        Ok(asset)
    }

    /// Builds a crewed lunar lander: faceted gold-kapton descent stage, ascent cabin, and
    /// four landing legs with footpads (origin on the footpad contact plane).
    pub fn lunar_lander() -> Result<Self, MapAdapterError> {
        assemble("lunar_lander", lunar_lander::parts())
    }
}

/// Rejects empty or non-finite geometry so a procedural asset fails loudly at construction.
fn assemble(name: &str, parts: Vec<UsdMeshPart>) -> Result<UsdAircraftAsset, MapAdapterError> {
    if parts.is_empty() {
        return Err(MapAdapterError::OpenUsd {
            detail: format!("procedural vehicle '{name}' produced zero mesh parts"),
        });
    }
    for part in &parts {
        let finite = part
            .vertices
            .iter()
            .all(|v| v.position.iter().chain(&v.normal).all(|c| c.is_finite()));
        if part.indices.len() < 3 || !finite || !part.rest_to_aircraft.is_finite() {
            return Err(MapAdapterError::OpenUsd {
                detail: format!(
                    "procedural vehicle '{name}' part '{}' has {} indices and finite={finite}",
                    part.prim_path,
                    part.indices.len()
                ),
            });
        }
    }
    Ok(UsdAircraftAsset::airframe(name, parts))
}

/// One single-sided part at `rest` in the aircraft frame.
fn part(
    path: &str,
    role: PartRole,
    material: PbrMaterial,
    rest: DMat4,
    mesh: MeshBuilder,
) -> UsdMeshPart {
    let (vertices, indices) = mesh.into_buffers();
    UsdMeshPart {
        prim_path: path.to_owned(),
        role,
        material,
        double_sided: false,
        rest_to_aircraft: rest,
        custom_hinge_axis: None,
        needle_channel: None,
        detail: SurfaceDetail::default(),
        surface: crate::usd_aircraft::PartSurface::Material,
        vertices,
        indices,
    }
}

/// One part whose geometry is authored directly in the aircraft frame.
fn fixed(path: &str, role: PartRole, material: PbrMaterial, mesh: MeshBuilder) -> UsdMeshPart {
    part(path, role, material, DMat4::IDENTITY, mesh)
}

const fn material(diffuse: [f32; 3], metallic: f32, roughness: f32) -> PbrMaterial {
    PbrMaterial {
        diffuse_rgb: diffuse,
        emissive_rgb: [0.0, 0.0, 0.0],
        metallic,
        roughness,
        opacity: 1.0,
    }
}

const fn glowing(diffuse: [f32; 3], emissive: [f32; 3]) -> PbrMaterial {
    PbrMaterial {
        diffuse_rgb: diffuse,
        emissive_rgb: emissive,
        metallic: 0.0,
        roughness: 0.2,
        opacity: 1.0,
    }
}

/// Woven carbon-fibre laminate.
const CARBON: PbrMaterial = material([0.035, 0.037, 0.042], 0.25, 0.32);
/// Gloss gel-coat composite skin.
const GLOSS_WHITE: PbrMaterial = material([0.82, 0.83, 0.85], 0.0, 0.22);
/// Matte graphite polymer.
const GRAPHITE: PbrMaterial = material([0.10, 0.105, 0.115], 0.05, 0.55);
/// Anodized motor bell.
const ANODIZED: PbrMaterial = material([0.55, 0.56, 0.60], 0.9, 0.3);
/// Optical payload window.
const OPTICAL_GLASS: PbrMaterial = PbrMaterial {
    diffuse_rgb: [0.05, 0.08, 0.12],
    emissive_rgb: [0.0, 0.0, 0.0],
    metallic: 0.0,
    roughness: 0.05,
    opacity: 0.35,
};

/// Spinner, blades, and motion-blur disc of one rotor whose spin axis is local `+Y`.
fn rotor(
    prefix: &str,
    rest: DMat4,
    radius: f32,
    blades: u32,
    blade_material: PbrMaterial,
) -> Vec<UsdMeshPart> {
    let hub_radius = radius * 0.11;
    let mut spinner = MeshBuilder::new();
    spinner.ellipsoid(
        Vec3::new(0.0, hub_radius * 0.35, 0.0),
        Vec3::new(hub_radius, hub_radius * 1.3, hub_radius),
        16,
        8,
    );
    let mut blade_mesh = MeshBuilder::new();
    for k in 0..blades.max(1) {
        let turn = Mat4::from_rotation_y(TAU * k as f32 / blades.max(1) as f32);
        blade_mesh.append(blade(radius, hub_radius * 0.6), turn);
    }
    let mut disc = MeshBuilder::new();
    disc.disc_xz(radius, 48);
    let mut disc_part = part(
        &format!("{prefix}/Disc"),
        PartRole::PropellerDisc,
        GRAPHITE,
        rest,
        disc,
    );
    disc_part.double_sided = true;
    vec![
        part(
            &format!("{prefix}/Spinner"),
            PartRole::PropellerSpinner,
            ANODIZED,
            rest,
            spinner,
        ),
        part(
            &format!("{prefix}/Blades"),
            PartRole::PropellerBlades,
            blade_material,
            rest,
            blade_mesh,
        ),
        disc_part,
    ]
}

/// One twisted, tapered blade along local `+X` in the rotation (`XZ`) plane.
fn blade(radius: f32, root: f32) -> MeshBuilder {
    const STATIONS: [(f32, f32, f32); 5] = [
        // (span fraction, chord / radius, pitch deg)
        (0.0, 0.10, 32.0),
        (0.25, 0.17, 24.0),
        (0.55, 0.15, 16.0),
        (0.85, 0.10, 11.0),
        (1.0, 0.05, 9.0),
    ];
    let rings: Vec<Vec<Vec3>> = STATIONS
        .iter()
        .map(|&(s, chord, pitch)| {
            let x = root + (radius - root) * s;
            let (sin, cos) = pitch.to_radians().sin_cos();
            let chord_dir = Vec3::new(0.0, sin, -cos);
            let thickness_dir = Vec3::new(0.0, cos, sin);
            let c = chord * radius;
            shapes::aerofoil(
                Vec3::new(x, 0.0, 0.0) - chord_dir * (0.45 * c),
                chord_dir * c,
                thickness_dir * (c * 0.12).max(0.0015),
            )
        })
        .collect();
    let mut mesh = MeshBuilder::new();
    mesh.loft(&rings, true, Inside::Axis);
    mesh
}

/// Rest transform placing local `+Y` (the spin axis) along aircraft `+Z` or `-Z`.
fn vertical_spin_rest(hub: Vec3, spin_up: bool) -> DMat4 {
    let tilt = if spin_up {
        std::f64::consts::FRAC_PI_2
    } else {
        -std::f64::consts::FRAC_PI_2
    };
    DMat4::from_translation(hub.as_dvec3()) * DMat4::from_rotation_x(tilt)
}
