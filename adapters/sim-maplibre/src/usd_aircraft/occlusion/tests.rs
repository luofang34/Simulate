use glam::{DMat4, Vec3};

use super::{bake, bounding_radius};
use crate::{PartRole, PbrMaterial, UsdMeshPart, UsdMeshVertex};

fn part_from(triangles: &[[Vec3; 3]], normal: Vec3) -> UsdMeshPart {
    let vertices = triangles
        .iter()
        .flatten()
        .map(|p| UsdMeshVertex {
            position: p.to_array(),
            normal: normal.to_array(),
            uv: [0.0, 0.0],
            occlusion: 1.0,
            tangent: crate::UsdMeshVertex::NO_TANGENT,
        })
        .collect::<Vec<_>>();
    UsdMeshPart {
        prim_path: "/Test".to_owned(),
        role: PartRole::CabinExterior,
        material: PbrMaterial::default(),
        double_sided: false,
        rest_to_aircraft: DMat4::IDENTITY,
        custom_hinge_axis: None,
        needle_channel: None,
        detail: crate::usd_aircraft::SurfaceDetail::default(),
        surface: crate::usd_aircraft::PartSurface::Material,
        indices: (0..vertices.len() as u32).collect(),
        vertices,
    }
}

/// A 4 m floor with sample points near and far from a 1 m wall standing on it.
fn floor_and_wall() -> Vec<UsdMeshPart> {
    let floor = part_from(
        &[
            [
                Vec3::new(-2.0, -2.0, 0.0),
                Vec3::new(2.0, -2.0, 0.0),
                Vec3::new(2.0, 2.0, 0.0),
            ],
            [
                Vec3::new(-2.0, -2.0, 0.0),
                Vec3::new(2.0, 2.0, 0.0),
                Vec3::new(-2.0, 2.0, 0.0),
            ],
        ],
        Vec3::Z,
    );
    let wall = part_from(
        &[
            [
                Vec3::new(0.0, -2.0, 0.0),
                Vec3::new(0.0, 2.0, 0.0),
                Vec3::new(0.0, 2.0, 1.0),
            ],
            [
                Vec3::new(0.0, -2.0, 0.0),
                Vec3::new(0.0, 2.0, 1.0),
                Vec3::new(0.0, -2.0, 1.0),
            ],
        ],
        Vec3::NEG_X,
    );
    let probes = part_from(
        &[[
            Vec3::new(-0.05, 0.0, 0.0),
            Vec3::new(-1.9, 0.0, 0.0),
            Vec3::new(-1.0, 0.0, 0.0),
        ]],
        Vec3::Z,
    );
    vec![floor, wall, probes]
}

#[test]
fn a_corner_is_darker_than_open_floor() {
    let mut parts = floor_and_wall();
    bake(&mut parts);
    let probes = &parts[2].vertices;
    let (corner, open) = (probes[0].occlusion, probes[1].occlusion);
    assert!(corner < 0.75, "corner {corner}");
    assert!(open > 0.97, "open {open}");
    assert!(corner < probes[2].occlusion && probes[2].occlusion <= open);
}

#[test]
fn a_convex_hull_never_occludes_itself() {
    let mut parts = vec![part_from(
        &[[
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
        ]],
        Vec3::Z,
    )];
    bake(&mut parts);
    assert!(parts[0].vertices.iter().all(|v| v.occlusion == 1.0));
    assert!((bounding_radius(&parts) - 1.0).abs() < 1e-6);
}

#[test]
fn the_p38_bakes_dark_junctions_and_open_tops() {
    let asset = crate::UsdAircraftAsset::p38g().expect("p38g");
    let skin = |path: &str| {
        asset
            .parts()
            .iter()
            .find(|p| p.prim_path == path)
            .expect(path)
    };
    let wing = skin("/P38g/Wing/UpperOlive");
    // Medians: the wing skin runs on inside the gondola and booms, where it is buried.
    let median = |part: &UsdMeshPart| {
        let mut values: Vec<f32> = part.vertices.iter().map(|v| v.occlusion).collect();
        values.sort_by(f32::total_cmp);
        values[values.len() / 2]
    };
    let darkest = wing
        .vertices
        .iter()
        .map(|v| v.occlusion)
        .fold(1.0, f32::min);
    assert!(median(wing) > 0.85, "upper wing median {}", median(wing));
    assert!(
        darkest < 0.7,
        "the wing roots meet the gondola and booms: {darkest}"
    );
    let wells = skin("/P38g/Gear/WheelWells");
    assert!(
        median(wells) < median(wing),
        "wheel wells {}",
        median(wells)
    );
}
