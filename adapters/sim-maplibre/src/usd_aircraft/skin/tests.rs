use super::{SkinFormat, SkinMap};

#[test]
fn mips_run_down_to_one_texel() {
    let map = SkinMap::with_mips(SkinFormat::Srgba8, 8, 4, vec![200; 8 * 4 * 4]);
    let sizes: Vec<usize> = map.levels().iter().map(Vec::len).collect();
    assert_eq!(sizes, [128, 32, 8, 4]);
    assert_eq!(map.byte_len(), 172);
}

#[test]
fn srgb_mips_average_in_linear_light() {
    // Black and white texels average to half the light, which sRGB encodes
    // near 188, not to the byte midpoint.
    let pixels = [[0, 0, 0, 255], [255, 255, 255, 255]].repeat(2).concat();
    let map = SkinMap::with_mips(SkinFormat::Srgba8, 2, 2, pixels);
    let coarse = map.levels().last().cloned().unwrap_or_default();
    assert_eq!(coarse, [188, 188, 188, 255]);
}

#[test]
fn normal_mips_shorten_toward_flat() {
    // Two opposite tilts average to a flat normal.
    let map = SkinMap::with_mips(SkinFormat::Normal8, 2, 1, vec![255, 128, 0, 128]);
    assert_eq!(map.levels().last().cloned().unwrap_or_default(), [128, 128]);
}

#[test]
fn oversized_maps_start_at_a_mip_that_fits() {
    use crate::usd_pass::first_fitting_level;
    assert_eq!(first_fitting_level(4096, 4096, 8192), 0);
    assert_eq!(first_fitting_level(4096, 4096, 2048), 1);
    assert_eq!(first_fitting_level(4096, 1024, 1024), 2);
}

use glam::{DMat4, Vec2, Vec3};

use super::{SkinLayout, SkinSample, SkinSource, SkinTexel, chart, pack};
use crate::{PartRole, PartSurface, PbrMaterial, SurfaceDetail, UsdMeshPart, UsdMeshVertex};

fn part(vertices: Vec<UsdMeshVertex>, indices: Vec<u32>) -> UsdMeshPart {
    UsdMeshPart {
        prim_path: "/Test/Skin".to_owned(),
        role: PartRole::CabinExterior,
        material: PbrMaterial::default(),
        double_sided: false,
        rest_to_aircraft: DMat4::IDENTITY,
        custom_hinge_axis: None,
        needle_channel: None,
        detail: SurfaceDetail::default(),
        surface: PartSurface::Material,
        vertices,
        indices,
    }
}

fn vertex(p: Vec3, n: Vec3, uv: Vec2) -> UsdMeshVertex {
    UsdMeshVertex {
        position: p.to_array(),
        normal: n.to_array(),
        uv: uv.to_array(),
        occlusion: 1.0,
        tangent: UsdMeshVertex::NO_TANGENT,
    }
}

/// A unit cube of six separate faces with no texture coordinates.
fn cube() -> UsdMeshPart {
    let mut vertices = Vec::new();
    let mut indices = Vec::new();
    for axis in [
        Vec3::X,
        Vec3::NEG_X,
        Vec3::Y,
        Vec3::NEG_Y,
        Vec3::Z,
        Vec3::NEG_Z,
    ] {
        let (u, v) = (
            axis.any_orthonormal_vector(),
            axis.cross(axis.any_orthonormal_vector()),
        );
        let base = u32::try_from(vertices.len()).unwrap();
        for (a, b) in [(-1.0, -1.0), (1.0, -1.0), (1.0, 1.0), (-1.0, 1.0)] {
            vertices.push(vertex(axis * 0.5 + (u * a + v * b) * 0.5, axis, Vec2::ZERO));
        }
        indices.extend([base, base + 1, base + 2, base, base + 2, base + 3]);
    }
    part(vertices, indices)
}

/// A tube of `sides` around `+y`, 2 m long, its seam column repeated as a
/// builder lays out a closed loft, with metric coordinates.
fn tube(sides: u32) -> UsdMeshPart {
    let mut vertices = Vec::new();
    let girth = std::f32::consts::TAU * 0.5;
    for ring in 0..2_u32 {
        for i in 0..=sides {
            let a = std::f32::consts::TAU * i as f32 / sides as f32;
            let n = Vec3::new(a.cos(), 0.0, a.sin());
            let p = n * 0.5 + Vec3::Y * ring as f32 * 2.0;
            vertices.push(vertex(
                p,
                n,
                Vec2::new(ring as f32 * 2.0, girth * i as f32 / sides as f32),
            ));
        }
    }
    let mut indices = Vec::new();
    for i in 0..sides {
        let (a, b, c, d) = (i, i + 1, sides + 1 + i, sides + 2 + i);
        indices.extend([a, b, d, a, d, c]);
    }
    part(vertices, indices)
}

#[test]
fn a_laid_out_tube_is_one_island_its_own_size() {
    let islands = chart::islands(&tube(16));
    assert_eq!(islands.len(), 1);
    let size = islands[0].size_m;
    assert!((size.x - 2.0).abs() < 1e-4 && (size.y - std::f32::consts::PI).abs() < 1e-3);
    assert_eq!(islands[0].triangles.len(), 32);
}

#[test]
fn faces_without_coordinates_are_projected_one_island_each() {
    let islands = chart::islands(&cube());
    assert_eq!(islands.len(), 6);
    for island in &islands {
        assert!(
            (island.size_m - Vec2::ONE).length() < 1e-4,
            "{:?}",
            island.size_m
        );
    }
}

#[test]
fn packed_islands_never_overlap_and_pack_the_same_way_twice() {
    let sizes: Vec<Vec2> = (0..40)
        .map(|i| Vec2::new(0.2 + (i % 7) as f32 * 0.3, 0.1 + (i % 5) as f32 * 0.25))
        .collect();
    let packing = pack::pack(&sizes, 512, 4).unwrap();
    assert_eq!(Some(&packing), pack::pack(&sizes, 512, 4).as_ref());
    let rects: Vec<(Vec2, Vec2)> = packing
        .placements
        .iter()
        .zip(&sizes)
        .map(|(p, s)| {
            (
                p.origin - Vec2::splat(4.0),
                p.origin + (*s * packing.texels_per_m).ceil() + Vec2::splat(4.0),
            )
        })
        .collect();
    for (i, a) in rects.iter().enumerate() {
        assert!(
            a.0.min_element() >= 0.0 && a.1.max_element() <= 512.0,
            "{a:?}"
        );
        for b in rects.iter().skip(i + 1) {
            let apart = a.1.x <= b.0.x || b.1.x <= a.0.x || a.1.y <= b.0.y || b.1.y <= a.0.y;
            assert!(apart, "{a:?} overlaps {b:?}");
        }
    }
}

/// Paints every texel the colour of its outward normal's axis.
struct Facing;

impl SkinSource for Facing {
    fn sample(&self, texel: &SkinTexel) -> SkinSample {
        SkinSample {
            albedo: texel.normal.abs(),
            occlusion: 1.0,
            roughness: 0.5,
            metallic: 0.0,
            normal: texel.normal,
        }
    }
}

#[test]
fn a_bake_paints_each_face_where_its_coordinates_point() {
    let mut parts = vec![cube()];
    let layout = SkinLayout {
        side: 256,
        gutter: 4,
        half_orm: true,
    };
    let skin = super::bake(&mut parts, &[0], layout, &Facing).unwrap();
    assert_eq!(skin.orm.size(), (128, 128));
    let part = &parts[0];
    assert_eq!(part.surface, PartSurface::Skin);
    let albedo = skin.albedo.levels()[0].clone();
    for triangle in part.indices.as_chunks::<3>().0 {
        let corners: Vec<&UsdMeshVertex> = triangle
            .iter()
            .map(|&i| &part.vertices[i as usize])
            .collect();
        let centre = corners.iter().fold(Vec2::ZERO, |s, v| s + Vec2::from(v.uv)) / 3.0;
        let texel = (centre * 256.0).floor();
        let at = (texel.y as usize * 256 + texel.x as usize) * 4;
        let n = Vec3::from(corners[0].normal).abs();
        // A face's own axis is lit full; the others stay dark.
        let lit: Vec<bool> = albedo[at..at + 3].iter().map(|&b| b > 200).collect();
        assert_eq!(lit, [n.x > 0.5, n.y > 0.5, n.z > 0.5]);
        // The flat map leaves the normal along the face: tangent-space z up.
        let normal =
            &skin.normal.levels()[0][(texel.y as usize * 256 + texel.x as usize) * 2..][..2];
        assert!(
            normal.iter().all(|&c| (126..=130).contains(&c)),
            "{normal:?}"
        );
    }
}
