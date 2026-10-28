use super::*;
use glam::Vec3;

#[test]
fn rejects_layouts_beyond_the_shader_table() {
    assert!(GlobeMesh::build(32, 32, 4).is_none());
    assert!(GlobeMesh::build(0, 8, 4).is_none());
    assert!(GlobeMesh::build(4, 8, 0).is_none());
}

#[test]
fn indices_stay_in_bounds_and_counts_match_layout() {
    let mesh = GlobeMesh::build(4, 8, 3).expect("layout fits");
    assert_eq!(mesh.patch_centres.len(), 32);
    assert_eq!(mesh.vertices.len(), 32 * 16);
    assert_eq!(mesh.indices.len(), 32 * 9 * 6);
    let count = u32::try_from(mesh.vertices.len()).unwrap();
    assert!(mesh.indices.iter().all(|&i| i < count));
}

#[test]
fn vertices_reconstruct_to_the_ellipsoid_within_centimetres() {
    let mesh = GlobeMesh::build(12, 24, 6).expect("layout fits");
    for v in mesh.vertices.iter().step_by(17) {
        let centre = mesh.patch_centres[v[8] as usize];
        let ecef = centre + Vec3::new(v[0], v[1], v[2]).as_dvec3();
        let back = fsg_geo::ecef_to_geodetic(ecef);
        assert!(
            back.height_m.abs() < 0.05,
            "height error {} m",
            back.height_m
        );
    }
}

#[test]
fn triangles_wind_counter_clockwise_from_space() {
    let mesh = GlobeMesh::build(6, 12, 2).expect("layout fits");
    for triangle in mesh.indices.as_chunks::<3>().0 {
        let [a, b, c] = [triangle[0], triangle[1], triangle[2]].map(|i| mesh.vertices[i as usize]);
        let pos = |v: GlobeVertex| Vec3::new(v[0], v[1], v[2]);
        let face = (pos(b) - pos(a)).cross(pos(c) - pos(a));
        let normal = Vec3::new(a[3], a[4], a[5]);
        // Degenerate pole triangles have no area and no orientation.
        if face.length() > 1.0 {
            assert!(face.dot(normal) > 0.0);
        }
    }
}
