use super::*;
use glam::Vec3;
use sim_geo::Geodetic;
use sim_scenery::{
    CatalogScenery, LunarTranquilityScenery, SceneryError, SurfaceRadiometry, TileId,
    tile_point_to_geodetic,
};

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
        let back = sim_geo::ecef_to_geodetic(ecef);
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

/// Ground that slopes up to the east, with a known height everywhere.
struct Ramp;

impl ElevationSource for Ramp {
    fn ground_height_m(&self, position: &Geodetic) -> Result<Option<f64>, SceneryError> {
        Ok(Some(100.0 + position.lon_deg() * 1000.0))
    }
}

fn delaware_block(body: BodyEllipsoid) -> TerrainRegion {
    TerrainRegion {
        body,
        south_rad: 39.0_f64.to_radians(),
        west_rad: (-75.6_f64).to_radians(),
        lat_span_rad: 0.1_f64.to_radians(),
        lon_span_rad: 0.2_f64.to_radians(),
        lat_patches: 2,
        lon_patches: 4,
        grid: 8,
    }
}

fn reconstructed(mesh: &GlobeMesh, vertex: &GlobeVertex) -> DVec3 {
    let centre = mesh.patch_centres[vertex[8] as usize];
    centre + Vec3::new(vertex[0], vertex[1], vertex[2]).as_dvec3()
}

#[test]
fn displacement_follows_the_ellipsoid_normal_to_the_sampled_height() {
    for body in [BodyEllipsoid::WGS84, BodyEllipsoid::MOON_IAU] {
        let sources = TerrainSources {
            elevation: Some(&Ramp),
            ..Default::default()
        };
        let mesh = GlobeMesh::build_region(&delaware_block(body), sources).unwrap();
        for vertex in mesh.vertices.iter().step_by(7) {
            let back = body.fixed_to_geodetic(reconstructed(&mesh, vertex));
            let expected = 100.0 + back.lon_deg() * 1000.0;
            assert!(
                (back.height_m - expected).abs() < 0.02,
                "height {} expected {expected}",
                back.height_m
            );
        }
    }
}

#[test]
fn patch_offsets_from_a_distant_eye_lose_nothing_to_f32() {
    // The eye is a low orbit away and moves by a millimetre; narrowing the
    // absolute position instead would quantise this to half a metre.
    let body = BodyEllipsoid::WGS84;
    let sources = TerrainSources {
        elevation: Some(&Ramp),
        ..Default::default()
    };
    let mesh = GlobeMesh::build_region(&delaware_block(body), sources).unwrap();
    let centre = mesh.patch_centres[3];
    let eye = centre + centre.normalize() * 2_500.0;
    let moved = eye + DVec3::new(0.001, 0.0, 0.0);
    let offset = (centre - eye).as_vec3();
    let offset_moved = (centre - moved).as_vec3();
    let step = (offset - offset_moved).as_dvec3();
    assert!((step.x - 0.001).abs() < 2e-4, "{step:?}");
    assert!(centre.length() > 6.3e6, "the centre stays in f64");
}

#[test]
fn vertices_stay_small_next_to_their_patch_centre() {
    let site = LunarTranquilityScenery::apollo_11_landing();
    let centre = site.at_local(0.0, 0.0);
    let span = 500.0 / BodyEllipsoid::MOON_IAU.semi_major_m;
    let region = TerrainRegion {
        body: BodyEllipsoid::MOON_IAU,
        south_rad: centre.lat_rad - span,
        west_rad: centre.lon_rad - span,
        lat_span_rad: 2.0 * span,
        lon_span_rad: 2.0 * span,
        lat_patches: 2,
        lon_patches: 2,
        grid: 32,
    };
    let sources = TerrainSources {
        elevation: Some(&site),
        radiometry: Some(&site),
        ..Default::default()
    };
    let mesh = GlobeMesh::build_region(&region, sources).unwrap();
    for vertex in &mesh.vertices {
        let relative = Vec3::new(vertex[0], vertex[1], vertex[2]);
        assert!(relative.length() < 800.0, "{relative:?}");
    }
    let floor = mesh
        .vertices
        .iter()
        .map(|v| {
            BodyEllipsoid::MOON_IAU
                .fixed_to_geodetic(reconstructed(&mesh, v))
                .height_m
        })
        .fold(f64::INFINITY, f64::min);
    assert!(floor < -25.0, "West Crater's floor is at {floor} m");
    let exitances = mesh.vertices.iter().map(|v| v[12]);
    let hottest = exitances.clone().fold(0.0, f32::max);
    let coldest = exitances.fold(f32::INFINITY, f32::min);
    assert!(coldest > 0.0, "every vertex has radiometry");
    assert!(hottest > 15.0 * coldest, "{coldest}..{hottest} W/m²");
}

#[test]
fn a_vertex_without_elevation_fails_instead_of_going_to_sea_level() {
    let mut catalog = CatalogScenery::new();
    let tile = TileId {
        zoom: 8,
        x: 74,
        y: 97,
    };
    let raster = sim_scenery::TerrariumTile::from_heights(1, 1, vec![50.0]).unwrap();
    catalog.insert_elevation(tile, raster).unwrap();
    let inside = tile_point_to_geodetic(tile, 0.5, 0.5).unwrap();
    let mut region = TerrainRegion {
        south_rad: inside.lat_rad,
        west_rad: inside.lon_rad,
        lat_span_rad: 0.001,
        lon_span_rad: 0.001,
        ..delaware_block(BodyEllipsoid::WGS84)
    };
    let sources = TerrainSources {
        elevation: Some(&catalog),
        ..Default::default()
    };
    assert!(GlobeMesh::build_region(&region, sources).is_ok());
    region.lon_span_rad = 0.2;
    let err = GlobeMesh::build_region(&region, sources).unwrap_err();
    assert!(matches!(err, RenderError::UncoveredTerrain { .. }), "{err}");
}

#[test]
fn a_tilted_surface_tilts_the_normal_and_patches_agree_along_seams() {
    let sources = TerrainSources {
        elevation: Some(&Ramp),
        ..Default::default()
    };
    let mesh = GlobeMesh::build_region(&delaware_block(BodyEllipsoid::WGS84), sources).unwrap();
    let frame = sim_geo::LocalFrame::at(&Geodetic::from_degrees(39.0, -75.6, 0.0));
    let normal = |v: &GlobeVertex| Vec3::new(v[3], v[4], v[5]).as_dvec3();
    // A rise of 1000 m per degree, about 86 km, up to the east.
    let tilt = normal(&mesh.vertices[0]).dot(frame.east());
    assert!((tilt + 0.0116).abs() < 2e-3, "tilts west by {tilt}");
    assert!(normal(&mesh.vertices[0]).dot(frame.up()) > 0.99);
    // Vertex 8 ends the first row of patch 0; 81 starts the first of patch 1.
    let (edge, next) = (mesh.vertices[8], mesh.vertices[81]);
    assert_eq!((edge[6], edge[7]), (next[6], next[7]), "same ground");
    assert!((normal(&edge) - normal(&next)).length() < 1e-6);
}

#[test]
fn missing_radiometry_and_ortho_are_marked_as_no_data() {
    struct Patchy;
    impl RadiometricSource for Patchy {
        fn surface_radiometry(
            &self,
            position: &Geodetic,
        ) -> Result<Option<SurfaceRadiometry>, SceneryError> {
            Ok((position.lon_deg() > -75.5).then_some(SurfaceRadiometry {
                albedo: 0.2,
                emissivity: 1.0,
                temperature_k: 300.0,
            }))
        }
    }
    let sources = TerrainSources {
        radiometry: Some(&Patchy),
        ..Default::default()
    };
    let mesh = GlobeMesh::build_region(&delaware_block(BodyEllipsoid::WGS84), sources).unwrap();
    let first = mesh.vertices.first().unwrap();
    assert_eq!(
        (first[9], first[10], first[11], first[12]),
        (-1.0, -1.0, -1.0, -1.0)
    );
    let last = mesh.vertices.last().unwrap();
    assert_eq!(last[11], 0.2);
    assert!((last[12] - 459.3).abs() < 0.1, "{}", last[12]);
}

#[test]
fn rejects_regions_with_no_area_or_tessellation() {
    let good = delaware_block(BodyEllipsoid::WGS84);
    let bad = [
        TerrainRegion {
            lat_span_rad: 0.0,
            ..good
        },
        TerrainRegion {
            lon_span_rad: -1.0,
            ..good
        },
        TerrainRegion {
            south_rad: f64::NAN,
            ..good
        },
        TerrainRegion {
            lat_patches: 0,
            ..good
        },
        TerrainRegion { grid: 0, ..good },
    ];
    for region in bad {
        let err = GlobeMesh::build_region(&region, TerrainSources::default()).unwrap_err();
        assert!(matches!(err, RenderError::EmptyRegion { .. }), "{err}");
    }
    let huge = TerrainRegion {
        lat_patches: 64,
        lon_patches: 64,
        ..good
    };
    let err = GlobeMesh::build_region(&huge, TerrainSources::default()).unwrap_err();
    assert!(
        matches!(
            err,
            RenderError::TooManyPatches {
                requested: 4096,
                ..
            }
        ),
        "{err}"
    );
}

#[test]
fn rejects_a_mesh_its_indices_cannot_address() {
    let dense = TerrainRegion {
        grid: 70_000,
        ..delaware_block(BodyEllipsoid::WGS84)
    };
    let err = GlobeMesh::build_region(&dense, TerrainSources::default()).unwrap_err();
    assert!(matches!(err, RenderError::TooManyVertices { .. }), "{err}");
}

#[test]
fn appending_a_mesh_renumbers_its_vertices_and_patches() {
    let mut first = GlobeMesh::build(2, 2, 2).unwrap();
    let second = GlobeMesh::build(2, 1, 2).unwrap();
    let (vertices, patches) = (first.vertices.len(), first.patch_centres.len());

    first.append(&second).unwrap();

    assert_eq!(first.vertices.len(), vertices + second.vertices.len());
    assert_eq!(
        first.patch_centres.len(),
        patches + second.patch_centres.len()
    );
    let appended = &first.vertices[vertices];
    assert_eq!(appended[PATCH_INDEX_FLOAT], patches as f32);
    assert_eq!(
        first.indices.iter().max().copied().unwrap() as usize,
        first.vertices.len() - 1
    );
    assert_eq!(first.vertices[vertices][..8], second.vertices[0][..8]);
}

#[test]
fn appending_past_the_patch_table_is_refused_and_changes_nothing() {
    let mut first = GlobeMesh::build(12, 24, 1).unwrap();
    let second = GlobeMesh::build(12, 24, 1).unwrap();
    let before = first.vertices.len();
    assert!(matches!(
        first.append(&second),
        Err(RenderError::TooManyPatches { requested: 576, .. })
    ));
    assert_eq!(first.vertices.len(), before);
}
