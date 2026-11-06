use sim_render::MAX_PATCHES;

use sim_geo::LocalFrame;

use super::*;

#[test]
fn delaware_is_displaced_and_draped_on_a_globe_below_it() {
    let scene = ScenarioWorld::DelawareCorridor.scene().unwrap();
    let globe = whole_body_at(BodyEllipsoid::WGS84, -EARTH_GLOBE_SINK_M, None).unwrap();
    let atlas = scene.atlas.as_ref().unwrap();

    assert!(scene.mesh.patch_centres.len() <= MAX_PATCHES);
    assert_eq!(
        scene.mesh.patch_centres.len(),
        globe.patch_centres.len() + (DELAWARE_LAT_PATCHES * DELAWARE_LON_PATCHES) as usize
    );
    assert_eq!((atlas.width, atlas.height), (768, 512));
    let (_, terrain) = scene.mesh.vertices.split_at(globe.vertices.len());
    assert!(
        terrain
            .iter()
            .all(|v| (0.0..=1.0).contains(&v[9]) && (0.0..=1.0).contains(&v[10]))
    );
}

#[test]
fn all_of_the_moon_carries_radiometry_so_none_draws_as_earth() {
    let scene = ScenarioWorld::LunarTranquility.scene().unwrap();
    let globe = whole_body_at(BodyEllipsoid::MOON_IAU, -MOON_GLOBE_SINK_M, None).unwrap();
    assert!(scene.atlas.is_none());
    assert!(scene.mesh.vertices.len() > globe.vertices.len());
    assert!(
        scene
            .mesh
            .vertices
            .iter()
            .all(|v| v[11] > 0.0 && v[12] >= 0.0),
        "albedo and exitance"
    );
}

#[test]
fn the_earth_globe_is_the_bare_ellipsoid() {
    let scene = ScenarioWorld::EarthGlobe.scene().unwrap();
    let bare = GlobeMesh::build(GLOBE_LAT_PATCHES, GLOBE_LON_PATCHES, GLOBE_PATCH_GRID).unwrap();
    assert_eq!(scene.mesh.vertices.len(), bare.vertices.len());
    assert!(scene.atlas.is_none());
}

#[test]
fn each_world_is_on_its_body_with_the_sun_up() {
    assert_eq!(
        ScenarioWorld::LunarTranquility.body(),
        BodyEllipsoid::MOON_IAU
    );
    assert_eq!(ScenarioWorld::DelawareCorridor.body(), BodyEllipsoid::WGS84);

    let site = SyntheticCorridorSpec::delaware().south_west;
    let sun = ScenarioWorld::DelawareCorridor.sun_direction_ecef();
    assert!(sun.dot(LocalFrame::at(&site).up()) > 0.5);

    let site = LunarTranquilityScenery::apollo_11_landing().at_local(0.0, 0.0);
    let frame = LocalFrame::at(&site);
    let sun = ScenarioWorld::LunarTranquility.sun_direction_ecef();
    assert!((sun.dot(frame.up()) - 10.8_f64.to_radians().sin()).abs() < 1e-9);
    assert!(sun.dot(frame.east()) > 0.9, "a morning Sun in the east");
}
