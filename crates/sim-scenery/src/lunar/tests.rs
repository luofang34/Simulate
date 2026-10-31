use super::surface::ROCK_ALBEDO;
use super::*;

fn scenery() -> LunarTranquilityScenery {
    LunarTranquilityScenery::apollo_11_landing()
}

fn height(east: f64, north: f64) -> f64 {
    let site = scenery();
    site.ground_height_m(&site.at_local(east, north))
        .unwrap()
        .unwrap()
}

fn surface(east: f64, north: f64) -> SurfaceRadiometry {
    let site = scenery();
    site.surface_radiometry(&site.at_local(east, north))
        .unwrap()
        .unwrap()
}

/// Share of a square of ground, on a half-metre lattice, standing on a
/// boulder at least 0.3 m tall.
fn hazard_cover(east_start: f64, north_start: f64, side_m: f64) -> f64 {
    let count = (side_m * 2.0) as u32;
    let lattice = (0..count).flat_map(|i| (0..count).map(move |j| (i, j)));
    let hazards = lattice
        .filter(|&(i, j)| {
            let east = east_start + f64::from(i) * 0.5;
            let north = north_start + f64::from(j) * 0.5;
            ground_at(east, north).height_m - terrain_height_m(east, north) >= 0.3
        })
        .count();
    hazards as f64 / f64::from(count * count)
}

#[test]
fn the_site_is_at_the_published_coordinates() {
    let position = scenery().at_local(0.0, 0.0);
    assert!((position.lat_deg() - 0.67416).abs() < 1e-12);
    assert!((position.lon_deg() - 23.47314).abs() < 1e-12);
}

#[test]
fn west_crater_is_a_deep_bowl_with_a_raised_rim() {
    let plain = height(-1500.0, 800.0);
    let floor = height(WEST_CRATER.east_m, WEST_CRATER.north_m);
    assert!(plain - floor > 30.0, "floor {floor} plain {plain}");
    let radius = WEST_CRATER.diameter_m / 2.0;
    let rim = terrain_height_m(WEST_CRATER.east_m + radius, 0.0);
    assert!(rim - floor > 35.0, "rim {rim} floor {floor}");
    assert!(rim > plain + 4.0, "rim {rim} plain {plain}");
}

#[test]
fn little_west_crater_is_a_smaller_nearby_bowl() {
    let floor = terrain_height_m(LITTLE_WEST_CRATER.east_m, 0.0);
    let beside = terrain_height_m(LITTLE_WEST_CRATER.east_m, 60.0);
    let depth = beside - floor;
    assert!((5.0..9.0).contains(&depth), "depth {depth}");
    let west_depth = terrain_height_m(-1500.0, 800.0) - terrain_height_m(WEST_CRATER.east_m, 0.0);
    assert!(depth < west_depth / 3.0);
}

#[test]
fn the_ejecta_is_a_boulder_field_where_the_pad_is_clear() {
    assert_eq!(hazard_cover(-30.0, -30.0, 60.0), 0.0);
    let radius = WEST_CRATER.diameter_m / 2.0;
    let ejecta = hazard_cover(WEST_CRATER.east_m + 1.1 * radius, -30.0, 60.0);
    assert!(ejecta > 0.05, "ejecta cover {ejecta}");
    let plain = hazard_cover(-1500.0, 800.0, 60.0);
    assert!(plain < 0.01, "plain cover {plain}");
}

#[test]
fn boulders_stand_on_the_terrain_rather_than_replacing_it() {
    let radius = WEST_CRATER.diameter_m / 2.0;
    let mut found = 0;
    for step in 0..2000 {
        let east = WEST_CRATER.east_m + radius * 1.05 + f64::from(step) * 0.05;
        let ground = ground_at(east, 0.0);
        let terrain = terrain_height_m(east, 0.0);
        assert!(ground.height_m >= terrain);
        if ground.height_m > terrain {
            found += 1;
            assert_eq!(ground.albedo, ROCK_ALBEDO);
        }
    }
    assert!(found > 10);
}

#[test]
fn the_ejecta_fades_into_the_plain_without_an_edge() {
    let radius = WEST_CRATER.diameter_m / 2.0;
    let albedo = |metres: f64| {
        let north = WEST_CRATER.north_m - radius * 1.2 - metres;
        (0..24)
            .map(|lane| ground_at(WEST_CRATER.east_m + f64::from(lane) * 0.25, north))
            .find(|ground| ground.emissivity > 0.92)
            .map_or(0.0, |ground| ground.albedo)
    };
    let steps = (0..200).map(|m| (albedo(f64::from(m)) - albedo(f64::from(m + 1))).abs());
    assert!(steps.fold(0.0, f32::max) < 0.002);
    assert!(albedo(0.0) > albedo(200.0) + 0.02);
}

#[test]
fn the_crater_floor_is_in_cold_shadow_while_the_plain_is_warm() {
    let plain = surface(-1500.0, 800.0);
    let floor = surface(WEST_CRATER.east_m, WEST_CRATER.north_m);
    assert_eq!(floor.temperature_k, SHADOW_TEMPERATURE_K);
    // Flat ground under a 10.8° Sun.
    assert!(
        (235.0..275.0).contains(&plain.temperature_k),
        "{}",
        plain.temperature_k
    );
    assert!(plain.exitance_w_m2() > 15.0 * floor.exitance_w_m2());
}

#[test]
fn the_wall_facing_the_sun_is_hotter_than_the_plain() {
    // The Sun is in the east, so the western inner wall faces it.
    let wall = surface(WEST_CRATER.east_m - 75.0, WEST_CRATER.north_m);
    let plain = surface(-1500.0, 800.0);
    assert!(
        wall.temperature_k > plain.temperature_k + 40.0,
        "wall {} plain {}",
        wall.temperature_k,
        plain.temperature_k
    );
}

#[test]
fn a_high_sun_clears_the_shadow_and_warms_the_plain() {
    let noon = LunarTranquilityScenery::new(90.0, 85.0);
    let floor = noon
        .surface_radiometry(&noon.at_local(WEST_CRATER.east_m, 0.0))
        .unwrap()
        .unwrap();
    assert!(floor.temperature_k > 330.0, "{}", floor.temperature_k);
}

#[test]
fn the_sun_below_the_horizon_leaves_everything_cold() {
    let night = LunarTranquilityScenery::new(90.0, -5.0);
    let ground = night
        .surface_radiometry(&night.at_local(-1500.0, 800.0))
        .unwrap()
        .unwrap();
    assert_eq!(ground.temperature_k, SHADOW_TEMPERATURE_K);
}

#[test]
fn outside_the_coverage_nothing_is_known() {
    let site = scenery();
    let far = site.at_local(4100.0, 0.0);
    assert_eq!(site.ground_height_m(&far), Ok(None));
    assert_eq!(site.surface_radiometry(&far), Ok(None));
}

#[test]
fn queries_are_repeatable() {
    assert_eq!(height(512.25, -37.5), height(512.25, -37.5));
    assert_eq!(surface(512.25, -37.5), surface(512.25, -37.5));
}
