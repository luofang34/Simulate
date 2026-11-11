use navigate_data::{GroundHeight, LayerKind};

use crate::fixtures::{CIRCUIT, camera};
use crate::{
    DEVIATION_M, ELEVATION_OFFSET, MAX_RANGE_M, NavdataError, SEARCH_RADIUS_M, SourceSet,
    plan_package, planned_camera,
};

/// The same ground everywhere.
struct Flat(f64);

impl GroundHeight for Flat {
    fn ground_m(&self, _point: [f64; 2]) -> Option<f64> {
        Some(self.0)
    }
}

/// No ground known.
struct Unknown;

impl GroundHeight for Unknown {
    fn ground_m(&self, _point: [f64; 2]) -> Option<f64> {
        None
    }
}

#[test]
fn the_plan_numbers_are_the_desktop_runs() {
    assert_eq!(
        (SEARCH_RADIUS_M, DEVIATION_M, MAX_RANGE_M),
        (500.0, 500.0, 4_000.0)
    );
    assert_eq!(ELEVATION_OFFSET, 3);
    let plan = plan_package(&CIRCUIT, camera(), &SourceSet::default(), &Flat(300.0)).unwrap();
    assert_eq!(
        (
            plan.request.search_radius_m,
            plan.request.deviation_m,
            plan.request.max_range_m
        ),
        (500.0, 500.0, 4_000.0)
    );
    assert_eq!(plan.request.elevation_offset, 3);
}

#[test]
fn the_focal_length_comes_from_the_horizontal_field_of_view() {
    let camera = planned_camera(800, 600, 90.0, 90.0);
    assert!((camera.focal_px - 400.0).abs() < 1e-9);
    assert_eq!((camera.width_px, camera.height_px), (800, 600));
    assert_eq!(camera.depression_rad, std::f64::consts::FRAC_PI_2);
    assert!((planned_camera(400, 300, 67.4, 45.0).focal_px - 299.85).abs() < 0.05);
}

#[test]
fn the_circuit_at_two_thousand_metres_needs_zoom_fifteen_imagery() {
    let sources = SourceSet::default();
    let plan = plan_package(&CIRCUIT, camera(), &sources, &Flat(300.0)).unwrap();
    assert_eq!(plan.resources.legs.len(), 4);
    for leg in &plan.resources.legs {
        assert_eq!(leg.agl_m, 1_700.0);
        let zooms: Vec<(LayerKind, u32)> = leg.bands.iter().map(|b| (b.layer, b.zoom)).collect();
        assert_eq!(
            zooms,
            [(LayerKind::Imagery, 15), (LayerKind::Elevation, 12)]
        );
    }
    // The desktop package of this route and camera, over the real ground,
    // holds 506 imagery and 15 elevation tiles.
    let imagery = plan.tiles[&LayerKind::Imagery].len();
    let elevation = plan.tiles[&LayerKind::Elevation].len();
    assert!((450..=560).contains(&imagery), "{imagery}");
    assert!((10..=20).contains(&elevation), "{elevation}");
    assert_eq!(plan.tile_count(), imagery + elevation);
    assert_eq!(plan.keys(&sources).len(), plan.tile_count());
    assert_eq!(
        plan.size.upper_bytes,
        imagery as u64 * 20_000 + elevation as u64 * 110_000
    );
    assert!(plan.size.estimate_bytes < plan.size.upper_bytes);
}

#[test]
fn a_leg_from_a_start_position_adds_its_tiles() {
    let sources = SourceSet::default();
    let circuit = plan_package(&CIRCUIT, camera(), &sources, &Flat(300.0)).unwrap();
    let mut from_start = vec![[41.7, -1.2, 2000.0]];
    from_start.extend(CIRCUIT);
    let with_start = plan_package(&from_start, camera(), &sources, &Flat(300.0)).unwrap();
    assert_eq!(with_start.resources.legs.len(), 5);
    assert!(with_start.tile_count() > circuit.tile_count() + 50);
    assert!(circuit.tiles[&LayerKind::Imagery].is_subset(&with_start.tiles[&LayerKind::Imagery]));
}

#[test]
fn a_larger_image_needs_finer_and_more_tiles() {
    let sources = SourceSet::default();
    let small = plan_package(&CIRCUIT, camera(), &sources, &Flat(300.0)).unwrap();
    let large = plan_package(
        &CIRCUIT,
        planned_camera(800, 600, 67.4, 90.0),
        &sources,
        &Flat(300.0),
    )
    .unwrap();
    assert!(
        large
            .resources
            .legs
            .iter()
            .all(|leg| leg.bands[0].zoom == 16)
    );
    assert!(large.tile_count() > 3 * small.tile_count());
}

#[test]
fn the_budget_is_judged_on_the_upper_bound() {
    let plan = plan_package(&CIRCUIT, camera(), &SourceSet::default(), &Flat(300.0)).unwrap();
    let upper = plan.size.upper_bytes;
    assert_eq!(plan.within(upper), Ok(()));
    assert_eq!(
        plan.within(upper - 1),
        Err(NavdataError::OverBudget {
            needed_bytes: upper,
            budget_bytes: upper - 1,
            tiles: plan.tile_count(),
        })
    );
    assert!(plan.size.estimate_bytes < upper - 1);
}

#[test]
fn a_route_navigate_cannot_plan_is_refused_with_its_reason() {
    let sources = SourceSet::default();
    let unknown = plan_package(&CIRCUIT, camera(), &sources, &Unknown).unwrap_err();
    assert!(
        unknown.to_string().contains("invalid ground: unknown at"),
        "{unknown}"
    );
    let buried = plan_package(&CIRCUIT, camera(), &sources, &Flat(2_500.0)).unwrap_err();
    assert!(
        buried
            .to_string()
            .contains("invalid leg: -500 m above ground"),
        "{buried}"
    );
    let single = plan_package(&CIRCUIT[..1], camera(), &sources, &Flat(300.0)).unwrap_err();
    assert!(matches!(single, NavdataError::Data { .. }));
}
