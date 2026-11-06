use glam::DQuat;
use sim_core::SimInstant;
use sim_geo::SensorIntrinsics;
use sim_scenario::RenderedSensorFrame;
use sim_scenery::{SyntheticCorridorSpec, render_luma_view};

use super::*;

const INTRINSICS: SensorIntrinsics = SensorIntrinsics {
    width: 96,
    height: 96,
    fx: 90.0,
    fy: 90.0,
    cx: 47.5,
    cy: 47.5,
};

fn reference() -> Arc<CatalogScenery> {
    let spec = SyntheticCorridorSpec {
        zoom: 14,
        tiles_east: 5,
        tiles_north: 3,
        ..SyntheticCorridorSpec::delaware()
    };
    Arc::new(CatalogScenery::synthetic_corridor(&spec).unwrap())
}

fn anchor() -> Geodetic {
    SyntheticCorridorSpec::delaware().south_west
}

/// A nadir camera at `enu` turned `yaw_rad` clockwise, image top forward.
fn nadir_pose(enu: DVec3, yaw_rad: f64) -> (DVec3, DQuat) {
    let origin = geodetic_to_ecef(&anchor());
    let position = origin + LocalFrame::at(&anchor()).enu_to_ecef() * enu;
    let frame = LocalFrame::at(&ecef_to_geodetic(position));
    let (sin, cos) = yaw_rad.sin_cos();
    let forward = frame.north() * cos + frame.east() * sin;
    let right = frame.east() * cos - frame.north() * sin;
    (
        position,
        DQuat::from_mat3(&DMat3::from_cols(right, forward, frame.up())),
    )
}

fn observe(
    catalog: &CatalogScenery,
    enu: DVec3,
    yaw_rad: f64,
    at: SimInstant,
) -> SensorObservation {
    let (eye, orientation) = nadir_pose(enu, yaw_rad);
    let image = render_luma_view(catalog, eye, orientation, &INTRINSICS).unwrap();
    let geodetic = ecef_to_geodetic(eye);
    let ground = catalog
        .ellipsoidal_ground_height_m(&geodetic)
        .unwrap()
        .unwrap()
        .0;
    SensorObservation {
        frame: RenderedSensorFrame {
            timestamp: at,
            intrinsics: INTRINSICS,
            image,
        },
        camera_to_ecef_estimate: orientation,
        agl_estimate_m: geodetic.height_m - ground,
    }
}

/// The eye 900 m above the ground under the anchor-frame point `east`,
/// `north`.
fn eye_over(
    estimator: &ImageCorridorEstimator,
    catalog: &CatalogScenery,
    east: f64,
    north: f64,
) -> DVec3 {
    let below = estimator.enu_to_geodetic(DVec3::new(east, north, 0.0));
    let ground = catalog
        .ellipsoidal_ground_height_m(&below)
        .unwrap()
        .unwrap()
        .0;
    estimator.geodetic_to_enu(&Geodetic {
        height_m: ground + 900.0,
        ..below
    })
}

/// A frame rendered at a known pose registers back onto that pose from a
/// prior well off it.
#[test]
fn an_anchor_registers_a_rendered_frame_onto_its_true_position() {
    let catalog = reference();
    let mut errors = Vec::new();
    for (east, north, yaw) in [
        (3_000.0, 1_500.0, 0.0),
        (4_200.0, 2_100.0, 1.4),
        (5_500.0, 900.0, -0.6),
    ] {
        let truth = DVec3::new(east, north, 0.0);
        let config = ImageEstimatorConfig::new(
            Arc::clone(&catalog),
            anchor(),
            truth + DVec3::new(150.0, -90.0, 0.0),
            120.0,
        );
        let mut estimator = ImageCorridorEstimator::new(config);
        let enu_eye = eye_over(&estimator, &catalog, east, north);
        let observation = observe(&catalog, enu_eye, yaw, SimInstant::START);
        let tiers = estimator.estimate(&observation);
        let fix = tiers.corridor_anchor.unwrap();
        let error = fix.position_enu_m - enu_eye;
        errors.push(error);
    }
    // Well inside the 15 m an anchor must reach: what remains with exact
    // attitude and height is sub-cell registration and relief.
    assert!(
        errors.iter().all(|e| e.truncate().length() < 5.0),
        "{errors:?}"
    );
}

/// Between anchors the warm track follows the ground's motion across
/// keyframe hops, and its claimed uncertainty covers its drift.
#[test]
fn the_warm_track_follows_motion_between_anchors() {
    let catalog = reference();
    let probe = ImageCorridorEstimator::new(ImageEstimatorConfig::new(
        Arc::clone(&catalog),
        anchor(),
        DVec3::ZERO,
        1.0,
    ));
    let start = eye_over(&probe, &catalog, 3_000.0, 1_500.0);
    let mut config = ImageEstimatorConfig::new(Arc::clone(&catalog), anchor(), start, 30.0);
    // One anchor at the start, then nothing but the track for 20 s.
    config.anchor_period = TickSpan::from_ticks(sim_core::TICKS_PER_SECOND * 60);
    let mut estimator = ImageCorridorEstimator::new(config);
    let step = TickSpan::from_ticks(sim_core::TICKS_PER_SECOND / 10);
    let mut at = SimInstant::START;
    let mut last = None;
    for frame in 0..200 {
        // 55 m/s north-east, turning the camera slowly as it goes.
        let travelled = 5.5 * f64::from(frame);
        let east = 3_000.0 + travelled * 0.8;
        let north = 1_500.0 + travelled * 0.6;
        let truth = eye_over(&probe, &catalog, east, north);
        let observation = observe(&catalog, truth, 0.002 * f64::from(frame), at);
        let tiers = estimator.estimate(&observation);
        if frame == 0 {
            assert!(tiers.corridor_anchor.is_some());
        } else {
            assert!(tiers.corridor_anchor.is_none());
        }
        let warm = tiers.warm_track.unwrap();
        last = Some((warm, truth));
        at = at.advanced(step);
    }
    let (warm, truth) = last.unwrap();
    let drift = (warm.position_enu_m - truth).truncate().length();
    let sigma = warm.covariance_enu_m2.x_axis.x.sqrt();
    assert!(
        drift < 3.0 * sigma && drift < 20.0,
        "drift {drift} sigma {sigma}"
    );
    assert_eq!(estimator.stats().warm_tracks, 200);
}
