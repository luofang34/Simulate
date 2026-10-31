use sim_geo::{BodyEllipsoid, Geodetic};

use super::*;
use crate::{
    Block, CameraMount, CameraPoseEcef, CameraSensorBlock, CameraSensorConfig, CameraSensorInput,
    EnuAnchor, SensorIntrinsics, VehicleState,
};

const THIRTIETH: TickSpan = TickSpan::from_ticks(TICKS_PER_SECOND / 30);

fn frame_at(index: u64, eye: DVec3) -> CameraFrameCapture {
    let vehicle =
        VehicleState::level_flight(&Geodetic::from_degrees(39.0, -75.5, 500.0), 0.0, 60.0);
    CameraFrameCapture {
        timestamp: SimInstant::START.advanced(TickSpan::from_ticks(THIRTIETH.ticks() * index)),
        vehicle,
        pose: CameraPoseEcef {
            position_ecef: vehicle.position_ecef,
            orientation: vehicle.attitude,
        },
        eye_enu: eye.to_array(),
        world_to_cam: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
        intrinsics: SensorIntrinsics {
            width: 800,
            height: 600,
            fx: 700.0,
            fy: 700.0,
            cx: 400.0,
            cy: 300.0,
        },
    }
}

fn estimate(position_enu_m: DVec3, sigma_m: f64) -> VisualEstimate {
    VisualEstimate {
        position_enu_m,
        covariance_enu_m2: DMat3::from_diagonal(DVec3::splat(sigma_m * sigma_m)),
    }
}

/// A frame every tier solved, with no estimation error.
fn solved(index: u64, eye: DVec3) -> VisualFrame {
    VisualFrame {
        capture: frame_at(index, eye),
        warm_track: Some(estimate(eye, 1.0)),
        corridor_anchor: Some(estimate(eye, 5.0)),
    }
}

fn harness() -> TwoTierVisualHarness {
    TwoTierVisualHarness::new(TwoTierConfig::default()).unwrap()
}

/// Runs frames through and collects every anchor fix in delivery order.
fn anchors_along(path: impl Iterator<Item = DVec3>) -> Vec<VisualFix> {
    let mut harness = harness();
    let mut last = SimInstant::START;
    for (index, eye) in (0u64..).zip(path) {
        let frame = solved(index, eye);
        last = frame.capture.timestamp;
        harness.submit(&frame).unwrap();
    }
    harness
        .take_due(last.advanced(TickSpan::SECOND))
        .into_iter()
        .filter(|fix| fix.map_cell.is_some())
        .collect()
}

#[test]
fn map_cells_floor_towards_negative_infinity() {
    let cell = |east, north| MapCell::containing(DVec3::new(east, north, 900.0), 500.0);
    let of = |cell_x, cell_y| MapCell { cell_x, cell_y };
    assert_eq!(cell(0.0, 0.0), of(0, 0));
    assert_eq!(cell(499.9, 500.0), of(0, 1));
    assert_eq!(cell(-0.1, -500.0), of(-1, -1));
    assert_eq!(cell(-500.1, 1_250.0), of(-2, 2));
}

#[test]
fn the_tiers_deliver_on_their_own_exact_delays() {
    let mut harness = harness();
    harness.submit(&solved(0, DVec3::ZERO)).unwrap();
    let start = SimInstant::START;
    let config = TwoTierConfig::default();
    assert_eq!(config.warm_delay, TickSpan::from_ticks(1_612_800_000));
    assert_eq!(config.anchor_delay, TickSpan::from_ticks(13_824_000_000));
    let tick = TickSpan::from_ticks(1);

    let warm_due = start.advanced(config.warm_delay);
    let before = TickSpan::from_ticks(config.warm_delay.ticks() - 1);
    assert!(harness.take_due(start.advanced(before)).is_empty());
    let warm = harness.take_due(warm_due);
    assert_eq!(warm.len(), 1);
    assert_eq!(warm[0].delivered_at, warm_due);
    assert_eq!(warm[0].eligibility, FusionEligibility::CorrelatedTrack);
    assert!(!warm[0].eligibility.independent_ekf_fix());
    assert_eq!(warm[0].map_cell, None);

    let anchor_due = start.advanced(config.anchor_delay);
    let before = TickSpan::from_ticks(config.anchor_delay.ticks() - 1);
    assert!(harness.take_due(start.advanced(before)).is_empty());
    let anchor = harness.take_due(anchor_due.advanced(tick));
    assert_eq!(anchor.len(), 1);
    assert_eq!(anchor[0].delivered_at, anchor_due);
    assert_eq!(anchor[0].eligibility, FusionEligibility::IndependentAnchor);
}

#[test]
fn thirty_frames_make_thirty_tracks_and_two_anchors() {
    let mut harness = harness();
    for index in 0..30 {
        harness.submit(&solved(index, DVec3::ZERO)).unwrap();
    }
    let fixes =
        harness.take_due(SimInstant::START.advanced(TickSpan::from_ticks(TICKS_PER_SECOND * 2)));
    let anchors = fixes.iter().filter(|fix| fix.map_cell.is_some()).count();
    assert_eq!(fixes.len() - anchors, 30);
    assert_eq!(anchors, 2);
    assert!(
        fixes
            .windows(2)
            .all(|p| p[0].delivered_at <= p[1].delivered_at)
    );
    let second_anchor = fixes.iter().rfind(|fix| fix.map_cell.is_some()).unwrap();
    assert_eq!(
        second_anchor.captured_at,
        SimInstant::START.advanced(TickSpan::from_ticks(TICKS_PER_SECOND / 2))
    );
}

#[test]
fn the_first_fix_in_a_cell_is_independent_and_loiter_is_shared() {
    let anchors = anchors_along((0..75).map(|_| DVec3::new(100.0, 100.0, 300.0)));
    assert_eq!(anchors.len(), 5);
    assert_eq!(anchors[0].eligibility, FusionEligibility::IndependentAnchor);
    assert!(anchors[0].eligibility.independent_ekf_fix());
    for anchor in &anchors[1..] {
        assert_eq!(anchor.eligibility, FusionEligibility::SharedMapCell);
        assert!(!anchor.eligibility.independent_ekf_fix());
        assert_eq!(anchor.map_cell, anchors[0].map_cell);
    }
}

#[test]
fn crossing_into_a_new_cell_earns_a_new_independent_fix_and_returning_does_not() {
    let east = |metres| DVec3::new(metres, 10.0, 300.0);
    // Dwell in three spots, half a second each: cell 0, cell 1, back to cell 0.
    let path = (0..15)
        .map(|_| east(100.0))
        .chain((0..15).map(|_| east(700.0)))
        .chain((0..15).map(|_| east(100.0)));
    let anchors = anchors_along(path);
    let labels: Vec<_> = anchors.iter().map(|fix| fix.eligibility).collect();
    assert_eq!(
        labels,
        [
            FusionEligibility::IndependentAnchor,
            FusionEligibility::IndependentAnchor,
            FusionEligibility::SharedMapCell,
        ]
    );
}

#[test]
fn an_anchor_is_filed_under_its_estimated_cell_as_navigate_does() {
    let mut harness = harness();
    let truth = DVec3::new(100.0, 100.0, 300.0);
    let frame = VisualFrame {
        capture: frame_at(0, truth),
        warm_track: None,
        corridor_anchor: Some(estimate(DVec3::new(600.0, 100.0, 300.0), 5.0)),
    };
    harness.submit(&frame).unwrap();
    let fixes = harness.take_due(SimInstant::START.advanced(TickSpan::SECOND));
    let cell = MapCell {
        cell_x: 1,
        cell_y: 0,
    };
    assert_eq!(fixes[0].map_cell, Some(cell));
}

#[test]
fn a_failed_anchor_does_not_use_up_its_cell() {
    let mut harness = harness();
    let eye = DVec3::new(100.0, 100.0, 300.0);
    let failed = VisualFrame {
        corridor_anchor: None,
        ..solved(0, eye)
    };
    harness.submit(&failed).unwrap();
    for index in 1..=15 {
        harness.submit(&solved(index, eye)).unwrap();
    }
    let fixes = harness.take_due(SimInstant::START.advanced(TickSpan::SECOND));
    let anchor = fixes.iter().find(|fix| fix.map_cell.is_some()).unwrap();
    assert_eq!(anchor.eligibility, FusionEligibility::IndependentAnchor);
}

#[test]
fn nees_weighs_the_error_at_the_exposure_by_the_claimed_covariance() {
    let mut harness = harness();
    let truth = DVec3::new(10.0, 20.0, 30.0);
    let frame = VisualFrame {
        capture: frame_at(0, truth),
        warm_track: Some(estimate(truth + DVec3::new(3.0, 0.0, 4.0), 2.0)),
        corridor_anchor: Some(VisualEstimate {
            position_enu_m: truth + DVec3::new(0.0, 6.0, 0.0),
            covariance_enu_m2: DMat3::from_diagonal(DVec3::new(1.0, 4.0, 9.0)),
        }),
    };
    harness.submit(&frame).unwrap();
    let fixes = harness.take_due(SimInstant::START.advanced(TickSpan::SECOND));
    assert!((fixes[0].position_nees - 25.0 / 4.0).abs() < 1e-12);
    assert!((fixes[1].position_nees - 9.0).abs() < 1e-12);
    assert_eq!(fixes[0].true_eye_enu_m, truth);

    let summary = harness.nees_summary(FusionEligibility::CorrelatedTrack);
    assert_eq!(summary.count, 1);
    assert!((summary.mean().unwrap() - 25.0 / 4.0).abs() < 1e-12);
    assert_eq!(
        harness
            .nees_summary(FusionEligibility::SharedMapCell)
            .mean(),
        None
    );
}

#[test]
fn an_overconfident_estimate_shows_up_in_the_mean_nees() {
    let mut harness = harness();
    for index in 0..30 {
        let eye = DVec3::new(50.0, 50.0, 300.0);
        let frame = VisualFrame {
            capture: frame_at(index, eye),
            warm_track: Some(estimate(eye + DVec3::X * 10.0, 1.0)),
            corridor_anchor: None,
        };
        harness.submit(&frame).unwrap();
    }
    assert!(harness.take_due(SimInstant::START).is_empty());
    harness.take_due(SimInstant::START.advanced(TickSpan::from_ticks(TICKS_PER_SECOND * 2)));
    let summary = harness.nees_summary(FusionEligibility::CorrelatedTrack);
    assert_eq!(summary.count, 30);
    assert!((summary.mean().unwrap() - 100.0).abs() < 1e-9);
    assert!((summary.max - 100.0).abs() < 1e-9);
}

#[test]
fn unusable_covariances_are_refused_and_change_nothing() {
    let mut harness = harness();
    let before = harness.clone();
    let singular = VisualEstimate {
        position_enu_m: DVec3::ZERO,
        covariance_enu_m2: DMat3::from_diagonal(DVec3::new(1.0, 0.0, 1.0)),
    };
    let indefinite = VisualEstimate {
        position_enu_m: DVec3::ZERO,
        covariance_enu_m2: DMat3::from_diagonal(DVec3::new(-1.0, -1.0, 1.0)),
    };
    for bad in [singular, indefinite] {
        let frame = VisualFrame {
            corridor_anchor: Some(bad),
            ..solved(0, DVec3::ZERO)
        };
        assert!(matches!(
            harness.submit(&frame),
            Err(VisualHarnessError::UnusableEstimate { .. })
        ));
    }
    assert_eq!(harness, before);
}

#[test]
fn frames_must_arrive_in_capture_order() {
    let mut harness = harness();
    harness.submit(&solved(3, DVec3::ZERO)).unwrap();
    assert!(matches!(
        harness.submit(&solved(3, DVec3::ZERO)),
        Err(VisualHarnessError::OutOfOrderFrame { .. })
    ));
    assert!(matches!(
        harness.submit(&solved(2, DVec3::ZERO)),
        Err(VisualHarnessError::OutOfOrderFrame { .. })
    ));
}

#[test]
fn a_cloned_harness_remembers_visited_cells_and_fixes_in_flight() {
    let mut harness = harness();
    let eye = DVec3::new(100.0, 100.0, 300.0);
    for index in 0..15 {
        harness.submit(&solved(index, eye)).unwrap();
    }
    let mut replay = harness.clone();
    let due = SimInstant::START.advanced(TickSpan::SECOND);
    harness.submit(&solved(15, eye)).unwrap();
    replay.submit(&solved(15, eye)).unwrap();
    assert_eq!(harness.take_due(due), replay.take_due(due));
    assert_eq!(harness, replay);
}

#[test]
fn bad_configurations_are_refused() {
    let zero_period = TwoTierConfig {
        anchor_period: TickSpan::ZERO,
        ..TwoTierConfig::default()
    };
    let no_cell = TwoTierConfig {
        map_cell_m: 0.0,
        ..TwoTierConfig::default()
    };
    assert!(TwoTierVisualHarness::new(zero_period).is_err());
    assert!(TwoTierVisualHarness::new(no_cell).is_err());
}

#[test]
fn the_camera_block_feeds_the_harness_with_consistent_truth() {
    let body = BodyEllipsoid::WGS84;
    let origin = Geodetic::from_degrees(39.0, -75.5, 0.0);
    let anchor = EnuAnchor::new(&body, &origin);
    let mut camera = CameraSensorBlock::new(CameraSensorConfig {
        mount: CameraMount::nadir_belly(DVec3::ZERO),
        intrinsics: frame_at(0, DVec3::ZERO).intrinsics,
        anchor,
        period: TwoTierConfig::default().warm_period,
    })
    .unwrap();
    let vehicle = VehicleState::level_flight(
        &Geodetic {
            height_m: 900.0,
            ..origin
        },
        0.0,
        60.0,
    );
    camera
        .step(&CameraSensorInput {
            vehicle,
            cg_body_m: DVec3::ZERO,
        })
        .unwrap();
    let capture = camera.output().unwrap();
    let truth = DVec3::from_array(capture.eye_enu);
    assert!((truth - DVec3::new(0.0, 0.0, 900.0)).length() < 1e-6);

    let mut harness = harness();
    let frame = VisualFrame {
        capture,
        warm_track: Some(estimate(truth, 1.0)),
        corridor_anchor: None,
    };
    harness.submit(&frame).unwrap();
    let fixes = harness.take_due(SimInstant::START.advanced(TickSpan::SECOND));
    assert_eq!(fixes[0].position_nees, 0.0);
}
