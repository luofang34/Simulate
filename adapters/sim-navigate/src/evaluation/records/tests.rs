use glam::DVec3;
use navigate_contract::{GeodeticPosition, SymmetricCov3};
use sim_core::{CameraMount, EnuAnchor, TICKS_PER_SECOND};
use sim_geo::{BodyEllipsoid, Geodetic, SensorIntrinsics};

use super::*;
use crate::{AblationSet, NavMeans};

#[test]
fn reversed_z_depth_becomes_range_and_sky_is_masked() {
    let near_m = 0.5;
    let depth = [0.5_f32, 0.0, 0.001, 0.25];
    let truth = ImageTruth::from_reversed_z(2, 2, &depth, near_m).unwrap();
    assert_eq!(truth.valid_mask, vec![255, 0, 255, 255]);
    assert_eq!(truth.depth_m.first(), Some(&1.0));
    assert_eq!(truth.depth_m.get(1), Some(&f32::INFINITY));
    assert!((truth.depth_m[2] - 500.0).abs() < 1e-3);
    // (0.4, 0.6) rounds to column 0, row 1.
    assert_eq!(
        truth.optical_axis_depth_m(0.4, 0.6),
        Some(f64::from(truth.depth_m[2]))
    );
    assert_eq!(truth.optical_axis_depth_m(1.0, 0.0), None);
    assert_eq!(truth.optical_axis_depth_m(5.0, 0.0), None);
}

#[test]
fn a_depth_buffer_of_the_wrong_size_is_refused() {
    assert!(matches!(
        ImageTruth::from_reversed_z(3, 2, &[0.1; 5], 0.5),
        Err(NavigateBridgeError::FrameSize {
            needed: 6,
            actual: 5,
            ..
        })
    ));
}

#[test]
fn a_capture_carries_its_true_pose_in_both_conventions() {
    let origin = Geodetic::from_degrees(39.1, -75.5, 0.0);
    let anchor = EnuAnchor::new(&BodyEllipsoid::WGS84, &origin);
    let vehicle =
        VehicleState::level_flight(&Geodetic::from_degrees(39.11, -75.49, 600.0), 0.3, 50.0);
    let pose = CameraMount::nadir_belly(DVec3::new(0.0, 0.0, 0.4)).pose(&vehicle, DVec3::ZERO);
    let navigate = pose.to_navigate(&anchor);
    let capture = CameraFrameCapture {
        timestamp: SimInstant::START.advanced(TickSpan::from_ticks(TICKS_PER_SECOND)),
        vehicle,
        pose,
        eye_enu: navigate.eye_enu,
        world_to_cam: navigate.world_to_cam,
        intrinsics: SensorIntrinsics {
            width: 4,
            height: 4,
            fx: 4.0,
            fy: 4.0,
            cx: 1.5,
            cy: 1.5,
        },
    };
    let truth = SimTruth::from_capture(&capture, Some(600.0));
    let camera = truth.camera.unwrap();
    assert_eq!(truth.instant, capture.timestamp);
    assert_eq!(camera.pose_ecef, pose);
    assert_eq!(camera.pose_enu, navigate);
    assert_eq!(camera.optical_axis_depth_m, Some(600.0));
    assert_eq!(SimTruth::vehicle(truth.instant, vehicle).camera, None);
}

#[test]
fn latency_runs_from_the_described_instant_to_delivery() {
    let valid_at = SimInstant::START.advanced(TickSpan::from_ticks(TICKS_PER_SECOND));
    let output = NavigationOutput {
        origin: NavigationOrigin::Fused(AblationSet::only(NavMeans::Visual)),
        source: SourceId::new(1),
        valid_at,
        delivered_at: valid_at.advanced(TickSpan::from_ticks(TICKS_PER_SECOND / 4)),
        position: GeodeticPosition::new(0.7, -1.3, 100.0),
        position_cov: SymmetricCov3::from_diagonal(1.0, 1.0, 1.0),
        velocity: None,
        labels: vec!["shared_map_cell".to_owned()],
    };
    assert_eq!(output.latency().as_seconds(), 0.25);
    let early = NavigationOutput {
        delivered_at: SimInstant::START,
        ..output
    };
    assert_eq!(early.latency(), TickSpan::ZERO);
}
