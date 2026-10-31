use sim_core::{
    Block, CAMERA_DEFAULT_PERIOD, CameraMount, CameraSensorBlock, CameraSensorConfig,
    CameraSensorInput, EnuAnchor, VehicleState,
};
use sim_geo::BodyEllipsoid;

use super::*;

const INTRINSICS: SensorIntrinsics = SensorIntrinsics {
    width: 800,
    height: 600,
    fx: 600.0,
    fy: 600.0,
    cx: 399.5,
    cy: 299.5,
};

fn capture(anchor: &Geodetic, mount: CameraMount) -> sim_core::CameraFrameCapture {
    let position = Geodetic::from_degrees(39.08, -75.62, 900.0);
    let vehicle = VehicleState::level_flight(&position, 1.2, 55.0);
    let mut camera = CameraSensorBlock::new(CameraSensorConfig {
        mount,
        intrinsics: INTRINSICS,
        anchor: EnuAnchor::new(&BodyEllipsoid::WGS84, anchor),
        period: CAMERA_DEFAULT_PERIOD,
    })
    .unwrap();
    camera
        .step(&CameraSensorInput {
            vehicle,
            cg_body_m: DVec3::ZERO,
        })
        .unwrap();
    camera.output().unwrap()
}

/// The pose Navigate solves in must be the eye `sim-core` reports, on every
/// axis of the frame and with the same rotation.
#[test]
fn a_camera_pose_lands_on_the_harness_eye_with_its_rotation() {
    let anchor = Geodetic::from_degrees(39.05, -75.70, 0.0);
    let frame = CorridorFrame::new(&anchor).unwrap();
    let capture = capture(&anchor, CameraMount::oblique(DVec3::X, 0.4, -0.3, 0.1));

    let pose = frame
        .camera_pose(capture.pose.position_ecef, capture.pose.orientation)
        .unwrap();

    let eye = DVec3::from_array(capture.eye_enu);
    assert!((pose.position.x - eye.x).abs() < 1e-3, "{pose:?}");
    assert!((pose.position.y - eye.y).abs() < 1e-3, "{pose:?}");
    assert!((pose.position.z - eye.z).abs() < 1e-3, "{pose:?}");
    // `world_to_cam` is row-major into right, down, forward; the eye axes are
    // right, up, back, so its second and third rows change sign.
    let [a, b, c, d, e, f, g, h, i] = capture.world_to_cam;
    let world_to_eye = nalgebra::Matrix3::new(a, b, c, -d, -e, -f, -g, -h, -i);
    let difference = (pose
        .orientation
        .to_rotation_matrix()
        .into_inner()
        .transpose()
        - world_to_eye)
        .norm();
    assert!(difference < 1e-9, "{difference}");
}

#[test]
fn enu_positions_return_through_ecef() {
    let anchor = Geodetic::from_degrees(39.05, -75.70, 0.0);
    let frame = CorridorFrame::new(&anchor).unwrap();
    let point = DVec3::new(3_000.0, 1_000.0, 900.0);
    let pose = frame
        .camera_pose(frame.enu_to_ecef(point), DQuat::IDENTITY)
        .unwrap();
    assert!((to_vector(point) - pose.position).norm() < 1e-3);
}

#[test]
fn an_anchor_outside_the_map_range_is_refused() {
    let polar = Geodetic::from_degrees(89.5, 0.0, 0.0);
    assert!(matches!(
        CorridorFrame::new(&polar),
        Err(NavigateBridgeError::InvalidAnchor { .. })
    ));
}

#[test]
fn every_exposure_has_its_own_digest_and_replays_it() {
    let later = SimInstant::from_nanos(2);
    let first = evidence_digest(1, 1, SimInstant::from_nanos(1), &INTRINSICS);
    assert_eq!(first.len(), 64);
    assert_eq!(
        first,
        evidence_digest(1, 1, SimInstant::from_nanos(1), &INTRINSICS)
    );
    assert_ne!(first, evidence_digest(2, 2, later, &INTRINSICS));
}

#[test]
fn an_estimate_keeps_the_declared_covariance_and_names_its_source() {
    let map = MapRevision {
        release_id: "synthetic".into(),
        manifest_sha256: "a".repeat(64),
    };
    let estimate = estimate(EstimateParts {
        stamp: FrameStamp {
            sequence: 1,
            capture_time_ns: 2,
        },
        observation_sha256: "b".repeat(64),
        map: &map,
        frame: LocalFrame::wgs84_enu(39.05, -75.70).unwrap(),
        pose: CameraPose {
            position: Vector3::zeros(),
            orientation: UnitQuaternion::identity(),
        },
        covariance_enu_m2: DMat3::from_cols(
            DVec3::new(4.0, 1.0, 0.0),
            DVec3::new(1.0, 9.0, 0.0),
            DVec3::new(0.0, 0.0, 16.0),
        ),
    });
    assert_eq!(estimate.geometry_covariance[(0, 1)], 1.0);
    assert_eq!(estimate.geometry_covariance[(2, 2)], 16.0);
    assert!(estimate.geometry_covariance[(3, 3)] > 0.0);
    assert_eq!(estimate.quality.inliers, 0);
    assert!(estimate.backend.contains("synthetic"));
}
