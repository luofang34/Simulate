use glam::{DQuat, DVec3, EulerRot};
use nalgebra::{Quaternion, UnitQuaternion, Vector2, Vector3};
use navigate_camera::{OpticalPose, Pixels};
use navigate_contract::{SourceEpoch, SourceId, WrappingSequence};
use sim_core::{CameraMount, EnuAnchor, SimInstant, TICKS_PER_SECOND, TickSpan, VehicleState};
use sim_geo::{BodyEllipsoid, Geodetic, LocalFrame, SensorIntrinsics};

use super::*;

const INTRINSICS: SensorIntrinsics = SensorIntrinsics {
    width: 800,
    height: 600,
    fx: 610.0,
    fy: 590.0,
    cx: 401.25,
    cy: 297.5,
};

fn origin() -> Geodetic {
    Geodetic::from_degrees(39.1, -75.5, 0.0)
}

fn vehicle() -> VehicleState {
    let position = Geodetic::from_degrees(39.102, -75.497, 900.0);
    let mut state = VehicleState::level_flight(&position, 0.6, 60.0);
    let bank = DQuat::from_euler(EulerRot::ZYX, 0.1, 0.17, -0.35);
    state.attitude = (state.attitude * bank).normalize();
    state
}

fn to_nalgebra(q: DQuat) -> UnitQuaternion<f64> {
    UnitQuaternion::from_quaternion(Quaternion::new(q.w, q.x, q.y, q.z))
}

fn to_vector(v: DVec3) -> Vector3<f64> {
    Vector3::new(v.x, v.y, v.z)
}

/// World points spread over the view: renderer camera directions (`-Z`
/// forward) at several ranges, carried into ECEF by the simulator's pose.
fn points_in_view(mount: &CameraMount, cg: DVec3) -> Vec<DVec3> {
    let pose = mount.pose(&vehicle(), cg);
    let mut points = Vec::new();
    for (x, y, range) in [
        (0.0, 0.0, 400.0),
        (0.3, -0.2, 800.0),
        (-0.45, 0.3, 1500.0),
        (0.6, 0.35, 250.0),
    ] {
        let direction = DVec3::new(x, y, -1.0).normalize();
        points.push(pose.position_ecef + pose.orientation * direction * range);
    }
    points
}

fn assert_routes_agree(mount: CameraMount, cg: DVec3) {
    let k = imager_model(&INTRINSICS).unwrap().intrinsics;
    let state = vehicle();
    let pose = mount.pose(&state, cg);
    let gl = OpticalPose::from_gl(to_vector(pose.position_ecef), to_nalgebra(pose.orientation));
    let body = navigate_mount(&mount, cg).camera_pose(
        &to_vector(state.position_ecef),
        &to_nalgebra(state.attitude),
    );
    let body_ecef = BodyEllipsoid::WGS84;
    let anchor = EnuAnchor::new(&body_ecef, &origin());
    let enu = pose.to_navigate(&anchor);
    let enu_pose = OpticalPose::from_row_major(enu.eye_enu, enu.world_to_cam).unwrap();
    let origin_ecef = body_ecef.geodetic_to_fixed(&origin());
    let frame = LocalFrame::at(&origin());
    for point in points_in_view(&mount, cg) {
        let from_gl = gl.project(&k, &to_vector(point)).unwrap();
        let from_mount = body.project(&k, &to_vector(point)).unwrap();
        let point_enu = frame.ecef_to_enu_vector(point - origin_ecef);
        let from_enu = enu_pose.project(&k, &to_vector(point_enu)).unwrap();
        assert!(
            (from_gl - from_mount).norm() < 1e-6,
            "{from_gl} {from_mount}"
        );
        assert!((from_gl - from_enu).norm() < 1e-6, "{from_gl} {from_enu}");
    }
}

#[test]
fn every_mount_projects_world_points_to_the_same_pixels_in_both_conventions() {
    let offset = DVec3::new(1.4, 0.3, -0.25);
    let cg = DVec3::new(0.35, -0.05, 0.12);
    for mount in [
        CameraMount::forward_nose(offset),
        CameraMount::nadir_belly(offset),
        CameraMount::oblique(offset, 1.2, -0.3, 0.4),
        CameraMount::looking_along(offset, DVec3::new(0.2, -1.0, 0.4), DVec3::NEG_Z),
    ] {
        assert_routes_agree(mount, cg);
        assert_routes_agree(mount, DVec3::ZERO);
    }
}

/// Pins the axes without the simulator's pose: a nose camera in level flight
/// sees the point dead ahead at the principal point, the right wing's side on
/// the right, and the ground below the centre.
#[test]
fn a_level_nose_camera_sees_ahead_right_and_down_where_an_image_puts_them() {
    let k = imager_model(&INTRINSICS).unwrap().intrinsics;
    let position = Geodetic::from_degrees(39.1, -75.5, 900.0);
    let state = VehicleState::level_flight(&position, 0.0, 60.0);
    let mount = navigate_mount(
        &CameraMount::forward_nose(DVec3::new(2.0, 0.0, 0.0)),
        DVec3::ZERO,
    );
    let pose = mount.camera_pose(
        &to_vector(state.position_ecef),
        &to_nalgebra(state.attitude),
    );
    let eye = state.position_ecef + state.attitude * DVec3::new(2.0, 0.0, 0.0);
    let ahead = eye + state.attitude * DVec3::new(500.0, 0.0, 0.0);
    let right = ahead + state.attitude * DVec3::new(0.0, 50.0, 0.0);
    let below = ahead + state.attitude * DVec3::new(0.0, 0.0, 50.0);
    let centre = pose.project(&k, &to_vector(ahead)).unwrap();
    assert!((centre - Vector2::new(INTRINSICS.cx, INTRINSICS.cy)).norm() < 1e-6);
    assert!(pose.project(&k, &to_vector(right)).unwrap().x > INTRINSICS.cx + 50.0);
    assert!(pose.project(&k, &to_vector(below)).unwrap().y > INTRINSICS.cy + 50.0);
}

fn solid_rgba(colour: [u8; 4]) -> Vec<u8> {
    let pixels = INTRINSICS.width as usize * INTRINSICS.height as usize;
    colour.iter().copied().cycle().take(pixels * 4).collect()
}

fn stamp() -> ObservationStamp {
    MasterClockBridge::default()
        .stamp(
            SourceId::new(7),
            SourceEpoch::new(0),
            WrappingSequence::new(0),
            SimInstant::START,
        )
        .unwrap()
}

#[test]
fn gray8_uses_rec709_luma_of_the_srgb_bytes() {
    for (colour, expected) in [
        ([255, 0, 0, 255], 54),
        ([0, 255, 0, 9], 182),
        ([0, 0, 255, 0], 18),
        ([255, 255, 255, 255], 255),
    ] {
        let frame = OwnedCameraFrame::from_rgba8(
            stamp(),
            &INTRINSICS,
            &solid_rgba(colour),
            FramePixelFormat::Gray8,
        )
        .unwrap();
        assert!(frame.pixels().iter().all(|v| *v == expected), "{colour:?}");
        let view = frame.frame().unwrap();
        let navigate_luma = view.image.luma(3, 4).unwrap() * 255.0;
        assert!((navigate_luma - f32::from(expected)).abs() < 0.5);
    }
}

#[test]
fn rgb8_drops_alpha_and_reads_as_the_same_luma_as_gray8() {
    let rgba = solid_rgba([200, 120, 40, 7]);
    let rgb =
        OwnedCameraFrame::from_rgba8(stamp(), &INTRINSICS, &rgba, FramePixelFormat::Rgb8).unwrap();
    let gray =
        OwnedCameraFrame::from_rgba8(stamp(), &INTRINSICS, &rgba, FramePixelFormat::Gray8).unwrap();
    assert_eq!(
        rgb.pixels().get(..6),
        Some(&[200, 120, 40, 200, 120, 40][..])
    );
    let rgb_view = rgb.frame().unwrap();
    assert!(matches!(
        rgb_view.image.pixels(),
        Pixels::Rgb8 { stride: 2400, .. }
    ));
    let from_rgb = rgb_view.image.luma(10, 10).unwrap();
    let from_gray = gray.frame().unwrap().image.luma(10, 10).unwrap();
    assert!((from_rgb - from_gray).abs() <= 0.5 / 255.0);
}

#[test]
fn a_buffer_of_the_wrong_size_is_refused() {
    let mut rgba = solid_rgba([1, 2, 3, 4]);
    rgba.pop();
    let refused =
        OwnedCameraFrame::from_rgba8(stamp(), &INTRINSICS, &rgba, FramePixelFormat::Gray8);
    assert!(matches!(
        refused,
        Err(NavigateBridgeError::FrameSize { .. })
    ));
}

#[test]
fn unusable_intrinsics_are_refused() {
    let flat = SensorIntrinsics {
        fx: 0.0,
        ..INTRINSICS
    };
    assert!(matches!(
        imager_model(&flat),
        Err(NavigateBridgeError::Camera { .. })
    ));
}

#[test]
fn the_link_stamps_each_exposure_at_its_instant_in_sequence() {
    let clock = MasterClockBridge::default();
    let mut link = CameraFrameLink::new(
        clock,
        SourceId::new(9),
        SourceEpoch::new(2),
        FramePixelFormat::Gray8,
    );
    let anchor = EnuAnchor::new(&BodyEllipsoid::WGS84, &origin());
    let rgba = solid_rgba([10, 20, 30, 255]);
    let mut previous = None;
    for frame_index in 0..3_u32 {
        let at = SimInstant::START.advanced(TickSpan::from_ticks(
            u64::from(frame_index) * TICKS_PER_SECOND / 30,
        ));
        let pose = CameraMount::forward_nose(DVec3::X).pose(&vehicle(), DVec3::ZERO);
        let navigate = pose.to_navigate(&anchor);
        let capture = CameraFrameCapture {
            timestamp: at,
            vehicle: vehicle(),
            pose,
            eye_enu: navigate.eye_enu,
            world_to_cam: navigate.world_to_cam,
            intrinsics: INTRINSICS,
        };
        let frame = link.link(&capture, &rgba).unwrap();
        let stamp = frame.stamp();
        assert_eq!(stamp.acquired_at, clock.to_monotonic(at).unwrap());
        assert_eq!(stamp.sequence, WrappingSequence::new(frame_index));
        if let Some(previous) = previous {
            assert!(stamp.acquired_at > previous);
        }
        previous = Some(stamp.acquired_at);
        frame.frame().unwrap().validate().unwrap();
    }
}
