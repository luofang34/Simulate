use std::f64::consts::FRAC_PI_2;

use super::*;
use sim_geo::ecef_to_geodetic;

const POSITION_DEG: (f64, f64) = (39.0, -75.5);

fn delaware() -> Geodetic {
    Geodetic::from_degrees(POSITION_DEG.0, POSITION_DEG.1, 0.0)
}

fn anchor() -> EnuAnchor {
    EnuAnchor::new(&BodyEllipsoid::WGS84, &delaware())
}

fn intrinsics() -> SensorIntrinsics {
    SensorIntrinsics {
        width: 800,
        height: 600,
        fx: 700.0,
        fy: 700.0,
        cx: 400.0,
        cy: 300.0,
    }
}

fn config(mount: CameraMount) -> CameraSensorConfig {
    CameraSensorConfig {
        mount,
        intrinsics: intrinsics(),
        anchor: anchor(),
        period: CAMERA_DEFAULT_PERIOD,
    }
}

fn level_north() -> VehicleState {
    VehicleState::level_flight(
        &Geodetic::from_degrees(POSITION_DEG.0, POSITION_DEG.1, 1_500.0),
        0.0,
        60.0,
    )
}

/// Pose after an arbitrary bank, pitch and heading, to catch composition
/// errors a level pose would hide.
fn manoeuvring() -> VehicleState {
    let level = VehicleState::level_flight(
        &Geodetic::from_degrees(POSITION_DEG.0 + 0.02, POSITION_DEG.1 + 0.03, 1_200.0),
        0.9,
        60.0,
    );
    VehicleState {
        attitude: (level.attitude * DQuat::from_euler(EulerRot::ZYX, 0.3, -0.2, 0.7)).normalize(),
        ..level
    }
}

fn optical_axis_ecef(pose: &CameraPoseEcef) -> DVec3 {
    pose.orientation * DVec3::NEG_Z
}

fn assert_vec_close(actual: DVec3, expected: DVec3) {
    assert!(
        (actual - expected).length() < 1e-9,
        "{actual} vs {expected}"
    );
}

#[test]
fn the_nose_camera_looks_forward_with_the_body_right_on_the_image_right() {
    let vehicle = manoeuvring();
    let pose = CameraMount::forward_nose(DVec3::ZERO).pose(&vehicle, DVec3::ZERO);
    assert_vec_close(optical_axis_ecef(&pose), vehicle.attitude * DVec3::X);
    assert_vec_close(pose.orientation * DVec3::X, vehicle.attitude * DVec3::Y);
    assert_vec_close(pose.orientation * DVec3::Y, vehicle.attitude * DVec3::NEG_Z);
}

#[test]
fn the_belly_camera_looks_at_the_ground_in_level_flight() {
    let vehicle = level_north();
    let pose = CameraMount::nadir_belly(DVec3::ZERO).pose(&vehicle, DVec3::ZERO);
    let frame = LocalFrame::at(&ecef_to_geodetic(vehicle.position_ecef));
    assert_vec_close(optical_axis_ecef(&pose), -frame.up());
    assert_vec_close(pose.orientation * DVec3::Y, frame.north());
    assert_vec_close(pose.orientation * DVec3::X, frame.east());
}

#[test]
fn an_oblique_camera_looks_out_of_the_right_window_and_below_the_horizon() {
    let vehicle = level_north();
    let depression = 20f64.to_radians();
    let mount = CameraMount::oblique(DVec3::ZERO, FRAC_PI_2, -depression, 0.0);
    let pose = mount.pose(&vehicle, DVec3::ZERO);
    let frame = LocalFrame::at(&ecef_to_geodetic(vehicle.position_ecef));
    let expected = frame.east() * depression.cos() - frame.up() * depression.sin();
    assert_vec_close(optical_axis_ecef(&pose), expected);
}

#[test]
fn zero_oblique_angles_are_the_nose_view() {
    let oblique = CameraMount::oblique(DVec3::ZERO, 0.0, 0.0, 0.0).body_to_camera;
    let nose = CameraMount::forward_nose(DVec3::ZERO).body_to_camera;
    assert!(oblique.dot(nose).abs() > 1.0 - 1e-12);
}

#[test]
fn the_lever_arm_follows_the_airframe_as_the_cg_moves() {
    let vehicle = manoeuvring();
    let mount = CameraMount::forward_nose(DVec3::new(4.0, 0.0, 0.5));
    let cg = DVec3::new(1.0, 0.0, 0.0);
    let pose = mount.pose(&vehicle, cg);
    let expected = vehicle.position_ecef + vehicle.attitude * DVec3::new(3.0, 0.0, 0.5);
    assert_vec_close(pose.position_ecef, expected);
}

fn matrix(pose: &NavigateCameraPose) -> DMat3 {
    DMat3::from_cols_array(&pose.world_to_cam).transpose()
}

#[test]
fn world_to_cam_is_a_proper_rotation_for_every_mount() {
    let mounts = [
        CameraMount::forward_nose(DVec3::X),
        CameraMount::nadir_belly(DVec3::Z),
        CameraMount::oblique(DVec3::ZERO, 1.1, -0.4, 0.3),
    ];
    for mount in mounts {
        for vehicle in [level_north(), manoeuvring()] {
            let pose = mount.pose(&vehicle, DVec3::ZERO).to_navigate(&anchor());
            let m = matrix(&pose);
            let residual = (m * m.transpose() - DMat3::IDENTITY)
                .to_cols_array()
                .map(f64::abs)
                .into_iter()
                .fold(0.0, f64::max);
            assert!(residual < 1e-12, "{residual}");
            assert!((m.determinant() - 1.0).abs() < 1e-12);
        }
    }
}

#[test]
fn world_to_cam_is_row_major_and_uses_the_optical_axes() {
    let vehicle = level_north();
    let pose = CameraMount::nadir_belly(DVec3::ZERO).pose(&vehicle, DVec3::ZERO);
    let navigate = pose.to_navigate(&anchor());
    let [r0, r1, r2, r3, r4, r5, r6, r7, r8] = navigate.world_to_cam;

    // Heading north with a nadir view: image right is east, image down is
    // south, and depth runs down. Rows are those camera axes in ENU.
    let tolerance = 1e-3;
    assert!((DVec3::new(r0, r1, r2) - DVec3::X).length() < tolerance);
    assert!((DVec3::new(r3, r4, r5) - DVec3::NEG_Y).length() < tolerance);
    assert!((DVec3::new(r6, r7, r8) - DVec3::NEG_Z).length() < tolerance);
}

#[test]
fn a_point_ahead_of_the_nose_camera_has_positive_optical_depth() {
    let anchor = anchor();
    let vehicle = manoeuvring();
    let pose = CameraMount::forward_nose(DVec3::ZERO).pose(&vehicle, DVec3::ZERO);
    let navigate = pose.to_navigate(&anchor);
    let ahead_ecef = pose.position_ecef + vehicle.attitude * DVec3::new(100.0, 0.0, 0.0);
    let ahead_enu = anchor
        .frame
        .ecef_to_enu_vector(ahead_ecef - anchor.origin_ecef);
    let in_camera = matrix(&navigate) * (ahead_enu - DVec3::from_array(navigate.eye_enu));
    assert!((in_camera - DVec3::new(0.0, 0.0, 100.0)).length() < 1e-6);
}

#[test]
fn eye_enu_is_measured_from_the_anchor() {
    let vehicle = VehicleState::level_flight(&delaware(), 0.0, 0.0);
    let at_anchor = CameraMount::forward_nose(DVec3::ZERO).pose(&vehicle, DVec3::ZERO);
    let eye = at_anchor.to_navigate(&anchor()).eye_enu;
    assert!(DVec3::from_array(eye).length() < 1e-9);

    let raised = CameraMount::forward_nose(DVec3::new(0.0, 0.0, -2.0)).pose(&vehicle, DVec3::ZERO);
    let eye = raised.to_navigate(&anchor()).eye_enu;
    assert_vec_close(DVec3::from_array(eye), DVec3::new(0.0, 0.0, 2.0));
}

#[test]
fn the_two_conventions_round_trip() {
    let anchor = anchor();
    let original = CameraMount::oblique(DVec3::new(1.0, 2.0, 3.0), 0.5, -0.6, 0.7)
        .pose(&manoeuvring(), DVec3::ZERO);
    let back = original.to_navigate(&anchor).to_ecef(&anchor);
    assert!((back.position_ecef - original.position_ecef).length() < 1e-6);
    assert!(back.orientation.dot(original.orientation).abs() > 1.0 - 1e-12);
}

#[test]
fn the_anchor_works_on_the_moon() {
    let moon = BodyEllipsoid::MOON_IAU;
    let site = Geodetic::from_degrees(0.67, 23.47, 0.0);
    let anchor = EnuAnchor::new(&moon, &site);
    let above = Geodetic {
        height_m: 100.0,
        ..site
    };
    let pose = CameraPoseEcef {
        position_ecef: moon.geodetic_to_fixed(&above),
        orientation: DQuat::IDENTITY,
    };
    let eye = pose.to_navigate(&anchor).eye_enu;
    assert_vec_close(DVec3::from_array(eye), DVec3::new(0.0, 0.0, 100.0));
}

#[test]
fn the_block_captures_at_thirty_hertz_on_exact_ticks() {
    let mut block = CameraSensorBlock::new(config(CameraMount::nadir_belly(DVec3::ZERO))).unwrap();
    assert_eq!(block.period(), TickSpan::period_of_hz(30).unwrap());
    assert_eq!(block.output(), None);

    let input = CameraSensorInput {
        vehicle: level_north(),
        cg_body_m: DVec3::ZERO,
    };
    block.step(&input).unwrap();
    assert_eq!(block.output().unwrap().timestamp, SimInstant::START);
    for _ in 0..30 {
        block.step(&input).unwrap();
    }
    let capture = block.output().unwrap();
    assert_eq!(
        capture.timestamp,
        SimInstant::START.advanced(TickSpan::SECOND)
    );
    assert_eq!(capture.vehicle, input.vehicle);
    assert_eq!(capture.intrinsics, intrinsics());
    let expected = capture.pose.to_navigate(&anchor());
    assert_eq!(capture.eye_enu, expected.eye_enu);
    assert_eq!(capture.world_to_cam, expected.world_to_cam);
}

#[test]
fn a_restored_camera_resumes_its_clock() {
    let mut block = CameraSensorBlock::new(config(CameraMount::nadir_belly(DVec3::ZERO))).unwrap();
    let input = CameraSensorInput {
        vehicle: level_north(),
        cg_body_m: DVec3::ZERO,
    };
    block.step(&input).unwrap();
    let saved = block.snapshot();
    block.step(&input).unwrap();
    let second = block.output();
    block.restore(&saved);
    block.step(&input).unwrap();
    assert_eq!(block.output(), second);
}

#[test]
fn unusable_setups_are_refused() {
    let mut flat = config(CameraMount::nadir_belly(DVec3::ZERO));
    flat.intrinsics.fx = 0.0;
    assert!(CameraSensorBlock::new(flat).is_err());
    let mut stopped = config(CameraMount::nadir_belly(DVec3::ZERO));
    stopped.period = TickSpan::ZERO;
    assert!(CameraSensorBlock::new(stopped).is_err());

    let mut block = CameraSensorBlock::new(config(CameraMount::nadir_belly(DVec3::ZERO))).unwrap();
    let bad = CameraSensorInput {
        vehicle: VehicleState {
            position_ecef: DVec3::NAN,
            ..level_north()
        },
        cg_body_m: DVec3::ZERO,
    };
    assert!(block.step(&bad).is_err());
    assert_eq!(block.output(), None);
}
