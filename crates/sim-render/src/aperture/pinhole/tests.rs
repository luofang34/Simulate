use glam::{DMat4, DQuat, Vec4};

use super::*;

const NEAR_M: f64 = 0.5;

fn intrinsics() -> SensorIntrinsics {
    SensorIntrinsics {
        width: 800,
        height: 600,
        fx: 700.0,
        fy: 700.0,
        cx: 399.5,
        cy: 299.5,
    }
}

fn projection(intrinsics: SensorIntrinsics) -> PinholeProjection {
    PinholeProjection::new(intrinsics, NEAR_M).unwrap()
}

fn ndc(matrix: DMat4, camera_point: DVec3) -> DVec3 {
    let clip = matrix * camera_point.extend(1.0);
    clip.truncate() / clip.w
}

/// Pixel coordinates (centres on integers) of a normalised device point.
fn pixel(ndc: DVec3, intrinsics: &SensorIntrinsics) -> [f64; 2] {
    [
        (ndc.x + 1.0) / 2.0 * f64::from(intrinsics.width) - 0.5,
        (1.0 - ndc.y) / 2.0 * f64::from(intrinsics.height) - 0.5,
    ]
}

#[test]
fn a_centred_camera_matches_the_symmetric_reversed_z_perspective() {
    let camera = intrinsics();
    let fov_y = 2.0 * (f64::from(camera.height) / (2.0 * camera.fy)).atan();
    let aspect = f64::from(camera.width) / f64::from(camera.height);
    let symmetric =
        glam::dcamera::rh::proj::directx::perspective_infinite_reverse(fov_y, aspect, NEAR_M);
    let ours = projection(camera).matrix();
    assert!(
        ours.abs_diff_eq(symmetric, 1e-12),
        "{ours:?} != {symmetric:?}"
    );
}

#[test]
fn the_optical_axis_lands_on_an_off_centre_principal_point() {
    let camera = SensorIntrinsics {
        cx: 420.25,
        cy: 287.0,
        ..intrinsics()
    };
    let centre = ndc(projection(camera).matrix(), DVec3::new(0.0, 0.0, -50.0));
    let [u, v] = pixel(centre, &camera);
    assert!((u - camera.cx).abs() < 1e-9, "{u}");
    assert!((v - camera.cy).abs() < 1e-9, "{v}");
}

#[test]
fn off_axis_points_follow_the_focal_lengths() {
    let camera = SensorIntrinsics {
        fx: 650.0,
        fy: 710.0,
        cx: 400.0,
        cy: 310.0,
        ..intrinsics()
    };
    // One metre right and half a metre up, at ten metres.
    let point = DVec3::new(1.0, 0.5, -10.0);
    let [u, v] = pixel(ndc(projection(camera).matrix(), point), &camera);
    assert!((u - (400.0 + 65.0)).abs() < 1e-9, "{u}");
    assert!((v - (310.0 - 35.5)).abs() < 1e-9, "{v}");
}

#[test]
fn the_matrix_and_the_pixel_formula_agree() {
    let camera = SensorIntrinsics {
        cx: 381.0,
        cy: 322.5,
        ..intrinsics()
    };
    let projection = projection(camera);
    let direction = DVec3::new(-0.2, 0.13, -1.0);
    let [u, v] = pixel(ndc(projection.matrix(), direction), &camera);
    let [x, y] = projection.pixel_of(direction).unwrap();
    assert!((u - x).abs() < 1e-9 && (v - y).abs() < 1e-9);
    assert_eq!(projection.pixel_of(DVec3::new(0.0, 0.0, 1.0)), None);
    assert_eq!(projection.pixel_of(DVec3::X), None);
}

#[test]
fn depth_is_reversed_and_inverts_to_range() {
    let matrix = projection(intrinsics()).matrix();
    let depth = |distance: f64| ndc(matrix, DVec3::new(0.0, 0.0, -distance)).z;
    assert!(
        (depth(NEAR_M) - 1.0).abs() < 1e-12,
        "near plane is depth one"
    );
    assert!(depth(10.0) > depth(10_000.0));
    assert!(depth(5_000_000.0) > 0.0);
    for distance in [1.0, 250.0, 12_000.0, 3_844_000.0] {
        let range = reversed_z_range_m(depth(distance) as f32, NEAR_M).unwrap();
        assert!(
            (range - distance).abs() / distance < 1e-6,
            "{distance} -> {range}"
        );
    }
}

#[test]
fn open_sky_has_no_range() {
    assert_eq!(reversed_z_range_m(0.0, NEAR_M), None);
    assert_eq!(projection(intrinsics()).range_m(0.0), None);
    assert_eq!(projection(intrinsics()).range_m(0.25), Some(2.0));
}

#[test]
fn the_view_rotates_world_axes_into_the_camera_then_projects() {
    let camera = intrinsics();
    let projection = projection(camera);
    // Looking along +X with +Z up.
    let pose = CameraPose::looking(DVec3::new(7e6, 0.0, 0.0), DVec3::X, DVec3::Z);
    let clip = projection.view_projection(&pose) * Vec4::new(100.0, 0.0, 10.0, 1.0);
    let ndc = clip.truncate() / clip.w;
    let [u, v] = pixel(ndc.as_dvec3(), &camera);
    assert!((u - camera.cx).abs() < 1e-3, "{u}");
    assert!((v - (camera.cy - 70.0)).abs() < 1e-3, "{v}");
    let identity = CameraPose {
        position_ecef: DVec3::ZERO,
        orientation: DQuat::IDENTITY,
    };
    let matrix = projection.matrix().as_mat4();
    assert_eq!(projection.view_projection(&identity), matrix);
}

#[test]
fn rejects_optics_and_near_planes_that_cannot_project() {
    let blind = SensorIntrinsics {
        fx: 0.0,
        ..intrinsics()
    };
    assert!(PinholeProjection::new(blind, 1.0).is_err());
    for near in [0.0, -1.0, f64::INFINITY, f64::NAN] {
        assert!(PinholeProjection::new(intrinsics(), near).is_err());
    }
}
