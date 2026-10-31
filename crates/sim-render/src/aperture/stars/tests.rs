use glam::DVec3;
use sim_geo::{SensorIntrinsics, UtcTime, nav_star_by_octal_id};

use super::*;

fn projection() -> PinholeProjection {
    PinholeProjection::new(
        SensorIntrinsics {
            width: 640,
            height: 480,
            fx: 500.0,
            fy: 500.0,
            cx: 319.5,
            cy: 239.5,
        },
        1.0,
    )
    .unwrap()
}

fn inertial() -> InertialTransform {
    InertialTransform::moon_at(UtcTime::from_unix_seconds(-14_182_940.0))
}

#[test]
fn the_star_the_camera_points_at_lands_on_the_principal_point() {
    let inertial = inertial();
    let star = nav_star_by_octal_id(0o17).unwrap();
    let forward = inertial.j2000_to_fixed(star.direction_j2000);
    let up = forward.cross(DVec3::Z).cross(forward);
    let pose = CameraPose::looking(DVec3::new(2e6, 0.0, 0.0), forward, up);
    let spots = star_spots(&projection(), &pose, &inertial);
    let spot = spots[usize::from(0o17_u8) - 1];
    assert!((spot[0] - 320.0).abs() < 1e-2, "{spot:?}");
    assert!((spot[1] - 240.0).abs() < 1e-2, "{spot:?}");
    assert!(spot[2] > 0.0);
}

#[test]
fn stars_behind_the_camera_are_dark() {
    let inertial = inertial();
    let star = nav_star_by_octal_id(0o17).unwrap();
    let backward = -inertial.j2000_to_fixed(star.direction_j2000);
    let up = backward.cross(DVec3::Z).cross(backward);
    let pose = CameraPose::looking(DVec3::ZERO, backward, up);
    let spot = star_spots(&projection(), &pose, &inertial)[usize::from(0o17_u8) - 1];
    assert_eq!(spot[2], 0.0);
}

#[test]
fn brighter_stars_peak_higher_on_a_magnitude_scale() {
    assert_eq!(peak_intensity(-1.46), 1.0);
    assert!((peak_intensity(0.0) - 1.0).abs() < 1e-6);
    assert!((peak_intensity(2.5) - 0.1).abs() < 1e-5);
    assert!((peak_intensity(5.0) - 0.01).abs() < 1e-6);
}
