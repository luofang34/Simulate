use glam::{DQuat, DVec3};
use sim_geo::WGS84_A;

use super::*;
use crate::aperture::{StarCameraConfig, StarVisibility};

const EXPOSURE_S: f64 = 0.1;

fn camera() -> StarCamera {
    StarCamera::new(StarCameraConfig::star_tracker_1024()).unwrap()
}

/// Looking at the zenith from 10 km over the equator.
fn start_pose() -> CameraPose {
    CameraPose::looking(DVec3::new(WGS84_A + 10_000.0, 0.0, 0.0), DVec3::X, DVec3::Z)
}

/// The camera panned about its own `+Y` axis by `angle_rad`.
fn panned(pose: &CameraPose, angle_rad: f64) -> CameraPose {
    CameraPose {
        position_ecef: pose.position_ecef,
        orientation: (pose.orientation * DQuat::from_rotation_y(angle_rad)).normalize(),
    }
}

/// A star on the boresight at mid-exposure.
fn star_at_mid(start: &CameraPose, end: &CameraPose, v_magnitude: f64) -> CatalogStar {
    let middle = pose_at(start, end, 0.5);
    CatalogStar {
        id: 1,
        direction_ecef: middle.orientation * DVec3::NEG_Z,
        v_magnitude,
    }
}

/// Signal total and intensity-weighted centroid within a box.
fn measure(image: &[f64], background: f64, x: [i64; 2], y: [i64; 2]) -> (f64, [f64; 2]) {
    let (mut sum, mut sx, mut sy) = (0.0, 0.0, 0.0);
    for row in y[0]..=y[1] {
        for column in x[0]..=x[1] {
            let value = image[(row * 1024 + column) as usize] - background;
            sum += value;
            sx += value * column as f64;
            sy += value * row as f64;
        }
    }
    (sum, [sx / sum, sy / sum])
}

#[test]
fn a_still_camera_exposes_exactly_as_the_static_path() {
    let camera = camera();
    let pose = start_pose();
    let star = star_at_mid(&pose, &pose, 3.0);
    assert_eq!(
        camera.expose_moving(&pose, &pose, &[star], 9),
        camera.expose(&pose, &[star], 9)
    );
}

#[test]
fn a_uniform_pan_draws_a_streak_of_rate_times_exposure_times_focal_length() {
    let camera = camera();
    let fx = camera.config().intrinsics.fx;
    let rate_rad_s = 0.1; // 5.7°/s, a brisk aircraft roll rate
    let start = start_pose();
    let end = panned(&start, rate_rad_s * EXPOSURE_S);
    let star = star_at_mid(&start, &end, 2.0);
    let (image, truth) = camera.expected_electrons_moving(&start, &end, &[star]);
    let star_truth = truth.stars[0];
    assert_eq!(star_truth.visibility, StarVisibility::Visible);
    let [first, last] = [
        star_truth.streak_start_px.unwrap(),
        star_truth.streak_end_px.unwrap(),
    ];
    let length = (last[0] - first[0]).hypot(last[1] - first[1]);
    let expected = rate_rad_s * EXPOSURE_S * fx;
    assert!(
        (length / expected - 1.0).abs() < 1e-4,
        "{length} px vs {expected} px"
    );
    assert!(last[0] > first[0], "panning about +Y moves stars toward +x");

    let mid = star_truth.centroid_px.unwrap();
    let (total, centroid) = measure(
        &image,
        truth.background_electrons_per_px,
        [first[0] as i64 - 8, last[0] as i64 + 8],
        [mid[1] as i64 - 8, mid[1] as i64 + 8],
    );
    let flux_error = total / star_truth.signal_electrons - 1.0;
    let centroid_error = (centroid[0] - mid[0]).hypot(centroid[1] - mid[1]);
    writeln!(
        std::io::stderr(),
        "streak {length:.3} px (expected {expected:.3}), flux error {flux_error:.2e}, \
         centroid error {centroid_error:.2e} px"
    )
    .ok();
    assert!(flux_error.abs() < 1e-3);
    assert!(centroid_error < 0.05);

    // Continuous: along the streak's middle third the row profile is flat.
    let row = mid[1].round() as i64;
    let third = (length / 3.0) as i64;
    let profile: Vec<f64> = (mid[0] as i64 - third / 2..=mid[0] as i64 + third / 2)
        .map(|x| image[(row * 1024 + x) as usize] - truth.background_electrons_per_px)
        .collect();
    let peak = profile.iter().copied().fold(f64::MIN, f64::max);
    let dip = profile.iter().copied().fold(f64::MAX, f64::min);
    assert!(dip / peak > 0.98, "streak ripples: {dip} vs {peak}");
}

#[test]
fn moving_exposures_repeat_and_differ_from_still_ones() {
    let camera = camera();
    let start = start_pose();
    let end = panned(&start, 0.004);
    let star = star_at_mid(&start, &end, 3.0);
    let moving = camera.expose_moving(&start, &end, &[star], 2);
    assert_eq!(moving, camera.expose_moving(&start, &end, &[star], 2));
    assert_ne!(moving.image, camera.expose(&start, &[star], 2).image);
}

use std::io::Write;
