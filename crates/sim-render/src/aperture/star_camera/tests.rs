use glam::DVec3;
use sim_geo::WGS84_A;

use super::*;

/// Over the equator at longitude zero, `height_m` up.
fn eye(height_m: f64) -> DVec3 {
    DVec3::new(WGS84_A + height_m, 0.0, 0.0)
}

/// A direction in the plane through the eye's zenith (`+X`) and the pole
/// (`+Z`), `angle_deg` from the nadir.
fn from_nadir(angle_deg: f64) -> DVec3 {
    let angle = angle_deg.to_radians();
    DVec3::new(-angle.cos(), 0.0, angle.sin())
}

fn camera(config: StarCameraConfig) -> StarCamera {
    StarCamera::new(config).unwrap()
}

/// The ECEF direction that projects to intrinsics pixel `(x, y)`.
fn direction_at(camera: &StarCamera, pose: &CameraPose, [x, y]: [f64; 2]) -> DVec3 {
    let intrinsics = camera.config().intrinsics;
    let in_camera = DVec3::new(
        (x - intrinsics.cx) / intrinsics.fx,
        -(y - intrinsics.cy) / intrinsics.fy,
        -1.0,
    );
    (pose.orientation * in_camera).normalize()
}

fn star(id: u16, direction_ecef: DVec3, v_magnitude: f64) -> CatalogStar {
    CatalogStar {
        id,
        direction_ecef,
        v_magnitude,
    }
}

/// Looking at the zenith from 10 km, so the Earth blocks nothing in view.
fn zenith_pose() -> CameraPose {
    CameraPose::looking(eye(10_000.0), DVec3::X, DVec3::Z)
}

/// Intensity-weighted centroid and total of the signal above `background`
/// within `half` pixels of `around`.
fn measure(
    image: &[f64],
    width: u32,
    around: [f64; 2],
    half: i64,
    background: f64,
) -> ([f64; 2], f64) {
    let (cx, cy) = (around[0].round() as i64, around[1].round() as i64);
    let (mut sum, mut sx, mut sy) = (0.0, 0.0, 0.0);
    for y in cy - half..=cy + half {
        for x in cx - half..=cx + half {
            let value = image[(y * i64::from(width) + x) as usize] - background;
            sum += value;
            sx += value * x as f64;
            sy += value * y as f64;
        }
    }
    ([sx / sum, sy / sum], sum)
}

#[test]
fn noiseless_centroid_matches_truth_within_five_hundredths_of_a_pixel() {
    let camera = camera(StarCameraConfig::star_tracker_1024());
    let pose = zenith_pose();
    let targets = [
        [511.5, 511.5],
        [100.27, 840.81],
        [733.5, 205.0],
        [402.93, 611.44],
    ];
    let stars: Vec<_> = targets
        .iter()
        .zip(0..)
        .map(|(&pixel, id)| star(id, direction_at(&camera, &pose, pixel), 2.0))
        .collect();
    let (image, truth) = camera.expected_electrons(&pose, &stars);
    let mut worst: f64 = 0.0;
    for (target, star) in targets.iter().zip(&truth.stars) {
        assert_eq!(star.visibility, StarVisibility::Visible);
        let centre = star.centroid_px.unwrap();
        assert!((centre[0] - target[0]).abs() < 1e-6 && (centre[1] - target[1]).abs() < 1e-6);
        let (measured, total) = measure(&image, 1024, centre, 6, truth.background_electrons_per_px);
        worst = worst.max((measured[0] - centre[0]).hypot(measured[1] - centre[1]));
        assert!(
            (total / star.signal_electrons - 1.0).abs() < 1e-3,
            "flux {total}"
        );
    }
    writeln_stderr(&format!("worst noiseless centroid error {worst:.2e} px"));
    assert!(worst < 0.05, "centroid error {worst} px");
}

#[test]
fn reruns_are_bit_identical_and_seeds_and_frames_differ() {
    let config = StarCameraConfig {
        seed: 11,
        ..StarCameraConfig::star_tracker_1024()
    };
    let camera_a = camera(config);
    let pose = zenith_pose();
    let stars = [star(1, direction_at(&camera_a, &pose, [300.0, 300.0]), 3.0)];
    let first = camera_a.expose(&pose, &stars, 5);
    assert_eq!(first, camera(config).expose(&pose, &stars, 5));
    assert_ne!(first.image, camera_a.expose(&pose, &stars, 6).image);
    let reseeded = camera(StarCameraConfig { seed: 12, ..config });
    assert_ne!(first.image, reseeded.expose(&pose, &stars, 5).image);
    assert!(first.image.counts.iter().all(|&count| count <= 4095));
    assert_eq!(first.image.significant_bits, 12);
}

#[test]
fn flat_frame_noise_matches_the_detector_model() {
    let config = StarCameraConfig::star_tracker_1024();
    let exposure = camera(config).expose(&zenith_pose(), &[], 0);
    let background = config.sky_electrons_per_px() + config.dark_electrons_per_px();
    let counts: Vec<f64> = exposure
        .image
        .counts
        .iter()
        .map(|&c| f64::from(c))
        .collect();
    let n = counts.len() as f64;
    let mean = counts.iter().sum::<f64>() / n;
    let std = (counts.iter().map(|c| (c - mean).powi(2)).sum::<f64>() / n).sqrt();
    let expected_mean = config.bias_dn + background / config.gain_e_per_dn;
    let expected_std = ((background + config.read_noise_e.powi(2)) / config.gain_e_per_dn.powi(2)
        + 1.0 / 12.0)
        .sqrt();
    assert!(
        (mean - expected_mean).abs() < 0.01,
        "{mean} vs {expected_mean}"
    );
    assert!(
        (std / expected_std - 1.0).abs() < 0.02,
        "{std} vs {expected_std}"
    );
}

#[test]
fn earth_and_its_atmosphere_hide_stars_behind_the_limb() {
    // From 700 km the solid limb is 64.3° from the nadir and the 50 km layer
    // 65.2°.
    let pose = CameraPose::looking(eye(700_000.0), from_nadir(66.0), DVec3::Y);
    let stars = [
        star(1, from_nadir(58.0), 2.0),
        star(2, from_nadir(64.8), 2.0),
        star(3, from_nadir(72.0), 2.0),
    ];
    let visibility = |atmosphere_height_m| {
        let config = StarCameraConfig {
            atmosphere_height_m,
            ..StarCameraConfig::star_tracker_1024()
        };
        let (_, truth) = camera(config).expected_electrons(&pose, &stars);
        truth.stars.iter().map(|s| s.visibility).collect::<Vec<_>>()
    };
    use StarVisibility::{Occluded, Visible};
    assert_eq!(
        visibility(STAR_ATMOSPHERE_HEIGHT_M),
        [Occluded, Occluded, Visible]
    );
    assert_eq!(visibility(0.0), [Occluded, Visible, Visible]);
}

#[test]
fn an_aircraft_sees_down_to_its_horizon_dip() {
    // From 10 km the geometric horizon dips 3.2° below the horizontal; the
    // 50 km layer is above the eye, so it does not hide the sky.
    let config = StarCameraConfig::star_tracker_1024();
    let pose = CameraPose::looking(eye(10_000.0), from_nadir(90.0), DVec3::Y);
    let stars = [
        star(1, from_nadir(92.0), 2.0),
        star(2, from_nadir(88.0), 2.0),
        star(3, from_nadir(85.0), 2.0),
    ];
    let (image, truth) = camera(config).expected_electrons(&pose, &stars);
    let visibility: Vec<_> = truth.stars.iter().map(|s| s.visibility).collect();
    use StarVisibility::{Occluded, Visible};
    assert_eq!(visibility, [Visible, Visible, Occluded]);
    let hidden = truth.stars[2];
    assert_eq!(hidden.signal_electrons, 0.0);
    let (_, light) = measure(
        &image,
        1024,
        hidden.centroid_px.unwrap(),
        6,
        truth.background_electrons_per_px,
    );
    assert!(light.abs() < 1e-6, "occluded star leaked {light} electrons");
}

#[test]
fn brighter_stars_give_more_signal_by_the_magnitude_ratio() {
    let camera = camera(StarCameraConfig::star_tracker_1024());
    let pose = zenith_pose();
    let magnitudes = [1.0, 2.0, 3.5, 5.0];
    let stars: Vec<_> = magnitudes
        .iter()
        .zip(0..)
        .map(|(&v, id)| {
            star(
                id,
                direction_at(&camera, &pose, [200.0 + 150.0 * f64::from(id), 500.0]),
                v,
            )
        })
        .collect();
    let (image, truth) = camera.expected_electrons(&pose, &stars);
    let measured: Vec<f64> = truth
        .stars
        .iter()
        .map(|s| {
            measure(
                &image,
                1024,
                s.centroid_px.unwrap(),
                6,
                truth.background_electrons_per_px,
            )
            .1
        })
        .collect();
    for (pair, mags) in measured.windows(2).zip(magnitudes.windows(2)) {
        assert!(pair[0] > pair[1]);
        let ratio = pair[0] / pair[1];
        let expected = 10.0_f64.powf(0.4 * (mags[1] - mags[0]));
        assert!(
            (ratio / expected - 1.0).abs() < 1e-3,
            "{ratio} vs {expected}"
        );
    }
}

#[test]
fn stars_off_the_detector_or_behind_are_out_of_view() {
    let camera = camera(StarCameraConfig::star_tracker_1024());
    let pose = zenith_pose();
    let stars = [
        star(1, -DVec3::X, 1.0),
        star(2, direction_at(&camera, &pose, [1030.0, 500.0]), 1.0),
        star(3, direction_at(&camera, &pose, [-0.4, 500.0]), 1.0),
    ];
    let (_, truth) = camera.expected_electrons(&pose, &stars);
    assert_eq!(truth.stars[0].visibility, StarVisibility::OutOfFov);
    assert_eq!(truth.stars[0].centroid_px, None);
    assert_eq!(truth.stars[0].signal_electrons, 0.0);
    assert_eq!(truth.stars[1].visibility, StarVisibility::OutOfFov);
    assert!(truth.stars[1].centroid_px.is_some());
    assert_eq!(truth.stars[2].visibility, StarVisibility::Visible);
}

#[test]
fn photometry_follows_the_zero_point() {
    let config = StarCameraConfig::star_tracker_1024();
    let area = std::f64::consts::PI * 0.006 * 0.006;
    let expected = V_ZERO_POINT_PHOTONS_PER_M2_S * area * 0.1 * 0.8 * 0.6;
    assert!((config.star_electrons(0.0) / expected - 1.0).abs() < 1e-12);
    assert!((config.star_electrons(5.0) / config.star_electrons(0.0) - 0.01).abs() < 1e-12);
    let v6 = config.star_electrons(6.0);
    assert!((150.0..250.0).contains(&v6), "V = 6 gives {v6} e-");
}

#[test]
fn erf_matches_reference_values() {
    for (x, expected) in [
        (0.0, 0.0),
        (0.5, 0.520_499_877_8),
        (1.0, 0.842_700_792_9),
        (-1.5, -0.966_105_146_5),
        (3.0, 0.999_977_909_5),
    ] {
        assert!((psf::erf(x) - expected).abs() < 2e-7, "erf({x})");
    }
}

#[test]
fn unusable_configurations_are_rejected() {
    let good = StarCameraConfig::star_tracker_1024();
    for bad in [
        StarCameraConfig {
            exposure_s: 0.0,
            ..good
        },
        StarCameraConfig {
            quantum_efficiency: 1.2,
            ..good
        },
        StarCameraConfig {
            psf_sigma_px: f64::NAN,
            ..good
        },
        StarCameraConfig {
            significant_bits: 17,
            ..good
        },
        StarCameraConfig {
            gain_e_per_dn: -1.0,
            ..good
        },
    ] {
        assert!(matches!(
            StarCamera::new(bad),
            Err(RenderError::InvalidSensor { .. })
        ));
    }
    assert!(StarCamera::new(StarCameraConfig::wide_field_2048()).is_ok());
}

fn writeln_stderr(line: &str) {
    use std::io::Write;
    writeln!(std::io::stderr(), "{line}").ok();
}
