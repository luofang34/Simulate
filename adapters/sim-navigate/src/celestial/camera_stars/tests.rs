use glam::DVec3;
use navigate_celestial::{Epoch, stars_ecef, stars_ecef_tt};
use sim_core::{SimInstant, TickSpan};
use sim_geo::{BodyEllipsoid, Geodetic, LocalFrame, UtcTime};
use sim_render::{CameraPose, StarCamera, StarCameraConfig, StarVisibility};

use super::*;

/// 2026-03-20T02:00:00Z, a night over the Delaware corridor.
const START_UNIX_S: f64 = 1_773_972_000.0;

fn start() -> UtcTime {
    UtcTime::from_unix_seconds(START_UNIX_S)
}

fn at_seconds(seconds: u64) -> SimInstant {
    SimInstant::START.advanced(TickSpan::from_seconds_and_subsec_ticks(seconds, 0))
}

fn angle_deg(a: DVec3, b: DVec3) -> f64 {
    a.angle_between(b).to_degrees()
}

#[test]
fn epoch_is_the_scenario_start_plus_simulated_time() {
    let epoch = star_camera_epoch(start(), at_seconds(90));
    let expected = Epoch::from_unix_seconds(START_UNIX_S + 90.0);
    assert!((epoch - expected).to_seconds().abs() < 1e-6);
}

#[test]
fn stars_match_navigates_civil_and_instant_paths() {
    let now = at_seconds(3_600);
    let stars = camera_stars(start(), now, 3.0).unwrap();
    let epoch = star_camera_epoch(start(), now);
    let civil = stars_ecef(epoch, 3.0).unwrap();
    let instant = stars_ecef_tt(epoch, 3.0).unwrap();
    assert_eq!(stars.len(), civil.len());
    assert!(stars.len() > 100);
    for ((star, civil), instant) in stars.iter().zip(&civil).zip(&instant) {
        assert_eq!(star.id, civil.hr);
        assert_eq!(star.v_magnitude, civil.v_magnitude);
        let direction = |sight: &navigate_celestial::StarSight| DVec3::from_array(sight.direction);
        assert!(angle_deg(star.direction_ecef, direction(civil)) < 1e-9);
        assert!(angle_deg(star.direction_ecef, direction(instant)) < 1e-9);
    }
}

#[test]
fn the_sky_turns_with_simulated_time() {
    let early = camera_stars(start(), at_seconds(0), 2.0).unwrap();
    let late = camera_stars(start(), at_seconds(240), 2.0).unwrap();
    let star = early
        .iter()
        .find(|s| s.direction_ecef.z.abs() < 0.2)
        .unwrap();
    let moved = late.iter().find(|s| s.id == star.id).unwrap();
    // Four minutes of sidereal rotation is 1.0027°.
    let turned = angle_deg(star.direction_ecef, moved.direction_ecef);
    let expected = 240.0 * 360.0 / 86_164.1 * star.direction_ecef.truncate().length();
    assert!(
        (turned - expected).abs() < 0.002,
        "{turned}° vs {expected}°"
    );
}

#[test]
fn reruns_give_the_same_sky() {
    let now = at_seconds(17);
    assert_eq!(
        camera_stars(start(), now, 4.0).unwrap(),
        camera_stars(start(), now, 4.0).unwrap()
    );
}

#[test]
fn an_epoch_outside_the_ephemeris_is_refused() {
    let ancient = UtcTime::from_unix_seconds(-200_000_000_000.0);
    assert!(matches!(
        camera_stars(ancient, SimInstant::START, 3.0),
        Err(NavigateBridgeError::StarEphemeris { .. })
    ));
}

#[test]
fn a_star_tracker_aimed_at_a_catalogue_star_images_it_at_the_boresight() {
    let site = Geodetic {
        lat_rad: 39.0_f64.to_radians(),
        lon_rad: (-75.5_f64).to_radians(),
        height_m: 9_000.0,
    };
    let eye = BodyEllipsoid::WGS84.geodetic_to_fixed(&site);
    let up = LocalFrame::at(&site).up();
    let stars = camera_stars(start(), at_seconds(600), 6.0).unwrap();
    let target = stars
        .iter()
        .find(|star| star.direction_ecef.dot(up) > 0.7)
        .unwrap();
    let side = target.direction_ecef.any_orthonormal_vector();
    let pose = CameraPose::looking(eye, target.direction_ecef, side);
    let camera = StarCamera::new(StarCameraConfig::star_tracker_1024()).unwrap();
    let exposure = camera.expose(&pose, &stars, 0);
    let truth = exposure
        .truth
        .stars
        .iter()
        .find(|star| star.id == target.id)
        .unwrap();
    assert_eq!(truth.visibility, StarVisibility::Visible);
    let [x, y] = truth.centroid_px.unwrap();
    assert!((x - 511.5).abs() < 1e-6 && (y - 511.5).abs() < 1e-6);
    let near_centre = exposure
        .image
        .count(511, 511)
        .unwrap()
        .max(exposure.image.count(512, 512).unwrap());
    let corner = exposure.image.count(3, 3).unwrap();
    assert!(near_centre > corner + 100, "{near_centre} vs {corner}");
    let visible = exposure
        .truth
        .stars
        .iter()
        .filter(|star| star.visibility == StarVisibility::Visible)
        .count();
    assert!(visible >= 1);
}
