use glam::{DQuat, DVec3};
use sim_core::{CameraFrameCapture, CameraPoseEcef, SimInstant};
use sim_geo::SensorIntrinsics;
use sim_navigate::{CelestialClock, ReferenceBody, SkyTimeMode};
use sim_render::{ApertureWaveband, OcclusionOverlayConfig};
use sim_scenario::{CaptureExposure, aperture_frame};

use super::*;
use crate::flight::Flight;
use crate::sky_rig::SkyRig;

const LIVE_UTC_S: f64 = 1_782_864_000.0;

/// A simulated camera's capture at the flight's own position.
fn camera_capture(flight: &Flight) -> CameraFrameCapture {
    let vehicle = flight.vehicle_state();
    CameraFrameCapture {
        timestamp: SimInstant::START,
        vehicle,
        pose: CameraPoseEcef {
            position_ecef: vehicle.position_ecef,
            orientation: DQuat::IDENTITY,
        },
        eye_enu: [0.0; 3],
        world_to_cam: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
        intrinsics: SensorIntrinsics {
            width: 800,
            height: 600,
            fx: 600.0,
            fy: 600.0,
            cx: 399.5,
            cy: 299.5,
        },
    }
}

#[test]
fn the_window_and_a_simulated_camera_take_their_sky_from_one_frame() {
    let flight = Flight::new().expect("free flight");
    let observer = flight.vehicle_state().position_ecef;
    let mut rig = SkyRig::new();
    let sky = rig.evaluate(&flight, observer, LIVE_UTC_S);
    let truth = TruthSky::of(&sky);
    let exposure = CaptureExposure {
        environment: sky.environment,
        waveband: ApertureWaveband::VisibleRgb,
        occlusion: OcclusionOverlayConfig::default(),
        near_m: 0.5,
    };
    let camera = aperture_frame(&camera_capture(&flight), &exposure);
    let env = camera.environment;

    assert_eq!(truth.sun_direction_ecef, env.sun.direction_fixed);
    assert_eq!(truth.atmosphere, env.atmosphere.is_some());
    assert!(truth.atmosphere, "free flight is on Earth");
    let celestial = truth.celestial.expect("live sky draws the Moon and stars");
    assert_eq!(celestial.sun_direction_ecef, env.sun.direction_fixed);
    assert_eq!(celestial.sun_angular_radius_deg, env.sun.angular_radius_deg);
    assert_eq!(celestial.sun_visible_fraction, env.sun.visible_fraction);
    assert_eq!(celestial.twilight.night_end_deg, env.twilight.night_end_deg);
    assert_eq!(celestial.twilight.day_start_deg, env.twilight.day_start_deg);
    assert_eq!(
        celestial.twilight.night_brightness,
        env.twilight.night_brightness
    );
}

#[test]
fn a_fixed_sun_frame_feeds_the_window_by_the_sun_alone() {
    let world = sim_scenario::ScenarioWorld::DelawareCorridor;
    let sky = SkyFrame::of_world(world);
    let truth = TruthSky::of(&sky);
    assert_eq!(truth.celestial, None);
    assert_eq!(
        truth.sun_direction_ecef,
        sky.environment.sun.direction_fixed
    );
    assert_eq!(sky.environment.body, world.body());
}

#[test]
fn the_lunar_frame_has_no_atmosphere_and_a_black_sky() {
    let sun = DVec3::new(0.3, 0.2, 0.9).normalize();
    let frame = environment_frame(
        &CelestialFrame::with_sun(sun, 0.26),
        BodyEllipsoid::MOON_IAU,
    );
    assert_eq!(frame.atmosphere, None);
    let under_the_sun = sun * BodyEllipsoid::MOON_IAU.semi_major_m;
    assert!(frame.sun_elevation_deg_at(under_the_sun) > 89.9);
    assert_eq!(frame.sky_linear_rgb_at(under_the_sun), [0.0; 3]);
    let sky = SkyFrame::of_world(sim_scenario::ScenarioWorld::LunarTranquility);
    assert_eq!(sky.environment.atmosphere, None);
    assert!(!TruthSky::of(&sky).atmosphere);
}

#[test]
fn frames_at_two_instants_of_one_epoch_turn_the_sun_with_the_earth() {
    let flight = Flight::new().expect("free flight");
    let base_utc_s = flight
        .sim_epoch()
        .start_utc()
        .expect("epoch")
        .unix_seconds();
    let observer = flight.vehicle_state().position_ecef;
    let frame_at = |elapsed_s: f64| {
        let mut clock = CelestialClock::default();
        let celestial = clock.evaluate(
            SkyTimeMode::Auto,
            ReferenceBody::Earth,
            observer,
            elapsed_s,
            Some(base_utc_s),
            LIVE_UTC_S,
        );
        environment_frame(&celestial, BodyEllipsoid::WGS84)
    };
    let start = frame_at(0.0);
    assert_eq!(frame_at(0.0), start, "one instant gives one frame");
    let later = frame_at(600.0);
    let (a, b) = (start.sun.direction_fixed, later.sun.direction_fixed);
    // Ten minutes of solar day: the Sun keeps its declination and swings 2.5° in longitude.
    assert!((a.z - b.z).abs() < 1e-4, "{a} -> {b}");
    let swing = 2.5_f64.to_radians();
    let expected = 2.0 * ((1.0 - a.z * a.z).sqrt() * (swing / 2.0).sin()).asin();
    let moved = a.angle_between(b);
    assert!(
        (moved - expected).abs() < 0.05_f64.to_radians(),
        "moved {}°, expected {}°",
        moved.to_degrees(),
        expected.to_degrees()
    );
    assert!(
        a.cross(b).z < 0.0,
        "the Sun moves west in the Earth-fixed frame"
    );
}
