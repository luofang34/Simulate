use glam::DVec3;
use sim_core::VehicleState;
use sim_geo::Geodetic;
use sim_scenario::OperatorViewportMode;

use super::{LookOffset, viewport_camera};

fn state() -> VehicleState {
    VehicleState::level_flight(&Geodetic::from_degrees(39.1, -75.5, 500.0), 0.5, 50.0)
}

#[test]
fn look_integrates_clamps_and_resets() {
    let mut look = LookOffset::default();
    look.integrate([1.0, 0.0], 0.5);
    assert!((look.yaw_rad - 60_f64.to_radians()).abs() < 1e-12);
    look.integrate([1.0, 1.0], 10.0);
    assert!((look.yaw_rad - 150_f64.to_radians()).abs() < 1e-12);
    assert!((look.pitch_rad - 75_f64.to_radians()).abs() < 1e-12);
    look.integrate([-1.0, -1.0], f64::NAN);
    assert!((look.yaw_rad - 150_f64.to_radians()).abs() < 1e-12);
    look.reset();
    assert_eq!(look, LookOffset::default());
}

#[test]
fn cockpit_stick_look_tracks_deflection_and_recentres_on_release() {
    let mut look = LookOffset::default();
    for _ in 0..30 {
        look.follow_stick([1.0, -0.5], 1.0 / 60.0);
    }
    assert!(look.yaw_rad > 80_f64.to_radians());
    assert!(look.pitch_rad < -20_f64.to_radians());
    for _ in 0..90 {
        look.follow_stick([0.0, 0.0], 1.0 / 60.0);
    }
    assert_eq!(look, LookOffset::default());
}

#[test]
fn look_turns_the_cockpit_eye_and_orbits_the_chase_camera() {
    let state = state();
    let look = LookOffset {
        yaw_rad: 90_f64.to_radians(),
        pitch_rad: 0.0,
    };
    let aboard = super::pilot_eye_mount().pose(&state, DVec3::ZERO);
    let cockpit = viewport_camera(OperatorViewportMode::Cockpit, &state, aboard);
    let turned = look.apply(OperatorViewportMode::Cockpit, cockpit, &state);
    assert_eq!(turned.pose.position_ecef, cockpit.pose.position_ecef);
    let forward = cockpit.pose.orientation * DVec3::NEG_Z;
    let right = cockpit.pose.orientation * DVec3::X;
    let now_forward = turned.pose.orientation * DVec3::NEG_Z;
    assert!(now_forward.dot(right) > 0.999, "pans right");
    assert!(now_forward.dot(forward).abs() < 1e-9);

    let chase = viewport_camera(OperatorViewportMode::Chase, &state, aboard);
    let orbited = look.apply(OperatorViewportMode::Chase, chase, &state);
    let range = |c: &sim_render::Camera| (c.pose.position_ecef - state.position_ecef).length();
    assert!((range(&orbited) - range(&chase)).abs() < 1e-6);
    assert!((orbited.pose.position_ecef - chase.pose.position_ecef).length() > 10.0);
    let sensor = viewport_camera(OperatorViewportMode::SensorAperture, &state, aboard);
    assert_eq!(
        look.apply(OperatorViewportMode::SensorAperture, sensor, &state),
        sensor
    );
}

/// Descending vertically, the Lunar Module's thrust axis points up; the external views
/// take its bearing from the crew windows and stand level on the Moon behind it.
#[test]
fn lunar_chase_frames_the_lander_from_behind_its_windows() {
    use glam::{DMat3, DQuat};
    use sim_geo::{BodyEllipsoid, LocalFrame};
    use sim_scenario::ScenarioProfile;

    use super::{DEFAULT_CHASE_BACK_UP_M, viewport_camera_for};

    let moon = BodyEllipsoid::MOON_IAU;
    let site = Geodetic::from_degrees(0.674, 23.473, 150.0);
    let frame = LocalFrame::at(&site);
    let lm = VehicleState {
        position_ecef: moon.geodetic_to_fixed(&site),
        velocity_ecef: -frame.up(),
        attitude: DQuat::from_mat3(&DMat3::from_cols(frame.up(), frame.east(), frame.north())),
        angular_rate_body: DVec3::ZERO,
    };
    let aboard = super::pilot_eye_mount().pose(&lm, DVec3::ZERO);
    let profile = Some(ScenarioProfile::Apollo11LunarLanding);
    for (mode, back_m, up_m) in [
        (OperatorViewportMode::Chase, 22.0, 6.5),
        (OperatorViewportMode::VolumetricScene, 140.0, 85.0),
    ] {
        let camera = viewport_camera_for(mode, profile, DEFAULT_CHASE_BACK_UP_M, &lm, aboard);
        let offset = camera.pose.position_ecef - lm.position_ecef;
        assert!(
            (offset.dot(frame.north()) + back_m).abs() < 1e-6,
            "{mode:?}"
        );
        assert!((offset.dot(frame.up()) - up_m).abs() < 1e-6, "{mode:?}");
        let looking = camera.pose.orientation * DVec3::NEG_Z;
        assert!(
            looking.dot(-offset.normalize()) > 0.999,
            "{mode:?} looks at the LM"
        );
        let height = moon.fixed_to_geodetic(camera.pose.position_ecef).height_m;
        assert!(height > site.height_m, "{mode:?} at {height} m");
    }
    let chart = viewport_camera_for(
        OperatorViewportMode::ChartMap,
        profile,
        DEFAULT_CHASE_BACK_UP_M,
        &lm,
        aboard,
    );
    let above = (chart.pose.position_ecef - lm.position_ecef).dot(frame.up());
    assert!((above - 4_500.0).abs() < 1e-6);
}
