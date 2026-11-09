use glam::DVec3;
use sim_geo::{BodyEllipsoid, Geodetic, LocalFrame};

use super::super::{
    FlybyTracker, VirtualCameraPlacement, VirtualCameraRigBlock, VirtualCameraSpec,
};
use crate::{Block, CAMERA_DEFAULT_PERIOD, CameraMount, CameraPoseEcef, EnuAnchor, VehicleState};

const FLYBY: FlybyTracker = FlybyTracker {
    grid_step_m: 600.0,
    lateral_offset_m: 220.0,
    vertical_offset_m: 4.0,
    lead_time_s: 0.0,
    reference_distance_m: 300.0,
    reference_fov_y_deg: 20.0,
    fov_clamp_deg: (4.0, 50.0),
};

fn spec(placement: VirtualCameraPlacement) -> VirtualCameraSpec {
    VirtualCameraSpec {
        id: "tracker".into(),
        title: "Tracker".into(),
        placement,
        airframe: crate::AirframeVisibility::External,
        base_intrinsics: VirtualCameraSpec::pinhole_800x600(50.0_f64.to_radians()),
        period: CAMERA_DEFAULT_PERIOD,
        depth_readout: true,
    }
}

/// Angle (`rad`) between the optical axis and the line of sight to `target`.
fn off_axis_rad(pose: &CameraPoseEcef, target: DVec3) -> f64 {
    let axis = pose.orientation * DVec3::NEG_Z;
    let sight = (target - pose.position_ecef).normalize();
    axis.cross(sight).length().atan2(axis.dot(sight))
}

fn on_body(body: &BodyEllipsoid, geo: &Geodetic, heading_rad: f64, speed: f64) -> VehicleState {
    let frame = LocalFrame::at(geo);
    let (s, c) = heading_rad.sin_cos();
    let along = frame.north() * c + frame.east() * s;
    VehicleState {
        position_ecef: body.geodetic_to_fixed(geo),
        velocity_ecef: along * speed,
        ..VehicleState::level_flight(geo, heading_rad, speed)
    }
}

#[test]
fn a_flyby_observer_holds_still_while_the_vehicle_passes_then_steps_ahead() {
    for start in [
        Geodetic::from_degrees(-12.0, -150.0, 900.0),
        Geodetic::from_degrees(45.3, 7.9, 2_400.0),
    ] {
        assert_flyby_holds_still(&start, 0.6);
    }
}

fn assert_flyby_holds_still(start: &Geodetic, heading: f64) {
    let body = BodyEllipsoid::WGS84;
    let tracker = spec(VirtualCameraPlacement::FlybyAutoTrack(FLYBY));
    let first = on_body(&body, start, heading, 60.0);
    let along = first.velocity_ecef.normalize();
    let mut previous: Option<CameraPoseEcef> = None;
    let mut steps_ahead = 0_u32;
    for tick in 0..300 {
        let mut state = first;
        state.position_ecef += along * (2.0 * f64::from(tick));
        let (pose, intrinsics) = tracker
            .pose_and_intrinsics_on(&body, &state, DVec3::ZERO)
            .unwrap();
        assert!(off_axis_rad(&pose, state.position_ecef) < 1e-9);
        let range = (pose.position_ecef - state.position_ecef).length();
        assert!(range < 0.5 * 600.0 + 230.0, "tick {tick}: {range} m");
        assert!(VirtualCameraSpec::vertical_fov_rad(&intrinsics) <= 50.0_f64.to_radians() + 1e-9);
        if let Some(previous) = previous {
            let moved = (pose.position_ecef - previous.position_ecef).length();
            if moved > 1.0 {
                // A new station: one cell further along the track.
                let ahead = (pose.position_ecef - previous.position_ecef).dot(along);
                assert!(
                    (ahead - 600.0).abs() < 10.0,
                    "tick {tick}: stepped {ahead} m"
                );
                steps_ahead = steps_ahead.wrapping_add(1);
            } else {
                assert!(moved < 0.05, "tick {tick}: drifted {moved} m in one step");
            }
        }
        previous = Some(pose);
    }
    assert!(
        (1..=2).contains(&steps_ahead),
        "{steps_ahead} station changes over 600 m"
    );
}

#[test]
fn a_range_limited_tower_tracks_nearby_and_hands_over_to_a_flyby_far_away() {
    let body = BodyEllipsoid::WGS84;
    let tower = Geodetic::from_degrees(40.692_574, -74.182_761, 64.5);
    let placement = VirtualCameraPlacement::RangeLimitedAutoTrack {
        position: tower,
        lead_time_s: 0.0,
        auto_zoom_span_m: Some(50.0),
        min_fov_y_rad: 3.5_f64.to_radians(),
        max_fov_y_rad: 48.0_f64.to_radians(),
        max_range_m: 18_000.0,
        fallback: FLYBY,
    };
    let camera = spec(placement);
    let tower_ecef = body.geodetic_to_fixed(&tower);

    let near = Geodetic::from_degrees(40.697, -74.182_761, 300.0);
    let state = on_body(&body, &near, 0.0, 50.0);
    let (pose, _) = camera.pose_and_intrinsics(&state, DVec3::ZERO).unwrap();
    assert!((pose.position_ecef - tower_ecef).length() < 1e-6);
    assert!(off_axis_rad(&pose, state.position_ecef) < 1e-9);

    // 50 km away and over the far side of the Earth: never aim through it.
    for far in [
        Geodetic::from_degrees(41.14, -74.182_761, 900.0),
        Geodetic::from_degrees(-40.0, 105.0, 900.0),
    ] {
        let state = on_body(&body, &far, 1.0, 60.0);
        let (pose, _) = camera.pose_and_intrinsics(&state, DVec3::ZERO).unwrap();
        let range = (pose.position_ecef - state.position_ecef).length();
        assert!(range < 18_000.0, "{range} m");
        assert!(off_axis_rad(&pose, state.position_ecef) < 1e-9);
        let up = LocalFrame::at(&body.fixed_to_geodetic(pose.position_ecef)).up();
        assert!(
            (pose.orientation * DVec3::NEG_Z).dot(up) > -0.2,
            "aims underground"
        );
    }
}

#[test]
fn a_lunar_rig_places_world_cameras_on_the_moon_and_frames_the_lander() {
    let moon = BodyEllipsoid::MOON_IAU;
    let site = Geodetic::from_degrees(0.674_16, 23.473_14, 3.0);
    let lander = Geodetic::from_degrees(0.675, 23.50, 500.0);
    let mut state = on_body(&moon, &lander, -std::f64::consts::FRAC_PI_2, 40.0);
    // The lander thrusts along body +x, straight up.
    let up = LocalFrame::at(&lander).up();
    state.attitude = glam::DQuat::from_rotation_arc(DVec3::X, up);
    let specs = vec![
        spec(VirtualCameraPlacement::WorldAutoTrack {
            position: site,
            lead_time_s: 0.0,
            auto_zoom_span_m: Some(40.0),
            min_fov_y_rad: 2.0_f64.to_radians(),
            max_fov_y_rad: 50.0_f64.to_radians(),
        }),
        spec(VirtualCameraPlacement::FlybyAutoTrack(FLYBY)),
        spec(VirtualCameraPlacement::BodyMounted(
            CameraMount::looking_along(DVec3::ZERO, DVec3::NEG_X, DVec3::Z),
        )),
    ];
    let anchor = EnuAnchor::new(&moon, &site);
    let mut rig = VirtualCameraRigBlock::at_default_rate(specs, anchor)
        .unwrap()
        .with_ellipsoid(moon);
    assert_eq!(rig.ellipsoid(), moon);
    rig.step(&(state, DVec3::ZERO)).unwrap();
    let captures = rig.output();
    for capture in &captures[..2] {
        let radius = capture.pose.position_ecef.length();
        assert!((radius - moon.semi_major_m).abs() < 2_000.0, "{radius} m");
        assert!(off_axis_rad(&capture.pose, state.position_ecef) < 1e-9);
    }
    let nadir = captures[2].pose.orientation * DVec3::NEG_Z;
    assert!(nadir.dot(-up) > 0.999, "the -x camera looks at the ground");
}
