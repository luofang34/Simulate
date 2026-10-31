use std::f64::consts::{FRAC_PI_2, PI};

use mbd_core::compute_site_kinematics;
use sim_core::VehicleState;
use sim_geo::WGS84_A;

use super::*;

fn origin() -> Geodetic {
    Geodetic::from_degrees(39.1, -75.6, 100.0)
}

fn world() -> EnuWorld {
    EnuWorld::new(&BodyEllipsoid::WGS84, &origin())
}

fn at_cg() -> LocalSiteKinematics {
    site_at(DVec3::ZERO, DQuat::IDENTITY, DVec3::ZERO)
}

fn site_at(offset: DVec3, rotation: DQuat, omega: DVec3) -> LocalSiteKinematics {
    compute_site_kinematics(
        DVec3::ZERO,
        omega,
        DVec3::ZERO,
        (offset, rotation),
        DVec3::ZERO,
    )
}

use mbd_core::LocalSiteKinematics;

fn stimulus(vehicle: VehicleState, site: LocalSiteKinematics) -> AviateStimulus {
    AviateStimulus { vehicle, site }
}

fn assert_close(actual: [f64; 3], expected: [f64; 3], tolerance: f64) {
    let gap = DVec3::from(actual) - DVec3::from(expected);
    assert!(gap.length() < tolerance, "{actual:?} vs {expected:?}");
}

#[test]
fn the_world_origin_is_the_zero_of_position() {
    let state = VehicleState::level_flight(&origin(), 0.0, 0.0);
    let lanes = world().model_lanes(&stimulus(state, at_cg()));
    assert_close(lanes.position_enu_m, [0.0; 3], 1e-6);
}

#[test]
fn nose_east_and_wings_level_is_the_identity_rotation() {
    let state = VehicleState::level_flight(&origin(), FRAC_PI_2, 0.0);
    let lanes = world().model_lanes(&stimulus(state, at_cg()));
    let [w, x, y, z] = lanes.attitude_wxyz;
    assert!(w.abs() > 1.0 - 1e-12, "{:?}", lanes.attitude_wxyz);
    assert!(
        x.abs() + y.abs() + z.abs() < 1e-6,
        "{:?}",
        lanes.attitude_wxyz
    );
}

#[test]
fn nose_north_maps_flu_forward_onto_enu_north_and_up_onto_up() {
    let state = VehicleState::level_flight(&origin(), 0.0, 60.0);
    let lanes = world().model_lanes(&stimulus(state, at_cg()));
    let [w, x, y, z] = lanes.attitude_wxyz;
    let site_to_enu = DQuat::from_xyzw(x, y, z, w);
    assert!((site_to_enu * DVec3::X - DVec3::Y).length() < 1e-9);
    assert!((site_to_enu * DVec3::Z - DVec3::Z).length() < 1e-9);
    assert!(
        (site_to_enu * DVec3::Y + DVec3::X).length() < 1e-9,
        "left is west"
    );
    assert_close(lanes.velocity_enu_m_s, [0.0, 60.0, 0.0], 1e-9);
}

#[test]
fn a_climb_above_the_origin_reads_up() {
    let high = Geodetic::from_degrees(39.1, -75.6, 350.0);
    let state = VehicleState::level_flight(&high, 0.0, 0.0);
    let lanes = world().model_lanes(&stimulus(state, at_cg()));
    assert_close(lanes.position_enu_m, [0.0, 0.0, 250.0], 1e-6);
}

#[test]
fn lanes_describe_the_site_rather_than_the_centre_of_gravity() {
    // One metre right of a north-pointing vehicle is one metre east.
    let state = VehicleState::level_flight(&origin(), 0.0, 0.0);
    let site = site_at(DVec3::Y, DQuat::IDENTITY, DVec3::ZERO);
    let lanes = world().model_lanes(&stimulus(state, site));
    assert_close(lanes.position_enu_m, [1.0, 0.0, 0.0], 1e-6);
}

#[test]
fn a_site_mounted_turned_reports_its_own_axes() {
    let state = VehicleState::level_flight(&origin(), FRAC_PI_2, 0.0);
    let site = site_at(DVec3::ZERO, DQuat::from_rotation_z(FRAC_PI_2), DVec3::ZERO);
    let lanes = world().model_lanes(&stimulus(state, site));
    let [w, x, y, z] = lanes.attitude_wxyz;
    let site_to_enu = DQuat::from_xyzw(x, y, z, w);
    assert!(
        (site_to_enu * DVec3::X + DVec3::Y).length() < 1e-9,
        "faces south"
    );
}

#[test]
fn a_spinning_vehicle_moves_an_offset_site_at_its_earth_relative_rate() {
    // Yawing right at 1 rad/s nose north: the right-hand site swings south.
    let mut state = VehicleState::level_flight(&origin(), 0.0, 0.0);
    state.angular_rate_body = DVec3::Z;
    let earth_rate_in_body =
        state.attitude.inverse() * (BodyEllipsoid::WGS84.omega_rad_s * DVec3::Z);
    let inertial = state.angular_rate_body + earth_rate_in_body;
    let site = site_at(DVec3::Y, DQuat::IDENTITY, inertial);
    let lanes = world().model_lanes(&stimulus(state, site));
    assert_close(lanes.velocity_enu_m_s, [0.0, -1.0, 0.0], 1e-9);
}

#[test]
fn the_rate_lane_is_world_axes_and_omits_the_earth_rate() {
    // A right yaw is clockwise from above: negative about ENU up, whatever
    // the heading, and no Earth rate appears on a parked vehicle.
    for heading in [0.0, 1.1, PI] {
        let mut state = VehicleState::level_flight(&origin(), heading, 0.0);
        state.angular_rate_body = DVec3::Z * 0.5;
        let lanes = world().model_lanes(&stimulus(state, at_cg()));
        assert_close(lanes.angular_rate_enu_rad_s, [0.0, 0.0, -0.5], 1e-9);
        state.angular_rate_body = DVec3::ZERO;
        let parked = world().model_lanes(&stimulus(state, at_cg()));
        assert_eq!(parked.angular_rate_enu_rad_s, [0.0; 3]);
    }
}

#[test]
fn a_site_already_in_ecef_is_not_transformed_twice() {
    let state = VehicleState::level_flight(&origin(), 0.7, 30.0);
    let site = site_at(DVec3::new(0.2, -0.1, 0.3), DQuat::IDENTITY, DVec3::ZERO);
    let world = world();
    let body_frame = world.model_lanes(&stimulus(state, site));
    let in_ecef = site.in_ecef(state.position_ecef, state.velocity_ecef, state.attitude);
    assert_eq!(world.model_lanes(&stimulus(state, in_ecef)), body_frame);
}

#[test]
fn a_body_other_than_the_default_keeps_its_own_origin() {
    let body = BodyEllipsoid {
        semi_major_m: WGS84_A / 4.0,
        ..BodyEllipsoid::WGS84
    };
    let at = origin();
    let world = EnuWorld::new(&body, &at);
    let state = VehicleState {
        position_ecef: body.geodetic_to_fixed(&at),
        ..VehicleState::level_flight(&at, 0.0, 0.0)
    };
    let lanes = world.model_lanes(&stimulus(state, at_cg()));
    assert_close(lanes.position_enu_m, [0.0; 3], 1e-9);
}

#[test]
fn non_finite_motion_shows_in_the_lanes() {
    let mut state = VehicleState::level_flight(&origin(), 0.0, 0.0);
    state.velocity_ecef = DVec3::NAN;
    let lanes = world().model_lanes(&stimulus(state, at_cg()));
    assert!(!lanes.is_finite());
    let good = world().model_lanes(&stimulus(
        VehicleState::level_flight(&origin(), 0.0, 0.0),
        at_cg(),
    ));
    assert!(good.is_finite());
}
