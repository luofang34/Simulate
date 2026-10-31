use indicate_instrument_state::down_in_body;
use sim_core::VehicleState;
use sim_geo::{BodyEllipsoid, UtcTime};

use super::*;

fn degrees(radians: f64) -> f64 {
    radians.to_degrees()
}

#[test]
fn level_flight_reads_wings_level_on_its_heading_anywhere() {
    for (body, lat, lon, height) in [
        (BodyEllipsoid::WGS84, 39.5, -75.5, 3_000.0),
        (BodyEllipsoid::WGS84, -62.0, 171.0, 11_000.0),
    ] {
        let at = Geodetic::from_degrees(lat, lon, height);
        let state = VehicleState::level_flight(&at, 1.2, 100.0);
        let local = LocalLevel::at(&body, state.position_ecef);
        let attitude = local.attitude_from_fixed(state.attitude, &DisplayAxes::AEROSPACE);
        let (bank, pitch) = bank_and_pitch_rad(attitude);
        assert!(bank.abs() < 1e-9 && pitch.abs() < 1e-9, "{bank} {pitch}");
        assert!((true_heading_rad(attitude).unwrap() - 1.2).abs() < 1e-9);
        let velocity = local.vector_in_ned(state.velocity_ecef);
        assert!((velocity.y.atan2(velocity.x) - 1.2).abs() < 1e-9);
        assert!(velocity.z.abs() < 1e-7, "{velocity}");
    }
}

#[test]
fn rotating_about_the_nose_to_the_right_banks_right_with_indicates_signs() {
    let attitude = DQuat::from_rotation_x(30f64.to_radians());
    let (bank, pitch) = bank_and_pitch_rad(attitude);
    assert!((degrees(bank) - 30.0).abs() < 1e-9);
    assert!(pitch.abs() < 1e-12);
    let shown = indicate_quat(attitude);
    let (_, down_y, _) = down_in_body(shown);
    assert!(down_y > 0.0, "the ground swings to the right wing");
    let (roll, _, _) = shown.to_euler();
    assert!((roll.to_degrees() - 30.0).abs() < 1e-4);
}

#[test]
fn climbing_is_nose_up() {
    let (_, pitch) = bank_and_pitch_rad(DQuat::from_rotation_y(0.2));
    assert!((pitch - 0.2).abs() < 1e-12);
}

#[test]
fn the_lm_reads_level_with_its_thrust_axis_vertical() {
    let body = BodyEllipsoid::MOON_IAU;
    let at = Geodetic::from_degrees(0.7, 23.5, 15_000.0);
    let local = LocalLevel::at(&body, body.geodetic_to_fixed(&at));
    let frame = LocalFrame::at(&at);
    // Thrust axis up, hatch north, crew's right east.
    let lm_to_fixed = DQuat::from_mat3(&DMat3::from_cols(frame.up(), frame.east(), frame.north()));
    let attitude = local.attitude_from_fixed(lm_to_fixed, &DisplayAxes::apollo_lm());
    let (bank, pitch) = bank_and_pitch_rad(attitude);
    assert!(bank.abs() < 1e-9 && pitch.abs() < 1e-9, "{bank} {pitch}");
    assert!(true_heading_rad(attitude).unwrap().abs() < 1e-9);
    let rate = DisplayAxes::apollo_lm().vector_in_display(DVec3::Z);
    assert!((rate - DVec3::X).length() < 1e-12, "LM +z is the roll axis");
}

#[test]
fn the_inertial_path_lands_on_the_fixed_path() {
    let body = BodyEllipsoid::MOON_IAU;
    let frame = InertialTransform::moon_at(UtcTime::from_unix_seconds(-14_182_940.0));
    let local = LocalLevel::at(&body, DVec3::new(1.6e6, 7.0e5, 1.0e5));
    let vehicle_to_fixed = DQuat::from_euler(glam::EulerRot::XYZ, 0.3, -0.2, 1.1);
    let vehicle_to_inertial = frame.fixed_to_j2000_rotation() * vehicle_to_fixed;
    let direct = local.attitude_from_fixed(vehicle_to_fixed, &DisplayAxes::AEROSPACE);
    let through =
        local.attitude_from_inertial(vehicle_to_inertial, &frame, &DisplayAxes::AEROSPACE);
    assert!(direct.angle_between(through) < 1e-7);
}

#[test]
fn heading_stops_at_the_vertical() {
    let straight_up = DQuat::from_rotation_y(90f64.to_radians());
    assert_eq!(true_heading_rad(straight_up), None);
    assert_eq!(heading_rate_rad_s(straight_up, DVec3::Z), None);
}

#[test]
fn a_banked_coordinated_turn_reads_its_heading_rate() {
    let bank = 30f64.to_radians();
    let turn_rate = 0.05;
    let earth_rate_in_body = DQuat::from_rotation_x(bank).inverse() * (DVec3::Z * turn_rate);
    let rate = heading_rate_rad_s(DQuat::from_rotation_x(bank), earth_rate_in_body).unwrap();
    assert!((rate - turn_rate).abs() < 1e-12);
}
