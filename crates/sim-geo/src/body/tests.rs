use super::*;
use crate::{WGS84_B, ecef_to_geodetic, geodetic_to_ecef};

const TRANQUILITY_LAT_DEG: f64 = 0.67416;
const TRANQUILITY_LON_DEG: f64 = 23.47314;

fn assert_round_trip(body: &BodyEllipsoid, position: Geodetic) {
    let back = body.fixed_to_geodetic(body.geodetic_to_fixed(&position));
    assert!((back.lat_rad - position.lat_rad).abs() < 1e-11, "{back:?}");
    assert!((back.lon_rad - position.lon_rad).abs() < 1e-11, "{back:?}");
    assert!((back.height_m - position.height_m).abs() < 1e-3, "{back:?}");
}

#[test]
fn sea_of_tranquility_round_trips_on_the_moon() {
    for height in [-2_000.0, 0.0, 15_000.0, 110_000.0] {
        let site = Geodetic::from_degrees(TRANQUILITY_LAT_DEG, TRANQUILITY_LON_DEG, height);
        assert_round_trip(&BodyEllipsoid::MOON_IAU, site);
    }
}

#[test]
fn moon_surface_lies_on_the_mean_sphere() {
    let site = Geodetic::from_degrees(TRANQUILITY_LAT_DEG, TRANQUILITY_LON_DEG, 0.0);
    let fixed = BodyEllipsoid::MOON_IAU.geodetic_to_fixed(&site);
    assert!((fixed.length() - 1_737_400.0).abs() < 1e-6);
}

#[test]
fn sphere_poles_and_axis_points_are_well_defined() {
    let moon = BodyEllipsoid::MOON_IAU;
    let north = moon.fixed_to_geodetic(DVec3::new(0.0, 0.0, 1_737_500.0));
    assert!((north.lat_deg() - 90.0).abs() < 1e-12);
    assert!((north.height_m - 100.0).abs() < 1e-9);
    let south = moon.fixed_to_geodetic(DVec3::new(0.0, 0.0, -1_737_400.0));
    assert!((south.lat_deg() + 90.0).abs() < 1e-12);
    assert!(south.height_m.abs() < 1e-9);
}

#[test]
fn oblate_axis_points_use_this_bodys_semi_minor_axis() {
    let flattened = BodyEllipsoid {
        semi_major_m: 3_396_190.0,
        flattening: 1.0 / 169.89,
        mu_m3_s2: 4.282_837e13,
        omega_rad_s: 7.088_218e-5,
    };
    let pole = flattened.fixed_to_geodetic(DVec3::new(0.0, 0.0, flattened.semi_minor_m() + 50.0));
    assert!((pole.height_m - 50.0).abs() < 1e-9);
    assert_round_trip(&flattened, Geodetic::from_degrees(-14.6, 175.5, 2_500.0));
}

#[test]
fn free_functions_are_the_wgs84_body() {
    let earth = BodyEllipsoid::WGS84;
    assert_eq!(earth.semi_minor_m(), WGS84_B);
    let position = Geodetic::from_degrees(28.5, -80.6, 121_920.0);
    let fixed = geodetic_to_ecef(&position);
    assert_eq!(fixed, earth.geodetic_to_fixed(&position));
    assert_eq!(ecef_to_geodetic(fixed), earth.fixed_to_geodetic(fixed));
    assert_round_trip(&earth, position);
}

#[test]
fn surface_gravity_points_down_at_the_known_strength() {
    let moon = BodyEllipsoid::MOON_IAU;
    let site = moon.geodetic_to_fixed(&Geodetic::from_degrees(
        TRANQUILITY_LAT_DEG,
        TRANQUILITY_LON_DEG,
        0.0,
    ));
    let gravity = moon.gravity_fixed(site);
    assert!((gravity.length() - 1.625).abs() < 5e-3, "{gravity}");
    assert!((gravity.normalize() + site.normalize()).length() < 1e-12);

    let earth = BodyEllipsoid::WGS84;
    let equator = DVec3::new(WGS84_A, 0.0, 0.0);
    assert!((earth.gravity_fixed(equator).length() - 9.798).abs() < 1e-3);
}

#[test]
fn gravity_at_the_centre_is_zero_rather_than_nan() {
    let earth = BodyEllipsoid::WGS84;
    assert_eq!(earth.gravity_fixed(DVec3::ZERO), DVec3::ZERO);
    assert_eq!(earth.gravity_fixed_j2(DVec3::ZERO, WGS84_J2), DVec3::ZERO);
}

/// `μ/r · (1 − J2 (a/r)² (3 sin²φ − 1) / 2)`, the potential the J2 field descends from.
fn j2_potential(body: &BodyEllipsoid, fixed: DVec3) -> f64 {
    let r = fixed.length();
    let sin_lat = fixed.z / r;
    let a_over_r = body.semi_major_m / r;
    body.mu_m3_s2 / r
        * (1.0 - WGS84_J2 * a_over_r * a_over_r * (3.0 * sin_lat * sin_lat - 1.0) / 2.0)
}

#[test]
fn j2_gravity_is_the_gradient_of_its_potential() {
    let earth = BodyEllipsoid::WGS84;
    let point = DVec3::new(4_100_000.0, -3_300_000.0, 4_400_000.0);
    let gravity = earth.gravity_fixed_j2(point, WGS84_J2);
    let step = 10.0;
    let gradient = DVec3::AXES.map(|axis| {
        (j2_potential(&earth, point + axis * step) - j2_potential(&earth, point - axis * step))
            / (2.0 * step)
    });
    let numeric = DVec3::from_array(gradient);
    assert!((gravity - numeric).length() < 1e-6, "{gravity} {numeric}");
}

#[test]
fn zero_j2_is_the_central_field() {
    let moon = BodyEllipsoid::MOON_IAU;
    let point = DVec3::new(1_000_000.0, 900_000.0, 1_100_000.0);
    assert_eq!(moon.gravity_fixed_j2(point, 0.0), moon.gravity_fixed(point));
}
