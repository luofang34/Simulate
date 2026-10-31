use super::*;
use crate::{BodyEllipsoid, Geodetic};

const APOLLO_11_LANDING_UNIX: f64 = -14_182_940.0;
const MOON_J2000_DEG: [f64; 3] = [266.857_733_444_95, 65.641_102_747_85, 41.195_263_980_75];
const MOON_APOLLO_11_DEG: [f64; 3] = [270.313_356_504_31, 68.090_278_584_76, -146_504.870_410_401];

fn assert_angle_deg(actual_rad: f64, expected_deg: f64, tolerance_deg: f64) {
    let difference = (actual_rad.to_degrees() - expected_deg + 180.0).rem_euclid(360.0) - 180.0;
    assert!(
        difference.abs() < tolerance_deg,
        "{} vs {expected_deg}",
        actual_rad.to_degrees()
    );
}

// Expected angles come from ERFA's `era00`.
#[test]
fn earth_rotation_angle_matches_erfa() {
    let at = UtcTime::from_unix_seconds;
    assert_angle_deg(
        earth_rotation_angle_rad(at(J2000_UNIX_SECONDS)),
        280.460_618_375_04,
        1e-9,
    );
    assert_angle_deg(
        earth_rotation_angle_rad(at(APOLLO_11_LANDING_UNIX)),
        243.238_046_363_25,
        1e-8,
    );
    assert_angle_deg(
        earth_rotation_angle_rad(at(1_793_318_400.0)),
        37.982_623_201_51,
        1e-8,
    );
}

#[test]
fn earth_spins_at_the_wgs84_rate() {
    let start = UtcTime::from_unix_seconds(0.0);
    let step = 1_000.0;
    let swept = earth_rotation_angle_rad(start.advanced_by(step)) - earth_rotation_angle_rad(start);
    let rate = swept.rem_euclid(TAU) / step;
    let relative = (rate - BodyEllipsoid::WGS84.omega_rad_s).abs() / rate;
    assert!(relative < 1e-7, "{rate}");
}

#[test]
fn earth_transform_turns_the_vernal_equinox_westward_by_the_angle() {
    let utc = UtcTime::from_unix_seconds(J2000_UNIX_SECONDS);
    let fixed = InertialTransform::earth_at(utc).j2000_to_fixed(DVec3::X);
    // The equinox lies at Greenwich hour angle ERA, so its body-fixed
    // longitude is minus that angle.
    assert_angle_deg(fixed.y.atan2(fixed.x), -280.460_618_375_04, 1e-9);
    assert!(fixed.z.abs() < 1e-15);
}

// Expected angles evaluate the IAU 2009 series as published in poliastro's
// `moon_rot_elements_at_epoch`.
#[test]
fn moon_orientation_matches_the_iau_series() {
    let fixtures = [
        (J2000_UNIX_SECONDS, MOON_J2000_DEG),
        (APOLLO_11_LANDING_UNIX, MOON_APOLLO_11_DEG),
    ];
    for (unix, expected) in fixtures {
        let actual = moon_orientation_rad(UtcTime::from_unix_seconds(unix));
        for (angle, degrees) in actual.into_iter().zip(expected) {
            assert_angle_deg(angle, degrees, 1e-9);
        }
    }
}

#[test]
fn moon_pole_points_at_the_iau_pole() {
    let utc = UtcTime::from_unix_seconds(APOLLO_11_LANDING_UNIX);
    let [right_ascension, declination, _] = moon_orientation_rad(utc);
    let pole = InertialTransform::moon_at(utc).fixed_to_j2000(DVec3::Z);
    assert!((pole.z.asin() - declination).abs() < 1e-12);
    assert_angle_deg(pole.y.atan2(pole.x), right_ascension.to_degrees(), 1e-9);
}

#[test]
fn moon_prime_meridian_is_measured_from_the_ascending_node() {
    let utc = UtcTime::from_unix_seconds(APOLLO_11_LANDING_UNIX);
    let [right_ascension, _, prime_meridian] = moon_orientation_rad(utc);
    let node = DVec3::new(
        (right_ascension + FRAC_PI_2).cos(),
        (right_ascension + FRAC_PI_2).sin(),
        0.0,
    );
    let fixed = InertialTransform::moon_at(utc).j2000_to_fixed(node);
    assert_angle_deg(fixed.y.atan2(fixed.x), -prime_meridian.to_degrees(), 1e-9);
    assert!(fixed.z.abs() < 1e-12);
}

#[test]
fn moon_mean_rate_matches_the_ellipsoid_constant() {
    // One 18.6-year nodal period cancels the largest periodic term.
    let days = 6_798.38;
    let start = UtcTime::from_unix_seconds(J2000_UNIX_SECONDS);
    let end = start.advanced_by(days * SECONDS_PER_DAY);
    let [_, _, begin] = moon_orientation_rad(start);
    let [_, _, finish] = moon_orientation_rad(end);
    let rate = (finish - begin) / (days * SECONDS_PER_DAY);
    let relative = (rate - BodyEllipsoid::MOON_IAU.omega_rad_s).abs() / rate;
    assert!(relative < 1e-5, "{rate}");
}

#[test]
fn transforms_round_trip_and_preserve_length() {
    let utc = UtcTime::from_unix_seconds(APOLLO_11_LANDING_UNIX);
    let site =
        BodyEllipsoid::MOON_IAU.geodetic_to_fixed(&Geodetic::from_degrees(0.67416, 23.47314, 0.0));
    for transform in [
        InertialTransform::earth_at(utc),
        InertialTransform::moon_at(utc),
    ] {
        let inertial = transform.fixed_to_j2000(site);
        assert!((inertial.length() - site.length()).abs() < 1e-8);
        assert!((transform.j2000_to_fixed(inertial) - site).length() < 1e-8);
        let rotated = transform.fixed_to_j2000_rotation() * site;
        assert!((rotated - inertial).length() < 1e-8);
    }
}
