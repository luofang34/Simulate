use super::*;

fn assert_round_trip(position: Geodetic) {
    let back = ecef_to_geodetic(geodetic_to_ecef(&position));
    assert!(
        (back.lat_rad - position.lat_rad).abs() < 1e-11,
        "lat {position:?} -> {back:?}"
    );
    assert!(
        (back.height_m - position.height_m).abs() < 1e-3,
        "height {position:?} -> {back:?}"
    );
    if position.lat_rad.abs() < std::f64::consts::FRAC_PI_2 - 1e-9 {
        assert!(
            (back.lon_rad - position.lon_rad).abs() < 1e-11,
            "lon {position:?} -> {back:?}"
        );
    }
}

#[test]
fn equator_prime_meridian_is_semi_major_axis() {
    let ecef = geodetic_to_ecef(&Geodetic::from_degrees(0.0, 0.0, 0.0));
    assert!((ecef.x - WGS84_A).abs() < 1e-9);
    assert!(ecef.y.abs() < 1e-9 && ecef.z.abs() < 1e-9);
}

#[test]
fn north_pole_is_semi_minor_axis() {
    let ecef = geodetic_to_ecef(&Geodetic::from_degrees(90.0, 0.0, 0.0));
    assert!((ecef.z - WGS84_B).abs() < 1e-6);
    let back = ecef_to_geodetic(ecef);
    assert!((back.lat_deg() - 90.0).abs() < 1e-12);
    assert!(back.height_m.abs() < 1e-6);
}

#[test]
fn round_trips_across_latitudes_and_heights() {
    for lat in [-89.9, -60.0, -33.9, 0.0, 12.5, 45.0, 51.47, 78.2, 89.999] {
        for lon in [-179.9, -122.38, 0.0, 2.35, 139.78] {
            for height in [-420.0, 0.0, 35.0, 3_000.0, 12_500.0, 400_000.0] {
                assert_round_trip(Geodetic::from_degrees(lat, lon, height));
            }
        }
    }
}

#[test]
fn south_pole_axis_point_reports_negative_latitude() {
    let back = ecef_to_geodetic(DVec3::new(0.0, 0.0, -WGS84_B - 100.0));
    assert!((back.lat_deg() + 90.0).abs() < 1e-12);
    assert!((back.height_m - 100.0).abs() < 1e-6);
}

#[test]
fn height_moves_along_the_geodetic_normal() {
    let lat = 37.6188_f64.to_radians();
    let lon = -122.3756_f64.to_radians();
    let ground = geodetic_to_ecef(&Geodetic {
        lat_rad: lat,
        lon_rad: lon,
        height_m: 0.0,
    });
    let raised = geodetic_to_ecef(&Geodetic {
        lat_rad: lat,
        lon_rad: lon,
        height_m: 1_000.0,
    });
    let normal = DVec3::new(lat.cos() * lon.cos(), lat.cos() * lon.sin(), lat.sin());
    assert!((raised - ground - normal * 1_000.0).length() < 1e-6);
}
