use super::*;

#[test]
fn the_field_over_delaware_points_north_and_down_and_slightly_west() {
    let dover = Geodetic::from_degrees(39.1582, -75.5244, 15.0);
    let field = earth_field_ned_ut(&dover, 2026.5).unwrap();
    assert!(field.x > 15.0 && field.x < 25.0, "north {} uT", field.x);
    assert!(field.z > 40.0 && field.z < 50.0, "down {} uT", field.z);
    let declination_deg = field.y.atan2(field.x).to_degrees();
    assert!(
        (-14.0..-9.0).contains(&declination_deg),
        "declination {declination_deg} deg"
    );
}

#[test]
fn a_date_the_model_does_not_cover_is_refused_with_its_context() {
    let dover = Geodetic::from_degrees(39.1582, -75.5244, 15.0);
    let refused = earth_field_ned_ut(&dover, 1970.0).unwrap_err();
    assert!(matches!(
        refused,
        DeviceError::NoMagneticField { decimal_year, .. } if decimal_year == 1970.0
    ));
}

#[test]
fn a_declared_local_anomaly_adds_to_the_main_field() {
    let dover = Geodetic::from_degrees(39.1582, -75.5244, 15.0);
    let main = MagneticEnvironment::main_field(2026.5);
    assert_eq!(
        main.field_ned_ut(&dover).unwrap(),
        earth_field_ned_ut(&dover, 2026.5).unwrap()
    );
    let anomaly = DVec3::new(0.0, 2.5, 0.0);
    let local = MagneticEnvironment {
        local_anomaly_ned_ut: anomaly,
        ..main
    };
    let moved = local.field_ned_ut(&dover).unwrap() - main.field_ned_ut(&dover).unwrap();
    assert!((moved - anomaly).length() < 1e-12);
}
