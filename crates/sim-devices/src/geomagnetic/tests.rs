use sim_geo::{Geodetic, geodetic_to_ecef};

use super::*;

fn ned_ut(environment: &MagneticEnvironment, at: &Geodetic) -> Result<DVec3, DeviceError> {
    let earth = sim_bodies::BodyDescriptor::earth()
        .unwrap()
        .fields()
        .magnetic;
    let fixed = environment.field_fixed_ut(earth.as_ref(), geodetic_to_ecef(at))?;
    Ok(LocalFrame::at(at).ecef_to_ned_vector(fixed))
}

#[test]
fn the_field_over_delaware_points_north_and_down_and_slightly_west() {
    let dover = Geodetic::from_degrees(39.1582, -75.5244, 15.0);
    let field = ned_ut(&MagneticEnvironment::main_field(2026.5), &dover).unwrap();
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
    let refused = ned_ut(&MagneticEnvironment::main_field(2031.0), &dover).unwrap_err();
    assert!(matches!(
        refused,
        DeviceError::NoMagneticField {
            source: sim_geo::MagneticFieldError::OutsideValidity { decimal_year, .. }
        } if decimal_year == 2031.0
    ));
}

#[test]
fn a_declared_local_anomaly_adds_to_the_main_field() {
    let dover = Geodetic::from_degrees(39.1582, -75.5244, 15.0);
    let main = MagneticEnvironment::main_field(2026.5);
    let anomaly = DVec3::new(0.0, 2.5, 0.0);
    let local = MagneticEnvironment {
        local_anomaly_ned_ut: anomaly,
        ..main
    };
    let moved = ned_ut(&local, &dover).unwrap() - ned_ut(&main, &dover).unwrap();
    assert!((moved - anomaly).length() < 1e-12);
}
