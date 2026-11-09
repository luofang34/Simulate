use super::feedback::fit_polar;
use super::{AceBridgeError, AceVehicleInputs};
use crate::{PackError, builtin_c172_package, builtin_p38g_package};

fn vehicle(id: &str, engines: f64, area: &str, omit: Option<&str>) -> Vec<u8> {
    let mut params = serde_json::json!({
        "engine.count": { "unit": "1", "value": { "value": engines, "unit": "1" } },
        "wing.area": serde_json::from_str::<serde_json::Value>(area).unwrap_or_default(),
        "wing.span": { "unit": "ft", "value": { "value": 36.0, "unit": "ft" } },
        "aero.cd0": { "unit": "1", "value": { "value": 0.031, "unit": "1" } },
        "aero.oswald": { "unit": "1", "value": { "value": 0.78, "unit": "1" } },
        "propeller.efficiency": { "unit": "1", "value": { "value": 0.8, "unit": "1" } },
        "engine.bsfc": { "unit": "kg/kWh", "value": { "value": 0.26, "unit": "kg/kWh" } }
    });
    if let (Some(key), Some(map)) = (omit, params.as_object_mut()) {
        map.remove(key);
    }
    serde_json::to_vec(&serde_json::json!({ "id": id, "name": id, "parameters": params }))
        .unwrap_or_default()
}

const AREA_M2: &str = r#"{ "unit": "m^2", "value": { "value": 16.17, "unit": "m^2" } }"#;

#[test]
fn units_are_converted_to_si() {
    let inputs =
        AceVehicleInputs::from_vehicle_json("v.json", &vehicle("v", 1.0, AREA_M2, None)).unwrap();
    assert!((inputs.wing_span_m - 36.0 * 0.3048).abs() < 1e-12);
    assert_eq!(inputs.wing_area_m2, 16.17);
    assert_eq!(inputs.engine_count, 1);
    assert_eq!(inputs.bsfc_kg_per_kwh, Some(0.26));
}

#[test]
fn a_missing_required_parameter_is_refused_not_defaulted() {
    let bytes = vehicle("v", 1.0, AREA_M2, Some("aero.cd0"));
    let error = AceVehicleInputs::from_vehicle_json("v.json", &bytes).unwrap_err();
    assert!(
        matches!(&error, AceBridgeError::MissingParameter { parameter, .. } if parameter == "aero.cd0"),
        "{error}"
    );
}

#[test]
fn an_unknown_unit_is_refused() {
    let area = r#"{ "unit": "acre", "value": { "value": 0.004, "unit": "acre" } }"#;
    let error =
        AceVehicleInputs::from_vehicle_json("v.json", &vehicle("v", 1.0, area, None)).unwrap_err();
    assert!(
        matches!(&error, AceBridgeError::UnsupportedUnit { unit, .. } if unit == "acre"),
        "{error}"
    );
}

#[test]
fn a_fractional_engine_count_is_refused() {
    let error = AceVehicleInputs::from_vehicle_json("v.json", &vehicle("v", 1.5, AREA_M2, None))
        .unwrap_err();
    assert!(
        matches!(error, AceBridgeError::InvalidEngineCount { .. }),
        "{error}"
    );
}

#[test]
fn a_twin_design_against_a_single_engine_package_fails_loudly() {
    let package = builtin_c172_package().unwrap();
    let id = package.manifest().id.clone();
    let twin =
        AceVehicleInputs::from_vehicle_json("v.json", &vehicle(&id, 2.0, AREA_M2, None)).unwrap();
    let error = twin.check_package(&package).unwrap_err();
    assert!(
        matches!(
            error,
            PackError::AceBridge(AceBridgeError::EngineCountMismatch {
                declared: 2,
                flown: 1,
                ..
            })
        ),
        "{error}"
    );
}

#[test]
fn a_twin_design_matches_a_twin_package() {
    let package = builtin_p38g_package().unwrap();
    let id = package.manifest().id.clone();
    let twin =
        AceVehicleInputs::from_vehicle_json("v.json", &vehicle(&id, 2.0, AREA_M2, None)).unwrap();
    twin.check_package(&package).unwrap();
    let single = AceVehicleInputs {
        engine_count: 1,
        ..twin
    };
    assert!(single.check_package(&package).is_err());
}

#[test]
fn a_package_from_another_design_is_refused() {
    let package = builtin_c172_package().unwrap();
    let other =
        AceVehicleInputs::from_vehicle_json("v.json", &vehicle("other", 1.0, AREA_M2, None))
            .unwrap();
    let error = other.check_package(&package).unwrap_err();
    assert!(
        matches!(
            error,
            PackError::AceBridge(AceBridgeError::PackageMismatch { .. })
        ),
        "{error}"
    );
}

#[test]
fn the_polar_fit_recovers_a_parabolic_polar() {
    let points: Vec<(f64, f64)> = [0.3, 0.5, 0.7, 0.9]
        .iter()
        .map(|cl| (*cl, 0.028 + 0.05 * cl * cl))
        .collect();
    let (cd0, k) = fit_polar(&points).unwrap();
    assert!((cd0 - 0.028).abs() < 1e-12 && (k - 0.05).abs() < 1e-12);
    assert_eq!(fit_polar(&points[..2]), None);
    let falling: Vec<(f64, f64)> = points.iter().map(|(cl, cd)| (*cl, 0.2 - cd)).collect();
    assert_eq!(fit_polar(&falling), None);
}
