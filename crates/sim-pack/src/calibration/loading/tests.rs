use super::{CgTarget, cg_target, load_point};
use crate::calibration::solver::{ParameterOverrides, build_fdm, probe_non_payload_lbs};
use crate::{PackError, builtin_c172_package};

const ORIGIN_IN: f64 = 78.8787;

fn target(fs_in: f64) -> CgTarget<'static> {
    CgTarget {
        label: "test point",
        fs_in,
        origin_in: ORIGIN_IN,
    }
}

#[test]
fn the_payload_is_split_to_reach_the_aft_limit() {
    let package = builtin_c172_package().unwrap();
    let mut fdm = build_fdm(&package).unwrap();
    let base = probe_non_payload_lbs(&mut fdm).unwrap();
    let mass_kg = 2_550.0 / 2.204_622_621_8;
    let flown = load_point(
        &mut fdm,
        mass_kg,
        base,
        ParameterOverrides::default(),
        Some(target(47.3)),
    )
    .unwrap()
    .unwrap();
    assert!((flown - 47.3).abs() < 0.01, "{flown}");
    let weight = fdm.get_property("inertia/weight-lbs").unwrap();
    assert!((weight - 2_550.0).abs() < 0.01, "{weight}");
}

#[test]
fn a_cg_ahead_of_every_station_is_refused() {
    let package = builtin_c172_package().unwrap();
    let mut fdm = build_fdm(&package).unwrap();
    let base = probe_non_payload_lbs(&mut fdm).unwrap();
    let mass_kg = 2_200.0 / 2.204_622_621_8;
    let refused = load_point(
        &mut fdm,
        mass_kg,
        base,
        ParameterOverrides::default(),
        Some(target(37.5)),
    );
    assert!(
        matches!(refused, Err(PackError::UnreachableCg { target_fs_in, .. }) if target_fs_in == 37.5),
        "{refused:?}"
    );
}

#[test]
fn a_cg_without_a_station_origin_is_refused() {
    assert!(matches!(
        cg_target("test point", Some(47.3), None),
        Err(PackError::InvalidCalibration { .. })
    ));
    assert!(cg_target("test point", None, None).unwrap().is_none());
}
