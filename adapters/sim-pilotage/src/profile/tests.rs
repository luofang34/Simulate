use std::collections::BTreeMap;

use pilotage_aircraft::{
    AircraftError, Draw, EnergyState, Remaining, endurance, weight_and_balance,
};
use sim_pack::{LoadedSyspack, MemoryPackageSource, builtin_c172_package};

use super::*;

const PILOT_SEAT: &str = "payload_station";
const NOSE_LOCKER: &str = "fpv_camera_mount";
const MAIN_TANK: &str = "left_fuel_tank";
const RIGHT_TANK: &str = "right_fuel_tank";
const REAR_SEATS: &str = "rear_seats";

fn cruise(draw: Draw) -> CruiseModel {
    CruiseModel {
        true_airspeed_mps: 55.0,
        draw,
    }
}

fn battery_spec() -> AircraftSpec {
    AircraftSpec {
        type_designator: "FSTR".into(),
        registration: None,
        source: "Simulate c172_class_reference package".into(),
        max_takeoff_weight_kg: 1_157.0,
        cg_envelope: vec![[700.0, 2.5], [1_157.0, 2.5], [1_157.0, 3.5], [700.0, 3.5]],
        stations: vec![
            StationSite {
                name: "pilot".into(),
                component: PILOT_SEAT.into(),
                max_kg: 120.0,
            },
            StationSite {
                name: "nose locker".into(),
                component: NOSE_LOCKER.into(),
                max_kg: 40.0,
            },
        ],
        energy: EnergySpec::Battery {
            usable_wh: 12_000.0,
        },
        cruise: cruise(Draw::PowerW(6_000.0)),
    }
}

fn fuel_spec() -> AircraftSpec {
    AircraftSpec {
        stations: vec![
            StationSite {
                name: "pilot".into(),
                component: PILOT_SEAT.into(),
                max_kg: 120.0,
            },
            StationSite {
                name: "rear".into(),
                component: REAR_SEATS.into(),
                max_kg: 160.0,
            },
        ],
        energy: EnergySpec::Fuel {
            density_kg_per_l: 0.72,
            tanks: vec![
                TankSite {
                    name: "main".into(),
                    component: MAIN_TANK.into(),
                    usable_l: 100.0,
                },
                TankSite {
                    name: "right".into(),
                    component: RIGHT_TANK.into(),
                    usable_l: 100.0,
                },
            ],
        },
        cruise: cruise(Draw::FuelFlowLph(25.0)),
        ..battery_spec()
    }
}

fn c172() -> LoadedSyspack {
    builtin_c172_package().unwrap()
}

/// The structure's mass summed straight from the manifest, independent of the
/// tree the bridge weighs.
fn c172_dry_kg() -> f64 {
    c172()
        .manifest()
        .components
        .iter()
        .map(|component| component.dry_mass_kg)
        .sum()
}

/// The arm of the root component's origin, where the C172 carries all of its
/// structural mass.
fn c172_root_arm_m() -> f64 {
    let package = c172();
    let root = package
        .manifest()
        .components
        .iter()
        .find(|component| component.parent.is_none())
        .unwrap();
    -root.translation_m[0]
}

/// Reloads a changed manifest as a package, through the verifying loader.
fn reload(manifest: &SyspackManifest) -> LoadedSyspack {
    let bytes = serde_json::to_vec(manifest).unwrap();
    let base = c172();
    let assets: BTreeMap<String, Vec<u8>> = manifest
        .assets
        .keys()
        .filter_map(|path| Some((path.clone(), base.asset(path)?.to_vec())))
        .collect();
    LoadedSyspack::load(&MemoryPackageSource::new(bytes, assets)).unwrap()
}

#[test]
fn the_c172s_structure_and_stations_come_from_its_component_tree() {
    let profile = aircraft_profile(&c172(), &battery_spec()).unwrap();

    assert!((profile.empty_weight_kg - c172_dry_kg()).abs() < 1e-9);
    assert!((profile.empty_arm_m - c172_root_arm_m()).abs() < 1e-12);
    let locker = &profile.stations[1];
    assert_eq!(locker.name, "nose locker");
    assert!(locker.arm_m < profile.stations[0].arm_m, "{profile:?}");
    assert_eq!(profile.max_takeoff_weight_kg, 1_157.0);
    profile.id().unwrap();
}

#[test]
fn weight_and_balance_moves_forward_with_load_in_the_nose() {
    let profile = aircraft_profile(&c172(), &battery_spec()).unwrap();
    let loading = Loading {
        profile: profile.id().unwrap(),
        revision: 1,
        stations_kg: vec![("pilot".into(), 80.0), ("nose locker".into(), 20.0)],
        fuel_l: vec![],
        battery_wh: Some(12_000.0),
    };

    let balance = weight_and_balance(&profile, &loading).unwrap();

    let expected_kg = c172_dry_kg() + 100.0;
    let moment_kg_m = profile.empty_weight_kg * profile.empty_arm_m
        + 80.0 * profile.stations[0].arm_m
        + 20.0 * profile.stations[1].arm_m;
    assert!((balance.weight_kg - expected_kg).abs() < 1e-9);
    assert!((balance.cg_arm_m - moment_kg_m / expected_kg).abs() < 1e-9);
    let unloaded = weight_and_balance(
        &profile,
        &Loading {
            stations_kg: vec![("pilot".into(), 80.0)],
            ..loading.clone()
        },
    )
    .unwrap();
    assert!(
        balance.cg_arm_m < unloaded.cg_arm_m,
        "the nose load moves the CG forward"
    );
    assert!(balance.within_max_weight);
    assert!(balance.within_envelope);
}

/// Pilotage's weight and balance and Simulate's mass properties are separate
/// calculations; flying the loading must give the number the pilot was told.
#[test]
fn a_loading_written_to_the_package_flies_the_balance_pilotage_computed() {
    let package = c172();
    let spec = fuel_spec();
    let profile = aircraft_profile(&package, &spec).unwrap();
    let loading = Loading {
        profile: profile.id().unwrap(),
        revision: 1,
        stations_kg: vec![("pilot".into(), 90.0), ("rear".into(), 0.0)],
        fuel_l: vec![("main".into(), 60.0), ("right".into(), 60.0)],
        battery_wh: None,
    };
    let balance = weight_and_balance(&profile, &loading).unwrap();

    let loaded = reload(&apply_loading(package.manifest(), &spec, &profile, &loading).unwrap());
    let mass = loaded.tree().mass_properties().unwrap();

    assert!(
        (mass.total_mass_kg - balance.weight_kg).abs() < 1e-9,
        "{mass:?}"
    );
    assert!(
        (-mass.cg_body_m.x - balance.cg_arm_m).abs() < 1e-9,
        "{mass:?}"
    );
}

#[test]
fn a_loading_survives_the_round_trip_through_the_package() {
    let package = c172();
    let spec = fuel_spec();
    let profile = aircraft_profile(&package, &spec).unwrap();
    let loading = Loading {
        profile: profile.id().unwrap(),
        revision: 0,
        stations_kg: vec![("pilot".into(), 90.0), ("rear".into(), 0.0)],
        fuel_l: vec![("main".into(), 60.0), ("right".into(), 60.0)],
        battery_wh: None,
    };
    let loaded = reload(&apply_loading(package.manifest(), &spec, &profile, &loading).unwrap());

    let read_back = package_loading(&loaded, &spec, &profile).unwrap();

    assert_eq!(read_back.stations_kg, loading.stations_kg);
    let (name, litres) = &read_back.fuel_l[0];
    assert_eq!(name, "main");
    assert!((litres - 60.0).abs() < 1e-9);
}

/// Loads are not structure: a package that already carries some keeps the same
/// empty weight, so the profile and its hash do not change.
#[test]
fn a_loaded_package_has_the_same_profile_as_an_empty_one() {
    let package = c172();
    let spec = fuel_spec();
    let profile = aircraft_profile(&package, &spec).unwrap();
    let loading = Loading {
        profile: profile.id().unwrap(),
        revision: 0,
        stations_kg: vec![("pilot".into(), 90.0), ("rear".into(), 0.0)],
        fuel_l: vec![("main".into(), 60.0), ("right".into(), 60.0)],
        battery_wh: None,
    };
    let loaded = reload(&apply_loading(package.manifest(), &spec, &profile, &loading).unwrap());
    let again = aircraft_profile(&loaded, &spec).unwrap();
    assert_eq!(again.id().unwrap(), profile.id().unwrap());
}

#[test]
fn endurance_on_the_battery_holds_back_the_reserve() {
    let profile = aircraft_profile(&c172(), &battery_spec()).unwrap();
    let energy = EnergyState {
        remaining: Remaining::BatteryWh(9_000.0),
        measured_draw: None,
        at_unix_ns: 1_000,
    };

    let result = endurance(&profile, energy, 30.0).unwrap();

    assert!((result.minutes - 60.0).abs() < 1e-9, "{result:?}");
    assert!((result.still_air_range_m - 60.0 * 60.0 * 55.0).abs() < 1e-6);
}

#[test]
fn endurance_on_fuel_uses_the_loaded_tanks() {
    let package = c172();
    let spec = fuel_spec();
    let profile = aircraft_profile(&package, &spec).unwrap();
    let loading = Loading {
        profile: profile.id().unwrap(),
        revision: 0,
        stations_kg: vec![],
        fuel_l: vec![("main".into(), 75.0)],
        battery_wh: None,
    };
    let energy = EnergyState {
        remaining: loading.remaining(&profile).unwrap(),
        measured_draw: None,
        at_unix_ns: 0,
    };
    let result = endurance(&profile, energy, 0.0).unwrap();
    assert!((result.minutes - 180.0).abs() < 1e-9);
}

#[test]
fn a_spacecraft_has_no_aircraft_profile() {
    let mut manifest = c172().manifest().clone();
    manifest.kind = SystemKind::Spacecraft;
    assert!(matches!(
        aircraft_profile(&reload(&manifest), &battery_spec()),
        Err(PilotageBridgeError::UnsupportedKind { .. })
    ));
}

#[test]
fn a_station_on_a_missing_component_is_refused_by_name() {
    let mut spec = battery_spec();
    spec.stations[0].component = "cargo_pod".into();
    let Err(PilotageBridgeError::UnknownComponent { site, component }) =
        aircraft_profile(&c172(), &spec)
    else {
        panic!("a missing component must be refused");
    };
    assert_eq!((site.as_str(), component.as_str()), ("pilot", "cargo_pod"));
}

#[test]
fn two_sites_on_one_component_are_refused() {
    let mut spec = fuel_spec();
    spec.stations[0].component = MAIN_TANK.into();
    assert!(matches!(
        aircraft_profile(&c172(), &spec),
        Err(PilotageBridgeError::SharedComponent { .. })
    ));
}

#[test]
fn pilotage_refuses_an_impossible_envelope_through_the_bridge() {
    let mut spec = battery_spec();
    spec.cg_envelope = vec![[500.0, 0.0], [800.0, 0.0]];
    assert!(matches!(
        aircraft_profile(&c172(), &spec),
        Err(PilotageBridgeError::Aircraft(
            AircraftError::InvalidProfile { .. }
        ))
    ));
}

#[test]
fn an_overweight_loading_is_refused_before_it_reaches_the_package() {
    let package = c172();
    let spec = battery_spec();
    let profile = aircraft_profile(&package, &spec).unwrap();
    let loading = Loading {
        profile: profile.id().unwrap(),
        revision: 0,
        stations_kg: vec![("pilot".into(), 500.0)],
        fuel_l: vec![],
        battery_wh: Some(1_000.0),
    };
    assert!(matches!(
        apply_loading(package.manifest(), &spec, &profile, &loading),
        Err(PilotageBridgeError::Aircraft(
            AircraftError::InvalidLoading { .. }
        ))
    ));
}
