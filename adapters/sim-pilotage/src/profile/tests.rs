use std::collections::BTreeMap;

use pilotage_aircraft::{
    AircraftError, Draw, EnergyState, Remaining, endurance, weight_and_balance,
};
use sim_pack::{LoadedSyspack, MemoryPackageSource, builtin_light_trainer_package};

use super::*;

const TRAINER_DRY_KG: f64 = 598.741_928;
const TRAINER_FDM: &str = "fdm/trainer.xml";
const NOSE_ARM_M: f64 = -1.0;

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
        source: "Simulate light-trainer package 1.0.0".into(),
        max_takeoff_weight_kg: 800.0,
        cg_envelope: vec![[500.0, -0.3], [800.0, -0.3], [800.0, 0.2], [500.0, 0.2]],
        stations: vec![
            StationSite {
                name: "pilot".into(),
                component: "avionics_bay".into(),
                max_kg: 120.0,
            },
            StationSite {
                name: "nose locker".into(),
                component: "nose_camera_mount".into(),
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
        stations: vec![StationSite {
            name: "pilot".into(),
            component: "avionics_bay".into(),
            max_kg: 120.0,
        }],
        energy: EnergySpec::Fuel {
            density_kg_per_l: 0.72,
            tanks: vec![TankSite {
                name: "main".into(),
                component: "nose_camera_mount".into(),
                usable_l: 100.0,
            }],
        },
        cruise: cruise(Draw::FuelFlowLph(25.0)),
        ..battery_spec()
    }
}

fn trainer() -> LoadedSyspack {
    builtin_light_trainer_package().unwrap()
}

/// Reloads a changed manifest as a package, through the verifying loader.
fn reload(manifest: &SyspackManifest) -> LoadedSyspack {
    let bytes = serde_json::to_vec(manifest).unwrap();
    let fdm = trainer().asset(TRAINER_FDM).unwrap().to_vec();
    let assets = BTreeMap::from([(TRAINER_FDM.to_owned(), fdm)]);
    LoadedSyspack::load(&MemoryPackageSource::new(bytes, assets)).unwrap()
}

#[test]
fn the_trainers_structure_and_stations_come_from_its_component_tree() {
    let profile = aircraft_profile(&trainer(), &battery_spec()).unwrap();

    assert!((profile.empty_weight_kg - TRAINER_DRY_KG).abs() < 1e-9);
    assert!(profile.empty_arm_m.abs() < 1e-12);
    let locker = &profile.stations[1];
    assert_eq!(locker.name, "nose locker");
    assert_eq!(locker.arm_m, NOSE_ARM_M);
    assert_eq!(profile.max_takeoff_weight_kg, 800.0);
    profile.id().unwrap();
}

#[test]
fn weight_and_balance_moves_forward_with_load_in_the_nose() {
    let profile = aircraft_profile(&trainer(), &battery_spec()).unwrap();
    let loading = Loading {
        profile: profile.id().unwrap(),
        revision: 1,
        stations_kg: vec![("pilot".into(), 80.0), ("nose locker".into(), 20.0)],
        fuel_l: vec![],
        battery_wh: Some(12_000.0),
    };

    let balance = weight_and_balance(&profile, &loading).unwrap();

    let expected_kg = TRAINER_DRY_KG + 100.0;
    assert!((balance.weight_kg - expected_kg).abs() < 1e-9);
    assert!((balance.cg_arm_m - 20.0 * NOSE_ARM_M / expected_kg).abs() < 1e-9);
    assert!(balance.within_max_weight);
    assert!(balance.within_envelope);
}

/// Pilotage's weight and balance and Simulate's mass properties are separate
/// calculations; flying the loading must give the number the pilot was told.
#[test]
fn a_loading_written_to_the_package_flies_the_balance_pilotage_computed() {
    let package = trainer();
    let spec = fuel_spec();
    let profile = aircraft_profile(&package, &spec).unwrap();
    let loading = Loading {
        profile: profile.id().unwrap(),
        revision: 1,
        stations_kg: vec![("pilot".into(), 90.0)],
        fuel_l: vec![("main".into(), 60.0)],
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
    let package = trainer();
    let spec = fuel_spec();
    let profile = aircraft_profile(&package, &spec).unwrap();
    let loading = Loading {
        profile: profile.id().unwrap(),
        revision: 0,
        stations_kg: vec![("pilot".into(), 90.0)],
        fuel_l: vec![("main".into(), 60.0)],
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
    let package = trainer();
    let spec = fuel_spec();
    let profile = aircraft_profile(&package, &spec).unwrap();
    let loading = Loading {
        profile: profile.id().unwrap(),
        revision: 0,
        stations_kg: vec![("pilot".into(), 90.0)],
        fuel_l: vec![("main".into(), 60.0)],
        battery_wh: None,
    };
    let loaded = reload(&apply_loading(package.manifest(), &spec, &profile, &loading).unwrap());
    let again = aircraft_profile(&loaded, &spec).unwrap();
    assert_eq!(again.id().unwrap(), profile.id().unwrap());
}

#[test]
fn endurance_on_the_battery_holds_back_the_reserve() {
    let profile = aircraft_profile(&trainer(), &battery_spec()).unwrap();
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
    let package = trainer();
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
    let mut manifest = trainer().manifest().clone();
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
        aircraft_profile(&trainer(), &spec)
    else {
        panic!("a missing component must be refused");
    };
    assert_eq!((site.as_str(), component.as_str()), ("pilot", "cargo_pod"));
}

#[test]
fn two_sites_on_one_component_are_refused() {
    let mut spec = fuel_spec();
    spec.stations[0].component = "nose_camera_mount".into();
    assert!(matches!(
        aircraft_profile(&trainer(), &spec),
        Err(PilotageBridgeError::SharedComponent { .. })
    ));
}

#[test]
fn pilotage_refuses_an_impossible_envelope_through_the_bridge() {
    let mut spec = battery_spec();
    spec.cg_envelope = vec![[500.0, 0.0], [800.0, 0.0]];
    assert!(matches!(
        aircraft_profile(&trainer(), &spec),
        Err(PilotageBridgeError::Aircraft(
            AircraftError::InvalidProfile { .. }
        ))
    ));
}

#[test]
fn an_overweight_loading_is_refused_before_it_reaches_the_package() {
    let package = trainer();
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
