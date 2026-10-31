//! Packages that are not vehicles: a ground station and a bench, built by the
//! same loader as an aeroplane, with no dynamics to fly.

use std::collections::BTreeMap;

use serde_json::{Value, json};
use sim_pack::{
    CotsDevice, DynamicsSpec, LoadedSyspack, MemoryPackageSource, PackError, SyspackError,
    SystemKind,
};

#[cfg(test)]
fn component(key: &str, parent: Option<&str>) -> Value {
    json!({
        "key": key,
        "name": key,
        "parent": parent,
        "translation_m": [0.0, 0.0, 0.0],
        "dry_mass_kg": 0.0,
        "dry_inertia_body": [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
        "initial_fluid_mass_kg": 0.0,
        "initial_payload_mass_kg": 0.0,
        "attached": true,
        "thermal_capacity_j_k": 0.0,
        "nominal_power_w": 45.0,
    })
}

#[cfg(test)]
fn stationary(kind: &str) -> Value {
    json!({
        "schema_version": 1,
        "id": "stationary-system",
        "name": "Stationary system",
        "version": "1.0.0",
        "kind": kind,
        "assets": {},
        "components": [component("rack", None), component("board", Some("rack"))],
        "dynamics": "None",
        "devices": [
            {"id": "imu", "site": "board", "kind": {"Icm42688": {"seed": 3}}},
            {"id": "baro", "site": "board", "kind": {"Bmp390": {"bus_id": 2, "i2c_addr": 118}}},
            {"id": "bus", "site": "rack", "kind": {
                "Arinc429Channel": {"high_speed": false, "gap_bits": 4},
            }},
            {"id": "controller", "site": "rack", "kind": {"Custom": {
                "type_id": "vendor.bench_controller",
                "parameters": {"channels": 8},
            }}},
        ],
    })
}

#[cfg(test)]
fn load(value: &Value) -> Result<LoadedSyspack, PackError> {
    let source = MemoryPackageSource::new(value.to_string().into_bytes(), BTreeMap::new());
    LoadedSyspack::load(&source)
}

#[test]
fn a_ground_station_and_a_bench_build_their_off_the_shelf_devices() {
    for (name, kind) in [
        ("GroundStation", SystemKind::GroundStation),
        ("TestBench", SystemKind::TestBench),
    ] {
        let package = load(&stationary(name)).unwrap();
        assert_eq!(package.manifest().kind, kind);
        assert_eq!(package.manifest().dynamics, DynamicsSpec::None);
        let devices = package.instantiate_cots_devices().unwrap();
        let [imu, baro, bus] = devices.as_slice() else {
            panic!("{devices:?}");
        };
        assert!(matches!(imu.device, CotsDevice::Icm42688(_)));
        assert!(matches!(baro.device, CotsDevice::Bmp390(_)));
        assert!(matches!(bus.device, CotsDevice::Arinc429(_)));
        assert_eq!(baro.site, package.component_id("board").unwrap());
        assert_eq!(bus.site, package.component_id("rack").unwrap());
        assert!(package.camera_mounts().unwrap().is_empty());
    }
}

#[test]
fn a_stationary_system_gives_a_simulator_nothing_to_fly() {
    let package = load(&stationary("TestBench")).unwrap();
    assert!(matches!(
        package.build_jsbsim_resolver(),
        Err(PackError::Syspack(SyspackError::DynamicsMismatch {
            declared: "None",
            ..
        }))
    ));
    assert!(matches!(
        package.jsbsim_model(),
        Err(PackError::Syspack(SyspackError::DynamicsMismatch { .. }))
    ));
    assert!(package.build_aero_model().is_err());
}

#[test]
fn a_device_a_stationary_system_cannot_build_still_refuses_it() {
    let mut value = stationary("GroundStation");
    value["devices"][1]["kind"]["Bmp390"]["i2c_addr"] = json!(200);
    assert!(matches!(
        load(&value),
        Err(PackError::DeviceBuild { device, .. }) if device == "baro"
    ));
}
