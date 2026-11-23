//! Packages that are not vehicles: a ground station and a bench, built by the
//! same loader as an aeroplane, with no dynamics to fly.

use std::collections::BTreeMap;

use serde_json::{Value, json};
use sim_atmosphere::VacuumAtmosphere;
use sim_core::{
    Block, MultibodyVehicleConfig, MultibodyVehicleInput, PropulsionCommands, TICKS_PER_SECOND,
    TickSpan, VehicleState,
};
use sim_geo::{BodyEllipsoid, Geodetic, UtcTime};
use sim_pack::{
    CotsDevice, DynamicsSpec, LoadedSyspack, MemoryPackageSource, PackError, SyspackError,
    SystemKind, builtin_c172s_package,
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

#[cfg(test)]
fn v2_multibody_manifest() -> Value {
    json!({
        "schema_version": 2,
        "id": "test-v2-spacecraft",
        "name": "Test v2 Spacecraft",
        "version": "1.0.0",
        "kind": "Spacecraft",
        "assets": {},
        "components": [{
            "key": "bus",
            "name": "bus",
            "parent": null,
            "translation_m": [0.0, 0.0, 0.0],
            "dry_mass_kg": 500.0,
            "dry_inertia_body": [[250.0, 0.0, 0.0], [0.0, 300.0, 0.0], [0.0, 0.0, 300.0]],
            "initial_fluid_mass_kg": 0.0,
            "initial_payload_mass_kg": 0.0,
            "attached": true,
            "thermal_capacity_j_k": 400_000.0,
            "nominal_power_w": 50.0,
        }],
        "dynamics": {
            "MultibodyComposed": {
                "aero_elements": [{
                    "site": "bus",
                    "kind": {
                        "BluntCapsule": {
                            "base_radius_m": 1.0,
                            "nose_radius_m": 2.0,
                            "reference_point_body_m": [-1.5, 0.0, 0.0],
                            "normal_force_slope": 0.1,
                            "axial_drag_coeff": 1.2,
                            "pitch_damping_coeff": -5.0,
                        }
                    }
                }]
            }
        },
        "devices": [],
        "subsystems": {
            "nominal_bus_voltage_v": 28.0,
            "tanks": [{
                "id": 0,
                "site": "bus",
                "center_local_m": [0.0, 0.0, 0.0],
                "geometry": { "Sphere": { "radius_m": 0.5 } },
                "species": "Hydrazine",
                "capacity_mass_kg": 400.0,
                "initial_mass_kg": 100.0,
                "feed_regime": "BladderOrDiaphragm",
            }],
            "liquid_rockets": [{
                "site": "bus",
                "mount_offset_local_m": [0.0, 0.0, 0.0],
                "thrust_axis_local": [1.0, 0.0, 0.0],
                "max_vacuum_thrust_n": 2_000.0,
                "isp_vac_s": 300.0,
                "isp_sl_s": 280.0,
                "fuel_tank": 0,
            }],
            "solar_arrays": [{
                "site": "bus",
                "area_m2": 2.0,
                "normal_local": [0.0, 0.0, -1.0],
                "efficiency": 0.28,
                "packing_factor": 0.9,
                "nominal_voltage_v": 28.0,
            }],
            "batteries": [{
                "site": "bus",
                "capacity_wh": 500.0,
                "nominal_voltage_v": 28.0,
                "max_discharge_w": 500.0,
            }],
        },
    })
}

#[test]
fn builtin_v1_package_audits_cleanly() {
    let package = builtin_c172s_package().unwrap();
    let report = package.audit_design().unwrap();
    assert!(report.passed(), "findings: {:?}", report.findings);
}

#[test]
fn v2_multibody_package_builds_and_steps_deterministically() {
    let package = load(&v2_multibody_manifest()).unwrap();
    let report = package.audit_design().unwrap();
    assert!(report.passed(), "findings: {:?}", report.findings);

    let pos = Geodetic::from_degrees(0.0, 0.0, 400_000.0);
    let initial = VehicleState::level_flight(&pos, 0.0, 7_670.0);
    let fwd0 = initial.forward_ecef();
    let speed0 = initial.velocity_ecef.dot(fwd0);
    let mut block = package
        .build_multibody_vehicle(
            MultibodyVehicleConfig {
                body: BodyEllipsoid::WGS84,
                period: TickSpan::from_ticks(TICKS_PER_SECOND / 10),
                epoch: UtcTime::from_unix_seconds(1_700_000_000.0),
                initial,
            },
            Box::new(VacuumAtmosphere),
        )
        .unwrap();

    let snap = block.snapshot();
    let burn = MultibodyVehicleInput {
        propulsion: PropulsionCommands {
            liquid_throttles: vec![1.0],
            ..PropulsionCommands::default()
        },
        ..MultibodyVehicleInput::default()
    };
    for _ in 0..10 {
        block.step(&burn).unwrap();
    }
    assert!(block.telemetry().total_propellant_kg < 100.0);
    assert!(block.output().velocity_ecef.dot(fwd0) > speed0 + 2.5);
    let end1 = block.output();

    block.restore(&snap);
    for _ in 0..10 {
        block.step(&burn).unwrap();
    }
    assert_eq!(block.output(), end1);
}
