use std::collections::BTreeMap;

use glam::{DQuat, DVec3};
use mbd_core::ComponentId;
use serde_json::{Value, json};
use sim_atmosphere::ReentryAtmosphere;
use sim_core::{
    Block, RigidBody6DofBlock, RigidBodyConfig, RigidBodyInput, TICKS_PER_SECOND, TickSpan,
    VehicleState,
};
use sim_geo::{BodyEllipsoid, Geodetic, UtcTime, geodetic_to_ecef};

use super::*;
use crate::{CotsDevice, MemoryPackageSource, SyspackError};

fn component(key: &str, parent: Option<&str>, x_m: f64, mass_kg: f64, inertia: f64) -> Value {
    json!({
        "key": key,
        "name": format!("The {key}"),
        "parent": parent,
        "translation_m": [x_m, 0.0, 0.0],
        "dry_mass_kg": mass_kg,
        "dry_inertia_body": [[inertia, 0.0, 0.0], [0.0, inertia, 0.0], [0.0, 0.0, inertia]],
        "initial_fluid_mass_kg": 0.0,
        "initial_payload_mass_kg": 0.0,
        "attached": true,
        "thermal_capacity_j_k": 0.0,
        "nominal_power_w": 0.0,
    })
}

fn capsule_dynamics() -> Value {
    json!({"RigidBodyBluntCapsule": {
        "base_radius_m": 1.0,
        "nose_radius_m": 2.0,
        "reference_point_body_m": [-2.0, 0.0, 0.0],
        "normal_force_slope": 0.1,
        "axial_drag_coeff": 1.2,
        "pitch_damping_coeff": -5.0,
        "sutton_graves_k": 1.7415e-4,
    }})
}

/// A vehicle nobody compiled: every part of it arrives as JSON.
fn downloaded_capsule() -> Value {
    let mut tank = component("tank", Some("hull"), -0.2, 5.0, 0.0);
    tank["initial_fluid_mass_kg"] = json!(40.0);
    let mut payload = component("payload_bay", Some("root"), 0.4, 2.0, 0.0);
    payload["initial_payload_mass_kg"] = json!(8.0);
    json!({
        "schema_version": 1,
        "id": "community-capsule",
        "name": "Community capsule",
        "version": "0.3.1",
        "kind": "ReentryCapsule",
        "assets": {},
        "components": [
            component("root", None, 0.0, 0.0, 0.0),
            component("hull", Some("root"), -0.5, 200.0, 80.0),
            tank,
            payload,
            component("imu_mount", Some("hull"), 0.1, 0.0, 0.0),
        ],
        "dynamics": capsule_dynamics(),
        "devices": [
            {"id": "imu", "site": "imu_mount", "kind": {"Icm42688": {"seed": 42}}},
            {"id": "baro", "site": "hull", "kind": {"Bmp390": {"bus_id": 1, "i2c_addr": 119}}},
            {"id": "bus", "site": "hull", "kind": {
                "Arinc429Channel": {"high_speed": true, "gap_bits": 4},
            }},
            {"id": "cam", "site": "payload_bay", "kind": {"PinholeCamera": {
                "offset_body_m": [0.4, 0.0, 0.1],
                "body_to_camera_xyzw": [0.5, 0.5, -0.5, 0.5],
                "width": 640, "height": 480,
                "fx": 500.0, "fy": 500.0, "cx": 319.5, "cy": 239.5,
                "rate_hz": 30,
            }}},
            {"id": "emulator", "site": "hull", "kind": {"Custom": {
                "type_id": "vendor.flight_computer",
                "parameters": {"rom": "not-built-here"},
            }}},
        ],
    })
}

fn load_json(value: &Value) -> Result<LoadedSyspack, PackError> {
    let source = MemoryPackageSource::new(value.to_string().into_bytes(), BTreeMap::new());
    LoadedSyspack::load(&source)
}

#[test]
fn cots_devices_are_built_on_their_sites_in_manifest_order() {
    let package = load_json(&downloaded_capsule()).unwrap();
    let devices = package.instantiate_cots_devices().unwrap();
    let [imu, baro, bus] = devices.as_slice() else {
        panic!("{devices:?}");
    };
    assert_eq!(imu.id, "imu");
    assert_eq!(imu.site, ComponentId(4));
    assert!(matches!(imu.device, CotsDevice::Icm42688(_)));
    assert!(matches!(baro.device, CotsDevice::Bmp390(_)));
    assert_eq!(bus.site, ComponentId(1));
    assert!(matches!(bus.device, CotsDevice::Arinc429(_)));
}

#[test]
fn cameras_carry_their_mount_intrinsics_and_period() {
    let package = load_json(&downloaded_capsule()).unwrap();
    let cameras = package.camera_mounts().unwrap();
    let [camera] = cameras.as_slice() else {
        panic!("{cameras:?}");
    };
    assert_eq!(camera.site, ComponentId(3));
    assert_eq!(camera.mount.offset_body_m, DVec3::new(0.4, 0.0, 0.1));
    assert_eq!(
        camera.mount.body_to_camera,
        DQuat::from_xyzw(0.5, 0.5, -0.5, 0.5)
    );
    assert_eq!(camera.intrinsics.width, 640);
    assert_eq!(camera.period, TickSpan::from_ticks(TICKS_PER_SECOND / 30));
}

#[test]
fn a_downloaded_capsule_flies_in_the_rigid_body_block() {
    let package = load_json(&downloaded_capsule()).unwrap();
    let start = Geodetic::from_degrees(10.0, 20.0, 60_000.0);
    let position_ecef = geodetic_to_ecef(&start);
    let initial = VehicleState {
        position_ecef,
        velocity_ecef: position_ecef.normalize().any_orthonormal_vector() * 4_000.0,
        attitude: DQuat::IDENTITY,
        angular_rate_body: DVec3::ZERO,
    };
    let fly = || {
        let mut block = RigidBody6DofBlock::new(
            RigidBodyConfig {
                body: BodyEllipsoid::WGS84,
                period: TickSpan::from_ticks(TICKS_PER_SECOND / 100),
                epoch: UtcTime::from_unix_seconds(0.0),
                initial,
            },
            package.tree().clone(),
            package.build_aero_model().unwrap(),
            Box::new(ReentryAtmosphere),
        )
        .unwrap();
        for _ in 0..200 {
            block.step(&RigidBodyInput::default()).unwrap();
        }
        block
    };
    let block = fly();
    let state = block.output();
    assert_ne!(state.position_ecef, position_ecef);
    assert!(state.velocity_ecef.length() < 4_000.0, "drag slows it down");
    assert!(block.aero_wrench().force_body_n.length() > 0.0);
    assert_eq!(fly().output(), state, "it replays exactly");
}

#[test]
fn a_jsbsim_package_has_no_aero_model_and_a_rigid_body_has_no_resolver() {
    let package = load_json(&downloaded_capsule()).unwrap();
    assert!(matches!(
        package.build_jsbsim_resolver(),
        Err(PackError::Syspack(SyspackError::DynamicsMismatch {
            needed: "JsbsimXml",
            ..
        }))
    ));
    assert!(matches!(
        package.jsbsim_model(),
        Err(PackError::Syspack(SyspackError::DynamicsMismatch { .. }))
    ));
    let trainer = crate::builtin_light_trainer_package().unwrap();
    assert!(matches!(
        trainer.build_aero_model(),
        Err(SyspackError::DynamicsMismatch {
            declared: "JsbsimXml",
            ..
        })
    ));
    assert!(matches!(
        trainer.capsule_aero(),
        Err(SyspackError::DynamicsMismatch { .. })
    ));
}

#[test]
fn bad_devices_fail_at_load() {
    let camera = |field: &str, bad: Value| {
        let mut value = downloaded_capsule();
        value["devices"][3]["kind"]["PinholeCamera"][field] = bad;
        value
    };
    for (expected, value) in [
        ("rate_hz", camera("rate_hz", json!(0))),
        ("rate_hz", camera("rate_hz", json!(7_919))),
        ("intrinsics", camera("width", json!(0))),
        ("intrinsics", camera("fx", json!(-1.0))),
        (
            "body_to_camera_xyzw",
            camera("body_to_camera_xyzw", json!([0.0, 0.0, 0.0, 0.0])),
        ),
    ] {
        assert!(
            matches!(
                load_json(&value),
                Err(PackError::Syspack(SyspackError::InvalidDevice { field, .. }))
                    if field == expected
            ),
            "{expected}"
        );
    }

    let mut bad_address = downloaded_capsule();
    bad_address["devices"][1]["kind"]["Bmp390"]["i2c_addr"] = json!(200);
    assert!(matches!(
        load_json(&bad_address),
        Err(PackError::DeviceBuild { device, .. }) if device == "baro"
    ));
}

#[test]
fn arinc_channels_are_numbered_in_manifest_order_up_to_the_limit() {
    let channel = |index: usize| {
        json!({
            "id": format!("bus{index}"), "site": "hull",
            "kind": {"Arinc429Channel": {"high_speed": false, "gap_bits": 0}},
        })
    };
    let mut value = downloaded_capsule();
    value["devices"] = Value::Array((0..256).map(channel).collect());
    let package = load_json(&value).unwrap();
    assert_eq!(package.instantiate_cots_devices().unwrap().len(), 256);

    value["devices"] = Value::Array((0..257).map(channel).collect());
    assert!(matches!(
        load_json(&value),
        Err(PackError::Syspack(SyspackError::InvalidDevice {
            field: "kind",
            ..
        }))
    ));
}

#[test]
fn an_unverifiable_package_never_reaches_the_devices() {
    let mut value = downloaded_capsule();
    value["assets"]["gone.bin"] = json!({"sha256": "0".repeat(64), "size_bytes": 3});
    assert!(matches!(
        load_json(&value),
        Err(PackError::Syspack(SyspackError::AssetMissing { path })) if path == "gone.bin"
    ));
}
