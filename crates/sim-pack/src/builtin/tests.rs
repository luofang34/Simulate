use std::collections::BTreeMap;
use std::path::{Path, PathBuf};

use glam::DVec3;
use sha2::{Digest, Sha256};
use sim_core::{
    Block, CAMERA_DEFAULT_PERIOD, CameraMount, ControlInputs, EngineStartMode,
    JSBSIM_DEFAULT_PERIOD, JsbsimFdmBlock, JsbsimFdmConfig, KinematicInput, VehicleState,
};
use sim_geo::Geodetic;

use super::*;
use crate::{
    CotsDevice, DirectoryPackageSource, DynamicsSpec, MANIFEST_FILE, SyspackError, SystemKind,
};

fn trainer_dir() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../../vehicles/light-trainer")
}

/// A copy of the trainer package, as a user's download would leave it.
fn downloaded_copy() -> tempfile::TempDir {
    let dir = tempfile::tempdir().unwrap();
    std::fs::create_dir(dir.path().join("fdm")).unwrap();
    for file in [MANIFEST_FILE, TRAINER_FDM_PATH] {
        std::fs::copy(trainer_dir().join(file), dir.path().join(file)).unwrap();
    }
    dir
}

fn fly(package: &LoadedSyspack) -> Vec<VehicleState> {
    let files = package.build_jsbsim_resolver().unwrap();
    let (aircraft_path, model) = package.jsbsim_model().unwrap();
    let mut block = JsbsimFdmBlock::new(&JsbsimFdmConfig {
        resolver: &files,
        aircraft_path,
        model,
        period: JSBSIM_DEFAULT_PERIOD,
        initial: Geodetic::from_degrees(39.1, -75.6, 900.0),
        heading_rad: 1.0,
        speed_m_s: 45.0,
        alpha_rad: 0.06,
        engine_start: EngineStartMode::Running,
    })
    .unwrap();
    let input = KinematicInput {
        controls: ControlInputs {
            throttle: 0.2,
            roll: 0.1,
            ..ControlInputs::default()
        },
    };
    (0..240)
        .map(|_| {
            block.step(&input).unwrap();
            block.output()
        })
        .collect()
}

#[test]
fn the_builtin_trainer_loads_as_a_jsbsim_fixed_wing() {
    let package = builtin_light_trainer_package().unwrap();
    let manifest = package.manifest();
    assert_eq!(manifest.id, "light-trainer");
    assert_eq!(manifest.kind, SystemKind::FixedWing);
    assert!(matches!(manifest.dynamics, DynamicsSpec::JsbsimXml { .. }));
    assert_eq!(package.jsbsim_model().unwrap(), ("aircraft", "sim_trainer"));
    assert_eq!(package.asset(TRAINER_FDM_PATH), Some(TRAINER_FDM));
}

#[test]
fn the_declared_digest_is_the_digest_of_the_shipped_file() {
    let package = builtin_light_trainer_package().unwrap();
    let declared = &package.manifest().assets[TRAINER_FDM_PATH];
    assert_eq!(declared.size_bytes, TRAINER_FDM.len() as u64);
    let digest: String = Sha256::digest(TRAINER_FDM)
        .iter()
        .map(|byte| format!("{byte:02x}"))
        .collect();
    assert_eq!(declared.sha256, digest);
}

#[test]
fn the_manifest_agrees_with_the_mass_jsbsim_flies() {
    let package = builtin_light_trainer_package().unwrap();
    let mass = package.tree().mass_properties().unwrap();
    let pounds = mass.total_mass_kg / 0.453_592_37;
    assert!((pounds - 1_320.0).abs() < 1e-3, "{pounds}");
    assert!(
        std::str::from_utf8(TRAINER_FDM)
            .unwrap()
            .contains("<emptywt unit=\"LBS\"> 1320 </emptywt>")
    );
}

#[test]
fn the_trainer_declares_the_corridor_sensor_suite() {
    let package = builtin_light_trainer_package().unwrap();
    let devices = package.instantiate_cots_devices().unwrap();
    let [imu, barometer] = devices.as_slice() else {
        panic!("{devices:?}");
    };
    assert!(matches!(imu.device, CotsDevice::Icm42688(_)));
    assert!(matches!(barometer.device, CotsDevice::Bmp390(_)));
    let cameras = package.camera_mounts().unwrap();
    let [camera] = cameras.as_slice() else {
        panic!("{cameras:?}");
    };
    let nose = CameraMount::forward_nose(DVec3::X);
    assert_eq!(camera.mount.offset_body_m, nose.offset_body_m);
    let difference = camera
        .mount
        .body_to_camera
        .angle_between(nose.body_to_camera);
    assert!(difference < 1e-7, "{difference}");
    assert_eq!(camera.period, CAMERA_DEFAULT_PERIOD);
}

#[test]
fn embedded_and_downloaded_trainers_are_the_same_vehicle() {
    let embedded = builtin_light_trainer_package().unwrap();
    let copy = downloaded_copy();
    let downloaded = LoadedSyspack::load(&DirectoryPackageSource::new(copy.path())).unwrap();
    assert_eq!(embedded, downloaded);
    assert_eq!(
        embedded.tree().mass_properties().unwrap(),
        downloaded.tree().mass_properties().unwrap()
    );
    let in_tree = LoadedSyspack::load(&DirectoryPackageSource::new(trainer_dir())).unwrap();
    assert_eq!(embedded, in_tree);
}

#[test]
fn embedded_and_downloaded_trainers_fly_the_same_trajectory() {
    let embedded = fly(&builtin_light_trainer_package().unwrap());
    let copy = downloaded_copy();
    let package = LoadedSyspack::load(&DirectoryPackageSource::new(copy.path())).unwrap();
    assert_eq!(embedded, fly(&package));
    assert_ne!(embedded.first(), embedded.last());
}

#[test]
fn one_corrupted_byte_of_the_aircraft_fails_on_disk_and_in_memory() {
    let copy = downloaded_copy();
    let path = copy.path().join(TRAINER_FDM_PATH);
    let mut xml = std::fs::read(&path).unwrap();
    let Some(byte) = xml.get_mut(100) else {
        panic!("the aircraft is shorter than 100 bytes");
    };
    *byte ^= 1;
    std::fs::write(&path, &xml).unwrap();
    assert!(matches!(
        LoadedSyspack::load(&DirectoryPackageSource::new(copy.path())),
        Err(PackError::Syspack(SyspackError::AssetDigestMismatch { path, .. })) if path == TRAINER_FDM_PATH
    ));

    let source = MemoryPackageSource::new(
        TRAINER_MANIFEST.to_vec(),
        BTreeMap::from([(TRAINER_FDM_PATH.to_owned(), xml)]),
    );
    assert!(matches!(
        LoadedSyspack::load(&source),
        Err(PackError::Syspack(SyspackError::AssetDigestMismatch { .. }))
    ));
}

#[test]
fn a_changed_size_fails_even_with_the_right_bytes() {
    let manifest = std::str::from_utf8(TRAINER_MANIFEST).unwrap();
    let declared = TRAINER_FDM.len();
    let edited = manifest.replace(
        &format!("\"size_bytes\": {declared}"),
        &format!("\"size_bytes\": {}", declared + 1),
    );
    assert_ne!(edited, manifest);
    let copy = downloaded_copy();
    std::fs::write(copy.path().join(MANIFEST_FILE), edited).unwrap();
    assert!(matches!(
        LoadedSyspack::load(&DirectoryPackageSource::new(copy.path())),
        Err(PackError::Syspack(SyspackError::AssetSizeMismatch { .. }))
    ));
}

#[test]
fn a_manifest_pointing_outside_the_package_fails() {
    let manifest = std::str::from_utf8(TRAINER_MANIFEST).unwrap();
    let edited = manifest.replace("fdm/trainer.xml", "../fdm/trainer.xml");
    let copy = downloaded_copy();
    std::fs::write(copy.path().join(MANIFEST_FILE), edited).unwrap();
    assert!(matches!(
        LoadedSyspack::load(&DirectoryPackageSource::new(copy.path())),
        Err(PackError::Syspack(SyspackError::UnsafeAssetPath { .. }))
    ));
}
