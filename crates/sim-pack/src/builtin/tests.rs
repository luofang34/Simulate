use std::collections::BTreeMap;
use std::path::{Path, PathBuf};

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

fn c172_dir() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../../vehicles/c172")
}

/// A copy of the C172 package, as a user's download would leave it.
fn downloaded_copy() -> tempfile::TempDir {
    let dir = tempfile::tempdir().unwrap();
    std::fs::create_dir(dir.path().join("fdm")).unwrap();
    std::fs::create_dir(dir.path().join("performance")).unwrap();
    std::fs::create_dir(dir.path().join("geometry")).unwrap();
    for file in [
        MANIFEST_FILE,
        C172_FDM_PATH,
        AIRSPEED_ENVELOPE_ASSET,
        PITOT_STATIC_CALIBRATION_ASSET,
        CALIBRATION_SUITE_ASSET,
        STRUCTURAL_LANDMARKS_ASSET,
    ] {
        std::fs::copy(c172_dir().join(file), dir.path().join(file)).unwrap();
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
fn the_builtin_c172_loads_as_a_jsbsim_fixed_wing() {
    let package = builtin_c172_package().unwrap();
    let manifest = package.manifest();
    assert_eq!(manifest.id, "c172_class_reference");
    assert_eq!(manifest.kind, SystemKind::FixedWing);
    assert!(matches!(manifest.dynamics, DynamicsSpec::JsbsimXml { .. }));
    assert_eq!(
        package.jsbsim_model().unwrap(),
        ("aircraft", "c172_class_reference")
    );
    assert_eq!(package.asset(C172_FDM_PATH), Some(C172_FDM));
}

#[test]
fn the_declared_digest_is_the_digest_of_the_shipped_file() {
    let package = builtin_c172_package().unwrap();
    let declared = &package.manifest().assets[C172_FDM_PATH];
    assert_eq!(declared.size_bytes, C172_FDM.len() as u64);
    let digest: String = Sha256::digest(C172_FDM)
        .iter()
        .map(|byte| format!("{byte:02x}"))
        .collect();
    assert_eq!(declared.sha256, digest);
}

#[test]
fn the_manifest_agrees_with_the_mass_jsbsim_flies() {
    let package = builtin_c172_package().unwrap();
    let mass = package.tree().mass_properties().unwrap();
    let pounds = mass.total_mass_kg / 0.453_592_37;
    let files = package.build_jsbsim_resolver().unwrap();
    let (aircraft_path, model) = package.jsbsim_model().unwrap();
    let mut fdm = jsbsim::Fdm::new().unwrap();
    fdm.load_model(&files, aircraft_path, model).unwrap();
    fdm.run_ic().unwrap();
    let flown = fdm.get_property("inertia/weight-lbs").unwrap();
    assert!((pounds - flown).abs() < 1e-2, "{pounds} vs {flown}");
}

#[test]
fn the_c172_declares_its_sensor_suite() {
    let package = builtin_c172_package().unwrap();
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
    let nose = CameraMount::forward_nose(camera.mount.offset_body_m);
    let difference = camera
        .mount
        .body_to_camera
        .angle_between(nose.body_to_camera);
    assert!(difference < 1e-7, "{difference}");
    assert_eq!(camera.period, CAMERA_DEFAULT_PERIOD);
    assert!(package.pitot_static_device("pitot_static").is_some());
}

#[test]
fn embedded_and_downloaded_c172s_are_the_same_vehicle() {
    let embedded = builtin_c172_package().unwrap();
    let copy = downloaded_copy();
    let downloaded = LoadedSyspack::load(&DirectoryPackageSource::new(copy.path())).unwrap();
    assert_eq!(embedded, downloaded);
    assert_eq!(
        embedded.tree().mass_properties().unwrap(),
        downloaded.tree().mass_properties().unwrap()
    );
    let in_tree = LoadedSyspack::load(&DirectoryPackageSource::new(c172_dir())).unwrap();
    assert_eq!(embedded, in_tree);
}

#[test]
fn embedded_and_downloaded_c172s_fly_the_same_trajectory() {
    let embedded = fly(&builtin_c172_package().unwrap());
    let copy = downloaded_copy();
    let package = LoadedSyspack::load(&DirectoryPackageSource::new(copy.path())).unwrap();
    assert_eq!(embedded, fly(&package));
    assert_ne!(embedded.first(), embedded.last());
}

#[test]
fn one_corrupted_byte_of_the_aircraft_fails_on_disk_and_in_memory() {
    let copy = downloaded_copy();
    let path = copy.path().join(C172_FDM_PATH);
    let mut xml = std::fs::read(&path).unwrap();
    let Some(byte) = xml.get_mut(100) else {
        panic!("the aircraft is shorter than 100 bytes");
    };
    *byte ^= 1;
    std::fs::write(&path, &xml).unwrap();
    assert!(matches!(
        LoadedSyspack::load(&DirectoryPackageSource::new(copy.path())),
        Err(PackError::Syspack(SyspackError::AssetDigestMismatch { path, .. })) if path == C172_FDM_PATH
    ));

    let source = MemoryPackageSource::new(
        C172_MANIFEST.to_vec(),
        BTreeMap::from([(C172_FDM_PATH.to_owned(), xml)]),
    );
    assert!(matches!(
        LoadedSyspack::load(&source),
        Err(PackError::Syspack(SyspackError::AssetDigestMismatch { .. }))
    ));
}

#[test]
fn a_changed_size_fails_even_with_the_right_bytes() {
    let manifest = std::str::from_utf8(C172_MANIFEST).unwrap();
    let declared = C172_FDM.len();
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
    let manifest = std::str::from_utf8(C172_MANIFEST).unwrap();
    let edited = manifest.replace(
        "\"fdm/c172_class_reference.xml\"",
        "\"../fdm/c172_class_reference.xml\"",
    );
    assert_ne!(edited, manifest);
    let copy = downloaded_copy();
    std::fs::write(copy.path().join(MANIFEST_FILE), edited).unwrap();
    assert!(matches!(
        LoadedSyspack::load(&DirectoryPackageSource::new(copy.path())),
        Err(PackError::Syspack(SyspackError::UnsafeAssetPath { .. }))
    ));
}

#[test]
fn the_builtin_p38g_loads_flies_level_and_drives_both_engines() {
    let package = builtin_p38g_package().unwrap();
    assert_eq!(package.manifest().id, "p38g");
    assert_eq!(package.manifest().name, "Lockheed P-38G-13-LO Lightning");
    assert_eq!(package.manifest().kind, SystemKind::FixedWing);
    let catalog = package.livery_catalog().expect("P-38G livery catalog");
    assert_eq!(catalog.default_livery, "miss-virginia");
    assert_eq!(catalog.liveries.len(), 3);
    let miss_virginia = catalog.default_entry().expect("default livery");
    assert_eq!(miss_virginia.serial_number, "43-2264");
    assert_eq!(miss_virginia.tactical_number, "147");
    assert_eq!(miss_virginia.tail_code, "32264");
    let mass = package.tree().mass_properties().unwrap();
    let pounds = mass.total_mass_kg / 0.453_592_37;
    assert!((pounds - 15_020.0).abs() < 1e-2, "{pounds}");

    let files = package.build_jsbsim_resolver().unwrap();
    let (aircraft_path, model) = package.jsbsim_model().unwrap();
    let mut block = JsbsimFdmBlock::new(&JsbsimFdmConfig {
        resolver: &files,
        aircraft_path,
        model,
        period: JSBSIM_DEFAULT_PERIOD,
        initial: Geodetic::from_degrees(39.1, -75.6, 1_500.0),
        heading_rad: 0.0,
        speed_m_s: 95.0,
        alpha_rad: 0.04,
        engine_start: EngineStartMode::Running,
    })
    .unwrap();
    block.set_system_controls(sim_core::SystemControls {
        gear_down: false,
        flaps_norm: 1.0 / 3.0,
        ..sim_core::SystemControls::running_defaults()
    });
    // A closed-loop settle rather than JSBSim's trim: the governed propellers' blade
    // angle comes from the flight control system, which the trim's 0.5 s steady-state
    // propulsion solve does not iterate, so the trim does not converge on this model.
    let mut controls = ControlInputs {
        throttle: 0.7,
        ..ControlInputs::default()
    };
    for _ in 0..600 {
        let state = block.output();
        let rate = state.angular_rate_body;
        controls.roll = (-0.8 * rate.x).clamp(-1.0, 1.0);
        controls.pitch = (-0.6 * rate.y).clamp(-1.0, 1.0);
        controls.yaw = (0.5 * rate.z).clamp(-1.0, 1.0);
        block.step(&KinematicInput { controls }).unwrap();
    }
    let rate = block.output().angular_rate_body;
    assert!(
        rate.x.abs() < 0.05 && rate.z.abs() < 0.05,
        "counter-rotating twin props fly without a roll or yaw tendency: {rate:?}"
    );
    assert_eq!(block.articulation().gear_pos_norm, 0.0);
    assert!(block.engine_status().running);
    assert!(block.articulation().propeller_thrust_n > 2_000.0);
    assert!(
        (block.articulation().flap_deg - 8.0).abs() < 0.05,
        "first notch must select 8 deg Combat Flaps, got {}",
        block.articulation().flap_deg
    );
}

#[test]
fn p38g_unboosted_ailerons_stiffen_at_high_airspeed_and_mach_tuck_noses_down() {
    let package = builtin_p38g_package().unwrap();
    let files = package.build_jsbsim_resolver().unwrap();
    let (aircraft_path, model) = package.jsbsim_model().unwrap();

    // 1. Low-speed roll command (75 m/s = ~145 kt IAS, qbar < 140 psf): full aileron travel (0.35 rad).
    let mut low_speed = JsbsimFdmBlock::new(&JsbsimFdmConfig {
        resolver: &files,
        aircraft_path,
        model,
        period: JSBSIM_DEFAULT_PERIOD,
        initial: Geodetic::from_degrees(39.1, -75.6, 1_000.0),
        heading_rad: 0.0,
        speed_m_s: 75.0,
        alpha_rad: 0.04,
        engine_start: EngineStartMode::Running,
    })
    .unwrap();
    low_speed
        .step(&KinematicInput {
            controls: ControlInputs {
                roll: 1.0,
                throttle: 0.6,
                ..ControlInputs::default()
            },
        })
        .unwrap();
    let low_ail_rad = low_speed.articulation().left_aileron_rad.abs();
    assert!(
        low_ail_rad > 0.33,
        "expected full ~0.35 rad aileron deflection below 140 psf, got {low_ail_rad}"
    );

    // 2. High-speed dive (235 m/s at 3,000 m = Mach ~0.715, qbar > 350 psf):
    //    unboosted aileron deflection is stick-force limited to < 35% of full travel,
    //    and transonic Mach tuck pitches the nose down at neutral elevator.
    let mut high_mach = JsbsimFdmBlock::new(&JsbsimFdmConfig {
        resolver: &files,
        aircraft_path,
        model,
        period: JSBSIM_DEFAULT_PERIOD,
        initial: Geodetic::from_degrees(39.1, -75.6, 3_000.0),
        heading_rad: 0.0,
        speed_m_s: 235.0,
        alpha_rad: 0.0,
        engine_start: EngineStartMode::Running,
    })
    .unwrap();
    high_mach
        .step(&KinematicInput {
            controls: ControlInputs {
                roll: 1.0,
                pitch: 0.0,
                throttle: 0.5,
                ..ControlInputs::default()
            },
        })
        .unwrap();
    let high_ail_rad = high_mach.articulation().left_aileron_rad.abs();
    assert!(
        high_ail_rad < 0.14,
        "unboosted P-38G aileron travel must stiffen at high dive qbar: low={low_ail_rad}, high={high_ail_rad}"
    );
    for _ in 0..24 {
        high_mach
            .step(&KinematicInput {
                controls: ControlInputs::default(),
            })
            .unwrap();
    }
    let pitch_rate_q = high_mach.output().angular_rate_body.y;
    assert!(
        pitch_rate_q < -0.05,
        "transonic Mach tuck above M = 0.65 without dive flaps must induce nose-down pitch rate (q < 0), got q={pitch_rate_q}"
    );
}

#[test]
fn p38g_maneuver_flaps_and_hydraulic_hold_valve_function() {
    let package = builtin_p38g_package().unwrap();
    let files = package.build_jsbsim_resolver().unwrap();
    let (aircraft_path, model) = package.jsbsim_model().unwrap();

    let make_high_alpha = |flaps_norm: f64, valve_closed: bool| {
        let mut block = JsbsimFdmBlock::new(&JsbsimFdmConfig {
            resolver: &files,
            aircraft_path,
            model,
            period: JSBSIM_DEFAULT_PERIOD,
            initial: Geodetic::from_degrees(39.1, -75.6, 7_620.0),
            heading_rad: 0.0,
            speed_m_s: 82.0,
            alpha_rad: 0.2618,
            engine_start: EngineStartMode::Running,
        })
        .unwrap();
        block.set_system_controls(sim_core::SystemControls {
            gear_down: false,
            flaps_norm,
            flap_valve_closed: false,
            ..sim_core::SystemControls::running_defaults()
        });
        let step_in = KinematicInput {
            controls: ControlInputs {
                throttle: 0.85,
                ..ControlInputs::default()
            },
        };
        block.step(&step_in).unwrap();
        if valve_closed {
            block.set_system_controls(sim_core::SystemControls {
                gear_down: false,
                flaps_norm: 0.0,
                flap_valve_closed: true,
                ..sim_core::SystemControls::running_defaults()
            });
        }
        for _ in 0..10 {
            block.step(&step_in).unwrap();
        }
        block
    };
    let clean_high_alpha = make_high_alpha(0.0, false);
    let combat_flap_high_alpha = make_high_alpha(1.0 / 3.0, true);
    assert!(
        (combat_flap_high_alpha.articulation().flap_deg - 8.0).abs() < 0.05,
        "CLOSED hydraulic flap valve must hold 8 deg Maneuver position"
    );
    assert!(
        combat_flap_high_alpha.output().position_ecef.length()
            > clean_high_alpha.output().position_ecef.length(),
        "8 deg Maneuver flaps at high alpha provide positive lift increment"
    );
}

#[test]
fn p38g_single_engine_feathering_and_trim_tabs_function() {
    let package = builtin_p38g_package().unwrap();
    let files = package.build_jsbsim_resolver().unwrap();
    let (aircraft_path, model) = package.jsbsim_model().unwrap();

    let run_single_engine = |feather_left: bool, rudder_trim: f64| {
        let mut block = JsbsimFdmBlock::new(&JsbsimFdmConfig {
            resolver: &files,
            aircraft_path,
            model,
            period: JSBSIM_DEFAULT_PERIOD,
            initial: Geodetic::from_degrees(39.1, -75.6, 2_000.0),
            heading_rad: 0.0,
            speed_m_s: 110.0,
            alpha_rad: 0.04,
            engine_start: EngineStartMode::Running,
        })
        .unwrap();
        block.set_system_controls(sim_core::SystemControls {
            gear_down: false,
            throttle_split: Some([0.0, 0.90]),
            mixture_split: Some([0.0, 0.85]),
            prop_feather: [feather_left, false],
            trim_norm: [0.0, 0.0, rudder_trim],
            coolant_flap_norm: [0.0, 0.65],
            oil_shutter_norm: [0.0, 0.55],
            ..sim_core::SystemControls::running_defaults()
        });
        // Five seconds: the live engine's turbo and governor take seconds to build the
        // thrust asymmetry, as the windmilling engine's drag does.
        for _ in 0..600 {
            block
                .step(&KinematicInput {
                    controls: ControlInputs {
                        throttle: 0.90,
                        ..ControlInputs::default()
                    },
                })
                .unwrap();
        }
        block
    };
    let unfeathered = run_single_engine(false, 0.0);
    let feathered = run_single_engine(true, 0.0);
    let trimmed = run_single_engine(true, -0.12);
    let yaw_unfeathered = unfeathered.output().angular_rate_body.z;
    let yaw_feathered = feathered.output().angular_rate_body.z;
    let yaw_trimmed = trimmed.output().angular_rate_body.z;
    assert!(yaw_unfeathered < -0.04);
    assert!(yaw_feathered > yaw_unfeathered);
    assert!(yaw_trimmed.abs() < yaw_feathered.abs());
    let left_vis = feathered
        .articulation()
        .propeller_visual_for_engine(0, 1.0 / 60.0);
    let right_vis = feathered
        .articulation()
        .propeller_visual_for_engine(1, 1.0 / 60.0);
    assert!((left_vis.blade_angle_deg - 85.0).abs() < 1e-3 && left_vis.disc_opacity == 0.0);
    assert!(right_vis.disc_opacity > 0.25 && right_vis.blade_opacity == 0.0);
}
