//! The exported C172 in flight: the package as `vehicles/c172` ships it, flown
//! by JSBSim with its engine running.

use std::collections::BTreeMap;

use glam::DVec3;
use jsbsim::{Fdm, LoadOptions};
use mbd_core::ComponentId;
use sim_core::{
    Block, ControlInputs, EngineStartMode, JSBSIM_DEFAULT_PERIOD, JsbsimFdmBlock, JsbsimFdmConfig,
    KinematicInput, VehicleState,
};
use sim_geo::{Geodetic, LocalFrame, ecef_to_geodetic};
use sim_pack::{
    ControlsSpec, DynamicsSpec, LoadedSyspack, MemoryPackageSource, PackError, PackageSource,
    SyspackError, SystemKind, builtin_c172_package, builtin_c172_source,
};

const STEPS_PER_SECOND: u32 = 120;
const POUNDS_PER_KG: f64 = 2.204_622_621_8;
const KG_M2_PER_SLUG_FT2: f64 = 1.355_817_962;
const INCHES_PER_METRE: f64 = 39.370_078_74;
const CRUISE_HEIGHT_M: f64 = 2_438.4;
const CRUISE_SPEED_M_S: f64 = 59.1611;
const CRUISE_ALPHA_RAD: f64 = 0.05;
/// Best-power mixture at 8,000 ft, where the air is 0.74 of sea-level pressure.
/// Full rich there cannot hold cruise, and a pilot leans for that reason.
const CRUISE_MIXTURE: f64 = 0.7;
const AIRFRAME_FUEL_KG: f64 = 114.0;
const LOADED_MASS_KG: f64 = 1_110.732_985;

#[cfg(test)]
fn start() -> Geodetic {
    Geodetic::from_degrees(39.1, -75.6, CRUISE_HEIGHT_M)
}

/// The aeroplane at its cruise point with the engine running, leaned out.
#[cfg(test)]
fn cruising(package: &LoadedSyspack) -> JsbsimFdmBlock {
    let files = package.build_jsbsim_resolver().unwrap();
    let (aircraft_path, model) = package.jsbsim_model().unwrap();
    let mut block = JsbsimFdmBlock::new(&JsbsimFdmConfig {
        resolver: &files,
        aircraft_path,
        model,
        period: JSBSIM_DEFAULT_PERIOD,
        initial: start(),
        heading_rad: 1.0,
        speed_m_s: CRUISE_SPEED_M_S,
        alpha_rad: CRUISE_ALPHA_RAD,
        engine_start: EngineStartMode::Running,
    })
    .unwrap();
    let mut controls = block.system_controls();
    controls.mixture_norm = CRUISE_MIXTURE;
    block.set_system_controls(controls);
    block
}

#[cfg(test)]
struct Flight {
    height_m: f64,
    speed_m_s: f64,
    bank_rad: f64,
    pitch_rad: f64,
}

#[cfg(test)]
fn flight_of(state: &VehicleState) -> Flight {
    let at = ecef_to_geodetic(state.position_ecef);
    let frame = LocalFrame::at(&at);
    let right = state.attitude * DVec3::Y;
    Flight {
        height_m: at.height_m,
        speed_m_s: state.velocity_ecef.length(),
        bank_rad: (-right.dot(frame.up())).asin(),
        pitch_rad: state.forward_ecef().dot(frame.up()).asin(),
    }
}

/// A wing leveller and a speed and altitude hold, the least a pilot would do.
#[cfg(test)]
fn pilot(state: &VehicleState, climb_rate_m_s: f64) -> ControlInputs {
    let flight = flight_of(state);
    let height_error = CRUISE_HEIGHT_M - flight.height_m;
    let wanted_pitch = (height_error * 0.002 - climb_rate_m_s * 0.01).clamp(-0.15, 0.15);
    ControlInputs {
        roll: -flight.bank_rad * 2.0 - state.angular_rate_body.x * 0.5,
        pitch: (wanted_pitch - flight.pitch_rad) * 2.0 - state.angular_rate_body.y * 0.5,
        yaw: 0.0,
        throttle: 0.9 + (CRUISE_SPEED_M_S - flight.speed_m_s) * 0.1,
    }
}

#[test]
fn the_exported_package_loads_and_declares_what_it_is() {
    let package = builtin_c172_package().unwrap();
    let manifest = package.manifest();
    assert_eq!(manifest.id, "c172s");
    assert_eq!(manifest.kind, SystemKind::FixedWing);
    assert_eq!(
        manifest.controls,
        ControlsSpec {
            has_magnetos: true,
            has_starter: true,
            has_mixture: true,
            has_fuel_cutoff: true,
            has_flaps: true,
            has_wheel_brakes: true,
            ..ControlsSpec::default()
        }
    );
    assert!(matches!(manifest.dynamics, DynamicsSpec::JsbsimXml { .. }));
    assert_eq!(manifest.components.len(), 8);
    assert_eq!(package.instantiate_cots_devices().unwrap().len(), 2);
    assert_eq!(package.camera_mounts().unwrap().len(), 1);
    let mass = package.tree().mass_properties().unwrap();
    assert!((mass.total_mass_kg - LOADED_MASS_KG).abs() < 1e-9);
    let fuel: f64 = ["left_fuel_tank", "right_fuel_tank"]
        .iter()
        .map(|key| {
            let tank = package.component_id(key).unwrap();
            package.tree().state(tank).unwrap().fluid_mass_kg
        })
        .sum();
    assert_eq!(fuel, AIRFRAME_FUEL_KG);
    assert_eq!(package.component_id("airframe").unwrap(), ComponentId(0));
}

#[test]
fn jsbsim_and_the_component_tree_agree_on_the_mass_properties() {
    let package = builtin_c172_package().unwrap();
    let files = package.build_jsbsim_resolver().unwrap();
    let (aircraft_dir, model_name) = package.jsbsim_model().unwrap();
    let mut fdm = Fdm::with_options(LoadOptions::new().with_strict(true)).unwrap();
    fdm.load_model(&files, aircraft_dir, model_name).unwrap();
    fdm.run_ic().unwrap();

    let mass = package.tree().mass_properties().unwrap();
    let weight = fdm.get_property("inertia/weight-lbs").unwrap();
    assert!(
        (weight / POUNDS_PER_KG - mass.total_mass_kg).abs() < 1e-2,
        "{weight}"
    );
    let fuel = fdm.get_property("propulsion/total-fuel-lbs").unwrap();
    assert!(
        (fuel / POUNDS_PER_KG - AIRFRAME_FUEL_KG).abs() < 1e-3,
        "{fuel}"
    );
    let cg_x = fdm.get_property("inertia/cg-x-in").unwrap() / INCHES_PER_METRE;
    assert!((cg_x + mass.cg_body_m.x).abs() < 1e-3, "{cg_x}");
    for (property, tree) in [
        ("inertia/ixx-slugs_ft2", mass.inertia_cg_body.x_axis.x),
        ("inertia/iyy-slugs_ft2", mass.inertia_cg_body.y_axis.y),
        ("inertia/izz-slugs_ft2", mass.inertia_cg_body.z_axis.z),
    ] {
        let jsbsim = fdm.get_property(property).unwrap() * KG_M2_PER_SLUG_FT2;
        assert!(
            (jsbsim - tree).abs() < 0.5,
            "{property}: {jsbsim} vs {tree}"
        );
    }
}

#[test]
fn a_running_start_has_the_engine_turning_at_altitude() {
    let package = builtin_c172_package().unwrap();
    let block = cruising(&package);
    let status = block.engine_status();
    assert!(status.running);
    assert!(status.propeller_rpm > 1_500.0, "{status:?}");
    let flight = flight_of(&block.output());
    assert!((flight.height_m - CRUISE_HEIGHT_M).abs() < 2.0);
    assert!((flight.speed_m_s - CRUISE_SPEED_M_S).abs() < 0.1);
}

#[test]
fn trimmed_at_cruise_it_flies_hands_off() {
    let package = builtin_c172_package().unwrap();
    let mut aircraft = cruising(&package);
    let controls = aircraft.trim_level_flight().unwrap();
    assert!(
        controls.throttle > 0.3 && controls.throttle < 1.0,
        "{controls:?}"
    );
    let input = KinematicInput { controls };
    for _ in 0..60 * STEPS_PER_SECOND {
        aircraft.step(&input).unwrap();
    }
    let flight = flight_of(&aircraft.output());
    assert!(
        (flight.height_m - CRUISE_HEIGHT_M).abs() < 5.0,
        "{}",
        flight.height_m
    );
    assert!(
        (flight.speed_m_s - CRUISE_SPEED_M_S).abs() < 0.5,
        "{}",
        flight.speed_m_s
    );
    assert!(flight.bank_rad.abs() < 0.02, "{}", flight.bank_rad);
    assert!(aircraft.engine_status().running);
}

#[test]
fn a_trimmed_cruise_replays_every_bit_from_a_snapshot() {
    let package = builtin_c172_package().unwrap();
    let mut aircraft = cruising(&package);
    let controls = aircraft.trim_level_flight().unwrap();
    let input = KinematicInput { controls };
    for _ in 0..STEPS_PER_SECOND {
        aircraft.step(&input).unwrap();
    }
    let snapshot = aircraft.snapshot();
    let fly_on = |aircraft: &mut JsbsimFdmBlock| {
        for _ in 0..5 * STEPS_PER_SECOND {
            aircraft.step(&input).unwrap();
        }
        (
            aircraft.output(),
            aircraft.engine_status(),
            aircraft.snapshot(),
        )
    };
    let first = fly_on(&mut aircraft);
    aircraft.restore(&snapshot);
    assert_eq!(fly_on(&mut aircraft), first);
}

#[test]
fn the_engine_burns_the_tank_down_in_flight() {
    let package = builtin_c172_package().unwrap();
    let files = package.build_jsbsim_resolver().unwrap();
    let (aircraft_dir, model_name) = package.jsbsim_model().unwrap();
    let mut fdm = Fdm::with_options(LoadOptions::new().with_strict(true)).unwrap();
    fdm.load_model(&files, aircraft_dir, model_name).unwrap();
    fdm.set_property("ic/h-sl-ft", 8_000.0).unwrap();
    fdm.set_property("ic/vt-fps", 194.0).unwrap();
    fdm.set_property("propulsion/magneto_cmd", 3.0).unwrap();
    fdm.set_property("propulsion/set-running", -1.0).unwrap();
    fdm.run_ic().unwrap();
    fdm.set_property("fcs/throttle-cmd-norm", 0.9).unwrap();
    fdm.set_property("fcs/mixture-cmd-norm", CRUISE_MIXTURE)
        .unwrap();
    let before = fdm.get_property("propulsion/total-fuel-lbs").unwrap();
    for _ in 0..30 * STEPS_PER_SECOND {
        fdm.run().unwrap();
    }
    let after = fdm.get_property("propulsion/total-fuel-lbs").unwrap();
    let burn_pph = (before - after) * 120.0;
    assert!(burn_pph > 20.0 && burn_pph < 80.0, "{burn_pph} lb/h");
}

#[test]
fn it_flies_its_cruise_point_for_two_minutes_under_a_light_hand() {
    let package = builtin_c172_package().unwrap();
    let mut aircraft = cruising(&package);
    let before = aircraft.output();
    let mut previous_height = flight_of(&before).height_m;
    for _ in 0..120 * STEPS_PER_SECOND {
        let state = aircraft.output();
        let height = flight_of(&state).height_m;
        let climb = (height - previous_height) * f64::from(STEPS_PER_SECOND);
        previous_height = height;
        let controls = pilot(&state, climb);
        aircraft.step(&KinematicInput { controls }).unwrap();
    }
    let flight = flight_of(&aircraft.output());
    assert!(
        (flight.height_m - CRUISE_HEIGHT_M).abs() < 50.0,
        "{}",
        flight.height_m
    );
    assert!(
        (flight.speed_m_s - CRUISE_SPEED_M_S).abs() < 3.0,
        "{}",
        flight.speed_m_s
    );
    assert!(flight.bank_rad.abs() < 0.05, "{}", flight.bank_rad);
    let travelled = (aircraft.output().position_ecef - before.position_ecef).length();
    assert!(travelled > 6_000.0, "{travelled}");
}

#[test]
fn closed_throttle_at_cruise_decelerates_and_full_power_holds_it() {
    let package = builtin_c172_package().unwrap();
    let speed_after = |throttle: f64| {
        let mut aircraft = cruising(&package);
        for _ in 0..10 * STEPS_PER_SECOND {
            let state = aircraft.output();
            let mut controls = pilot(&state, 0.0);
            controls.throttle = throttle;
            aircraft.step(&KinematicInput { controls }).unwrap();
        }
        flight_of(&aircraft.output()).speed_m_s
    };
    let (idle, full) = (speed_after(0.0), speed_after(1.0));
    assert!(idle < CRUISE_SPEED_M_S - 2.0, "{idle}");
    assert!(full > CRUISE_SPEED_M_S, "{full}");
}

#[test]
fn a_tampered_aircraft_is_caught_by_its_digest() {
    let source = builtin_c172_source();
    let package = builtin_c172_package().unwrap();
    let asset = "fdm/c172s.xml";
    let manifest = source.read_manifest_bytes().unwrap();
    let mut base_assets: BTreeMap<String, Vec<u8>> = BTreeMap::new();
    for key in package.manifest().assets.keys() {
        base_assets.insert(key.clone(), source.read_asset_bytes(key).unwrap());
    }
    let mut xml = source.read_asset_bytes(asset).unwrap();
    let flip = xml.iter().position(|byte| *byte == b'1').unwrap();
    xml[flip] = b'2';
    let mut tampered_map = base_assets.clone();
    tampered_map.insert(asset.to_owned(), xml);
    let tampered = MemoryPackageSource::new(manifest.clone(), tampered_map);
    assert!(matches!(
        LoadedSyspack::load(&tampered),
        Err(PackError::Syspack(SyspackError::AssetDigestMismatch { path, .. })) if path == asset
    ));
    let mut trunc_map = base_assets;
    trunc_map.insert(asset.to_owned(), vec![b'<']);
    let truncated = MemoryPackageSource::new(manifest, trunc_map);
    assert!(matches!(
        LoadedSyspack::load(&truncated),
        Err(PackError::Syspack(SyspackError::AssetSizeMismatch { .. }))
    ));
}
