//! The piston engine of `vehicles/c172` from the ground up: a parked,
//! cold-and-dark aeroplane started through its magnetos and starter.

use sim_core::{
    Block, ControlInputs, EngineStartMode, JSBSIM_DEFAULT_PERIOD, JsbsimFdmBlock, JsbsimFdmConfig,
    KinematicInput, MagnetoSwitch, SystemControls,
};
use sim_geo::{Geodetic, ecef_to_geodetic};
use sim_pack::{LoadedSyspack, builtin_c172_package};

const STEPS_PER_SECOND: u32 = 120;
/// The main wheels' reach below the centre of gravity, which puts them just on
/// the ground at the start.
const RIDE_HEIGHT_M: f64 = 1.3;
const IDLE_THROTTLE: f64 = 0.2;

#[cfg(test)]
fn parked(package: &LoadedSyspack) -> JsbsimFdmBlock {
    let files = package.build_jsbsim_resolver().unwrap();
    let (aircraft_path, model) = package.jsbsim_model().unwrap();
    JsbsimFdmBlock::new(&JsbsimFdmConfig {
        resolver: &files,
        aircraft_path,
        model,
        period: JSBSIM_DEFAULT_PERIOD,
        initial: Geodetic::from_degrees(39.1, -75.6, RIDE_HEIGHT_M),
        heading_rad: 1.0,
        speed_m_s: 0.0,
        alpha_rad: 0.0,
        engine_start: EngineStartMode::ColdAndDark,
    })
    .unwrap()
}

#[cfg(test)]
fn run(block: &mut JsbsimFdmBlock, seconds: u32, throttle: f64) {
    let input = KinematicInput {
        controls: ControlInputs {
            throttle,
            ..ControlInputs::default()
        },
    };
    for _ in 0..seconds * STEPS_PER_SECOND {
        block.step(&input).unwrap();
    }
}

/// `after` applied to the block's controls.
#[cfg(test)]
fn set(block: &mut JsbsimFdmBlock, after: impl Fn(&mut SystemControls)) {
    let mut controls = block.system_controls();
    after(&mut controls);
    block.set_system_controls(controls);
}

/// A settled aeroplane with its fuel on and full rich, nothing else touched.
#[cfg(test)]
fn fuelled_and_settled(package: &LoadedSyspack) -> JsbsimFdmBlock {
    let mut block = parked(package);
    run(&mut block, 3, 0.0);
    set(&mut block, |controls| {
        controls.fuel_cutoff = false;
        controls.mixture_norm = 1.0;
    });
    block
}

/// Key to Start for long enough to catch, then back to Both.
#[cfg(test)]
fn started(package: &LoadedSyspack) -> JsbsimFdmBlock {
    let mut block = fuelled_and_settled(package);
    set(&mut block, |controls| {
        controls.magnetos = MagnetoSwitch::Start;
    });
    run(&mut block, 4, IDLE_THROTTLE);
    set(&mut block, |controls| {
        controls.magnetos = MagnetoSwitch::Both;
    });
    run(&mut block, 10, IDLE_THROTTLE);
    block
}

#[cfg(test)]
fn settled_rpm_on(block: &mut JsbsimFdmBlock, magnetos: MagnetoSwitch) -> f64 {
    set(block, |controls| controls.magnetos = magnetos);
    run(block, 10, IDLE_THROTTLE);
    assert!(block.engine_status().running, "{magnetos:?}");
    block.engine_status().propeller_rpm
}

#[test]
fn a_cold_and_dark_aeroplane_sits_stopped_on_its_gear() {
    let package = builtin_c172_package().unwrap();
    let mut block = parked(&package);
    assert_eq!(block.system_controls(), SystemControls::cold_and_dark());
    let status = block.engine_status();
    assert!(!status.running);
    assert_eq!(status.propeller_rpm, 0.0);
    run(&mut block, 5, 0.0);
    let status = block.engine_status();
    assert!(!status.running);
    assert_eq!(status.propeller_rpm, 0.0);
    assert!(block.output().velocity_ecef.length() < 1e-2);
    let height = ecef_to_geodetic(block.output().position_ecef).height_m;
    assert!(height > 1.0 && height < RIDE_HEIGHT_M, "{height}");
}

#[test]
fn cranking_with_the_magnetos_off_turns_the_engine_but_never_fires_it() {
    let package = builtin_c172_package().unwrap();
    let mut block = fuelled_and_settled(&package);
    set(&mut block, |controls| controls.starter = true);
    run(&mut block, 5, IDLE_THROTTLE);
    let status = block.engine_status();
    assert!(!status.running);
    assert!(status.propeller_rpm > 300.0, "{status:?}");
    set(&mut block, |controls| controls.starter = false);
    run(&mut block, 5, IDLE_THROTTLE);
    assert_eq!(block.engine_status().propeller_rpm, 0.0);
}

#[test]
fn the_key_starts_the_engine_and_it_idles_after_release() {
    let package = builtin_c172_package().unwrap();
    let mut block = started(&package);
    let status = block.engine_status();
    assert_eq!(block.system_controls().magnetos, MagnetoSwitch::Both);
    assert!(status.running);
    assert!(
        status.propeller_rpm > 700.0 && status.propeller_rpm < 1_300.0,
        "{status:?}"
    );
    run(&mut block, 30, IDLE_THROTTLE);
    let later = block.engine_status();
    assert!(later.running);
    assert!(
        (later.propeller_rpm - status.propeller_rpm).abs() < 10.0,
        "{later:?}"
    );
}

#[test]
fn one_magneto_drops_the_rpm_and_the_switch_off_stops_the_engine() {
    let package = builtin_c172_package().unwrap();
    let mut block = started(&package);
    let both = settled_rpm_on(&mut block, MagnetoSwitch::Both);
    let left = settled_rpm_on(&mut block, MagnetoSwitch::Left);
    let back = settled_rpm_on(&mut block, MagnetoSwitch::Both);
    let right = settled_rpm_on(&mut block, MagnetoSwitch::Right);
    assert!(both - left > 25.0, "{both} on both, {left} on left");
    assert!(both - right > 25.0, "{both} on both, {right} on right");
    assert!((back - both).abs() < 1.0, "{back} back on both");

    set(&mut block, |controls| {
        controls.magnetos = MagnetoSwitch::Off;
    });
    run(&mut block, 8, IDLE_THROTTLE);
    let status = block.engine_status();
    assert!(!status.running);
    assert_eq!(status.propeller_rpm, 0.0);
}

#[test]
fn the_fuel_shutoff_stops_a_piston_that_has_no_cutoff_property() {
    let package = builtin_c172_package().unwrap();
    let mut flooded = fuelled_and_settled(&package);
    set(&mut flooded, |controls| {
        controls.fuel_cutoff = true;
        controls.magnetos = MagnetoSwitch::Start;
    });
    run(&mut flooded, 6, IDLE_THROTTLE);
    assert!(!flooded.engine_status().running, "no fuel, no start");

    let mut running = started(&package);
    set(&mut running, |controls| controls.fuel_cutoff = true);
    run(&mut running, 8, IDLE_THROTTLE);
    assert!(!running.engine_status().running);
    assert_eq!(running.engine_status().propeller_rpm, 0.0);
}

#[test]
fn idle_cutoff_stops_the_engine() {
    let package = builtin_c172_package().unwrap();
    let mut block = started(&package);
    set(&mut block, |controls| controls.mixture_norm = 0.0);
    run(&mut block, 8, IDLE_THROTTLE);
    assert!(!block.engine_status().running);
}

#[test]
fn the_brakes_hold_against_power_until_released() {
    let package = builtin_c172_package().unwrap();
    let mut block = started(&package);
    assert_eq!(block.system_controls().wheel_brake_norm, 1.0);
    let parked_at = block.output().position_ecef;
    run(&mut block, 5, 1.0);
    assert!(block.engine_status().propeller_rpm > 1_500.0);
    let crept = (block.output().position_ecef - parked_at).length();
    assert!(crept < 0.5, "{crept}");

    set(&mut block, |controls| controls.wheel_brake_norm = 0.0);
    run(&mut block, 5, 1.0);
    assert!(block.output().velocity_ecef.length() > 5.0);
}

#[test]
fn a_snapshot_taken_mid_start_replays_every_bit_with_its_switches() {
    let package = builtin_c172_package().unwrap();
    let mut block = fuelled_and_settled(&package);
    set(&mut block, |controls| {
        controls.magnetos = MagnetoSwitch::Start;
    });
    run(&mut block, 1, IDLE_THROTTLE);
    let snapshot = block.snapshot();
    let fly_on = |block: &mut JsbsimFdmBlock| {
        run(block, 2, IDLE_THROTTLE);
        set(block, |controls| controls.magnetos = MagnetoSwitch::Both);
        run(block, 2, IDLE_THROTTLE);
        (block.output(), block.engine_status(), block.snapshot())
    };
    let first = fly_on(&mut block);
    assert_eq!(block.system_controls().magnetos, MagnetoSwitch::Both);
    block.restore(&snapshot);
    assert_eq!(block.system_controls().magnetos, MagnetoSwitch::Start);
    let second = fly_on(&mut block);
    assert_eq!(first, second);
    assert!(first.1.running);
}
