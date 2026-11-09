//! The piston engine of `vehicles/c172` from the ground up: a parked,
//! cold-and-dark aeroplane started through its magnetos and starter.

use sim_core::{
    Block, ControlInputs, EngineStartMode, JSBSIM_DEFAULT_PERIOD, JsbsimFdmBlock, JsbsimFdmConfig,
    KinematicInput, MagnetoSwitch, SystemControls,
};
use sim_geo::{Geodetic, ecef_to_geodetic};
use sim_pack::{LoadedSyspack, builtin_c172_package};

const STEPS_PER_SECOND: u32 = 120;
/// The main wheels' reach below the centre of gravity at the shipped load (51.2 in
/// below the reference line, which the high wing tanks lift the CG 3.1 in above),
/// which puts them just on the ground at the start.
const RIDE_HEIGHT_M: f64 = 1.378;
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

/// The shipped FDM with its propeller's power coefficient replaced by a flat
/// `cp`, so full throttle on the brakes settles the engine wherever that load
/// balances it.
#[cfg(test)]
fn fdm_loaded_by_flat_cp(cp: f64) -> jsbsim::Fdm {
    const FDM_XML: &str = include_str!("../../../vehicles/c172/fdm/c172_class_reference.xml");
    const VIRTUAL_PATH: &str = "aircraft/c172_class_reference/c172_class_reference.xml";
    let table = FDM_XML.find(r#"<table name="C_POWER""#).unwrap();
    let open = table + FDM_XML[table..].find("<tableData>").unwrap() + "<tableData>".len();
    let close = open + FDM_XML[open..].find("</tableData>").unwrap();
    let xml = format!(
        "{}\n0.0 {cp}\n2.0 {cp}\n{}",
        &FDM_XML[..open],
        &FDM_XML[close..]
    );
    let package = builtin_c172_package().unwrap();
    let mut files = package.build_jsbsim_resolver().unwrap();
    files.insert(VIRTUAL_PATH, xml.into_bytes());
    let (aircraft_path, model) = package.jsbsim_model().unwrap();
    let mut fdm = jsbsim::Fdm::new().unwrap();
    fdm.load_model(&files, aircraft_path, model).unwrap();
    fdm
}

/// `(rpm, brake horsepower)` at full throttle and full rich on the brakes at
/// sea level, standard day, against a flat power coefficient `cp`.
#[cfg(test)]
fn static_full_throttle(cp: f64) -> (f64, f64) {
    let mut fdm = fdm_loaded_by_flat_cp(cp);
    fdm.run_ic().unwrap();
    let cg_z_in = fdm.get_property("inertia/cg-z-in").unwrap();
    let wheels_z_in = (0..3)
        .map(|n| {
            fdm.get_property(&format!("gear/unit[{n}]/z-position"))
                .unwrap()
        })
        .fold(f64::INFINITY, f64::min);
    for (property, value) in [
        ("ic/terrain-elevation-ft", 0.0),
        ("ic/h-sl-ft", (cg_z_in - wheels_z_in) / 12.0),
        ("ic/vt-fps", 0.0),
        ("ic/theta-deg", 0.0),
        ("fcs/throttle-cmd-norm", 1.0),
        ("fcs/mixture-cmd-norm", 1.0),
        ("propulsion/magneto_cmd", 3.0),
        ("propulsion/set-running", -1.0),
    ] {
        fdm.set_property(property, value).unwrap();
    }
    fdm.run_ic().unwrap();
    for _ in 0..30 * STEPS_PER_SECOND {
        fdm.set_property("fcs/left-brake-cmd-norm", 1.0).unwrap();
        fdm.set_property("fcs/right-brake-cmd-norm", 1.0).unwrap();
        fdm.run().unwrap();
    }
    (
        fdm.get_property("propulsion/engine[0]/engine-rpm").unwrap(),
        fdm.get_property("propulsion/engine[0]/power-hp").unwrap(),
    )
}

/// Full-throttle power at `rpm`, interpolated between two flat loads that
/// settle the engine either side of it.
#[cfg(test)]
fn full_throttle_hp_at(rpm: f64, heavy_cp: f64, light_cp: f64) -> f64 {
    let (rpm_low, hp_low) = static_full_throttle(heavy_cp);
    let (rpm_high, hp_high) = static_full_throttle(light_cp);
    assert!(rpm_low < rpm && rpm < rpm_high, "{rpm_low} {rpm_high}");
    hp_low + (hp_high - hp_low) * (rpm - rpm_low) / (rpm_high - rpm_low)
}

/// The Lycoming IO-360-L2A is rated 180 BHP at 2,700 rpm at sea level,
/// standard day, full throttle and full rich; the engine model must make
/// that within 1% before any propeller is fitted against it.
#[test]
fn full_throttle_sea_level_power_is_the_lycoming_rating() {
    let hp = full_throttle_hp_at(2_700.0, 0.0460, 0.0440);
    assert!((hp - 180.0).abs() <= 1.8, "{hp:.1} hp at 2,700 rpm");
}

/// Lycoming 60297-12 Fig. 3-25 puts the same full-throttle line at about
/// 155 BHP at 2,300 rpm (midway between its 151 hp at 2,200 and 160 hp at
/// 2,400), the static run-up's power; pinned within 2%.
#[test]
fn full_throttle_power_at_the_static_run_up_follows_the_lycoming_curve() {
    let hp = full_throttle_hp_at(2_300.0, 0.0660, 0.0610);
    assert!((hp - 155.0).abs() <= 3.1, "{hp:.1} hp at 2,300 rpm");
}
