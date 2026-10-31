use std::f64::consts::FRAC_PI_2;

use jsbsim::MemoryResolver;
use sim_geo::{LocalFrame, ecef_to_geodetic, geodetic_to_ecef};

use super::*;

const TRAINER_XML: &str = include_str!("../../../../vehicles/light-trainer/fdm/trainer.xml");
const JSBSIM_TRAINER_AIRCRAFT_PATH: &str = "aircraft";
const JSBSIM_TRAINER_MODEL: &str = "sim_trainer";

const CRUISE_M_S: f64 = 45.0;
const CRUISE_ALPHA_RAD: f64 = 0.06;

fn ned_axes(position: &Geodetic) -> [DVec3; 3] {
    let frame = LocalFrame::at(position);
    [frame.north(), frame.east(), -frame.up()]
}

fn start() -> Geodetic {
    Geodetic::from_degrees(39.1, -75.6, 900.0)
}

fn jsbsim_trainer_resolver() -> MemoryResolver {
    let mut files = MemoryResolver::new();
    files.insert("aircraft/sim_trainer/sim_trainer.xml", TRAINER_XML);
    files
}

fn trainer(heading_rad: f64) -> JsbsimFdmBlock {
    let files = jsbsim_trainer_resolver();
    JsbsimFdmBlock::new(&JsbsimFdmConfig {
        resolver: &files,
        aircraft_path: JSBSIM_TRAINER_AIRCRAFT_PATH,
        model: JSBSIM_TRAINER_MODEL,
        period: JSBSIM_DEFAULT_PERIOD,
        initial: start(),
        heading_rad,
        speed_m_s: CRUISE_M_S,
        alpha_rad: CRUISE_ALPHA_RAD,
        engine_start: EngineStartMode::Running,
    })
    .unwrap()
}

fn run(block: &mut JsbsimFdmBlock, steps: u32, controls: ControlInputs) {
    let input = KinematicInput { controls };
    for _ in 0..steps {
        block.step(&input).unwrap();
    }
}

fn cruise() -> ControlInputs {
    ControlInputs {
        throttle: 0.2,
        ..ControlInputs::default()
    }
}

/// Exact bits, because a replay that merely lands close has diverged.
fn bits(state: &VehicleState) -> [u64; 13] {
    let [px, py, pz] = state.position_ecef.to_array();
    let [vx, vy, vz] = state.velocity_ecef.to_array();
    let [qx, qy, qz, qw] = state.attitude.to_array();
    let [wx, wy, wz] = state.angular_rate_body.to_array();
    [px, py, pz, vx, vy, vz, qx, qy, qz, qw, wx, wy, wz].map(f64::to_bits)
}

#[test]
fn the_initial_state_lands_on_the_requested_wgs84_pose() {
    let block = trainer(FRAC_PI_2);
    let state = block.output();
    let [north, east, down] = ned_axes(&start());

    assert!((state.position_ecef - geodetic_to_ecef(&start())).length() < 2.0);
    assert!((ecef_to_geodetic(state.position_ecef).height_m - 900.0).abs() < 2.0);
    // Heading east at 0.06 rad of alpha, so the nose is east and raised.
    let nose = state.forward_ecef();
    assert!(nose.dot(east) > 0.99, "{nose}");
    assert!(nose.dot(north).abs() < 1e-3, "{nose}");
    assert!(
        (nose.dot(-down) - CRUISE_ALPHA_RAD.sin()).abs() < 1e-3,
        "{nose}"
    );
    assert!(state.down_ecef().dot(down) > 0.99);
    assert!((state.velocity_ecef.length() - CRUISE_M_S).abs() < 0.1);
    assert!(state.velocity_ecef.normalize().dot(east) > 0.999);
}

#[test]
fn the_velocity_integrates_to_the_position() {
    let mut block = trainer(0.0);
    let before = block.output();
    run(&mut block, 12, cruise());
    let after = block.output();

    let mean = (before.velocity_ecef + after.velocity_ecef) / 2.0;
    let travelled = after.position_ecef - before.position_ecef;
    assert!(
        (travelled - mean * 0.1).length() < 0.05,
        "{travelled} vs {mean}"
    );
}

#[test]
fn each_control_moves_the_aircraft_in_pilot_sense() {
    let deflect = |roll, pitch, yaw| ControlInputs {
        roll,
        pitch,
        yaw,
        ..cruise()
    };
    let cases = [
        (deflect(0.5, 0.0, 0.0), DVec3::X, "roll right"),
        (deflect(0.0, 0.5, 0.0), DVec3::Y, "nose up"),
        (deflect(0.0, 0.0, 0.5), DVec3::Z, "nose right"),
    ];
    for (controls, axis, what) in cases {
        let mut block = trainer(0.0);
        run(&mut block, 30, controls);
        let achieved = block.output().angular_rate_body.dot(axis);
        assert!(achieved > 0.05, "{what}: {achieved} rad/s");
    }
}

#[test]
fn throttle_changes_the_speed_it_settles_at() {
    let settled = |throttle| {
        let mut block = trainer(0.0);
        run(
            &mut block,
            1_200,
            ControlInputs {
                throttle,
                ..ControlInputs::default()
            },
        );
        block.output().velocity_ecef.length()
    };
    assert!(settled(0.9) > settled(0.0) + 5.0);
}

#[test]
fn a_restored_block_replays_bit_identically() {
    let mut block = trainer(0.3);
    let busy = ControlInputs {
        roll: 0.2,
        pitch: 0.1,
        yaw: -0.05,
        throttle: 0.6,
    };
    run(&mut block, 240, busy);
    let saved = block.snapshot();

    run(&mut block, 240, busy);
    let first = (bits(&block.output()), block.snapshot());
    assert_ne!(saved, first.1);

    block.restore(&saved);
    assert_eq!(bits(&block.output()), bits(&saved.output));
    run(&mut block, 240, busy);
    assert_eq!(bits(&block.output()), first.0);
    assert_eq!(block.snapshot(), first.1, "the whole executive state");
}

#[test]
fn two_blocks_from_one_configuration_fly_the_same_bits() {
    let [mut one, mut two] = [trainer(1.0), trainer(1.0)];
    run(&mut one, 120, cruise());
    run(&mut two, 120, cruise());
    assert_eq!(bits(&one.output()), bits(&two.output()));
}

#[test]
fn the_step_matches_the_master_clock_period() {
    let mut block = trainer(0.0);
    run(&mut block, 120, cruise());
    assert_eq!(block.period().ticks(), TICKS_PER_SECOND / 120);
    assert!((block.fdm.sim_time() - 1.0).abs() < 1e-12);
    assert_eq!(block.fdm.dt().to_bits(), (1.0_f64 / 120.0).to_bits());
}

#[test]
fn a_snapshot_of_another_aircraft_is_refused_and_changes_nothing() {
    let mut files = jsbsim_trainer_resolver();
    let xml = TRAINER_XML.replace("1320", "1400");
    files.insert("aircraft/heavier/heavier.xml", xml);
    let mut other = JsbsimFdmBlock::new(&JsbsimFdmConfig {
        resolver: &files,
        aircraft_path: JSBSIM_TRAINER_AIRCRAFT_PATH,
        model: "heavier",
        period: JSBSIM_DEFAULT_PERIOD,
        initial: start(),
        heading_rad: 0.0,
        speed_m_s: CRUISE_M_S,
        alpha_rad: CRUISE_ALPHA_RAD,
        engine_start: EngineStartMode::Running,
    })
    .unwrap();
    let before = bits(&other.output());

    let foreign = trainer(0.0).snapshot();
    assert!(matches!(
        other.try_restore(&foreign),
        Err(BlockError::Fdm { .. })
    ));
    assert_eq!(bits(&other.output()), before);
}

#[test]
fn a_zero_period_and_an_unknown_aircraft_are_refused() {
    let files = jsbsim_trainer_resolver();
    let config = |period, model| JsbsimFdmConfig {
        resolver: &files,
        aircraft_path: JSBSIM_TRAINER_AIRCRAFT_PATH,
        model,
        period,
        initial: start(),
        heading_rad: 0.0,
        speed_m_s: CRUISE_M_S,
        alpha_rad: CRUISE_ALPHA_RAD,
        engine_start: EngineStartMode::Running,
    };
    assert!(matches!(
        JsbsimFdmBlock::new(&config(TickSpan::ZERO, JSBSIM_TRAINER_MODEL)),
        Err(BlockError::InputOutOfRange { .. })
    ));
    assert!(matches!(
        JsbsimFdmBlock::new(&config(JSBSIM_DEFAULT_PERIOD, "no_such_aircraft")),
        Err(BlockError::Fdm { .. })
    ));
}

#[test]
fn magneto_positions_are_the_values_jsbsim_reads() {
    assert_eq!(MagnetoSwitch::Off.magneto_cmd(), 0.0);
    assert_eq!(MagnetoSwitch::Left.magneto_cmd(), 1.0);
    assert_eq!(MagnetoSwitch::Right.magneto_cmd(), 2.0);
    assert_eq!(MagnetoSwitch::Both.magneto_cmd(), 3.0);
    assert_eq!(MagnetoSwitch::Start.magneto_cmd(), 3.0);
}

#[test]
fn the_start_modes_latch_their_controls() {
    let running = SystemControls::running_defaults();
    assert_eq!(running.magnetos, MagnetoSwitch::Both);
    assert_eq!(running.mixture_norm, 1.0);
    assert_eq!(running.prop_pitch_norm, 1.0);
    assert!(running.gear_down && !running.fuel_cutoff && !running.starter);
    let cold = SystemControls::cold_and_dark();
    assert_eq!(cold.magnetos, MagnetoSwitch::Off);
    assert_eq!(cold.mixture_norm, 0.0);
    assert_eq!(cold.wheel_brake_norm, 1.0);
    assert!(cold.fuel_cutoff && !cold.starter);
    assert_eq!(trainer(0.0).system_controls(), running);
}

#[test]
fn levers_are_limited_to_their_travel_and_nan_is_the_lower_stop() {
    let mut block = trainer(0.0);
    block.set_system_controls(SystemControls {
        mixture_norm: f64::NAN,
        prop_pitch_norm: 2.0,
        flaps_norm: -1.0,
        wheel_brake_norm: 0.5,
        ..SystemControls::running_defaults()
    });
    let controls = block.system_controls();
    assert_eq!(controls.mixture_norm, 0.0);
    assert_eq!(controls.prop_pitch_norm, 1.0);
    assert_eq!(controls.flaps_norm, 0.0);
    assert_eq!(controls.wheel_brake_norm, 0.5);
}

#[test]
fn an_electric_aircraft_ignores_the_controls_it_lacks() {
    let mut plain = trainer(0.0);
    let mut switched = trainer(0.0);
    switched.set_system_controls(SystemControls {
        magnetos: MagnetoSwitch::Start,
        starter: true,
        mixture_norm: 0.0,
        fuel_cutoff: true,
        ..SystemControls::running_defaults()
    });
    run(&mut plain, 120, cruise());
    run(&mut switched, 120, cruise());
    assert_eq!(bits(&plain.output()), bits(&switched.output()));
}

#[test]
fn the_controls_travel_with_the_snapshot() {
    let mut block = trainer(0.0);
    let snapshot = block.snapshot();
    block.set_system_controls(SystemControls::cold_and_dark());
    block.restore(&snapshot);
    assert_eq!(block.system_controls(), SystemControls::running_defaults());
}

#[test]
fn an_aircraft_without_pitch_trim_reports_that_it_cannot_trim() {
    let mut block = trainer(0.0);
    assert!(matches!(
        block.trim_level_flight(),
        Err(BlockError::Fdm { .. })
    ));
    assert!(block.output().position_ecef.is_finite());
}
