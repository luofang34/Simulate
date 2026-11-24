//! An aeroplane the host built, flown by hand with the autopilot's servos released: the
//! chain around it changes nothing of its flight, armed or not, and its flight controller
//! runs from a parked start.

use sim_core::{
    Block, ControlInputs, EngineStartMode, JsbsimFdmBlock, JsbsimFdmConfig, KinematicInput,
    SystemControls, TICKS_PER_SECOND, TickSpan,
};
use sim_geo::ecef_to_geodetic;

use super::*;
use crate::{C172Variant, c172_board_on};

type Chain = HostChain<C172Airframe, FixedWingController, FixedWingMixer>;

/// Ground under the aeroplane, metres above the ellipsoid.
const FIELD_M: f64 = 240.0;
/// Static height of the C172's reference point over the ground on its gear, metres.
const GEAR_HEIGHT_M: f64 = 1.324;

fn cycle() -> TickSpan {
    TickSpan::from_ticks(TICKS_PER_SECOND / C172_RATE_HZ)
}

/// The C172S as a host builds it over its own terrain, at the flight controller's cycle.
fn host_fdm(position: Geodetic, speed_m_s: f64, engine_start: EngineStartMode) -> JsbsimFdmBlock {
    let package = C172Variant::C172s.load_package().unwrap();
    let resolver = package.build_jsbsim_resolver().unwrap();
    let (aircraft_path, model) = package.jsbsim_model().unwrap();
    JsbsimFdmBlock::new_on_terrain(
        &JsbsimFdmConfig {
            resolver: &resolver,
            aircraft_path,
            model,
            period: cycle(),
            initial: position,
            heading_rad: START_HEADING_DEG.to_radians(),
            speed_m_s,
            alpha_rad: if speed_m_s > 0.0 { 0.04 } else { 0.0 },
            engine_start,
        },
        FIELD_M,
    )
    .unwrap()
}

/// A host-built C172 in trimmed flight.
fn flying_fdm() -> JsbsimFdmBlock {
    let mut fdm = host_fdm(start(), TRUE_M_S, EngineStartMode::Running);
    fdm.trim_level_flight().unwrap();
    fdm
}

/// `fdm` under a chain flying `route`, its servos released.
fn hand_flown(fdm: JsbsimFdmBlock, route: &ChainRoute) -> Chain {
    let origin = ecef_to_geodetic(fdm.output().position_ecef);
    let airframe = C172Airframe::around(
        fdm,
        ControlInputs::default(),
        SharedAir::new(GeoidStandardAtmosphere::new()),
        UtcTime::from_unix_seconds(START_UNIX_S),
    )
    .unwrap();
    let board = c172_board_on(airframe, &c172_sensor_suite(2026.45, 7), origin, 2026.45).unwrap();
    let mut chain = HostChain::new(board, route).unwrap();
    chain.board_mut().airframe_mut().set_servos_engaged(false);
    chain
}

fn no_plan() -> ChainRoute {
    ChainRoute {
        fixes: Vec::new(),
        ..route(PLAN_LOADED_AT_S)
    }
}

/// What a pilot does with the controls over a flight: slow turns, a pitch doublet and
/// a throttle change, a different input every cycle.
fn pilot_at(cycle: usize) -> ControlInputs {
    let t_s = cycle as f64 / C172_RATE_HZ as f64;
    ControlInputs {
        roll: 0.3 * (0.7 * t_s).sin(),
        pitch: 0.05 + 0.08 * (0.4 * t_s).sin(),
        yaw: 0.1 * (0.3 * t_s).cos(),
        throttle: 0.7 + 0.2 * (0.1 * t_s).sin(),
    }
}

/// The ground the host states at `cycle`: it rises a metre every two seconds, as terrain
/// streaming in under a flight does.
fn ground_at(cycle: usize) -> (f64, u64) {
    let version = (cycle / 500) as u64;
    (FIELD_M + version as f64, version)
}

#[test]
fn released_servos_leave_the_flight_as_the_bare_flight_model_flies_it() {
    let mut bare = flying_fdm();
    // The initial conditions leave a wind of rounding size; the chain's air is calm.
    bare.set_wind_ned_m_s(glam::DVec3::ZERO).unwrap();
    let mut armed = hand_flown(flying_fdm(), &route(PLAN_LOADED_AT_S));
    let mut idle = hand_flown(flying_fdm(), &no_plan());
    let mut commanded = false;
    for cycle in 0..(40 * C172_RATE_HZ as usize) {
        let controls = pilot_at(cycle);
        let (ground_m, version) = ground_at(cycle);
        bare.sync_ground_versioned_m(ground_m, version).unwrap();
        bare.step(&KinematicInput { controls }).unwrap();
        for chain in [&mut armed, &mut idle] {
            let airframe = chain.board_mut().airframe_mut();
            airframe.sync_ground_versioned_m(ground_m, version).unwrap();
            airframe.set_pilot_controls(controls);
            chain.advance_tick().unwrap();
            assert_eq!(chain.true_state(), bare.output(), "cycle {cycle}");
        }
        let command = armed.board().last_command();
        commanded |= command.armed && sim_aviate::direct_fixed_wing_controls(command) != controls;
    }
    assert!(armed.status().armed, "Pilotage armed the flight controller");
    assert!(commanded, "the armed controller commanded other controls");
    assert!(
        !idle.status().armed,
        "nothing arms a controller with no plan"
    );
    assert_eq!(
        armed.board().airframe().fdm().articulation(),
        bare.articulation()
    );
    assert_eq!(
        armed.board().airframe().fdm().engine_status(),
        bare.engine_status()
    );
}

/// A parked aeroplane, engine running or cold, with its flight controller powered.
fn parked(engine_start: EngineStartMode) -> Chain {
    let position = Geodetic {
        height_m: FIELD_M + GEAR_HEIGHT_M,
        ..start()
    };
    hand_flown(host_fdm(position, 0.0, engine_start), &no_plan())
}

fn stand(chain: &mut Chain, seconds: f64, controls: ControlInputs) {
    for _ in 0..(seconds * C172_RATE_HZ as f64) as usize {
        let airframe = chain.board_mut().airframe_mut();
        airframe.sync_ground_versioned_m(FIELD_M, 0).unwrap();
        airframe.set_pilot_controls(controls);
        chain.advance_tick().unwrap();
    }
}

fn moved_m(chain: &Chain, from: &VehicleState) -> f64 {
    (chain.true_state().position_ecef - from.position_ecef).length()
}

#[test]
fn a_parked_aeroplane_stands_on_its_gear_with_the_flight_controller_running() {
    for engine_start in [EngineStartMode::Running, EngineStartMode::ColdAndDark] {
        let mut chain = parked(engine_start);
        let brakes = SystemControls {
            wheel_brake_norm: 1.0,
            ..chain.board().airframe().fdm().system_controls()
        };
        chain.board_mut().airframe_mut().set_system_controls(brakes);
        let start = chain.true_state();
        stand(&mut chain, 20.0, ControlInputs::default());

        let airframe = chain.board().airframe();
        let moved = moved_m(&chain, &start);
        assert!(moved < 0.5, "{engine_start:?}: moved {moved} m");
        assert!(chain.true_state().velocity_ecef.length() < 0.05);
        let articulation = airframe.fdm().articulation();
        assert!(articulation.gear_wow.iter().all(|w| *w), "{articulation:?}");
        assert_eq!(
            airframe.fdm().engine_status().running,
            engine_start == EngineStartMode::Running
        );
        assert!(!chain.status().armed, "{engine_start:?}");
        assert!(
            chain.counters().datagrams_from_fc > 0,
            "the flight controller reports to the host"
        );
    }
}

#[test]
fn a_parked_aeroplanes_flaps_and_brakes_answer_the_pilot() {
    let mut chain = parked(EngineStartMode::Running);
    let running = chain.board().airframe().fdm().system_controls();
    let throttle = ControlInputs {
        throttle: 0.6,
        ..ControlInputs::default()
    };
    let set = |chain: &mut Chain, wheel_brake_norm, flaps_norm| {
        chain
            .board_mut()
            .airframe_mut()
            .set_system_controls(SystemControls {
                wheel_brake_norm,
                flaps_norm,
                ..running
            });
    };

    set(&mut chain, 1.0, 1.0);
    stand(&mut chain, 3.0, ControlInputs::default());
    let held_from = chain.true_state();
    stand(&mut chain, 8.0, throttle);
    let held = moved_m(&chain, &held_from);
    let flap_deg = chain.board().airframe().fdm().articulation().flap_deg;
    assert!(flap_deg > 20.0, "flaps at {flap_deg} deg");

    set(&mut chain, 0.0, 0.0);
    let rolled_from = chain.true_state();
    stand(&mut chain, 8.0, throttle);
    let rolled = moved_m(&chain, &rolled_from);
    assert!(held < 1.0, "the brakes held: {held} m");
    assert!(rolled > 10.0, "it rolled with the brakes off: {rolled} m");
    let flap_deg = chain.board().airframe().fdm().articulation().flap_deg;
    assert!(flap_deg < 1.0, "flaps at {flap_deg} deg");
}
