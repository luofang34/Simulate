//! The hub's hand-flown C172, on its own control chain, through the plant block the
//! session steps: it rewinds with the whole chain and runs from a parked start.

use sim_core::{Block, ControlInputs, SystemControls, TICKS_PER_SECOND, TickSpan, VehicleState};
use sim_flight_chain::C172_RATE_HZ;
use sim_geo::{Geodetic, UtcTime};
use sim_scenario::ScenarioProfile;

use super::PlantBlock;
use crate::vehicles::{AutopilotC172, FreeFlightPlant, FreeFlightVehicle};
use crate::world_surface::WorldSurfaceHandle;

const HEADING_RAD: f64 = 0.6;
const FIELD_M: f64 = 8.5;

fn test_utc() -> UtcTime {
    UtcTime::from_unix_seconds(ScenarioProfile::CorridorNominalCruise.start_utc_unix_s())
}

fn aloft() -> Geodetic {
    Geodetic::from_degrees(39.1582, -75.5244, 600.0)
}

fn cycle() -> TickSpan {
    TickSpan::from_ticks(TICKS_PER_SECOND / C172_RATE_HZ)
}

/// What a pilot does with the controls: a different input every cycle.
fn pilot_at(cycle: usize) -> ControlInputs {
    let t_s = cycle as f64 / C172_RATE_HZ as f64;
    ControlInputs {
        roll: 0.3 * (0.7 * t_s).sin(),
        pitch: 0.05 + 0.08 * (0.4 * t_s).sin(),
        yaw: 0.1 * (0.3 * t_s).cos(),
        throttle: 0.7 + 0.2 * (0.1 * t_s).sin(),
    }
}

fn autopilot(plant: &FreeFlightPlant) -> &AutopilotC172 {
    match plant {
        FreeFlightPlant::C172Autopilot(c172) => c172,
        _ => panic!("the C172 flies on its control chain"),
    }
}

fn fly(block: &mut PlantBlock, from_cycle: usize, cycles: usize) -> Vec<VehicleState> {
    (from_cycle..from_cycle + cycles)
        .map(|cycle| {
            block.step(&pilot_at(cycle)).unwrap();
            block.output()
        })
        .collect()
}

/// The hub's plant block returns the whole chain to a snapshot: the flight after it
/// repeats bit for bit, as often as the snapshot is returned to.
#[test]
fn the_hand_flown_c172_rewinds_through_the_plant_block() {
    let plant = FreeFlightPlant::new_at(
        FreeFlightVehicle::JsbsimC172,
        aloft(),
        HEADING_RAD,
        FIELD_M,
        test_utc(),
    )
    .unwrap();
    let mut block = PlantBlock::new(plant, WorldSurfaceHandle::new());
    fly(&mut block, 0, 1000);
    let at = block.output();
    let snapshot = block.snapshot();
    let first = fly(&mut block, 1000, 1500);
    assert_ne!(first.last(), Some(&at));

    for _ in 0..2 {
        block.restore(&snapshot);
        assert!(block.take_fault().is_none());
        assert_eq!(block.output(), at);
        assert_eq!(fly(&mut block, 1000, 1500), first);
        let c172 = autopilot(block.plant());
        assert!(!c172.chain().board().airframe().servos_engaged());
    }

    // A snapshot of the chain is not one of a bare JSBSim plant, and the other way.
    let p38g = FreeFlightPlant::new_at(
        FreeFlightVehicle::JsbsimP38g,
        aloft(),
        HEADING_RAD,
        FIELD_M,
        test_utc(),
    )
    .unwrap();
    block.restore(&p38g.snapshot());
    assert!(block.take_fault().is_some());
}

fn distance_m(a: &VehicleState, b: &VehicleState) -> f64 {
    (a.position_ecef - b.position_ecef).length()
}

/// A parked hub C172, engine running or cold, stands on its gear on the terrain with
/// its flight controller running, and the switches queued through the plant block take
/// effect: the brakes hold it against power, and released it rolls; the flaps travel.
#[test]
fn a_parked_hand_flown_c172_stands_and_answers_its_switches() {
    let world = WorldSurfaceHandle::new();
    let mut ground = Geodetic::from_degrees(40.524, -74.598, 25.0);
    if let Some(height_m) = world.ellipsoidal_ground_m(&ground) {
        ground.height_m = height_m;
    }
    let idle = ControlInputs::default();
    let power = ControlInputs {
        throttle: 0.6,
        ..idle
    };
    for (vehicle, cold) in [
        (FreeFlightVehicle::JsbsimC172, false),
        (FreeFlightVehicle::JsbsimC172, true),
        (FreeFlightVehicle::JsbsimC172p, false),
    ] {
        let plant = FreeFlightPlant::new_on_ground(vehicle, ground, 1.2, cold, test_utc()).unwrap();
        assert_eq!(plant.period(), cycle());
        let mut block = PlantBlock::new(plant, world.clone());
        let switches = block.system_controls().unwrap();
        let stand = |block: &mut PlantBlock, seconds: usize, controls| {
            let from = block.output();
            for _ in 0..seconds * C172_RATE_HZ as usize {
                block.step(&controls).unwrap();
            }
            distance_m(&block.output(), &from)
        };

        block.queue_system_controls(SystemControls {
            wheel_brake_norm: 1.0,
            flaps_norm: 1.0,
            ..switches
        });
        let settled = stand(&mut block, 12, idle);
        assert!(settled < 0.5, "{vehicle:?} cold={cold}: moved {settled} m");
        assert!(block.output().velocity_ecef.length() < 0.05);
        let art = block.plant().articulation(0.0);
        assert!(art.gear_wow.iter().all(|w| *w), "{vehicle:?}: {art:?}");
        assert!(art.flap_deg > 20.0, "flaps at {} deg", art.flap_deg);
        assert_eq!(art.engine_running, !cold, "{vehicle:?}");
        let chain = autopilot(block.plant()).chain();
        assert!(chain.counters().datagrams_from_fc > 0, "the controller ran");
        assert!(!chain.status().armed);
        if cold {
            continue;
        }

        let held = stand(&mut block, 6, power);
        block.queue_system_controls(SystemControls {
            wheel_brake_norm: 0.0,
            flaps_norm: 0.0,
            ..switches
        });
        let rolled = stand(&mut block, 6, power);
        assert!(held < 1.0, "{vehicle:?}: the brakes held, {held} m");
        assert!(rolled > 10.0, "{vehicle:?}: it rolled {rolled} m");
        assert!(block.plant().articulation(0.0).flap_deg < 1.0);
    }
}
