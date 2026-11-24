//! The hub's hand-flown C172 on its own control chain: the chain changes nothing of
//! the flight.

use glam::DVec3;
use sim_core::{Block, ControlInputs, SystemControls, TICKS_PER_SECOND, TickSpan};
use sim_flight_chain::C172_RATE_HZ;
use sim_geo::Geodetic;
use sim_pack::{builtin_c172p_package, builtin_c172s_package};

use super::super::jsbsim::{build_jsbsim_fdm, step_jsbsim_with_force};
use super::super::{FreeFlightPlant, FreeFlightVehicle};
use crate::world_surface::WorldSurfaceHandle;

const HEADING_RAD: f64 = 0.6;
const FIELD_M: f64 = 8.5;

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

fn autopilot(plant: &FreeFlightPlant) -> &super::super::AutopilotC172 {
    match plant {
        FreeFlightPlant::C172Autopilot(c172) => c172,
        _ => panic!("the C172 flies on its control chain"),
    }
}

/// Both hub C172s fly bit for bit as the bare JSBSim model does under the same controls,
/// systems and ground at the flight controller's cycle, while the controller runs.
#[test]
fn the_hand_flown_c172_flies_as_its_bare_flight_model_does() {
    let world = WorldSurfaceHandle::new();
    for (vehicle, pkg, speed_m_s) in [
        (FreeFlightVehicle::JsbsimC172, builtin_c172s_package(), 52.0),
        (
            FreeFlightVehicle::JsbsimC172p,
            builtin_c172p_package(),
            50.0,
        ),
    ] {
        let mut plant = FreeFlightPlant::new_at(vehicle, aloft(), HEADING_RAD, FIELD_M).unwrap();
        assert_eq!(plant.period(), cycle(), "{vehicle:?}");
        let (mut bare, mut bare_force) = build_jsbsim_fdm(
            &pkg.unwrap(),
            aloft(),
            HEADING_RAD,
            speed_m_s,
            FIELD_M,
            cycle(),
        )
        .unwrap();
        // The initial conditions leave a wind of rounding size; the chain's air is calm.
        bare.set_wind_ned_m_s(DVec3::ZERO).unwrap();
        assert_eq!(plant.state(), bare.output(), "{vehicle:?} at the start");

        let mut worst_force_m_s2 = 0.0_f64;
        for cycle in 0..(20 * C172_RATE_HZ as usize) {
            if cycle == 1500 {
                let flaps = SystemControls {
                    flaps_norm: 0.5,
                    ..bare.system_controls()
                };
                plant.set_system_controls(flaps);
                bare.set_system_controls(flaps);
            }
            let controls = pilot_at(cycle);
            plant.step(controls, &world).unwrap();
            step_jsbsim_with_force(&mut bare, &mut bare_force, controls, &world).unwrap();
            assert_eq!(plant.state(), bare.output(), "{vehicle:?} cycle {cycle}");
            let force = plant.specific_force_frd_m_s2().unwrap();
            worst_force_m_s2 = worst_force_m_s2.max((force - bare_force).length());
        }
        assert_eq!(plant.articulation(0.0), bare.articulation());
        assert_eq!(plant.engine_status(), Some(bare.engine_status()));
        assert_eq!(plant.system_controls(), Some(bare.system_controls()));
        assert_eq!(plant.cg_structural_in(), Some(bare.cg_structural_in()));
        assert_eq!(plant.fdm_ground_m(), bare.last_ground_sync_m());
        assert!(plant.articulation(0.0).flap_deg > 5.0);
        // The chain's specific force is the inertial unit's: it carries the Earth-rate
        // Coriolis and centrifugal terms the display-only figure of a bare plant omits.
        assert!(
            (0.001..0.06).contains(&worst_force_m_s2),
            "{vehicle:?}: specific force differs by {worst_force_m_s2} m/s²"
        );

        let chain = autopilot(&plant).chain();
        assert!(chain.counters().datagrams_from_fc > 0, "the controller ran");
        assert!(chain.board().estimated_state().is_some(), "and navigates");
        assert!(!chain.status().armed, "nothing arms it without a plan");
        assert!(!chain.board().airframe().servos_engaged());
        assert_eq!(plant.aviate_armed(), None);
    }
}
