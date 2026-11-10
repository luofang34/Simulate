use sim_core::{
    Block, ControlInputs, EngineStartMode, FlapSelector, JSBSIM_DEFAULT_PERIOD, JsbsimFdmBlock,
    JsbsimFdmConfig, KinematicInput, SystemControls,
};
use sim_geo::Geodetic;

use super::super::builtin_p38g_package;

/// The cockpit's controls are drawn where the flight model was commanded: each
/// engine's levers apart, the column and pedals in the pilot's sense (opposite to
/// JSBSim's elevator and rudder commands), and the flap selector's detent.
#[test]
fn p38g_cockpit_controls_follow_the_flight_model_commands() {
    let package = builtin_p38g_package().unwrap();
    let files = package.build_jsbsim_resolver().unwrap();
    let (aircraft_path, model) = package.jsbsim_model().unwrap();
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
    let step = |block: &mut JsbsimFdmBlock, flaps_norm: f64, valve_closed: bool| {
        block.set_system_controls(SystemControls {
            gear_down: false,
            flaps_norm,
            flap_valve_closed: valve_closed,
            throttle_split: Some([0.30, 0.70]),
            mixture_split: Some([0.40, 0.90]),
            prop_pitch_split: Some([0.20, 0.80]),
            trim_norm: [0.10, -0.20, 0.30],
            ..SystemControls::running_defaults()
        });
        block
            .step(&KinematicInput {
                controls: ControlInputs {
                    roll: 0.5,
                    pitch: -0.4,
                    yaw: 0.25,
                    throttle: 0.5,
                },
            })
            .unwrap();
        block.articulation().cockpit
    };
    let cockpit = step(&mut block, 1.0 / 3.0, false);
    let close = |a: f64, b: f64| (a - b).abs() < 1e-9;
    assert!(close(cockpit.roll_norm, 0.5));
    assert!(close(cockpit.pitch_norm, -0.4));
    assert!(close(cockpit.yaw_norm, 0.25));
    assert!(close(cockpit.throttle_norm[0], 0.30) && close(cockpit.throttle_norm[1], 0.70));
    assert!(close(cockpit.mixture_norm[0], 0.40) && close(cockpit.mixture_norm[1], 0.90));
    assert!(close(cockpit.prop_norm[0], 0.20) && close(cockpit.prop_norm[1], 0.80));
    assert!(close(cockpit.trim_norm[1], -0.20) && close(cockpit.trim_norm[2], 0.30));
    assert_eq!(cockpit.flap_selector, FlapSelector::Maneuver);
    assert!(!cockpit.gear_lever_down);
    assert_eq!(
        step(&mut block, 1.0, false).flap_selector,
        FlapSelector::Down
    );
    assert_eq!(
        step(&mut block, 1.0, true).flap_selector,
        FlapSelector::Closed
    );
    assert_eq!(step(&mut block, 0.0, false).flap_selector, FlapSelector::Up);
}
