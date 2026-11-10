use glam::DVec3;
use sim_core::ControlInputs;
use sim_geo::{LocalFrame, ecef_to_geodetic};
use sim_maplibre::OwnshipVisualKind;
use sim_scenario::SimTimeScale;

use super::*;
use crate::map_rig::aircraft::ownship_visual;

const FRAME_S: f64 = 0.25;

fn fly(flight: &mut Flight, total_s: f64, controls: ControlInputs) {
    let frames = (total_s / FRAME_S).round() as usize;
    for _ in 0..frames {
        flight
            .advance_by(FRAME_S, controls, SimTimeScale::RealTime)
            .expect("spaceflight step");
    }
}

fn height_m(flight: &Flight) -> f64 {
    ecef_to_geodetic(flight.vehicle_state().position_ecef).height_m
}

/// Angle between the rocket's nose and the local vertical, radians. The app flies it in
/// aircraft axes whose "up" (`-Z`) is the nose.
fn nose_off_vertical_rad(flight: &Flight) -> f64 {
    let state = flight.vehicle_state();
    let up = LocalFrame::at(&ecef_to_geodetic(state.position_ecef)).up();
    (state.attitude * DVec3::NEG_Z).angle_between(up)
}

#[test]
fn a_spaceflight_lifts_raketa_off_its_pad_under_guidance() {
    let mut flight = Flight::new().expect("flight");
    flight.fly_raketa().expect("spaceflight");
    assert!(flight.in_space());
    assert_eq!(flight.ascent_guidance(), Some(true));
    assert_eq!(flight.coasting_in_orbit(), Some(false));
    assert_eq!(flight.profile(), None);
    assert_eq!(flight.free_vehicle(), None);
    assert_eq!(
        ownship_visual(flight.profile(), flight.free_vehicle()),
        OwnshipVisualKind::Raketa
    );
    let pad_m = height_m(&flight);
    assert!(
        nose_off_vertical_rad(&flight) < 1e-6,
        "it stands nose up on the pad"
    );

    fly(&mut flight, 20.0, ControlInputs::default());
    let climbed = height_m(&flight) - pad_m;
    assert!(
        (4_000.0..9_000.0).contains(&climbed),
        "20 s of guided ascent climbed {climbed:.0} m"
    );
    let shown = flight.display_vehicle_state().position_ecef;
    let truth = flight.vehicle_state().position_ecef;
    assert!((shown - truth).length() < 50.0, "display tracks truth");
}

#[test]
fn the_pilot_takes_raketa_from_guidance_and_restarts_on_the_pad() {
    let mut flight = Flight::new().expect("flight");
    flight.fly_raketa().expect("spaceflight");
    fly(&mut flight, 8.0, ControlInputs::default());
    flight.take_manual_control().expect("manual");
    assert_eq!(flight.ascent_guidance(), Some(false));

    // Engine off: the vessel is in free fall, and it loses the speed gravity takes.
    let before = flight.vehicle_state().velocity_ecef.length();
    fly(&mut flight, 2.0, ControlInputs::default());
    let coasting = flight.vehicle_state().velocity_ecef.length();
    assert!(
        coasting < before - 10.0,
        "{before} m/s, then {coasting} m/s"
    );

    // Full throttle and a held stick: it speeds up, and the reaction wheels start the
    // nose off the vertical against the air load.
    let stick = ControlInputs {
        throttle: 1.0,
        pitch: -1.0,
        ..ControlInputs::default()
    };
    fly(&mut flight, 3.0, stick);
    assert!(flight.vehicle_state().velocity_ecef.length() > coasting + 50.0);
    assert!(
        nose_off_vertical_rad(&flight) > 0.1,
        "the stick swung the nose {} rad",
        nose_off_vertical_rad(&flight)
    );

    flight.restart().expect("restart");
    assert!(flight.in_space(), "a restart is another spaceflight");
    assert_eq!(flight.ascent_guidance(), Some(true));
    assert!(flight.vehicle_state().velocity_ecef.length() < 1e-6);
    flight.select(None).expect("free flight");
    assert!(!flight.in_space());
}
