use glam::DVec3;
use sim_core::ControlInputs;
use sim_geo::{LocalFrame, ecef_to_geodetic};
use sim_maplibre::OwnshipVisualKind;
use sim_scenario::SimTimeScale;

use sim_orbital::NaifId;
use sim_scenario::ScenarioWorld;

use super::mode::Mode;
use super::rewind::seconds;
use super::*;
use crate::map_rig::aircraft::ownship_visual;
use crate::sky_rig::{SkyRig, wall_clock_utc_s};

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

/// Flies the guided ascent at the flown warp limit until the vessel coasts in orbit.
fn fly_to_orbit(flight: &mut Flight) {
    for _ in 0..1_200 {
        if flight.coasting_in_orbit() == Some(true) {
            return;
        }
        flight
            .advance_by(FRAME_S, ControlInputs::default(), SimTimeScale::Warp(4.0))
            .expect("ascent frame");
    }
    panic!("no orbit after 1200 s of guided ascent");
}

#[test]
fn a_spaceflight_warps_only_while_it_coasts_and_follows_its_central_body() {
    let mut flight = Flight::new().expect("flight");
    flight.fly_raketa().expect("spaceflight");
    let warp = SimTimeScale::Warp(1_000.0);
    let seconds_now = |flight: &Flight| seconds(flight.now());

    // Under guidance, flight dynamics carry the vessel: a thousandfold warp runs at four.
    flight
        .advance_by(FRAME_S, ControlInputs::default(), warp)
        .expect("governed frame");
    assert!((seconds_now(&flight) - 4.0 * FRAME_S).abs() < 0.05);

    fly_to_orbit(&mut flight);
    assert_eq!(flight.ascent_guidance(), Some(false), "guidance let go");
    assert_eq!(flight.world(), ScenarioWorld::EarthGlobe);
    let before = seconds_now(&flight);
    flight
        .advance_by(FRAME_S, ControlInputs::default(), warp)
        .expect("coasting frame");
    assert!((seconds_now(&flight) - before - 1_000.0 * FRAME_S).abs() < 1.0);
    assert_eq!(flight.coasting_in_orbit(), Some(true));
    let height = height_m(&flight);
    assert!(
        (150_000.0..230_000.0).contains(&height),
        "coasting at {height} m"
    );

    // About Mars the world is Mars and the Sun is where the dynamics have it.
    let Mode::Space(world) = &mut flight.mode else {
        panic!("a spaceflight");
    };
    world
        .switch_central_body(NaifId::MARS)
        .expect("origin switch");
    assert_eq!(flight.world(), ScenarioWorld::MarsGlobe);
    let observer = flight.vehicle_state().position_ecef;
    assert!(
        observer.length() > 5.0e10,
        "Mars is far from low Earth orbit"
    );
    let mut rig = SkyRig::new();
    let sky = rig.evaluate(&flight, observer, wall_clock_utc_s());
    let sun = flight
        .dynamics_direction_fixed(NaifId::SUN, observer)
        .expect("the Sun from Mars");
    assert!(sky.bodies.is_none(), "no Earth sky about Mars");
    assert!(sun.angle_between(sky.environment.sun.direction_fixed) < 1e-9);

    let Mode::Space(world) = &mut flight.mode else {
        panic!("a spaceflight");
    };
    world
        .switch_central_body(NaifId::MOON)
        .expect("origin switch");
    assert_eq!(flight.world(), ScenarioWorld::LunarTranquility);
}

#[test]
fn spaceflight_navigation_ux_warp_governor_and_light_time_dsn_link_are_live() {
    use glam::DQuat;
    use sim_orbital::WarpBottleneckReason;

    let mut flight = Flight::new().expect("flight");
    flight.fly_raketa().expect("spaceflight");
    assert_eq!(
        flight.space_reference_frame(),
        Some(SpaceReferenceFrame::BodyInertial)
    );

    fly(&mut flight, 10.0, ControlInputs::default());
    let ascent_warp = flight
        .space_warp_status(1_000.0, ControlInputs::default())
        .expect("warp status");
    assert_eq!(
        ascent_warp.bottleneck_reason,
        WarpBottleneckReason::AtmosphericFlight
    );
    assert!(ascent_warp.achieved_warp <= 4.0);
    let plume = flight.space_aerospike_plume().expect("plume");
    assert!(plume.visible && plume.shock_diamond_spacing_m > 0.0);

    let navball = flight.space_fdai_navball().expect("fdai navball");
    assert!((navball.prograde.dir_body.length() - 1.0).abs() < 1e-12);
    assert!((navball.radial_out.dir_body.length() - 1.0).abs() < 1e-12);

    let orrery = flight
        .space_orrery_projection(DVec3::new(0.0, 0.0, 2.0e9), DQuat::IDENTITY, 1.0, 1.5)
        .expect("orrery");
    assert_eq!(orrery.len(), 5);

    let (pos_inert, _) = flight.space_state_in_reference_frame().expect("inertial");
    flight.set_space_reference_frame(SpaceReferenceFrame::BodyFixed);
    let (pos_fixed, _) = flight.space_state_in_reference_frame().expect("fixed");
    assert!((pos_inert.length() - pos_fixed.length()).abs() < 1.0);
    flight.set_space_reference_frame(SpaceReferenceFrame::SunEarthSynodic);
    let (pos_syn, _) = flight.space_state_in_reference_frame().expect("synodic");
    assert!((pos_inert.length() - pos_syn.length()).abs() < 1.0);

    fly_to_orbit(&mut flight);
    let coast_warp = flight
        .space_warp_status(1_000.0, ControlInputs::default())
        .expect("coast warp");
    assert_eq!(
        coast_warp.bottleneck_reason,
        WarpBottleneckReason::Unconstrained
    );
    let leo_link = flight.space_dsn_link_status().expect("leo dsn link");
    assert!((0.005..0.06).contains(&leo_link.light_time_s));

    let Mode::Space(world) = &mut flight.mode else {
        panic!("a spaceflight");
    };
    world
        .switch_central_body(NaifId::MARS)
        .expect("origin switch");
    let mars_origin_link = flight
        .space_dsn_link_status()
        .expect("mars-origin dsn link");
    assert!(
        (mars_origin_link.light_time_s - leo_link.light_time_s).abs() < 1e-6,
        "switching coordinate origin to Mars preserves physical SSB light-time"
    );

    let jezero_fixed = sim_geo::BodyEllipsoid::MARS_IAU
        .geodetic_to_fixed(&sim_geo::Geodetic::from_degrees(18.38, 77.58, 0.0));
    let mars_surface_link = flight
        .space_dsn_link_from_body_fixed(NaifId::MARS, jezero_fixed, DVec3::ZERO)
        .expect("mars surface dsn link");
    assert!(mars_surface_link.light_time_s > 180.0);
}
