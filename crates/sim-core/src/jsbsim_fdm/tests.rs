use std::f64::consts::FRAC_PI_2;

use jsbsim::MemoryResolver;
use sim_geo::{LocalFrame, ecef_to_geodetic, geodetic_to_ecef};

use super::*;

mod ground;
mod systems;

const C172_XML: &str = include_str!("../../../../vehicles/c172s/fdm/c172s.xml");
const JSBSIM_AIRCRAFT_PATH: &str = "aircraft";
const JSBSIM_C172_MODEL: &str = "c172s";

const CRUISE_M_S: f64 = 45.0;
const CRUISE_ALPHA_RAD: f64 = 0.06;

fn ned_axes(position: &Geodetic) -> [DVec3; 3] {
    let frame = LocalFrame::at(position);
    [frame.north(), frame.east(), -frame.up()]
}

fn start() -> Geodetic {
    Geodetic::from_degrees(39.1, -75.6, 900.0)
}

fn c172_resolver() -> MemoryResolver {
    let mut files = MemoryResolver::new();
    files.insert("aircraft/c172s/c172s.xml", C172_XML);
    files
}

/// The C172 with 100 lb more empty weight: the same model under another name.
fn heavier_c172_xml() -> String {
    const OPEN: &str = "<emptywt unit=\"LBS\">";
    const CLOSE: &str = "</emptywt>";
    let start = C172_XML.find(OPEN).unwrap() + OPEN.len();
    let end = start + C172_XML[start..].find(CLOSE).unwrap();
    let pounds: f64 = C172_XML[start..end].trim().parse().unwrap();
    format!(
        "{} {} {}",
        &C172_XML[..start],
        pounds + 100.0,
        &C172_XML[end..]
    )
}

fn c172(heading_rad: f64) -> JsbsimFdmBlock {
    let files = c172_resolver();
    JsbsimFdmBlock::new(&JsbsimFdmConfig {
        resolver: &files,
        aircraft_path: JSBSIM_AIRCRAFT_PATH,
        model: JSBSIM_C172_MODEL,
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
    let block = c172(FRAC_PI_2);
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
    let mut block = c172(0.0);
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
        let mut block = c172(0.0);
        run(&mut block, 30, controls);
        let achieved = block.output().angular_rate_body.dot(axis);
        assert!(achieved > 0.05, "{what}: {achieved} rad/s");
    }
}

#[test]
fn throttle_changes_the_speed_it_settles_at() {
    // 100 s: hands off, the aeroplane trades speed for height in a phugoid of
    // roughly half a minute, so a shorter run samples the oscillation rather
    // than the speed the throttle settles it at.
    let settled = |throttle| {
        let mut block = c172(0.0);
        run(
            &mut block,
            12_000,
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
    let mut block = c172(0.3);
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
    let [mut one, mut two] = [c172(1.0), c172(1.0)];
    run(&mut one, 120, cruise());
    run(&mut two, 120, cruise());
    assert_eq!(bits(&one.output()), bits(&two.output()));
}

#[test]
fn the_step_matches_the_master_clock_period() {
    let mut block = c172(0.0);
    run(&mut block, 120, cruise());
    assert_eq!(block.period().ticks(), TICKS_PER_SECOND / 120);
    assert!((block.fdm.sim_time() - 1.0).abs() < 1e-12);
    assert_eq!(block.fdm.dt().to_bits(), (1.0_f64 / 120.0).to_bits());
}

#[test]
fn a_snapshot_of_another_aircraft_is_refused_and_changes_nothing() {
    let mut files = c172_resolver();
    let xml = heavier_c172_xml();
    files.insert("aircraft/heavier/heavier.xml", xml);
    let mut other = JsbsimFdmBlock::new(&JsbsimFdmConfig {
        resolver: &files,
        aircraft_path: JSBSIM_AIRCRAFT_PATH,
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

    let foreign = c172(0.0).snapshot();
    assert!(matches!(
        other.try_restore(&foreign),
        Err(BlockError::Fdm { .. })
    ));
    assert_eq!(bits(&other.output()), before);
}

#[test]
fn a_zero_period_and_an_unknown_aircraft_are_refused() {
    let files = c172_resolver();
    let config = |period, model| JsbsimFdmConfig {
        resolver: &files,
        aircraft_path: JSBSIM_AIRCRAFT_PATH,
        model,
        period,
        initial: start(),
        heading_rad: 0.0,
        speed_m_s: CRUISE_M_S,
        alpha_rad: CRUISE_ALPHA_RAD,
        engine_start: EngineStartMode::Running,
    };
    assert!(matches!(
        JsbsimFdmBlock::new(&config(TickSpan::ZERO, JSBSIM_C172_MODEL)),
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
    assert_eq!(c172(0.0).system_controls(), running);
}

#[test]
fn levers_are_limited_to_their_travel_and_nan_is_the_lower_stop() {
    let mut block = c172(0.0);
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
fn the_controls_travel_with_the_snapshot() {
    let mut block = c172(0.0);
    let snapshot = block.snapshot();
    block.set_system_controls(SystemControls::cold_and_dark());
    block.restore(&snapshot);
    assert_eq!(block.system_controls(), SystemControls::running_defaults());
}

#[test]
fn c172_articulation_tracks_propeller_phase_disc_blur_and_control_surfaces() {
    let files = c172_resolver();

    let cold = JsbsimFdmBlock::new(&JsbsimFdmConfig {
        resolver: &files,
        aircraft_path: JSBSIM_AIRCRAFT_PATH,
        model: JSBSIM_C172_MODEL,
        period: JSBSIM_DEFAULT_PERIOD,
        initial: start(),
        heading_rad: 0.0,
        speed_m_s: 0.0,
        alpha_rad: 0.0,
        engine_start: EngineStartMode::ColdAndDark,
    })
    .unwrap();
    let cold_vis = cold.articulation().propeller_visual(1.0 / 250.0);
    assert_eq!(cold_vis.blade_opacity, 1.0);
    assert_eq!(cold_vis.disc_opacity, 0.0);

    let mut running = JsbsimFdmBlock::new(&JsbsimFdmConfig {
        resolver: &files,
        aircraft_path: JSBSIM_AIRCRAFT_PATH,
        model: JSBSIM_C172_MODEL,
        period: JSBSIM_DEFAULT_PERIOD,
        initial: start(),
        heading_rad: 0.0,
        speed_m_s: CRUISE_M_S,
        alpha_rad: CRUISE_ALPHA_RAD,
        engine_start: EngineStartMode::Running,
    })
    .unwrap();
    running.set_system_controls(SystemControls {
        flaps_norm: 1.0,
        ..SystemControls::running_defaults()
    });
    let initial_phase = running.articulation().propeller_phase_rad;
    run(
        &mut running,
        60,
        ControlInputs {
            roll: 0.4,
            pitch: -0.25,
            yaw: 0.3,
            throttle: 0.85,
        },
    );
    let art = running.articulation();
    assert!(art.propeller_rpm > 1_200.0, "rpm={}", art.propeller_rpm);
    assert_ne!(art.propeller_phase_rad, initial_phase);
    let spin_vis = art.propeller_visual(1.0 / 250.0);
    assert_eq!(spin_vis.blade_opacity, 0.0);
    assert!(
        (0.25..=0.40).contains(&spin_vis.disc_opacity),
        "disc_opacity={}",
        spin_vis.disc_opacity
    );
    assert!(spin_vis.shutter_blur_angle_rad > 0.2);
    assert!((art.flap_deg - 2.5).abs() < 0.25, "flap={}", art.flap_deg);
    assert!(art.left_aileron_rad.abs() > 0.02);
    assert!((art.right_aileron_rad + art.left_aileron_rad).abs() < 1e-4);
    assert!(art.elevator_rad.abs() > 0.02);
    assert!(art.rudder_rad.abs() > 0.02);

    let snap = running.snapshot();
    assert_eq!(snap.articulation, art);

    run(&mut running, 720, cruise());
    assert!(
        (running.articulation().flap_deg - 30.0).abs() < 0.5,
        "full flap={}",
        running.articulation().flap_deg
    );
}

/// Body-axis downward gear force, lbf: negative while the ground pushes up.
fn gear_down_lbf(block: &mut JsbsimFdmBlock) -> f64 {
    block.fdm.get_property("forces/fbz-gear-lbs").unwrap()
}

/// JSBSim reports terrain from its own geodetic round trip, good to well
/// under a millimetre.
const TERRAIN_READBACK_M: f64 = 1e-4;

#[test]
fn syncing_the_ground_leaves_the_latest_step_untouched() {
    let mut block = c172(0.0);
    run(&mut block, 30, cruise());
    let before = block.snapshot();

    block.sync_ground_elevation_m(123.25).unwrap();

    assert_eq!(block.snapshot(), before);
    let terrain_ft = block
        .fdm
        .read_node(block.nodes.terrain_elevation_asl_ft)
        .unwrap();
    assert!((terrain_ft * METRES_PER_FOOT - 123.25).abs() < TERRAIN_READBACK_M);
    assert!(block.sync_ground_elevation_m(f64::NAN).is_err());
}

#[test]
fn ground_moving_far_below_every_step_does_not_disturb_the_flight() {
    let [mut steady, mut tracked] = [c172(0.4), c172(0.4)];
    let input = KinematicInput { controls: cruise() };
    for step in 0..240_u32 {
        tracked
            .sync_ground_elevation_m(f64::from(step % 37) * 2.5)
            .unwrap();
        steady.step(&input).unwrap();
        tracked.step(&input).unwrap();
        assert_eq!(
            bits(&tracked.output()),
            bits(&steady.output()),
            "step {step}"
        );
    }
}

#[test]
fn ground_raised_into_the_gear_pushes_up_and_lowering_it_lets_go() {
    let [mut free, mut grounded] = [c172(0.0), c172(0.0)];
    run(&mut free, 10, cruise());
    run(&mut grounded, 10, cruise());
    let height = ecef_to_geodetic(grounded.output().position_ecef).height_m;
    assert_eq!(gear_down_lbf(&mut grounded), 0.0);

    // The extended wheels reach about 1.3 m below the reference point, so
    // ground 1 m below it compresses the struts.
    // JSBSim integrates the contact force on the step after it measures it.
    grounded.sync_ground_elevation_m(height - 1.0).unwrap();
    run(&mut free, 3, cruise());
    run(&mut grounded, 3, cruise());
    assert!(
        gear_down_lbf(&mut grounded) < -100.0,
        "{}",
        gear_down_lbf(&mut grounded)
    );
    let up = LocalFrame::at(&start()).up();
    let climb = |block: &JsbsimFdmBlock| block.output().velocity_ecef.dot(up);
    assert!(climb(&grounded) > climb(&free), "the contact pushes up");
    let along = |block: &JsbsimFdmBlock| block.output().velocity_ecef.reject_from(up).length();
    let (sliding, flying) = (along(&grounded), along(&free));
    assert!(
        sliding < flying && sliding > flying - 2.0,
        "rolling friction drags a little: {sliding} vs {flying}"
    );

    grounded.sync_ground_elevation_m(height - 300.0).unwrap();
    run(&mut grounded, 1, cruise());
    assert_eq!(gear_down_lbf(&mut grounded), 0.0);
    assert!(grounded.output().angular_rate_body.is_finite());
}

#[test]
fn msl_ground_is_raised_by_the_geoid_onto_the_ellipsoid() {
    let mut block = c172(0.0);
    let site = start();
    block
        .sync_ground_msl_egm96_m(site.lat_rad, site.lon_rad, 10.0)
        .unwrap();
    let terrain_m = block
        .fdm
        .read_node(block.nodes.terrain_elevation_asl_ft)
        .unwrap()
        * METRES_PER_FOOT;
    let undulation = sim_geo::egm96_undulation_m(site.lat_rad, site.lon_rad);
    assert!(undulation < -30.0, "{undulation}");
    assert!(
        (terrain_m - (10.0 + undulation)).abs() < TERRAIN_READBACK_M,
        "{terrain_m}"
    );
}

/// Between stopped and cruise RPM a turning propeller is never drawn as nothing: as the
/// solid blades fade the blurred disc takes over (a P-38G idles near 860 propeller RPM,
/// where the old ramps left 1 % blades and a 0.11 disc).
#[test]
fn a_turning_propeller_is_always_drawn_as_blades_or_a_disc() {
    for rpm in [0.0, 200.0, 400.0, 600.0, 863.0, 1_200.0, 2_600.0] {
        let art = AirframeArticulation {
            propeller_rpm: rpm,
            ..AirframeArticulation::default()
        };
        let vis = art.propeller_visual(1.0 / 60.0);
        let coverage = f64::from(vis.blade_opacity) + f64::from(vis.disc_opacity) / 0.34;
        assert!((coverage - 1.0).abs() < 1e-3, "rpm {rpm}: {vis:?}");
    }
}
