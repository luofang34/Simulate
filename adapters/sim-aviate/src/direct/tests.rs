use aviate_core::control::{Command, CommandSource, ControlMode, Setpoint};
use aviate_core::math::Quaternion;
use aviate_core::types::{Meters, NormalizedThrust};
use aviate_hal_io::SystemCommand;
use aviate_xil_direct::DirectNoiseTier;
use glam::DVec3;
use mbd_core::{ComponentId, ComponentSpec, ComponentState, ComponentTree, Placement, VacuumAero};
use sim_atmosphere::StandardAtmosphere;
use sim_core::{
    Block, RigidBody6DofBlock, RigidBodyConfig, RigidBodyInput, SimInstant, TICKS_PER_SECOND,
    TickSpan, VehicleState,
};
use sim_geo::{BodyEllipsoid, Geodetic, UtcTime};
use world_core::normal_gravity;

use super::*;
use crate::RotorLayout;

mod gnss_denial;

fn x500_rigid_body(origin: &Geodetic, period: TickSpan) -> RigidBody6DofBlock {
    let g = normal_gravity(GeodeticPosition::wgs84(
        origin.lat_rad.to_degrees(),
        origin.lon_rad.to_degrees(),
        origin.height_m,
    ))
    .unwrap();
    let mass_kg = 20.25 / g;
    let spec = ComponentSpec {
        name: "X500 Quadrotor".to_owned(),
        placement: Placement::ROOT,
        dry_mass_kg: mass_kg,
        dry_inertia_body: glam::DMat3::from_diagonal(DVec3::new(0.029, 0.029, 0.055)),
        thermal_capacity_j_k: 0.0,
        nominal_power_w: 0.0,
    };
    let tree = ComponentTree::new([(spec, ComponentState::at_temperature(293.15))]).unwrap();
    let cfg = RigidBodyConfig {
        body: BodyEllipsoid::WGS84,
        period,
        epoch: UtcTime::from_unix_seconds(0.0),
        initial: VehicleState::level_flight(origin, 0.0, 0.0),
    };
    RigidBody6DofBlock::new(
        cfg,
        tree,
        Box::new(VacuumAero),
        Box::new(StandardAtmosphere),
    )
    .unwrap()
}

fn hold_cmd(sequence: u32, z_ned: f32) -> SystemCommand {
    SystemCommand::FlightControl(Command {
        mode: ControlMode::PositionHold,
        setpoint: Setpoint {
            position: Some([Meters(0.0), Meters(0.0), Meters(z_ned)]),
            ..Setpoint::default()
        },
        config_mode_request: None,
        sensor_overrides: None,
        sequence,
        source: CommandSource::Gcs,
    })
}

#[test]
fn tier0_direct_runner_climbs_6dof_x500_rigid_body_and_emits_mavlink_tx() {
    let origin = Geodetic::from_degrees(39.1582, -75.5244, 15.0);
    let period = TickSpan::from_ticks(TICKS_PER_SECOND / 400); // 400 Hz
    let mut plant = x500_rigid_body(&origin, period);
    let rotors = RotorLayout::x500();

    let mut runner = AviateDirectRunner::new(
        build_direct_x500_kernel().unwrap(),
        AviateDirectConfig {
            noise: DirectNoiseTier::Off,
            ..AviateDirectConfig::ideal(origin)
        },
    )
    .unwrap();

    let g_rest = DVec3::new(0.0, 0.0, -9.801);
    let mut now = SimInstant::START;
    for _ in 0..400 {
        let _ = runner.step_cg(now, &plant.output(), g_rest).unwrap();
        now = now.advanced(period);
    }
    assert!(runner.is_ready());

    let mut seq = 1_u32;
    runner.send_command(hold_cmd(seq, -4.0));
    runner.send_command(SystemCommand::Arm);
    let act0 = runner.step_cg(now, &plant.output(), g_rest).unwrap();
    assert!(runner.is_armed());
    plant
        .step(&RigidBodyInput {
            thrusters: rotors.direct_thrusters(&act0).unwrap(),
            ..RigidBodyInput::default()
        })
        .unwrap();
    now = now.advanced(period);

    let mut tx_bytes_total = 0_usize;
    let mut tx_buf = [0_u8; 512];
    for k in 0..2_400_u32 {
        if k > 0 && k % 40 == 0 {
            seq = seq.wrapping_add(1);
            runner.send_command(hold_cmd(seq, -4.0));
        }
        let site = plant.site_kinematics(ComponentId(0)).unwrap();
        let stim = AviateStimulus {
            vehicle: plant.output(),
            site,
        };
        let act = runner.step_site(now, &stim).unwrap();
        let thrusters = rotors.direct_thrusters(&act).unwrap();
        plant
            .step(&RigidBodyInput {
                thrusters,
                ..RigidBodyInput::default()
            })
            .unwrap();
        now = now.advanced(period);
        tx_bytes_total += runner.pop_mavlink_tx(&mut tx_buf);
    }

    let final_geo = sim_geo::ecef_to_geodetic(plant.output().position_ecef);
    let climbed_m = final_geo.height_m - origin.height_m;
    assert!(
        climbed_m > 2.5 && climbed_m < 5.5,
        "expected climb near +4 m, got {climbed_m:.3} m"
    );
    assert!(tx_bytes_total > 0);
}

#[test]
fn direct_fixed_wing_kernel_drives_aileron_elevator_and_throttle() {
    let origin = Geodetic::from_degrees(39.1582, -75.5244, 300.0);
    let period = TickSpan::from_ticks(TICKS_PER_SECOND / 400);
    let kernel = build_direct_fixed_wing_kernel(&crate::c172_airframe()).unwrap();
    let mut runner = AviateDirectRunner::new(kernel, AviateDirectConfig::ideal(origin)).unwrap();

    let cruise = VehicleState::level_flight(&origin, 0.0, 28.0);
    let f_level = DVec3::new(0.0, 0.0, -9.80);
    let mut now = SimInstant::START;
    while !runner.is_ready() {
        let _ = runner.step_cg(now, &cruise, f_level).unwrap();
        now = now.advanced(period);
    }

    runner.send_command(SystemCommand::Arm);
    let _ = runner.step_cg(now, &cruise, f_level).unwrap();
    now = now.advanced(period);

    runner.send_command(SystemCommand::FlightControl(Command {
        mode: ControlMode::Attitude,
        setpoint: Setpoint {
            attitude: Some(Quaternion::IDENTITY),
            collective_thrust: NormalizedThrust(0.72),
            ..Setpoint::default()
        },
        config_mode_request: None,
        sensor_overrides: None,
        sequence: 1,
        source: CommandSource::Gcs,
    }));

    let act = runner.step_cg(now, &cruise, f_level).unwrap();
    let controls = direct_fixed_wing_controls(&act);
    assert!((controls.throttle - 0.72).abs() < 1.0e-4);
    assert!(controls.roll.abs() < 0.05);
}

#[test]
fn industrial_noise_ekf_estimate_tracks_truth_without_copying_it() {
    let origin = Geodetic::from_degrees(39.1582, -75.5244, 15.0);
    let period = TickSpan::from_ticks(TICKS_PER_SECOND / 400);
    let plant = x500_rigid_body(&origin, period);
    let config = AviateDirectConfig::industrial(origin);
    assert_eq!(config.noise, DirectNoiseTier::Industrial);
    let mut runner = AviateDirectRunner::new(build_direct_x500_kernel().unwrap(), config).unwrap();
    let truth = plant.output();
    let g_rest = DVec3::new(0.0, 0.0, -9.801);
    let mut now = SimInstant::START;
    for _ in 0..800 {
        let _ = runner.step_cg(now, &truth, g_rest).unwrap();
        now = now.advanced(period);
    }
    assert!(runner.is_ready(), "industrial-noise EKF never became ready");
    let estimate = runner.estimated_state().expect("usable estimate");
    let error_m = (estimate.position_ecef - truth.position_ecef).length();
    assert!(error_m < 10.0, "position error {error_m} m");
    assert_ne!(estimate.position_ecef, truth.position_ecef);
    let attitude_err_rad = estimate.attitude.angle_between(truth.attitude);
    assert!(
        attitude_err_rad < 3_f64.to_radians(),
        "attitude error {attitude_err_rad}"
    );
}
