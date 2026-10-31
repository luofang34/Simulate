use aviate_xil_contract::{AttachError, FcState, unpack_fc_status};

use super::*;

const LANES: ModelLanes = ModelLanes {
    position_enu_m: [1.0, 2.0, 3.0],
    attitude_wxyz: [1.0, 0.0, 0.0, 0.0],
    velocity_enu_m_s: [4.0, 5.0, 6.0],
    angular_rate_enu_rad_s: [7.0, 8.0, 9.0],
};

fn write_motors(shared: &mut SharedStateV2, speeds: &[f64]) {
    shared.command.num_motors = speeds.len() as u32;
    for (lane, speed) in shared.command.motor_vel_bits.iter_mut().zip(speeds) {
        *lane = speed.to_bits();
    }
}

#[test]
fn a_new_block_passes_its_own_validation_and_names_no_flight_controller() {
    let shared = new_shared_state(7);
    validate_shared_state(&shared).unwrap();
    assert_eq!(shared.header.writer_incarnation, 7);
    assert_eq!(shared.header.declared_size as usize, EXPECTED_SIZE);
    assert_ne!(shared.control.lockstep_enabled, 0);
    assert_eq!(shared.state.valid, 0, "no model state has been published");
    let (generation, state) = unpack_fc_status(shared.control.fc_status);
    assert_eq!((generation, state), (1, FcState::Init));
}

#[test]
fn attach_fails_closed_on_a_foreign_magic() {
    let mut shared = new_shared_state(1);
    shared.header.magic = 0xDEAD;
    assert_eq!(
        validate_shared_state(&shared),
        Err(AviateBridgeError::Attach(AttachError::BadMagic {
            found: 0xDEAD
        })),
    );
}

#[test]
fn attach_fails_closed_on_a_stale_layout() {
    let mut shared = new_shared_state(1);
    shared.header.layout_version = LAYOUT_VERSION - 1;
    assert_eq!(
        validate_shared_state(&shared),
        Err(AviateBridgeError::Attach(AttachError::VersionMismatch {
            found: LAYOUT_VERSION - 1
        })),
    );
}

#[test]
fn attach_fails_closed_on_a_writer_of_another_size() {
    let mut shared = new_shared_state(1);
    shared.header.declared_size = 304;
    assert_eq!(
        validate_shared_state(&shared),
        Err(AviateBridgeError::Attach(
            AttachError::DeclaredSizeMismatch { found: 304 }
        )),
    );
}

#[test]
fn a_block_no_writer_owns_is_not_ready() {
    let mut shared = new_shared_state(1);
    shared.header.plugin_ready = 0;
    assert_eq!(
        validate_shared_state(&shared),
        Err(AviateBridgeError::NotReady)
    );
}

#[test]
fn a_publish_leaves_the_seqlock_even_with_every_lane_readable() {
    let mut shared = new_shared_state(1);
    publish_model_state(&mut shared, 42, 1_500, &LANES).unwrap();
    let state = shared.state;
    assert_eq!(state.seq, 2);
    assert_eq!(state.sim_step, 42);
    assert_eq!(state.time_us, 1_500);
    assert_eq!(state.reset_generation, 1);
    assert_eq!(state.valid, 1);
    assert_eq!(state.pos_bits.map(f64::from_bits), LANES.position_enu_m);
    assert_eq!(state.quat_bits.map(f64::from_bits), LANES.attitude_wxyz);
    assert_eq!(state.vel_bits.map(f64::from_bits), LANES.velocity_enu_m_s);
    assert_eq!(
        state.ang_vel_bits.map(f64::from_bits),
        LANES.angular_rate_enu_rad_s
    );
}

#[test]
fn a_snapshot_carries_the_epoch_it_was_published_in() {
    let mut shared = new_shared_state(1);
    shared.header.reset_generation = 9;
    publish_model_state(&mut shared, 1, 0, &LANES).unwrap();
    assert_eq!(shared.state.reset_generation, 9);
}

#[test]
fn non_finite_lanes_publish_invalid_and_keep_the_last_good_ones() {
    let mut shared = new_shared_state(1);
    publish_model_state(&mut shared, 1, 0, &LANES).unwrap();
    let bad = ModelLanes {
        velocity_enu_m_s: [f64::NAN, 0.0, 0.0],
        ..LANES
    };
    publish_model_state(&mut shared, 2, 10, &bad).unwrap();
    assert_eq!(shared.state.valid, 0, "a reader must withhold this step");
    assert_eq!(shared.state.sim_step, 2, "the step itself still counts");
    assert_eq!(
        shared.state.vel_bits.map(f64::from_bits),
        LANES.velocity_enu_m_s
    );
}

#[test]
fn an_unfinished_write_stops_the_next_publish() {
    let mut shared = new_shared_state(1);
    shared.state.seq = 3;
    assert_eq!(
        publish_model_state(&mut shared, 1, 0, &LANES),
        Err(AviateBridgeError::ModelStateInFlight { seq: 3 }),
    );
    assert_eq!(shared.state.sim_step, 0, "nothing was written");
}

#[test]
fn a_consistent_command_reads_only_its_populated_lanes() {
    let mut shared = new_shared_state(1);
    write_motors(&mut shared, &[100.0, 200.0, 300.0, 400.0]);
    shared.command.motor_vel_bits[6] = f64::NAN.to_bits();
    let command = read_motor_command(&mut shared).unwrap();
    assert_eq!(command.speeds_rad_s(), &[100.0, 200.0, 300.0, 400.0]);
}

#[test]
fn a_writer_stuck_mid_command_gives_no_snapshot() {
    let mut shared = new_shared_state(1);
    shared.command.seq = 5;
    assert_eq!(
        read_motor_command(&mut shared),
        Err(AviateBridgeError::MotorCommandTorn { seq: 5 }),
    );
}

#[test]
fn a_command_claiming_more_lanes_than_the_block_has_is_refused() {
    let mut shared = new_shared_state(1);
    shared.command.num_motors = 9;
    assert_eq!(
        read_motor_command(&mut shared),
        Err(AviateBridgeError::MotorCountOutOfRange { count: 9 }),
    );
}

#[test]
fn a_populated_lane_that_is_not_a_speed_is_refused_by_name() {
    let mut shared = new_shared_state(1);
    write_motors(&mut shared, &[100.0, f64::INFINITY]);
    assert_eq!(
        read_motor_command(&mut shared),
        Err(AviateBridgeError::MotorSpeedNotFinite {
            lane: 1,
            bits: f64::INFINITY.to_bits()
        }),
    );
}
