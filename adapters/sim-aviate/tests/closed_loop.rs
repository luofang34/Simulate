//! An X500 flown by a flight controller over the real shared block, with its
//! compute time charged on the master clock.

use core::sync::atomic::AtomicU32;
use std::f64::consts::FRAC_PI_2;

use aviate_xil_contract::{FcState, SharedStateV2, pack_fc_status, seqlock_read, seqlock_write};
use glam::{DMat3, DQuat, DVec3};
use mbd_core::{ComponentId, ComponentSpec, ComponentState, ComponentTree, Placement, VacuumAero};
use sim_atmosphere::VacuumAtmosphere;
use sim_aviate::{
    AviateSharedMemoryEndpoint, AviateStimulus, AviateXilBridge, EnuWorld, FlightControllerCommand,
    RotorLayout, new_shared_state,
};
use sim_core::{
    Block, BridgeError, HitlSyncMode, RigidBody6DofBlock, RigidBodyConfig, RigidBodyInput,
    TICKS_PER_SECOND, TickSpan, VehicleState,
};
use sim_geo::{BodyEllipsoid, Geodetic, LocalFrame, UtcTime};

const STEP: TickSpan = TickSpan::from_ticks(TICKS_PER_SECOND / 250);
const STEPS_PER_SECOND: u32 = 250;
const FC_COMPUTE: TickSpan = TickSpan::from_ticks(TICKS_PER_SECOND / 500);
const AIRFRAME: ComponentId = ComponentId(0);
const FC_MOUNT: ComponentId = ComponentId(1);
const MASS_KG: f64 = 2.0;
const INERTIA_KG_M2: DVec3 = DVec3::new(0.0217, 0.0217, 0.04);
const GRAVITY_M_S2: f64 = 9.80;
/// X500 motor constant, repeated because a controller knows its own airframe
/// from its configuration rather than from the simulator.
const MOTOR_CONSTANT: f64 = 8.54858e-6;
const ARM_M: f64 = 0.174;
const MOMENT_CONSTANT_M: f64 = 0.016;
const MOUNT_OFFSET_M: DVec3 = DVec3::new(0.02, 0.0, -0.03);
const TARGET_UP_M: f64 = 5.0;

#[cfg(test)]
fn origin() -> Geodetic {
    Geodetic::from_degrees(39.1, -75.6, 100.0)
}

#[cfg(test)]
fn x500_tree() -> ComponentTree {
    let state = ComponentState::at_temperature(293.15);
    ComponentTree::new([
        (
            ComponentSpec {
                name: "airframe".to_owned(),
                placement: Placement::ROOT,
                dry_mass_kg: MASS_KG,
                dry_inertia_body: DMat3::from_diagonal(INERTIA_KG_M2),
                thermal_capacity_j_k: 1.0,
                nominal_power_w: 0.0,
            },
            state,
        ),
        (
            ComponentSpec {
                name: "fc_mount".to_owned(),
                placement: Placement::child_of(AIRFRAME, MOUNT_OFFSET_M),
                dry_mass_kg: 0.0,
                dry_inertia_body: DMat3::ZERO,
                thermal_capacity_j_k: 1.0,
                nominal_power_w: 0.0,
            },
            state,
        ),
    ])
    .unwrap()
}

/// The vehicle hovering over the world origin, nose east so its FLU axes start
/// on the world's ENU axes, rolled `roll_rad` and `offset_up_m` off target.
#[cfg(test)]
fn x500(roll_rad: f64, offset_up_m: f64) -> RigidBody6DofBlock {
    let up = TARGET_UP_M + offset_up_m;
    let at = Geodetic::from_degrees(39.1, -75.6, 100.0 + up);
    let mut initial = VehicleState::level_flight(&at, FRAC_PI_2, 0.0);
    initial.attitude = (initial.attitude * DQuat::from_rotation_x(roll_rad)).normalize();
    RigidBody6DofBlock::new(
        RigidBodyConfig {
            body: BodyEllipsoid::WGS84,
            period: STEP,
            epoch: UtcTime::from_unix_seconds(1_700_000_000.0),
            initial,
        },
        x500_tree(),
        Box::new(VacuumAero),
        Box::new(VacuumAtmosphere),
    )
    .unwrap()
}

/// Attitude and altitude hold, fed by nothing but the block: the four rotor
/// speeds for one snapshot, reading tilt and rates as FLU axes near level.
#[cfg(test)]
fn hold_position(state: &aviate_xil_contract::ModelStateBlock) -> [f64; 4] {
    assert_ne!(state.valid, 0, "the simulator published an unreadable step");
    let up = f64::from_bits(state.pos_bits[2]);
    let climb = f64::from_bits(state.vel_bits[2]);
    let [w, x, y, z] = state.quat_bits.map(f64::from_bits);
    let tilt = DVec3::new(x, y, z) * 2.0 * w.signum();
    let rate = DVec3::from_array(state.ang_vel_bits.map(f64::from_bits));

    let thrust = MASS_KG * (GRAVITY_M_S2 + 4.0 * (TARGET_UP_M - up) - 3.0 * climb);
    let torque = -INERTIA_KG_M2 * (tilt * 64.0 + rate * 12.8);
    let roll = torque.x / (4.0 * ARM_M);
    let pitch = torque.y / (4.0 * ARM_M);
    let yaw = torque.z / (4.0 * MOMENT_CONSTANT_M);
    let per_rotor = thrust / 4.0;
    // Lane order: front right, rear left, front left, rear right, with FLU
    // roll positive right wing down and pitch positive nose down.
    [
        per_rotor - roll - pitch - yaw,
        per_rotor + roll + pitch - yaw,
        per_rotor + roll - pitch + yaw,
        per_rotor - roll + pitch + yaw,
    ]
    .map(|rotor_n: f64| (rotor_n.max(0.0) / MOTOR_CONSTANT).sqrt())
}

/// The contract's flight-controller half: read a coherent snapshot, publish
/// the command, acknowledge the step, and declare `Ready` for this epoch.
#[cfg(test)]
fn controller(fc_state: FcState) -> impl FnMut(&mut SharedStateV2) {
    move |shared| {
        let state_seq = shared.state.seq;
        let state = shared.state;
        let snapshot = seqlock_read(&AtomicU32::new(state_seq), || state).unwrap();
        let speeds = hold_position(&snapshot);
        let block = &mut shared.command;
        let (seq, count, bits) = (
            &mut block.seq,
            &mut block.num_motors,
            &mut block.motor_vel_bits,
        );
        seqlock_write(AtomicU32::from_mut(seq), || {
            *count = 4;
            for (lane, speed) in bits.iter_mut().zip(speeds) {
                *lane = speed.to_bits();
            }
        });
        block.fc_step_ack = snapshot.sim_step;
        shared.control.fc_status = pack_fc_status(shared.header.reset_generation, fc_state);
    }
}

#[cfg(test)]
struct Flight<F: FnMut(&mut SharedStateV2)> {
    body: RigidBody6DofBlock,
    bridge: AviateXilBridge<F>,
    layout: RotorLayout,
    command: Option<FlightControllerCommand>,
}

#[cfg(test)]
impl<F: FnMut(&mut SharedStateV2)> Flight<F> {
    fn new(body: RigidBody6DofBlock, peer: F, processing_delay: TickSpan) -> Self {
        let world = EnuWorld::new(&BodyEllipsoid::WGS84, &origin());
        let endpoint =
            AviateSharedMemoryEndpoint::attach(new_shared_state(1), world, peer).unwrap();
        let mode = HitlSyncMode::LockstepBarrier { processing_delay };
        Self {
            body,
            bridge: AviateXilBridge::new(endpoint, mode),
            layout: RotorLayout::x500(),
            command: None,
        }
    }

    /// One master period: publish, deliver what is due, apply it, integrate.
    fn step(&mut self) -> Result<(), BridgeError> {
        let now = self.body.now();
        let stimulus = AviateStimulus {
            vehicle: self.body.output(),
            site: self.body.site_kinematics(FC_MOUNT).unwrap(),
        };
        self.bridge.stimulate(now, &stimulus)?;
        for delivered in self.bridge.poll(now)? {
            self.command = Some(delivered.response);
        }
        let thrusters = match &self.command {
            Some(command) => self.layout.thrusters(command).unwrap(),
            None => Vec::new(),
        };
        self.body
            .step(&RigidBodyInput {
                thrusters,
                ..RigidBodyInput::default()
            })
            .unwrap();
        Ok(())
    }

    fn fly(&mut self, steps: u32) {
        for _ in 0..steps {
            self.step().unwrap();
        }
    }

    fn up_m(&self) -> f64 {
        let state = self.body.output();
        let world_origin = BodyEllipsoid::WGS84.geodetic_to_fixed(&origin());
        let up = LocalFrame::at(&origin()).up();
        (state.position_ecef - world_origin).dot(up)
    }

    /// Angle between body down and the local down, radians.
    fn tilt_rad(&self) -> f64 {
        let down = -LocalFrame::at(&origin()).up();
        self.body.output().down_ecef().angle_between(down)
    }
}

#[cfg(test)]
fn bits(state: &VehicleState) -> Vec<u64> {
    state
        .position_ecef
        .to_array()
        .into_iter()
        .chain(state.velocity_ecef.to_array())
        .chain(state.attitude.to_array())
        .chain(state.angular_rate_body.to_array())
        .map(f64::to_bits)
        .collect()
}

#[test]
fn the_controller_recovers_a_tilted_low_vehicle_and_holds_it_over_the_block() {
    let mut flight = Flight::new(x500(0.12, -1.5), controller(FcState::Ready), FC_COMPUTE);
    flight.fly(STEPS_PER_SECOND / 4);
    assert!(flight.body.output().velocity_ecef.length() > 0.0);
    flight.fly(STEPS_PER_SECOND * 10);

    assert!(
        (flight.up_m() - TARGET_UP_M).abs() < 0.05,
        "{}",
        flight.up_m()
    );
    assert!(flight.tilt_rad() < 0.01, "{}", flight.tilt_rad());
    let total = u64::from(STEPS_PER_SECOND / 4 + STEPS_PER_SECOND * 10);
    let stats = flight.bridge.stats();
    assert_eq!(
        stats.delivered,
        total - 1,
        "the last answer is still in flight"
    );
    assert_eq!(stats.dropped, 0);
    assert_eq!(
        stats.max_staleness,
        TickSpan::from_ticks(STEP.ticks() - FC_COMPUTE.ticks())
    );

    // The rotors, not a shortcut, carry the vehicle: its mount feels one g up.
    let sensed = flight.body.site_kinematics(FC_MOUNT).unwrap();
    assert!(
        (sensed.specific_force_site_m_s2 + DVec3::Z * GRAVITY_M_S2).length() < 0.1,
        "{}",
        sensed.specific_force_site_m_s2,
    );
}

#[test]
fn identical_flights_replay_bit_for_bit() {
    let fly = || {
        let mut flight = Flight::new(x500(0.08, 0.5), controller(FcState::Ready), FC_COMPUTE);
        flight.fly(STEPS_PER_SECOND * 3);
        bits(&flight.body.output())
    };
    assert_eq!(fly(), fly());
}

#[test]
fn compute_time_on_the_master_clock_changes_the_flight() {
    let fly = |delay| {
        let mut flight = Flight::new(x500(0.08, 0.5), controller(FcState::Ready), delay);
        flight.fly(STEPS_PER_SECOND);
        bits(&flight.body.output())
    };
    let late = TickSpan::from_ticks(STEP.ticks() * 3);
    assert_ne!(fly(FC_COMPUTE), fly(late));
}

#[test]
fn a_controller_that_never_reports_ready_never_spins_the_rotors() {
    let mut flight = Flight::new(x500(0.0, 0.0), controller(FcState::Converging), FC_COMPUTE);
    flight.fly(STEPS_PER_SECOND);
    let sensed = flight.body.site_kinematics(FC_MOUNT).unwrap();
    assert!(sensed.specific_force_site_m_s2.length() < 1e-3);
    let fall = TARGET_UP_M - flight.up_m();
    assert!((fall - 0.5 * GRAVITY_M_S2).abs() < 0.1, "{fall}");
}

#[test]
fn a_block_corrupted_in_flight_stops_the_loop_rather_than_flying_on_it() {
    let mut healthy = controller(FcState::Ready);
    let mut remaining = STEPS_PER_SECOND;
    let peer = move |shared: &mut SharedStateV2| {
        healthy(shared);
        remaining = remaining.saturating_sub(1);
        if remaining == 0 {
            shared.header.magic = 0;
        }
    };
    let mut flight = Flight::new(x500(0.0, 0.0), peer, FC_COMPUTE);
    flight.fly(STEPS_PER_SECOND - 1);
    let BridgeError::Endpoint { source, .. } = flight.step().unwrap_err();
    assert!(source.detail.contains("BadMagic"), "{source}");
}
