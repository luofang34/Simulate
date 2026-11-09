use std::f64::consts::PI;

use pilotage_adapter_api::{ActionResult, AvionicsAttitudeSample, Pose2d};
use pilotage_protocol::{
    BodyRateIntent, ControlAction, ControlIntent, ControlPayload, Generation, IntentFamily,
    LogicalAxisId, ReferenceFrame, ScopeId, ScopedControlFrame, SequenceNum, SessionId, VehicleId,
    VelocityIntent,
};
use pilotage_timing::MonoTimestamp;
use sim_core::{JSBSIM_DEFAULT_PERIOD, KinematicModel};
use sim_pack::{SystemKind, builtin_c172_package};

use super::*;

mod approach;
mod attitude;
mod checkpoint;
mod link_loss;

const VEHICLE: VehicleId = VehicleId::new(5);
const CONFIG: SimulateAdapterConfig = SimulateAdapterConfig {
    vehicle: VEHICLE,
    seed: 0xF5,
};

fn start() -> Geodetic {
    Geodetic::from_degrees(39.1, -75.6, 900.0)
}

fn kinematic() -> SimulateVehicleAdapter<KinematicModel> {
    let state = VehicleState::level_flight(&start(), PI / 2.0, 55.0);
    SimulateVehicleAdapter::new(CONFIG, KinematicModel::new(state))
}

fn c172() -> SimulateVehicleAdapter<JsbsimFdmBlock> {
    let package = builtin_c172_package().unwrap();
    let start = PackageStart {
        position: start(),
        heading_rad: PI / 2.0,
        speed_m_s: 55.0,
        alpha_rad: 0.05,
        period: JSBSIM_DEFAULT_PERIOD,
    };
    SimulateVehicleAdapter::from_package(CONFIG, &package, &start).unwrap()
}

fn frame(
    scope: &str,
    intent: Option<ControlIntent>,
    actions: Vec<ControlAction>,
) -> ScopedControlFrame {
    ScopedControlFrame {
        timing: None,
        admitted_timing: None,
        action_ids: vec![],
        session: SessionId::new(1),
        vehicle: VEHICLE,
        scope: ScopeId::new(scope),
        generation: Generation::new(1),
        sequence: SequenceNum::new(1),
        sampled_at: MonoTimestamp::from_nanos(0),
        profile_revision: 1,
        activation_revision: 0,
        payload: ControlPayload::default(),
        intent,
        actions,
    }
}

fn body_rate(roll_rate: f32, pitch_rate: f32, yaw_rate: f32, thrust: f32) -> Option<ControlIntent> {
    Some(ControlIntent::BodyRate(BodyRateIntent {
        roll_rate,
        pitch_rate,
        yaw_rate,
        thrust,
    }))
}

fn command(rates: Option<ControlIntent>) -> ScopedControlFrame {
    frame(MOTION_SCOPE, rates, vec![])
}

fn sample<B: Block<Input = KinematicInput, Output = VehicleState>>(
    adapter: &mut SimulateVehicleAdapter<B>,
) -> TelemetrySample {
    adapter.sample_telemetry().samples.remove(0)
}

fn bank_rad(state: &VehicleState) -> f64 {
    let frame = LocalFrame::at(&ecef_to_geodetic(state.position_ecef));
    let right = state.attitude * glam::DVec3::Y;
    (-right.dot(frame.up())).atan2(-state.down_ecef().dot(frame.up()))
}

#[test]
fn capabilities_advertise_a_stepped_deterministic_simulation() {
    let capabilities = kinematic().capabilities();
    let execution = capabilities.execution;
    assert!(execution.stepped && execution.deterministic && execution.accelerated);
    assert!(!execution.physically_embodied && !execution.render_capable);
    let vehicle = &capabilities.vehicles[0];
    assert_eq!(vehicle.id, VEHICLE);
    let motion = &vehicle.scopes[0];
    assert_eq!(motion.scope.as_str(), MOTION_SCOPE);
    assert_eq!(motion.intents[0].family, IntentFamily::BodyRate);
    assert_eq!(motion.intents[0].max_angular, 1.0);
    assert!(motion.legacy.is_none());
    assert_eq!(vehicle.scopes[1].scope.as_str(), SIM_LIFECYCLE_SCOPE);
    assert!(!vehicle.link_loss_actions.is_empty());
    assert!(kinematic().video_sources().is_empty());
}

#[test]
fn a_step_budget_advances_exactly_that_many_periods() {
    let mut adapter = kinematic();
    let outcome = adapter.step(StepBudget { ticks: 60 });
    assert_eq!(outcome.advanced, 60);
    assert_eq!(outcome.now, SimTick::new(60));
    assert_eq!(adapter.now().to_nanos(), Some(500_000_000));
    assert_eq!(sample(&mut adapter).tick, SimTick::new(60));
}

/// The same intent must fly the same way on both dynamics sources, though one
/// reads stick as rate and the other as surface deflection.
#[test]
fn a_roll_rate_intent_banks_either_dynamics_source() {
    let intent = body_rate(0.2, 0.0, 0.0, 0.4);
    let mut kinematic = kinematic();
    assert_eq!(
        kinematic.apply_control(&command(intent)).disposition,
        Disposition::Accepted
    );
    kinematic.step(StepBudget { ticks: 240 });
    let mut jsbsim = c172();
    jsbsim.apply_control(&command(intent));
    jsbsim.step(StepBudget { ticks: 240 });

    // Two seconds at 0.2 rad/s, less the rate loop's rise.
    for bank in [
        bank_rad(&kinematic.vehicle_state()),
        bank_rad(&jsbsim.vehicle_state()),
    ] {
        assert!(bank > 0.25 && bank < 0.5, "{bank}");
    }
}

#[test]
fn thrust_reaches_the_vehicle_and_disarming_cuts_it() {
    let speed_after = |actions: Vec<ControlAction>| {
        let mut adapter = kinematic();
        adapter.apply_control(&frame(MOTION_SCOPE, body_rate(0.0, 0.0, 0.0, 1.0), actions));
        adapter.step(StepBudget { ticks: 600 });
        adapter.vehicle_state().velocity_ecef.length()
    };
    let armed = speed_after(vec![]);
    let disarmed = speed_after(vec![ControlAction::Disarm]);
    assert!(armed > 80.0, "{armed}");
    assert!(disarmed < 45.0, "{disarmed}");
}

#[test]
fn arming_actions_answer_and_reach_the_fc_state() {
    let mut adapter = kinematic();
    let outcome = adapter.apply_control(&frame(MOTION_SCOPE, None, vec![ControlAction::Disarm]));
    assert_eq!(
        outcome.action_results,
        vec![ActionResult::accepted(ControlAction::Disarm)]
    );
    adapter.step(StepBudget { ticks: 1 });
    assert_eq!(sample(&mut adapter).fc_state.unwrap().arm_state, 1);
    adapter.apply_control(&frame(MOTION_SCOPE, None, vec![ControlAction::Arm]));
    adapter.step(StepBudget { ticks: 1 });
    assert_eq!(sample(&mut adapter).fc_state.unwrap().arm_state, 2);
}

#[test]
fn an_action_this_scope_does_not_take_is_refused_by_name() {
    let mut adapter = kinematic();
    let outcome = adapter.apply_control(&frame(MOTION_SCOPE, None, vec![ControlAction::SimReset]));
    assert!(!outcome.action_results[0].accepted);
}

#[test]
fn commands_past_the_envelope_are_clamped_and_say_so() {
    for intent in [
        body_rate(5.0, 0.0, 0.0, 0.5),
        body_rate(0.0, 0.5, 0.0, 0.5),
        body_rate(0.0, 0.0, 0.3, 0.5),
        body_rate(0.0, 0.0, 0.0, 1.5),
        body_rate(f32::NAN, 0.0, 0.0, 0.5),
    ] {
        let mut adapter = kinematic();
        assert_eq!(
            adapter.apply_control(&command(intent)).disposition,
            Disposition::Constrained
        );
        adapter.step(StepBudget { ticks: 120 });
        let state = adapter.vehicle_state();
        assert!(state.position_ecef.is_finite() && state.attitude.is_finite());
    }
}

#[test]
fn foreign_frames_are_refused_with_their_reason() {
    let mut adapter = kinematic();
    let mut foreign = command(body_rate(0.0, 0.0, 0.0, 0.5));
    foreign.vehicle = VehicleId::new(9);
    assert_eq!(
        adapter.apply_control(&foreign).disposition,
        Disposition::Rejected(RejectReason::UnknownVehicle)
    );
    assert_eq!(
        adapter
            .apply_control(&frame("vehicle.gimbal", None, vec![]))
            .disposition,
        Disposition::Rejected(RejectReason::UnknownScope)
    );
    let velocity = Some(ControlIntent::Velocity(VelocityIntent {
        frame: ReferenceFrame::BodyFrd,
        vx: 1.0,
        vy: 0.0,
        vz: 0.0,
        yaw_rate: 0.0,
    }));
    assert!(matches!(
        adapter.apply_control(&command(velocity)).disposition,
        Disposition::Rejected(RejectReason::Other(_))
    ));
    let mut legacy = command(None);
    legacy.payload.axes.push((LogicalAxisId::new(1), 0.5));
    assert!(matches!(
        adapter.apply_control(&legacy).disposition,
        Disposition::Rejected(RejectReason::Other(_))
    ));
}

#[test]
fn every_outcome_carries_the_tick_it_happened_on() {
    let mut adapter = kinematic();
    adapter.step(StepBudget { ticks: 7 });
    let accepted = adapter.apply_control(&command(body_rate(0.0, 0.0, 0.0, 0.5)));
    assert_eq!(accepted.tick, SimTick::new(7));
    let refused = adapter.apply_control(&command(None));
    assert_eq!(refused.tick, SimTick::new(7));
}

/// ADR-0018: truth is not an estimate, and may never pass for one.
#[test]
fn avionics_and_truth_are_separate_sources_and_never_the_same_values() {
    let mut adapter = kinematic();
    adapter.step(StepBudget { ticks: 10 });
    let sample = sample(&mut adapter);
    let avionics = sample.avionics.unwrap();
    let truth = sample.sim_truth.unwrap();
    let AvionicsAttitudeSample::Canonical {
        quat_wxyz, stamp, ..
    } = avionics.attitude.unwrap()
    else {
        panic!("the estimator reports a canonical attitude");
    };
    let kinematics = avionics.kinematics.unwrap();

    assert_eq!(
        stamp.role,
        pilotage_adapter_api::SourceRole::OperationalEstimate
    );
    assert_eq!(
        truth.stamp.role,
        pilotage_adapter_api::SourceRole::SimulationTruth
    );
    assert_ne!(stamp.source_id, truth.stamp.source_id);
    assert_ne!(quat_wxyz, truth.quat_wxyz);
    assert_ne!(kinematics.pos_ned_m, truth.pos_ned_m);
    assert_ne!(kinematics.vel_ned_mps, truth.vel_ned_mps);
}

#[test]
fn telemetry_reads_the_vehicle_in_local_axes() {
    let mut adapter = kinematic();
    adapter.apply_control(&command(body_rate(0.0, 0.0, 0.0, 0.3)));
    adapter.step(StepBudget { ticks: 240 });
    let sample = sample(&mut adapter);
    let truth = sample.sim_truth.unwrap();
    let [north, east, down] = truth.pos_ned_m;
    let [v_north, v_east, v_down] = truth.vel_ned_mps;
    assert!(east > 100.0, "due east at 55 m/s for 2 s: {east}");
    assert!(
        north.abs() < 1.0 && down.abs() < 2.0,
        "{:?}",
        truth.pos_ned_m
    );
    assert!(v_east > 50.0 && v_north.abs() < 1.0 && v_down.abs() < 1.0);

    let avionics = sample.avionics.unwrap();
    let kinematics = avionics.kinematics.unwrap();
    assert!((kinematics.pos_ned_m[1] - east).abs() < 10.0);
    assert!((kinematics.vel_ned_mps[1] - v_east).abs() < 1.0);
    assert!((avionics.baro.unwrap().pressure_alt_m - 900.0).abs() < 10.0);
    let Pose2d { heading, .. } = sample.pose.unwrap();
    assert!((heading - PI / 2.0).abs() < 0.01, "{heading}");
    assert!((sample.speed.unwrap() - 55.0).abs() < 5.0);

    let fix = truth.geodetic.unwrap();
    assert!((fix.position.latitude_deg - 39.1).abs() < 0.01);
    assert!((fix.position.vertical.height_m - 900.0).abs() < 2.0);
    let estimated = avionics.geodetic.unwrap();
    assert!(estimated.quality.stated_horizontal_mm().is_some());
    assert_ne!(estimated.position, fix.position);
}

/// A sequence advances only for a new measurement: re-reading a sample must
/// not look like fresh data to Pilotage's ingress gate.
#[test]
fn reading_twice_claims_no_new_measurement() {
    let mut adapter = kinematic();
    adapter.step(StepBudget { ticks: 3 });
    let (first, second) = (sample(&mut adapter), sample(&mut adapter));
    assert_eq!(first, second);
    adapter.step(StepBudget { ticks: 1 });
    let third = sample(&mut adapter);
    let [a, b] = [first, third].map(|s| s.avionics.unwrap().attitude.unwrap().stamp());
    assert_eq!(b.sequence, a.sequence.wrapping_add(1));
    assert!(b.acquired_at_ns > a.acquired_at_ns);
}

fn truth_bits(
    adapter: &SimulateVehicleAdapter<impl Block<Input = KinematicInput, Output = VehicleState>>,
) -> [u64; 7] {
    let state = adapter.vehicle_state();
    let [px, py, pz] = state.position_ecef.to_array();
    let [qx, qy, qz, qw] = state.attitude.to_array();
    [px, py, pz, qx, qy, qz, qw].map(f64::to_bits)
}

#[test]
fn identical_commands_fly_identical_bits_however_the_budget_is_cut() {
    let intent = body_rate(0.1, 0.05, 0.0, 0.5);
    let mut whole = c172();
    whole.apply_control(&command(intent));
    whole.step(StepBudget { ticks: 120 });
    let mut cut = c172();
    cut.apply_control(&command(intent));
    for _ in 0..12 {
        cut.step(StepBudget { ticks: 10 });
    }
    assert_eq!(truth_bits(&whole), truth_bits(&cut));
    assert_eq!(sample(&mut whole), sample(&mut cut));
}

#[test]
fn a_reset_returns_to_the_start_in_a_new_epoch_with_time_still_running() {
    let intent = body_rate(0.2, 0.0, 0.0, 0.8);
    let mut adapter = c172();
    let first_bits = truth_bits(&adapter);
    adapter.apply_control(&command(intent));
    adapter.step(StepBudget { ticks: 120 });
    let before = sample(&mut adapter).sim_truth.unwrap().stamp;

    let reset = frame(SIM_LIFECYCLE_SCOPE, None, vec![ControlAction::SimReset]);
    let outcome = adapter.apply_control(&reset);

    assert_eq!(outcome.disposition, Disposition::Accepted);
    assert!(outcome.action_results[0].accepted);
    assert_eq!(truth_bits(&adapter), first_bits);
    let after = sample(&mut adapter).sim_truth.unwrap().stamp;
    assert_eq!(after.source_epoch, before.source_epoch.wrapping_add(1));
    assert!(after.acquired_at_ns >= before.acquired_at_ns);
    assert_eq!(adapter.step(StepBudget { ticks: 1 }).now, SimTick::new(121));
}

#[test]
fn a_reset_refuses_anything_but_one_reset_action() {
    let mut adapter = kinematic();
    assert!(matches!(
        adapter
            .apply_control(&frame(SIM_LIFECYCLE_SCOPE, None, vec![]))
            .disposition,
        Disposition::Rejected(RejectReason::Other(_))
    ));
}

/// A block that fails after `good` steps.
struct Failing {
    state: VehicleState,
    good: u32,
}

impl Block for Failing {
    type Input = KinematicInput;
    type Output = VehicleState;
    type Snapshot = u32;

    fn period(&self) -> TickSpan {
        JSBSIM_DEFAULT_PERIOD
    }

    fn step(&mut self, _: &KinematicInput) -> Result<(), BlockError> {
        match self.good.checked_sub(1) {
            Some(left) => {
                self.good = left;
                Ok(())
            }
            None => Err(BlockError::FdmTerminated { block: "failing" }),
        }
    }

    fn output(&self) -> VehicleState {
        self.state
    }

    fn snapshot(&self) -> u32 {
        self.good
    }

    fn restore(&mut self, snapshot: &u32) {
        self.good = *snapshot;
    }
}

#[test]
fn a_block_failure_stops_stepping_and_a_reset_clears_it() {
    let state = VehicleState::level_flight(&start(), 0.0, 55.0);
    let mut adapter = SimulateVehicleAdapter::new(CONFIG, Failing { state, good: 3 });

    let outcome = adapter.step(StepBudget { ticks: 10 });

    assert_eq!(outcome.advanced, 3);
    assert!(adapter.fault().is_some());
    assert_eq!(adapter.step(StepBudget { ticks: 1 }).advanced, 0);
    adapter.apply_control(&frame(
        SIM_LIFECYCLE_SCOPE,
        None,
        vec![ControlAction::SimReset],
    ));
    assert!(adapter.fault().is_none());
    assert_eq!(adapter.step(StepBudget { ticks: 2 }).advanced, 2);
}

#[test]
fn a_system_kind_names_its_pilotage_class_where_there_is_one() {
    for (kind, class) in [
        (SystemKind::FixedWing, Some(VehicleClass::FixedWing)),
        (SystemKind::Rotorcraft, Some(VehicleClass::Multicopter)),
        (SystemKind::Multirotor, Some(VehicleClass::Multicopter)),
        (SystemKind::SurfaceVehicle, Some(VehicleClass::GroundRover)),
        (SystemKind::LunarLander, None),
        (SystemKind::ReentryCapsule, None),
        (SystemKind::Spacecraft, None),
        (SystemKind::GroundStation, None),
        (SystemKind::TestBench, None),
        (SystemKind::GenericSystem, None),
    ] {
        assert_eq!(vehicle_class(kind), class, "{kind:?}");
    }
}

#[test]
fn a_package_is_described_to_pilotage_as_its_kind_says() {
    let package = builtin_c172_package().unwrap();
    let start = PackageStart {
        position: Geodetic::from_degrees(39.1, -75.6, 2_438.4),
        heading_rad: 0.0,
        speed_m_s: 59.0,
        alpha_rad: 0.05,
        period: JSBSIM_DEFAULT_PERIOD,
    };
    let adapter = SimulateVehicleAdapter::from_package(CONFIG, &package, &start).unwrap();
    let [descriptor] = adapter.capabilities().vehicles.try_into().unwrap();
    assert_eq!(descriptor.class, VehicleClass::FixedWing);
}
