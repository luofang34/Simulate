//! The host of Pilotage's mission engine: it converts the vehicle's
//! telemetry into ownship samples, ticks the engine, and frames what the
//! engine emits onto the vehicle adapter.
//!
//! The mission engine is Pilotage's own and so is the guidance it runs.
//! This host stands in for Pilotage's session host: there are no sessions,
//! leases or wire, and one holder owns the motion scope throughout. It reads
//! the flight controller's estimate and never the simulation's truth sample.

mod route;

use navigate_contract::{ClockDomainId, DurationNanos, GeodeticPosition, MonotonicNanos};
use pilotage_adapter_api::{
    AvionicsAttitudeSample, AvionicsSample, Disposition, TelemetrySample, VehicleAdapter,
};
use pilotage_mavlink::link::estimator::QUALITY_DEGRADED;
use pilotage_mavlink::sampling::yaw_of;
use pilotage_mission::{
    MissionAction, MissionConfig, MissionEngine, MissionEvent, MissionLimits, MissionOutput,
    MissionPlanRecord, MissionState, OwnshipSample, OwnshipSource, TruthRole,
};
use pilotage_protocol::{
    ControlAction, ControlIntent, ControlPayload, DIRECT_SCOPE, Generation, MOTION_SCOPE, ScopeId,
    ScopedControlFrame, SequenceNum, SessionId, VehicleId,
};
use pilotage_timing::MonoTimestamp;
use sim_geo::Geodetic;

use crate::ChainError;
pub use route::RouteFix;

/// The one clock the mission, the flight controller's boot time and the
/// simulation share.
const MISSION_CLOCK: ClockDomainId = ClockDomainId::new(1);
/// Attitude, as Pilotage numbers the estimate groups.
const ATTITUDE_VALID: u32 = 0b0001;
/// Position and velocity.
const NAVIGATION_VALID: u32 = 0b1100;
/// Arm state code of an armed flight controller.
const ARMED: u32 = 2;
/// How long the flight controller has to report itself armed: two of its
/// heartbeats and a margin.
const ARM_CONFIRMATION: DurationNanos = DurationNanos::from_millis(2_500);

/// A route and how the mission flies it.
#[derive(Debug, Clone, PartialEq)]
pub struct MissionSetup {
    /// The vehicle the mission commands.
    pub vehicle: VehicleId,
    /// The mission anchor: the pad, which the flight controller measures its
    /// local frame from.
    pub anchor: Geodetic,
    /// The route's fixes in fly order.
    pub fixes: Vec<RouteFix>,
    /// Height of the whole route above the anchor, metres.
    pub cruise_height_m: f64,
    /// Along-track speed, m/s.
    pub cruise_mps: f64,
    /// Climb rate to the cruise height, m/s.
    pub climb_rate_mps: f64,
}

/// What one mission frame did.
#[derive(Debug, Clone, PartialEq)]
pub struct FrameReport {
    /// The mission phase after the frame.
    pub state: MissionState,
    /// What the engine reported, in order.
    pub events: Vec<MissionEvent>,
    /// The intent the engine emitted, if any.
    pub intent: Option<ControlIntent>,
    /// How the vehicle disposed of the frame, when one was sent.
    pub disposition: Option<Disposition>,
}

/// An arm action the vehicle link took and the flight controller has yet
/// to confirm.
#[derive(Debug, Clone, Copy, PartialEq)]
struct PendingArm {
    action_id: u64,
    sent_at: MonotonicNanos,
}

/// Pilotage's mission engine and the state of its conversation with one
/// vehicle.
pub struct MissionHost {
    engine: MissionEngine,
    record: MissionPlanRecord,
    vehicle: VehicleId,
    sequence: u32,
    pending_arm: Option<PendingArm>,
    last_estimate: Option<u32>,
    arms_refused: u32,
}

impl MissionHost {
    /// The mission engine over `setup`'s route, flying the vehicle's own
    /// estimate.
    pub fn new(setup: &MissionSetup) -> Result<Self, ChainError> {
        let mission = |error| ChainError::Mission(Box::new(error));
        let (snapshot, provenance, route) = route::packed_route(&setup.fixes).map_err(mission)?;
        let anchor = GeodeticPosition::new(
            setup.anchor.lat_rad,
            setup.anchor.lon_rad,
            setup.anchor.height_m,
        );
        let mut config = MissionConfig::new(route, anchor, MISSION_CLOCK);
        config.cruise_height_m = setup.cruise_height_m;
        config.cruise_mps = setup.cruise_mps;
        config.climb_rate_mps = setup.climb_rate_mps;
        config.limits = MissionLimits {
            max_horizontal_mps: setup.cruise_mps,
            max_vertical_mps: setup.climb_rate_mps,
            ..MissionLimits::default()
        };
        config.ownship_source = OwnshipSource::vehicle_estimate();
        let (engine, record) =
            MissionEngine::new(&snapshot, provenance, config).map_err(mission)?;
        Ok(Self {
            engine,
            record,
            vehicle: setup.vehicle,
            sequence: 0,
            pending_arm: None,
            last_estimate: None,
            arms_refused: 0,
        })
    }

    /// The pack-for-flight record of the built mission.
    pub const fn record(&self) -> &MissionPlanRecord {
        &self.record
    }

    /// The mission engine.
    pub const fn engine(&self) -> &MissionEngine {
        &self.engine
    }

    /// Arm actions the flight controller did not confirm in time.
    pub const fn arms_refused(&self) -> u32 {
        self.arms_refused
    }

    /// One mission frame at `now`: the vehicle's telemetry in, the engine's
    /// tick, and its intent and action out to the vehicle.
    pub fn frame<A: VehicleAdapter>(
        &mut self,
        adapter: &mut A,
        now: MonotonicNanos,
    ) -> FrameReport {
        let telemetry = adapter.sample_telemetry();
        let sample = telemetry
            .samples
            .iter()
            .find(|sample| sample.vehicle == self.vehicle);
        if let Some(sample) = sample {
            self.offer_estimate(sample, now);
            self.settle_arm(sample, now);
        }
        let output = self.engine.tick(now);
        let disposition = self.send(adapter, &output, now);
        FrameReport {
            state: output.state,
            events: output.events,
            intent: output.intent,
            disposition,
        }
    }

    /// Offers the flight controller's estimate when it carries a new,
    /// authorized position and velocity.
    fn offer_estimate(&mut self, sample: &TelemetrySample, now: MonotonicNanos) {
        let Some(ownship) = sample.avionics.as_ref().and_then(ownship_of) else {
            return;
        };
        if self.last_estimate == Some(ownship.sequence) {
            return;
        }
        self.last_estimate = Some(ownship.sequence);
        self.engine.on_ownship(&ownship, now);
    }

    /// Answers a pending arm action from the flight controller's own report:
    /// the vehicle link taking the command is not the controller arming.
    fn settle_arm(&mut self, sample: &TelemetrySample, now: MonotonicNanos) {
        let Some(pending) = self.pending_arm else {
            return;
        };
        let arm_state = sample.fc_state.as_ref().map(|state| state.arm_state);
        let waited = now.as_nanos().saturating_sub(pending.sent_at.as_nanos());
        let accepted = if arm_state == Some(ARMED) {
            true
        } else if waited > ARM_CONFIRMATION.as_nanos() {
            self.arms_refused = self.arms_refused.wrapping_add(1);
            false
        } else {
            return;
        };
        self.pending_arm = None;
        self.engine.on_action_result(pending.action_id, accepted);
    }

    fn send<A: VehicleAdapter>(
        &mut self,
        adapter: &mut A,
        output: &MissionOutput,
        now: MonotonicNanos,
    ) -> Option<Disposition> {
        if output.intent.is_none() && output.action.is_none() {
            return None;
        }
        let frame = self.frame_of(output, now);
        let outcome = adapter.apply_control(&frame);
        if let Some(MissionAction { action_id, .. }) = output.action {
            let taken = !matches!(outcome.disposition, Disposition::Rejected(_))
                && outcome.action_results.iter().all(|result| result.accepted);
            if taken {
                self.pending_arm = Some(PendingArm {
                    action_id,
                    sent_at: now,
                });
            } else {
                self.arms_refused = self.arms_refused.wrapping_add(1);
                self.engine.on_action_result(action_id, false);
            }
        }
        Some(outcome.disposition)
    }

    fn frame_of(&mut self, output: &MissionOutput, now: MonotonicNanos) -> ScopedControlFrame {
        self.sequence = self.sequence.wrapping_add(1);
        let scope = match output.intent {
            Some(ControlIntent::AttitudeThrust(_)) => DIRECT_SCOPE,
            _ => MOTION_SCOPE,
        };
        let action: Option<(ControlAction, u32)> = output
            .action
            .map(|action| (action.action, action.action_id as u32));
        ScopedControlFrame {
            timing: None,
            admitted_timing: None,
            session: SessionId::new(1),
            vehicle: self.vehicle,
            scope: ScopeId::new(scope),
            generation: Generation::new(1),
            sequence: SequenceNum::new(self.sequence),
            sampled_at: MonoTimestamp::from_nanos(now.as_nanos()),
            profile_revision: 0,
            activation_revision: 0,
            payload: ControlPayload::default(),
            intent: output.intent,
            actions: action.iter().map(|(action, _)| *action).collect(),
            action_ids: action.iter().map(|(_, id)| *id).collect(),
        }
    }
}

/// The estimate as an ownship sample, when the flight controller authorizes
/// its position and velocity. The flight controller's local frame is the
/// mission's: both are anchored at the pad.
fn ownship_of(avionics: &AvionicsSample) -> Option<OwnshipSample> {
    let kinematics = avionics.kinematics?;
    let authorized = avionics.quality <= QUALITY_DEGRADED
        && avionics.valid_flags & NAVIGATION_VALID == NAVIGATION_VALID;
    if !authorized {
        return None;
    }
    let yaw_rad = match avionics.attitude {
        Some(AvionicsAttitudeSample::Canonical { quat_wxyz, .. })
            if avionics.valid_flags & ATTITUDE_VALID != 0 =>
        {
            Some(yaw_of(quat_wxyz))
        }
        _ => None,
    };
    Some(OwnshipSample {
        ned: kinematics.pos_ned_m.map(f64::from),
        ned_velocity: kinematics.vel_ned_mps.map(f64::from),
        yaw_rad,
        role: TruthRole::OperationalEstimate,
        acquired_at: MonotonicNanos::from_nanos(kinematics.stamp.acquired_at_ns),
        sequence: kinematics.stamp.sequence,
    })
}
