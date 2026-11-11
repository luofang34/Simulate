//! The host of Pilotage's mission engine: it converts the vehicle's
//! telemetry into ownship samples, ticks the engine, and frames what the
//! engine emits onto the vehicle adapter.
//!
//! The mission engine is Pilotage's own and so is the guidance it runs.
//! This host stands in for Pilotage's session host: there are no sessions,
//! leases or wire, and one holder owns the motion scope throughout. The
//! mission flies on the flight controller's estimate or on a navigator's
//! solution, as [`OwnshipFeed`] says, and never on the simulation's truth
//! sample.

use navigate_contract::{
    AttitudeQuaternion, ClockDomainId, DurationNanos, GeodeticPosition, MonotonicNanos,
    NavigationSolution,
};
use pilotage_adapter_api::{
    AdapterCapabilities, AvionicsAttitudeSample, AvionicsSample, Disposition, IntentCapability,
    TelemetrySample, VehicleAdapter,
};
use pilotage_mavlink::link::estimator::QUALITY_DEGRADED;
use pilotage_mavlink::sampling::yaw_of;
use pilotage_mission::{
    MissionAction, MissionConfig, MissionEngine, MissionEvent, MissionLimits, MissionOutput,
    MissionState, OwnshipAltitude, OwnshipSample, OwnshipSource, PlannedWaypoint, TruthRole,
    WaypointPlan, WaypointPlanRecord,
};
use pilotage_protocol::{
    ControlAction, ControlIntent, ControlPayload, DIRECT_SCOPE, Generation, IntentFamily,
    MOTION_SCOPE, ScopeId, ScopedControlFrame, SequenceNum, SessionId, VehicleId,
};
use pilotage_timing::MonoTimestamp;
use sim_geo::Geodetic;

use crate::ChainError;

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

/// Whose navigation the mission flies on.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub enum OwnshipFeed {
    /// The flight controller's own estimate, read from its telemetry.
    #[default]
    FlightController,
    /// A navigator outside the flight controller, whose solutions the
    /// caller hands to [`MissionHost::offer_navigation`]. The flight
    /// controller's estimate then steers nothing in the mission.
    Navigator,
}

/// One fix of a route.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RouteFix {
    /// Geodetic latitude, degrees.
    pub lat_deg: f64,
    /// Longitude, degrees east.
    pub lon_deg: f64,
    /// Height above the WGS84 ellipsoid the vehicle holds at the fix, metres.
    pub altitude_m: f64,
}

/// A route and how the mission flies it.
#[derive(Debug, Clone, PartialEq)]
pub struct MissionSetup {
    /// The vehicle the mission commands.
    pub vehicle: VehicleId,
    /// The mission anchor: the pad, which the flight controller measures its
    /// local frame from.
    pub anchor: Geodetic,
    /// The route's fixes in fly order. The mission climbs over the anchor to
    /// the first fix's height before it sets out.
    pub fixes: Vec<RouteFix>,
    /// Distance from a fix at which it is sequenced, metres: the room the
    /// vehicle needs to turn at its speed.
    pub capture_radius_m: f64,
    /// Along-track speed, m/s.
    pub cruise_mps: f64,
    /// Climb rate to the cruise height, m/s.
    pub climb_rate_mps: f64,
    /// Whose navigation the mission flies on.
    pub ownship: OwnshipFeed,
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
    /// On the heap: the engine is tens of kilobytes, and a host is moved by
    /// value through its owners.
    engine: Box<MissionEngine>,
    record: WaypointPlanRecord,
    vehicle: VehicleId,
    ownship: OwnshipFeed,
    sequence: u32,
    pending_arm: Option<PendingArm>,
    last_estimate: Option<(u32, Option<u32>)>,
    arms_refused: u32,
}

impl MissionHost {
    /// The mission engine over `setup`'s route, flying the vehicle's own
    /// estimate inside the velocity envelope `capabilities` advertises for
    /// it.
    pub fn new(
        setup: &MissionSetup,
        capabilities: &AdapterCapabilities,
    ) -> Result<Self, ChainError> {
        let mission = |error| ChainError::Mission(Box::new(error));
        let waypoints = setup
            .fixes
            .iter()
            .map(|fix| PlannedWaypoint::new(fix.lat_deg, fix.lon_deg, fix.altitude_m))
            .collect();
        let plan = WaypointPlan {
            capture_radius_m: Some(setup.capture_radius_m),
            ..WaypointPlan::new(waypoints)
        };
        let first_height_m = setup.fixes.first().map_or(0.0, |fix| fix.altitude_m);
        let anchor = GeodeticPosition::new(
            setup.anchor.lat_rad,
            setup.anchor.lon_rad,
            setup.anchor.height_m,
        );
        let envelope = velocity_envelope(capabilities, setup.vehicle)?;
        let within = |asked: f64, bound: f32| match f64::from(bound) {
            bound if bound > 0.0 => asked.min(bound),
            _ => asked,
        };
        let mut config = MissionConfig::new(String::new(), anchor, MISSION_CLOCK);
        config.cruise_height_m = first_height_m - setup.anchor.height_m;
        config.cruise_mps = within(setup.cruise_mps, envelope.max_linear);
        config.climb_rate_mps = within(setup.climb_rate_mps, envelope.effective_vertical());
        config.limits = MissionLimits {
            max_horizontal_mps: config.cruise_mps,
            max_vertical_mps: config.climb_rate_mps,
            max_yaw_rate_rps: within(
                MissionLimits::default().max_yaw_rate_rps,
                envelope.max_angular,
            ),
        };
        config.ownship_source = OwnshipSource::vehicle_estimate();
        // The class decides how the plan ends: a vehicle that cannot stop
        // orbits its last fix.
        if let Some(descriptor) = capabilities
            .vehicles
            .iter()
            .find(|descriptor| descriptor.id == setup.vehicle)
        {
            config.vehicle_class = descriptor.class;
        }
        let (engine, record) = MissionEngine::from_waypoints(&plan, config).map_err(mission)?;
        Ok(Self {
            engine: Box::new(engine),
            record,
            vehicle: setup.vehicle,
            ownship: setup.ownship,
            sequence: 0,
            pending_arm: None,
            last_estimate: None,
            arms_refused: 0,
        })
    }

    /// The record of the built mission: its waypoints and identity.
    pub const fn record(&self) -> &WaypointPlanRecord {
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
            if self.ownship == OwnshipFeed::FlightController {
                self.offer_estimate(sample, now);
            }
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

    /// Offers a navigator's solution as the mission's ownship. Ignored unless
    /// the mission flies on [`OwnshipFeed::Navigator`].
    ///
    /// Pilotage has no ownship source for a navigator's solution, so it
    /// enters through the vehicle-estimate source: the engine flies its
    /// position, velocity and heading, and re-declares its uncertainty and
    /// composition as that source's.
    pub fn offer_navigation(&mut self, solution: &NavigationSolution, now: MonotonicNanos) {
        if self.ownship != OwnshipFeed::Navigator {
            return;
        }
        let position = solution.position;
        let velocity = solution.velocity;
        let ownship = OwnshipSample {
            ned: self.engine.ned_of(
                position.latitude_rad,
                position.longitude_rad,
                OwnshipAltitude::Ellipsoid(position.altitude_m),
            ),
            ned_velocity: [velocity.north_mps, velocity.east_mps, velocity.down_mps],
            yaw_rad: solution.attitude.map(heading_of),
            role: TruthRole::OperationalEstimate,
            acquired_at: solution.stamp.solved_at,
            sequence: solution.stamp.sequence.get(),
        };
        self.engine.on_ownship(&ownship, now);
    }

    /// Offers the flight controller's estimate when it carries a new,
    /// authorized position and velocity.
    fn offer_estimate(&mut self, sample: &TelemetrySample, now: MonotonicNanos) {
        let Some(avionics) = sample.avionics.as_ref() else {
            return;
        };
        let Some(ownship) = ownship_of(avionics) else {
            return;
        };
        // Attitude arrives twice as often as position, and the engine turns
        // guidance into the body frame with the heading it last heard.
        let attitude = avionics.attitude.map(|attitude| attitude.stamp().sequence);
        let received = (ownship.sequence, attitude);
        if self.last_estimate == Some(received) {
            return;
        }
        self.last_estimate = Some(received);
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

/// Heading of a body-to-NED attitude, radians clockwise from north.
fn heading_of(attitude: AttitudeQuaternion) -> f64 {
    let AttitudeQuaternion { w, x, y, z, .. } = attitude;
    (2.0 * (w * z + x * y)).atan2(1.0 - 2.0 * (y * y + z * z))
}

/// The velocity envelope `capabilities` advertises on the motion scope of
/// `vehicle`.
fn velocity_envelope(
    capabilities: &AdapterCapabilities,
    vehicle: VehicleId,
) -> Result<&IntentCapability, ChainError> {
    capabilities
        .vehicles
        .iter()
        .filter(|descriptor| descriptor.id == vehicle)
        .flat_map(|descriptor| &descriptor.scopes)
        .filter(|scope| scope.scope.as_str() == MOTION_SCOPE)
        .flat_map(|scope| &scope.intents)
        .find(|intent| intent.family == IntentFamily::Velocity)
        .ok_or(ChainError::NoVelocityScope { vehicle })
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
