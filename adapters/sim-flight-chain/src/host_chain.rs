//! A flight controller's board under Pilotage's own vehicle host.
//!
//! Pilotage's embeddable host core holds the mission, the session engine
//! and the Aviate adapter. This chain gives it the simulation's time and
//! moves the MAVLink bytes between it and the board, the way a radio or a
//! serial line moves them on the vehicle. Nothing between the two is
//! Simulate's: the core encodes every command and reads every report.

#[cfg(test)]
mod tests;

use aviate_core::control::{ControlMode, VehicleController};
use aviate_core::mixer::Mixer;
use navigate_contract::{GeodeticPosition, MonotonicNanos};
use pilotage_adapter_api::SourceIncarnation;
use pilotage_adapter_aviate::{AviateAirframe, EmbeddedIncarnations};
use pilotage_host_core::{HostCore, HostCoreConfig, HostEvent, HostStatus, MISSION_CLOCK};
use pilotage_mavlink::link::LinkConfig;
use pilotage_mission::{
    LegRole, MissionConfig, MissionLimits, OnboardSolution, OwnshipSource, PlannedAltitude,
    PlannedPath, PlannedWaypoint, TerminalHoldingPattern, WaypointPlan, WaypointPlanRecord,
    WaypointTurn, WaypointTurnDirection,
};
use pilotage_protocol::VehicleClass;
use sim_core::{SimInstant, VehicleState};
use sim_geo::UtcTime;

use crate::{Airframe, ChainError, HostedBoard};

/// Bytes the flight controller's transmit ring holds: one read empties it,
/// so no frame is split across two datagrams.
const TRANSMIT_RING_BYTES: usize = 8_192;
const NANOS_PER_SECOND: f64 = 1.0e9;

/// Whose navigation the mission flies on.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub enum OwnshipFeed {
    /// The flight controller's own estimate, read from its telemetry.
    #[default]
    FlightController,
    /// An onboard navigator outside the flight controller, whose solutions
    /// the embedding host offers; the flight controller's estimate then
    /// steers nothing in the mission.
    Navigator,
}

/// One fix of a route.
#[derive(Debug, Clone, PartialEq)]
pub struct RouteFix {
    /// The fix's name, which the mission and its displays report; `None`
    /// takes the host's generated `WP<n>`.
    pub ident: Option<String>,
    /// Geodetic latitude, degrees.
    pub lat_deg: f64,
    /// Longitude, degrees east.
    pub lon_deg: f64,
    /// Height above the WGS84 ellipsoid the vehicle holds at the fix when
    /// `altitude` is absent, metres.
    pub altitude_m: f64,
    /// Optional explicit altitude constraint (`At`, `AtOrAbove`, `AtOrBelow`,
    /// `Window`, or `Unconstrained`) above the WGS84 ellipsoid.
    pub altitude: Option<PlannedAltitude>,
    /// Cross the fix before turning (`true`) rather than anticipating (`false`).
    pub fly_over: bool,
    /// Optional mandatory turn direction (`Left` or `Right`) onto this leg.
    pub turn_direction: Option<WaypointTurnDirection>,
    /// Optional ARINC 424 path terminator of the leg ending here.
    pub path: Option<PlannedPath>,
    /// Optional operational flight-phase role of this leg.
    pub role: Option<LegRole>,
    /// Optional procedure-published RNP on this leg, nautical miles (`> 0`).
    pub rnp_nm: Option<f64>,
}

impl RouteFix {
    /// A fly-by fix crossed at `altitude_m` on a default track leg.
    #[must_use]
    pub const fn new(ident: Option<String>, lat_deg: f64, lon_deg: f64, altitude_m: f64) -> Self {
        Self {
            ident,
            lat_deg,
            lon_deg,
            altitude_m,
            altitude: None,
            fly_over: false,
            turn_direction: None,
            path: None,
            role: None,
            rnp_nm: None,
        }
    }
}

/// A route and how the host's mission flies it.
#[derive(Debug, Clone, PartialEq)]
pub struct ChainRoute {
    /// The route's fixes in fly order, heights above the ellipsoid.
    pub fixes: Vec<RouteFix>,
    /// Along-track speed asked of guidance, m/s.
    pub cruise_mps: f64,
    /// The bank the mission plans its turns at, degrees: it sizes the turn
    /// lead and the orbit.
    pub route_bank_deg: f64,
    /// Climb rate toward a fix above the vehicle, m/s.
    pub climb_rate_mps: f64,
    /// Greatest horizontal speed the vehicle may be asked for, m/s.
    pub max_horizontal_mps: f64,
    /// Whose navigation the mission flies on.
    pub ownship: OwnshipFeed,
    /// The vehicle's class, which decides how the plan ends.
    pub class: VehicleClass,
    /// What the flight controller flies, as Pilotage's adapter bounds its
    /// demands.
    pub airframe: AviateAirframe,
    /// Route time at which the plan is loaded into the host, seconds: when
    /// the crew hands the mission over after powering the flight
    /// controller. The host flies nothing before it.
    pub plan_loaded_at_s: f64,
    /// The civil time the run starts at, which fixes the magnetic field.
    pub utc: UtcTime,
    /// The holding pattern the mission flies at the last fix when its plan
    /// ends; each value left unset takes Pilotage's standard.
    pub terminal_hold: TerminalHoldingPattern,
}

/// What crossed the link between the host and the flight controller.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct LinkCounters {
    /// Frames the host sent toward the flight controller.
    pub frames_to_fc: u64,
    /// Command bytes the flight controller's receive ring had no room for.
    pub refused_bytes_to_fc: u64,
    /// Datagrams read from the flight controller's transmit ring.
    pub datagrams_from_fc: u64,
    /// Bytes read from it.
    pub bytes_from_fc: u64,
}

/// Pilotage's host core and the board it commands.
pub struct HostChain<A, C, M>
where
    C: VehicleController,
    M: Mixer,
{
    board: HostedBoard<A, C, M>,
    /// On the heap: the core carries the mission engine by value.
    core: Box<HostCore>,
    record: Option<WaypointPlanRecord>,
    /// The plan, until it is loaded at `load_at`.
    plan: Option<WaypointPlan>,
    load_at: MonotonicNanos,
    next_wake: MonotonicNanos,
    transmit: Vec<u8>,
    transmit_len: usize,
    /// The frames the host sent toward the controller in the last cycle.
    sent: Vec<Vec<u8>>,
    events: Vec<(SimInstant, HostEvent)>,
    counters: LinkCounters,
}

impl<A, C, M> HostChain<A, C, M>
where
    A: Airframe,
    C: VehicleController,
    M: Mixer,
{
    /// `board` under a host core flying `route`. The mission anchor is the
    /// board's origin, where its flight controller measures from and
    /// where the embedding host placed the vehicle.
    pub fn new(board: HostedBoard<A, C, M>, route: &ChainRoute) -> Result<Self, ChainError> {
        let host = |source| ChainError::Host(Box::new(source));
        let config = host_config(&board, route)?;
        // The core counts time from this instant and reads no clock; any
        // instant serves, and the run depends on none.
        let core = HostCore::new(config, web_time::Instant::now()).map_err(host)?;
        Ok(Self {
            board,
            core: Box::new(core),
            record: None,
            plan: Some(waypoint_plan(route)),
            load_at: nanos_of(route.plan_loaded_at_s),
            next_wake: MonotonicNanos::from_nanos(0),
            transmit: vec![0; TRANSMIT_RING_BYTES],
            transmit_len: 0,
            sent: Vec::new(),
            events: Vec::new(),
            counters: LinkCounters::default(),
        })
    }

    /// One cycle of the flight controller. The host runs what fell due up
    /// to now first, its commands reach the controller's receive ring, the
    /// controller and airframe step, and what the controller transmitted
    /// in the cycle reaches the host at the cycle's end.
    pub fn advance_tick(&mut self) -> Result<(), ChainError> {
        let now = self.host_now();
        if now >= self.load_at
            && let Some(plan) = self.plan.take()
        {
            let record = self
                .core
                .load_mission(plan)
                .map_err(|source| ChainError::Host(Box::new(source)))?;
            self.record = Some(record);
            self.next_wake = now;
        }
        if now >= self.next_wake {
            self.next_wake = self
                .core
                .tick(now)
                .map_err(|source| ChainError::Host(Box::new(source)))?
                .at;
            let at = self.board.now();
            self.events
                .extend(self.core.drain_events().into_iter().map(|e| (at, e)));
        }
        self.sent = self.core.drain_uplink();
        for frame in &self.sent {
            let accepted = self.board.uplink(frame);
            let refused = frame.len().saturating_sub(accepted) as u64;
            self.counters.frames_to_fc = self.counters.frames_to_fc.wrapping_add(1);
            self.counters.refused_bytes_to_fc =
                self.counters.refused_bytes_to_fc.wrapping_add(refused);
        }
        self.board.step()?;
        self.transmit_len = self.board.downlink(&mut self.transmit);
        if let Some(datagram) = self.transmit.get(..self.transmit_len)
            && !datagram.is_empty()
        {
            let now = self.host_now();
            // The board has one serial link: its reports and its answers to
            // commands travel together, and each side of the host takes what
            // is addressed to it.
            self.core.on_downlink(datagram, now);
            self.core.on_uplink_reply(datagram, now);
            self.counters.datagrams_from_fc = self.counters.datagrams_from_fc.wrapping_add(1);
            self.counters.bytes_from_fc = self
                .counters
                .bytes_from_fc
                .wrapping_add(datagram.len() as u64);
        }
        Ok(())
    }

    /// Hands an onboard navigator's solution to the mission, which flies
    /// it when its ownship is the navigator, with what the navigator states
    /// of a position sensor behind it. The mission keeps the statement of
    /// the latest solution it admitted, so each solution carries its own.
    pub fn offer_navigation(&mut self, offered: impl Into<OnboardSolution>) {
        let now = self.host_now();
        self.core.on_onboard_solution(offered, now);
    }

    /// The events the host reported since the last call, each with the
    /// instant of the tick that reported it.
    pub fn drain_events(&mut self) -> Vec<(SimInstant, HostEvent)> {
        core::mem::take(&mut self.events)
    }

    /// Where the host stands after its last tick.
    pub fn status(&self) -> &HostStatus {
        self.core.status()
    }

    /// The record of the plan once it is loaded: its waypoints and identity.
    pub const fn record(&self) -> Option<&WaypointPlanRecord> {
        self.record.as_ref()
    }

    /// What crossed the link so far.
    pub const fn counters(&self) -> LinkCounters {
        self.counters
    }

    /// What the flight controller transmitted in the last cycle.
    pub fn last_downlink(&self) -> &[u8] {
        self.transmit.get(..self.transmit_len).unwrap_or_default()
    }

    /// The frames the host sent toward the flight controller in the last
    /// cycle, in send order.
    pub fn last_uplink(&self) -> &[Vec<u8>] {
        &self.sent
    }

    /// The board and its airframe.
    pub const fn board(&self) -> &HostedBoard<A, C, M> {
        &self.board
    }

    /// The board, to reach its devices.
    pub const fn board_mut(&mut self) -> &mut HostedBoard<A, C, M> {
        &mut self.board
    }

    /// The instant the vehicle state describes.
    pub const fn now(&self) -> SimInstant {
        self.board.now()
    }

    /// The true vehicle state, for scoring and drawing.
    pub fn true_state(&self) -> VehicleState {
        self.board.true_state()
    }

    /// The control mode the flight controller flew on its last cycle.
    pub fn flight_mode(&self) -> ControlMode {
        self.board.flight_mode()
    }

    /// The simulation's time on the host's clock, which starts with the run.
    fn host_now(&self) -> MonotonicNanos {
        let elapsed_s = self
            .board
            .now()
            .duration_since(SimInstant::START)
            .as_seconds();
        MonotonicNanos::from_nanos((elapsed_s * NANOS_PER_SECOND).round() as u64)
    }
}

/// The host's configuration for `board` flying `route`.
fn host_config<A, C, M>(
    board: &HostedBoard<A, C, M>,
    route: &ChainRoute,
) -> Result<HostCoreConfig, ChainError>
where
    A: Airframe,
    C: VehicleController,
    M: Mixer,
{
    let origin = board.origin();
    let anchor = GeodeticPosition::new(origin.lat_rad, origin.lon_rad, origin.height_m);
    let mut mission = MissionConfig::new(String::new(), anchor, MISSION_CLOCK);
    // A vehicle that starts at the first fix's height climbs nowhere.
    mission.cruise_height_m = route.fixes.first().map_or(0.0, |fix| {
        let target_m = fix
            .altitude
            .map_or(fix.altitude_m, |altitude| altitude.reference_m());
        (target_m - origin.height_m).max(0.0)
    });
    mission.cruise_mps = route.cruise_mps;
    mission.climb_rate_mps = route.climb_rate_mps;
    mission.route_bank_deg = route.route_bank_deg;
    mission.terminal_holding_pattern = route.terminal_hold;
    mission.limits = MissionLimits {
        max_horizontal_mps: route.max_horizontal_mps,
        max_vertical_mps: route.climb_rate_mps,
        ..MissionLimits::default()
    };
    mission.vehicle_class = route.class;
    mission.ownship_source = match route.ownship {
        OwnshipFeed::FlightController => OwnshipSource::vehicle_estimate(),
        // A navigator's solution that degrades, a visual one over water or a
        // GNSS one in an outage, is flown on by dead reckoning with the
        // degradation shown, never dropped for a straight line.
        OwnshipFeed::Navigator => OwnshipSource::onboard_navigation_gnss_denied(),
    };
    Ok(HostCoreConfig {
        link: LinkConfig::simulator(),
        airframe: route.airframe.clone(),
        fc_origin_height_m: Some(origin.height_m),
        incarnations: EmbeddedIncarnations {
            estimate: SourceIncarnation::new([1; 16]),
            fc_state: SourceIncarnation::new([2; 16]),
        },
        mission,
        magnetic_epoch_unix_s: route.utc.unix_seconds().round() as i64,
    })
}

fn nanos_of(seconds: f64) -> MonotonicNanos {
    MonotonicNanos::from_nanos((seconds.max(0.0) * NANOS_PER_SECOND).round() as u64)
}

fn waypoint_plan(route: &ChainRoute) -> WaypointPlan {
    WaypointPlan::new(
        route
            .fixes
            .iter()
            .map(|fix| PlannedWaypoint {
                latitude_deg: fix.lat_deg,
                longitude_deg: fix.lon_deg,
                altitude: fix.altitude.unwrap_or(PlannedAltitude::At(fix.altitude_m)),
                turn: if fix.fly_over {
                    WaypointTurn::FlyOver
                } else {
                    WaypointTurn::FlyBy
                },
                turn_direction: fix.turn_direction,
                name: fix.ident.clone(),
                path: fix.path,
                max_speed_mps: None,
                gradient: None,
                role: fix.role,
                rnp_nm: fix.rnp_nm,
            })
            .collect(),
    )
}
