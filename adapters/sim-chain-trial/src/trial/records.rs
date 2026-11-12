//! The progress records and mission updates a chain trial reports.

use navigate_contract::{NavigationSolution, SensorClass};
use sim_core::VehicleState;
use sim_flight_chain::{
    HostEvent, MISSION_CLOCK, MissionEvent, SourceIntegrity, SourceIntegrityState,
};
use sim_geo::{Geodetic, geodetic_to_ecef};
use sim_navigate::trial::{
    EventKind, EventRecord, GnssCheck, GnssCheckState, GuidanceSource, PROGRESS_SCHEMA,
    ProgressRecord, ProgressRoute, ProgressSet, ProgressWaypoint, SensorKind, SensorLog,
    SensorState, SensorStates, StartedRecord, TrialRequest,
};

use super::sets::{FLIGHT_CONTROLLER_SET, SetSlot, flight_controller_sample, solution_sample};
use crate::{ChainTrialError, ChainTrialSpec, DeviceOutage, MissionUpdate};

/// The name device outages are reported under, as a schedule.
const OUTAGE_SCHEDULE: &str = "device";
/// The reason they give.
const OUTAGE_REASON: &str = "device out of service";

/// What the records remember between steps.
pub(super) struct Recorder {
    started: Option<StartedRecord>,
    last_phase: Option<String>,
    /// The plan's waypoint identifiers, in fly order.
    idents: Vec<String>,
}

impl Recorder {
    pub(super) fn new(request: &TrialRequest, digest: &str, spec: &ChainTrialSpec) -> Self {
        let waypoints: Vec<ProgressWaypoint> = request
            .waypoints
            .iter()
            .map(|w| ProgressWaypoint {
                ident: w.ident.clone(),
                lat_deg: w.lat_deg,
                lon_deg: w.lon_deg,
                altitude_m: w.altitude_m,
            })
            .collect();
        let s = spec.start;
        let mut along_m = 0.0;
        let mut from = Geodetic::from_degrees(s.lat_deg, s.lon_deg, s.height_m);
        for w in &request.waypoints {
            let to = Geodetic::from_degrees(w.lat_deg, w.lon_deg, w.altitude_m);
            along_m += (geodetic_to_ecef(&to) - geodetic_to_ecef(&from)).length();
            from = to;
        }
        let mut sets: Vec<ProgressSet> = request
            .sensor_sets
            .iter()
            .map(|set| ProgressSet {
                name: set.name.clone(),
                means: set.means.clone(),
            })
            .collect();
        sets.push(ProgressSet {
            name: FLIGHT_CONTROLLER_SET.to_owned(),
            means: Vec::new(),
        });
        Self {
            started: Some(StartedRecord {
                schema: PROGRESS_SCHEMA,
                request_sha256: digest.to_owned(),
                epoch_utc: request.epoch_utc.clone(),
                route: ProgressRoute {
                    waypoints,
                    planned_duration_s: along_m / s.indicated_m_s.max(1.0),
                },
                sets,
                guidance: GuidanceSource::Estimate,
            }),
            last_phase: None,
            idents: request.waypoints.iter().map(|w| w.ident.clone()).collect(),
        }
    }

    /// The run's first record, once.
    pub(super) fn start(&mut self) -> Option<ProgressRecord> {
        self.started.take().map(ProgressRecord::Started)
    }

    /// The mission's update at `t_s`, when its phase changed or it reported
    /// anything.
    pub(super) fn mission(
        &mut self,
        t_s: f64,
        phase: Option<String>,
        events: &[(sim_core::SimInstant, HostEvent)],
    ) -> Option<MissionUpdate> {
        if events.is_empty() && phase == self.last_phase {
            return None;
        }
        self.last_phase.clone_from(&phase);
        Some(MissionUpdate {
            t_s,
            phase,
            events: events
                .iter()
                .map(|(_, event)| format!("{event:?}"))
                .collect(),
        })
    }
}

impl Recorder {
    /// The waypoints Pilotage's mission sequenced among `events`, as the
    /// trial reports them: a leg advancing to fix `n` sequences fix `n − 1`,
    /// and the last fix sequenced completes the plan.
    pub(super) fn sequenced(
        &self,
        events: &[(sim_core::SimInstant, HostEvent)],
    ) -> Vec<ProgressRecord> {
        let last = self.idents.len().saturating_sub(1);
        events
            .iter()
            .filter_map(|(at, event)| {
                let (waypoint_index, plan_complete) = match event {
                    HostEvent::Mission(MissionEvent::LegAdvanced { to_index, .. }) => {
                        (to_index.checked_sub(1)?, false)
                    }
                    HostEvent::Mission(MissionEvent::LastWaypointSequenced { .. }) => (last, true),
                    _ => return None,
                };
                Some(ProgressRecord::Event(EventRecord {
                    t_s: at.duration_since(sim_core::SimInstant::START).as_seconds(),
                    set: None,
                    kind: EventKind::WaypointSequenced {
                        waypoint_index,
                        ident: self.idents.get(waypoint_index).cloned().unwrap_or_default(),
                        plan_complete,
                    },
                }))
            })
            .collect()
    }
}

/// `solution` stamped on the mission's clock, which counts the same
/// nanoseconds from the run's start as the navigators' clock does.
pub(super) fn on_mission_clock(solution: &NavigationSolution) -> NavigationSolution {
    let mut stamped = *solution;
    stamped.stamp.clock = MISSION_CLOCK;
    stamped
}

/// What a navigator's check of its receiver states to the mission: the
/// navigator's own verdict and its time, on the navigator's clock, which
/// counts as the mission's does.
pub(super) fn gnss_statement(check: GnssCheck) -> SourceIntegrity {
    SourceIntegrity {
        class: SensorClass::Gnss,
        state: match check.state {
            GnssCheckState::Consistent => SourceIntegrityState::Consistent,
            GnssCheckState::Disagreeing => SourceIntegrityState::Disagreeing,
            GnssCheckState::Excluded => SourceIntegrityState::Excluded,
        },
        since: check.since,
    }
}

/// The outages that began or ended in `(from_s, to_s]`, for every set
/// that has the device.
pub(super) fn availability(
    sets: &[SetSlot],
    outages: &[DeviceOutage],
    (from_s, to_s): (f64, f64),
) -> Vec<ProgressRecord> {
    let mut records = Vec::new();
    for outage in outages {
        let (was, is) = (outage.covers_up_to(from_s), outage.covers_up_to(to_s));
        let kind = match (was, is) {
            (false, true) => EventKind::AvailabilityStart {
                sensor: outage.sensor,
                schedule: OUTAGE_SCHEDULE.to_owned(),
                reason: Some(OUTAGE_REASON.to_owned()),
            },
            (true, false) => EventKind::AvailabilityEnd {
                sensor: outage.sensor,
                schedule: OUTAGE_SCHEDULE.to_owned(),
                reason: Some(OUTAGE_REASON.to_owned()),
            },
            _ => continue,
        };
        for slot in sets.iter().filter(|slot| slot.has(outage.sensor)) {
            records.push(ProgressRecord::Event(EventRecord {
                t_s: to_s,
                set: Some(slot.name.clone()),
                kind: kind.clone(),
            }));
        }
    }
    records
}

/// Every set's sample at `t_s` against `truth`, and the flight
/// controller's estimate against the truth of its own instant.
pub(super) fn samples(
    (t_s, truth): (f64, &VehicleState),
    sets: &mut [SetSlot],
    log: &SensorLog,
    controller: Option<&(VehicleState, VehicleState)>,
    outages: &[DeviceOutage],
) -> Result<Vec<ProgressRecord>, ChainTrialError> {
    let mut records = Vec::new();
    for slot in sets.iter_mut() {
        let sensors = slot.states(t_s, outages);
        if let Some(solution) = slot.run.solution(log)? {
            records.push(solution_sample(
                (t_s, &slot.name),
                truth,
                solution.get(),
                sensors,
            )?);
        }
    }
    if let Some((truth, estimate)) = controller {
        let device = |kind: SensorKind| {
            if outages
                .iter()
                .any(|o| o.sensor == kind && o.covers_up_to(t_s))
            {
                SensorState::ScheduledUnavailable
            } else {
                SensorState::Aiding
            }
        };
        let sensors = SensorStates {
            gnss: device(SensorKind::Gnss),
            baro: device(SensorKind::Baro),
            stars: SensorState::NotInSet,
            terrain_camera: SensorState::NotInSet,
        };
        records.push(flight_controller_sample(t_s, truth, estimate, sensors));
    }
    Ok(records)
}
