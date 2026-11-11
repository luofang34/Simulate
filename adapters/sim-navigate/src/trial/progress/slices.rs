//! Building progress records and handing them to the sink one time slice
//! at a time, ordered by route time.

use navigate_contract::NavigationSolution;
use sim_core::VehicleState;
use sim_geo::ecef_to_geodetic;

use super::{
    EventKind, EventRecord, PROGRESS_SCHEMA, ProgressPosition, ProgressRecord, ProgressRoute,
    ProgressSet, ProgressSink, ProgressWaypoint, SampleRecord, SensorStates, StartedRecord,
};
use crate::trial::availability::{SensorKind, SetAvailability};
use crate::trial::replay::SetRun;
use crate::trial::report::SeriesSample;
use crate::trial::vertical::LegVertical;
use crate::trial::{TrialError, TrialRequest};

/// Records waiting for their slice, and the events known in advance.
pub(in crate::trial) struct Slices<'s> {
    sink: &'s mut dyn ProgressSink,
    pending: Vec<ProgressRecord>,
    scheduled: Vec<EventRecord>,
    next_scheduled: usize,
    period_steps: usize,
}

impl<'s> Slices<'s> {
    /// Slices a progress sample every `period_steps`, with the request's
    /// availability events and `known` events placed by their times.
    pub(in crate::trial) fn new(
        sink: &'s mut dyn ProgressSink,
        request: &TrialRequest,
        period_steps: usize,
        known: Vec<EventRecord>,
    ) -> Result<Self, TrialError> {
        let mut scheduled = known;
        for set in &request.sensor_sets {
            let availability = SetAvailability::resolve(request, set)?;
            for scheduled_interval in availability.intervals() {
                let interval = &scheduled_interval.interval;
                let event = |t_s: f64, start: bool| {
                    let (sensor, schedule, reason) = (
                        scheduled_interval.sensor,
                        scheduled_interval.schedule.clone(),
                        interval.reason.clone(),
                    );
                    EventRecord {
                        t_s,
                        set: Some(set.name.clone()),
                        kind: if start {
                            EventKind::AvailabilityStart {
                                sensor,
                                schedule,
                                reason,
                            }
                        } else {
                            EventKind::AvailabilityEnd {
                                sensor,
                                schedule,
                                reason,
                            }
                        },
                    }
                };
                scheduled.push(event(interval.start_s, true));
                scheduled.push(event(interval.end_s, false));
            }
        }
        scheduled.sort_by(|a, b| a.t_s.total_cmp(&b.t_s));
        Ok(Self {
            sink,
            pending: Vec::new(),
            scheduled,
            next_scheduled: 0,
            period_steps: period_steps.max(1),
        })
    }

    /// Whether a scored sample at `step` is also a progress sample.
    pub(in crate::trial) const fn due(&self, step: usize, last: bool) -> bool {
        last || step.is_multiple_of(self.period_steps)
    }

    /// Holds a record for the slice being built.
    pub(in crate::trial) fn push(&mut self, record: ProgressRecord) {
        self.pending.push(record);
    }

    /// Hands the sink every held record and every known event up to
    /// `through_s`, by route time; records of one instant keep their order.
    pub(in crate::trial) fn flush(&mut self, through_s: f64) -> Result<(), TrialError> {
        while let Some(event) = self
            .scheduled
            .get(self.next_scheduled)
            .filter(|event| event.t_s <= through_s)
        {
            self.pending.push(ProgressRecord::Event(event.clone()));
            self.next_scheduled = self.next_scheduled.wrapping_add(1);
        }
        self.pending
            .sort_by(|a, b| time_of(a).total_cmp(&time_of(b)));
        for record in self.pending.drain(..) {
            self.sink.record(&record).map_err(TrialError::Progress)?;
        }
        Ok(())
    }
}

fn time_of(record: &ProgressRecord) -> f64 {
    match record {
        ProgressRecord::Sample(sample) => sample.t_s,
        ProgressRecord::Event(event) => event.t_s,
        ProgressRecord::Started(_) | ProgressRecord::Finished(_) => 0.0,
    }
}

/// The `started` record of `request`, whose legs are `legs`.
pub(in crate::trial) fn started(
    request: &TrialRequest,
    request_sha256: String,
    legs: &[LegVertical],
) -> ProgressRecord {
    ProgressRecord::Started(StartedRecord {
        schema: PROGRESS_SCHEMA,
        request_sha256,
        epoch_utc: request.epoch_utc.clone(),
        route: ProgressRoute {
            waypoints: request
                .waypoints
                .iter()
                .map(|w| ProgressWaypoint {
                    ident: w.ident.clone(),
                    lat_deg: w.lat_deg,
                    lon_deg: w.lon_deg,
                    altitude_m: w.altitude_m,
                })
                .collect(),
            planned_duration_s: legs.iter().map(|l| l.length_m / l.speed_m_s).sum(),
        },
        sets: request
            .sensor_sets
            .iter()
            .map(|set| ProgressSet {
                name: set.name.clone(),
                means: set.ablation_set().means().collect(),
            })
            .collect(),
        guidance: request.vehicle.guidance,
    })
}

/// One scored instant of one set.
pub(in crate::trial) struct Scored<'a> {
    /// The truth scored against.
    pub(in crate::trial) truth: &'a VehicleState,
    /// The set's estimate.
    pub(in crate::trial) estimate: &'a NavigationSolution,
    /// The score.
    pub(in crate::trial) sample: &'a SeriesSample,
}

impl Scored<'_> {
    /// The progress sample of `set`, run by `run`.
    pub(in crate::trial) fn record(
        &self,
        set: &str,
        run: &SetRun,
        cross_track_true_m: Option<f64>,
    ) -> ProgressRecord {
        let t_s = self.sample.time_s;
        let truth = ecef_to_geodetic(self.truth.position_ecef);
        let estimate = self.estimate.position;
        ProgressRecord::Sample(SampleRecord {
            t_s,
            set: set.to_owned(),
            truth: ProgressPosition {
                lat_deg: truth.lat_rad.to_degrees(),
                lon_deg: truth.lon_rad.to_degrees(),
                altitude_m: truth.height_m,
            },
            estimate: ProgressPosition {
                lat_deg: estimate.latitude_rad.to_degrees(),
                lon_deg: estimate.longitude_rad.to_degrees(),
                altitude_m: estimate.altitude_m,
            },
            horizontal_error_m: self.sample.horizontal_error_m,
            vertical_error_m: self.sample.vertical_error_m,
            sigma_horizontal_m: self.sample.horizontal_sigma_m,
            cross_track_true_m,
            sensors: SensorStates {
                gnss: run.sensor_state(SensorKind::Gnss, t_s),
                baro: run.sensor_state(SensorKind::Baro, t_s),
                stars: run.sensor_state(SensorKind::Stars, t_s),
            },
        })
    }
}
