//! Replaying a sensor log through the linked navigator under one sensor
//! set: each reading becomes a sensor input, and what the navigator did
//! with it comes back as acceptance records. Nothing here takes truth: the
//! log and the navigator's own solution are all a replay sees.

mod tally;
mod terrain;

use std::collections::BTreeMap;

use navigate_contract::{MonotonicNanos, NavigationSolution, SourceId};
use navigate_fusion::RejectionReason;
use navigate_onboard::{AcceptanceRecord, Alignment, Navigator, Outcome, SensorData, SensorInput};
use sim_core::SimInstant;

use super::record::SensorLog;
use tally::Ledger;
pub use tally::{ObservationTally, SensorTallies, StarAvailability};
use tally::{Reading, sensor_state};

use super::TrialError;
use super::availability::{ScheduledInterval, SensorKind, SensorState, SetAvailability};
use super::onboard::{
    BARO_SOURCE, GNSS_SOURCE, IMU_SOURCE, STAR_SOURCE, baro_altitude_data, gnss_fix_data,
};
use super::route::{STEP_HZ, step_seconds};
use super::stars::{exposure_steps, star_camera_config};
use crate::{AblationSet, Estimate, MasterClockBridge, NavMeans};

/// The sensors a replay may use: its means, and the schedules that
/// withhold them.
#[derive(Debug, Clone, PartialEq)]
pub struct SetSelection {
    /// The means the filter may take observations from.
    pub means: AblationSet,
    /// When each sensor is withheld.
    pub availability: SetAvailability,
}

/// One sensor set's navigator stepping through a log. Cloning it forks the
/// run, the navigator through its own snapshot.
pub struct SetRun {
    selection: SetSelection,
    navigator: Box<dyn Navigator>,
    clock: MasterClockBridge,
    step: usize,
    next_gnss: usize,
    next_baro: usize,
    next_opening: usize,
    next_reading: usize,
    star_half_exposure_steps: usize,
    next_terrain: usize,
    terrain_fixes: BTreeMap<String, u64>,
    /// Wall-clock seconds the navigator spent on terrain frames, and the
    /// frames; for the log only, never a result.
    terrain_wall: (f64, u64, f64),
    records: Vec<AcceptanceRecord>,
    ledger: Ledger,
    stars: StarAvailability,
}

impl Clone for SetRun {
    fn clone(&self) -> Self {
        Self {
            selection: self.selection.clone(),
            navigator: self.navigator.snapshot(),
            clock: self.clock,
            step: self.step,
            next_gnss: self.next_gnss,
            next_baro: self.next_baro,
            next_opening: self.next_opening,
            next_reading: self.next_reading,
            star_half_exposure_steps: self.star_half_exposure_steps,
            next_terrain: self.next_terrain,
            terrain_fixes: self.terrain_fixes.clone(),
            terrain_wall: self.terrain_wall,
            records: Vec::new(),
            ledger: self.ledger.clone(),
            stars: self.stars,
        }
    }
}

impl std::fmt::Debug for SetRun {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("SetRun")
            .field("selection", &self.selection)
            .field("step", &self.step)
            .field("tallies", self.ledger.total())
            .finish_non_exhaustive()
    }
}

impl SetRun {
    /// `navigator` aligned from the log's alignment fix at step 0, taking
    /// the inputs `selection` allows.
    pub fn new(
        log: &SensorLog,
        selection: SetSelection,
        mut navigator: Box<dyn Navigator>,
    ) -> Result<Self, TrialError> {
        let clock = MasterClockBridge::default();
        let start = log.instants.first().copied().unwrap_or(SimInstant::START);
        let fix = &log.alignment;
        navigator.align(&Alignment {
            position: fix.position,
            velocity: fix.velocity,
            attitude: fix.attitude,
            position_cov: fix.position_cov,
            velocity_cov: fix.velocity_cov,
            attitude_cov: fix.attitude_cov,
            at: clock.to_monotonic(start)?,
            imu_bias_prior: fix.imu_bias_prior,
        })?;
        let star_half_exposure_steps = if selection.means.contains(NavMeans::Celestial) {
            exposure_steps(&star_camera_config(), STEP_HZ as f64)? / 2
        } else {
            0
        };
        let ledger = Ledger::new(&selection.availability);
        Ok(Self {
            selection,
            navigator,
            clock,
            step: 0,
            next_gnss: 0,
            next_baro: 0,
            next_opening: 0,
            next_reading: 0,
            star_half_exposure_steps,
            next_terrain: 0,
            terrain_fixes: BTreeMap::new(),
            terrain_wall: (0.0, 0, 0.0),
            records: Vec::new(),
            ledger,
            stars: StarAvailability::default(),
        })
    }

    /// This run continued from its present state under `selection`: the
    /// navigator is its snapshot, and only the inputs change.
    pub fn fork(&self, selection: SetSelection) -> Self {
        let mut fork = self.clone();
        fork.selection = selection;
        fork
    }

    /// The last step processed.
    pub const fn step(&self) -> usize {
        self.step
    }

    /// Per-sensor tallies so far.
    pub const fn tallies(&self) -> &SensorTallies {
        self.ledger.total()
    }

    /// Per-sensor tallies inside each scheduled interval so far, in the
    /// order of the set's schedules.
    pub fn interval_tallies(&self) -> &[(ScheduledInterval, SensorTallies)] {
        self.ledger.intervals()
    }

    /// What `sensor` is doing for this set at route time `t_s`, as of the
    /// last step processed.
    pub fn sensor_state(&self, sensor: SensorKind, t_s: f64) -> SensorState {
        let has = self.selection.means.contains(sensor.means());
        sensor_state(&self.ledger, &self.selection.availability, has, sensor, t_s)
    }

    /// Star tracker availability so far.
    pub const fn star_availability(&self) -> StarAvailability {
        self.stars
    }

    /// The solution at the last step processed of `log`.
    pub fn solution(
        &mut self,
        log: &SensorLog,
    ) -> Result<Option<Estimate<NavigationSolution>>, TrialError> {
        let now = self.now(log, self.step)?;
        Ok(self.navigator.solution(now).map(Estimate::new))
    }

    /// Processes every step up to and including `target`: the IMU increment
    /// first, then the readings delivered at that step.
    pub fn advance_to(&mut self, log: &SensorLog, target: usize) -> Result<(), TrialError> {
        while self.step < target {
            let step = self.step.wrapping_add(1);
            let now = self.now(log, step)?;
            let Some(increment) = log.imu.get(step.wrapping_sub(1)) else {
                return Err(TrialError::InvalidRequest {
                    field: "replay",
                    detail: format!("no IMU increment for step {step}"),
                });
            };
            let data = SensorData::ImuIncrement {
                dt_s: increment.dt_s,
                delta_theta_rad: increment.delta_theta_rad.to_array(),
                delta_velocity_m_s: increment.delta_velocity_m_s.to_array(),
            };
            self.ingest(IMU_SOURCE, (now, now), data)?;
            self.step = step;
            self.stars_at(log, step, now)?;
            self.terrain_at(log, step, now)?;
            self.gnss_at(log, step, now)?;
            self.baro_at(log, step, now)?;
        }
        Ok(())
    }

    fn now(&self, log: &SensorLog, step: usize) -> Result<MonotonicNanos, TrialError> {
        Ok(self.clock.to_monotonic(instant(log, step)?)?)
    }

    /// Hands one input to the navigator and keeps its acceptance records.
    fn ingest(
        &mut self,
        sensor: SourceId,
        (acquired_at, delivered_at): (MonotonicNanos, MonotonicNanos),
        data: SensorData,
    ) -> Result<(), TrialError> {
        self.records.clear();
        let input = SensorInput {
            sensor,
            acquired_at,
            delivered_at,
            data,
        };
        self.navigator.ingest(&input, &mut self.records)?;
        Ok(())
    }

    /// The star frame opening or read out at `step`. A frame any instant
    /// of whose mid-exposure to readout a schedule covers is neither
    /// announced at mid-exposure nor read: the camera gave nothing.
    fn stars_at(
        &mut self,
        log: &SensorLog,
        step: usize,
        now: MonotonicNanos,
    ) -> Result<(), TrialError> {
        if !self.selection.means.contains(NavMeans::Celestial) {
            return Ok(());
        }
        if log.star_mid_steps.get(self.next_opening) == Some(&step) {
            let end = step.wrapping_add(self.star_half_exposure_steps);
            let availability = &self.selection.availability;
            if !availability.withheld(SensorKind::Stars, step_seconds(step), step_seconds(end)) {
                self.ingest(STAR_SOURCE, (now, now), SensorData::StarExposureMid)?;
            }
            self.next_opening = self.next_opening.wrapping_add(1);
        }
        let Some(frame) = log.stars.get(self.next_reading) else {
            return Ok(());
        };
        if frame.image.end_step != step {
            return Ok(());
        }
        self.next_reading = self.next_reading.wrapping_add(1);
        let window = (step_seconds(frame.image.mid_step), step_seconds(step));
        if self
            .selection
            .availability
            .withheld(SensorKind::Stars, window.0, window.1)
        {
            self.ledger
                .record(SensorKind::Stars, window, &Reading::Withheld);
            return Ok(());
        }
        let mid = self.clock.to_monotonic(frame.image.mid)?;
        let centroids = frame.image.centroids.clone();
        self.ingest(
            STAR_SOURCE,
            (mid, now),
            SensorData::StarCentroids { centroids },
        )?;
        let verdicts = verdicts(&self.records);
        let lost_in_space = self
            .records
            .iter()
            .any(|record| record.context == Some("lost_in_space"));
        let stars = &mut self.stars;
        stars.frames_attempted = stars.frames_attempted.wrapping_add(1);
        if lost_in_space {
            stars.lost_in_space_attempted = stars.lost_in_space_attempted.wrapping_add(1);
        }
        if !verdicts.is_empty() {
            if lost_in_space {
                stars.lost_in_space_solved = stars.lost_in_space_solved.wrapping_add(1);
            }
            stars.frames_solved = stars.frames_solved.wrapping_add(1);
            stars.stars_in_solved_frames = stars
                .stars_in_solved_frames
                .wrapping_add(verdicts.len() as u64);
            stars.stars_per_solved_frame =
                stars.stars_in_solved_frames as f64 / stars.frames_solved as f64;
        }
        let reading = Reading::Offered(&verdicts);
        self.ledger.record(SensorKind::Stars, window, &reading);
        Ok(())
    }

    fn gnss_at(
        &mut self,
        log: &SensorLog,
        step: usize,
        now: MonotonicNanos,
    ) -> Result<(), TrialError> {
        let time_s = step_seconds(step);
        while let Some(logged) = log.gnss.get(self.next_gnss).filter(|f| f.step <= step) {
            self.next_gnss = self.next_gnss.wrapping_add(1);
            if !self.selection.means.contains(NavMeans::Gnss) {
                continue;
            }
            let window = (time_s, time_s);
            if self
                .selection
                .availability
                .withheld(SensorKind::Gnss, time_s, time_s)
            {
                self.ledger
                    .record(SensorKind::Gnss, window, &Reading::Withheld);
                continue;
            }
            self.ingest(GNSS_SOURCE, (now, now), gnss_fix_data(&logged.reading))?;
            let verdicts = verdicts(&self.records);
            self.ledger
                .record(SensorKind::Gnss, window, &Reading::Offered(&verdicts));
        }
        Ok(())
    }

    fn baro_at(
        &mut self,
        log: &SensorLog,
        step: usize,
        now: MonotonicNanos,
    ) -> Result<(), TrialError> {
        let time_s = step_seconds(step);
        while let Some(logged) = log.baro.get(self.next_baro).filter(|b| b.step <= step) {
            self.next_baro = self.next_baro.wrapping_add(1);
            if !self.selection.means.contains(NavMeans::Barometric) {
                continue;
            }
            let window = (time_s, time_s);
            if self
                .selection
                .availability
                .withheld(SensorKind::Baro, time_s, time_s)
            {
                self.ledger
                    .record(SensorKind::Baro, window, &Reading::Withheld);
                continue;
            }
            // The undulation only needs the host's position to kilometres,
            // so a navigator without a solution falls back on where it was
            // aligned rather than losing the sample.
            let position = self
                .navigator
                .solution(now)
                .map_or(log.alignment.position, |s| s.position);
            let data = baro_altitude_data(
                &logged.reading,
                position.latitude_rad,
                position.longitude_rad,
            )?;
            self.ingest(BARO_SOURCE, (now, now), data)?;
            let verdicts = verdicts(&self.records);
            self.ledger
                .record(SensorKind::Baro, window, &Reading::Offered(&verdicts));
        }
        Ok(())
    }
}

/// The filter's verdicts on the measurements of one input; empty when the
/// navigator offered none to the filter.
fn verdicts(records: &[AcceptanceRecord]) -> Vec<Result<(), &RejectionReason>> {
    records
        .iter()
        .filter_map(|record| match &record.outcome {
            Outcome::Accepted => Some(Ok(())),
            Outcome::Rejected(reason) => Some(Err(reason)),
            Outcome::Unused(_) => None,
        })
        .collect()
}

/// The instant of `step` in `log`.
fn instant(log: &SensorLog, step: usize) -> Result<SimInstant, TrialError> {
    log.instants
        .get(step)
        .copied()
        .ok_or_else(|| TrialError::InvalidRequest {
            field: "replay",
            detail: format!("step {step} is past the log"),
        })
}
