//! Replaying a sensor log through Navigate's inertial filter under one
//! sensor set. Nothing here takes truth: the log and the navigator's own
//! solution are all a replay sees.

mod tally;

use navigate_contract::{
    NavigationSolution, SensorClass, SourceComposition, SourceEpoch, SourceId, WrappingSequence,
};
use navigate_fusion::{
    FogmBias, InertialConfig, IngestOutcome, Observation, ObservationValue, RejectionReason,
};
use sim_core::SimInstant;

use super::record::SensorLog;
use tally::Ledger;
pub use tally::{ObservationTally, SensorTallies, StarAvailability};
use tally::{Reading, sensor_state};

use super::availability::{ScheduledInterval, SensorKind, SensorState, SetAvailability};
use super::route::{STEP_HZ, step_seconds};
use super::stars::sighting::{SightingOutcome, StarSighting};
use super::stars::{exposure_steps, star_camera_config};
use super::{BaroSpec, TrialError};
use crate::{AblationSet, Estimate, InertialNavigator, MasterClockBridge, NavMeans};

/// Navigate source identifiers of the trial's sensors.
pub const IMU_SOURCE: SourceId = SourceId::new(90);
/// GNSS position fixes.
pub const GNSS_POSITION_SOURCE: SourceId = SourceId::new(91);
/// GNSS velocity fixes.
pub const GNSS_VELOCITY_SOURCE: SourceId = SourceId::new(92);
/// Barometric altitude.
pub const BARO_SOURCE: SourceId = SourceId::new(93);
/// Star sightings.
pub const STAR_SOURCE: SourceId = SourceId::new(94);

/// The sensors a replay may use: its means, and the schedules that
/// withhold them.
#[derive(Debug, Clone, PartialEq)]
pub struct SetSelection {
    /// The means the filter may take observations from.
    pub means: AblationSet,
    /// When each sensor is withheld.
    pub availability: SetAvailability,
}

/// Navigate's inertial configuration for the trial: the IMU grade's
/// statistics, and the barometric bias state set to the altimeter's.
pub fn trial_inertial_config(spec: &sim_devices::ImuErrorSpec, baro: &BaroSpec) -> InertialConfig {
    InertialConfig {
        baro_bias: FogmBias::new(baro.bias_correlation_time_s, baro.bias_sigma_m),
        ..crate::inertial_config(spec)
    }
}

/// One sensor set's navigator stepping through a log. Cloning it forks the
/// run, filter state included.
#[derive(Debug, Clone)]
pub struct SetRun<'a> {
    selection: SetSelection,
    /// Boxed: the filter's inline history makes it hundreds of kilobytes,
    /// too much to move about a test thread's stack.
    navigator: Box<InertialNavigator>,
    step: usize,
    next_gnss: usize,
    next_baro: usize,
    next_opening: usize,
    next_reading: usize,
    sighting: Option<StarSighting<'a>>,
    gnss_position_sequence: WrappingSequence,
    gnss_velocity_sequence: WrappingSequence,
    baro_sequence: WrappingSequence,
    star_half_exposure_steps: usize,
    ledger: Ledger,
    stars: StarAvailability,
}

impl<'a> SetRun<'a> {
    /// A navigator aligned from the log's alignment fix at step 0, taking
    /// star sightings through `sighting` when the selection has stars.
    pub fn new(
        log: &SensorLog,
        selection: SetSelection,
        config: InertialConfig,
        sighting: Option<StarSighting<'a>>,
    ) -> Result<Self, TrialError> {
        let clock = MasterClockBridge::default();
        let mut navigator = Box::new(InertialNavigator::new(config, clock, IMU_SOURCE));
        let start = log.instants.first().copied().unwrap_or(SimInstant::START);
        navigator.align(&log.alignment, start)?;
        let sighting = sighting.filter(|_| selection.means.contains(NavMeans::Celestial));
        let star_half_exposure_steps = match sighting {
            Some(_) => exposure_steps(&star_camera_config(), STEP_HZ as f64)? / 2,
            None => 0,
        };
        let ledger = Ledger::new(&selection.availability);
        Ok(Self {
            selection,
            navigator,
            step: 0,
            next_gnss: 0,
            next_baro: 0,
            next_opening: 0,
            next_reading: 0,
            sighting,
            gnss_position_sequence: WrappingSequence::new(0),
            gnss_velocity_sequence: WrappingSequence::new(0),
            baro_sequence: WrappingSequence::new(0),
            star_half_exposure_steps,
            ledger,
            stars: StarAvailability::default(),
        })
    }

    /// This run continued from its present state under `selection`.
    pub fn fork(&self, selection: SetSelection) -> Self {
        let mut fork = self.clone();
        if !selection.means.contains(NavMeans::Celestial) {
            fork.sighting = None;
        }
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
        let at = instant(log, self.step)?;
        Ok(self.navigator.solution(at)?.map(Estimate::new))
    }

    /// Processes every step up to and including `target`: the IMU increment
    /// first, then the observations delivered at that step.
    pub fn advance_to(&mut self, log: &SensorLog, target: usize) -> Result<(), TrialError> {
        while self.step < target {
            let step = self.step.wrapping_add(1);
            let at = instant(log, step)?;
            let Some(increment) = log.imu.get(step.wrapping_sub(1)) else {
                return Err(TrialError::InvalidRequest {
                    field: "replay",
                    detail: format!("no IMU increment for step {step}"),
                });
            };
            self.navigator.ingest(increment, at)?;
            self.step = step;
            self.stars_at(log, step, at)?;
            self.gnss_at(log, step, at)?;
            self.baro_at(log, step, at)?;
        }
        Ok(())
    }

    /// The star frame opening or read out at `step`. A frame any instant
    /// of whose mid-exposure to readout a schedule covers is neither primed
    /// with the navigator's prior nor read: the camera gave nothing.
    fn stars_at(&mut self, log: &SensorLog, step: usize, at: SimInstant) -> Result<(), TrialError> {
        let frames = &log.stars;
        let Some(sighting) = self.sighting.as_mut() else {
            return Ok(());
        };
        let availability = &self.selection.availability;
        if log.star_mid_steps.get(self.next_opening) == Some(&step) {
            let end = step.wrapping_add(self.star_half_exposure_steps);
            if !availability.withheld(SensorKind::Stars, step_seconds(step), step_seconds(end)) {
                sighting.hold_prior(&mut self.navigator, at)?;
            }
            self.next_opening = self.next_opening.wrapping_add(1);
        }
        let Some(frame) = frames.get(self.next_reading) else {
            return Ok(());
        };
        if frame.image.end_step != step {
            return Ok(());
        }
        self.next_reading = self.next_reading.wrapping_add(1);
        let window = (step_seconds(frame.image.mid_step), step_seconds(step));
        if availability.withheld(SensorKind::Stars, window.0, window.1) {
            self.ledger
                .record(SensorKind::Stars, window, &Reading::Withheld);
            return Ok(());
        }
        let outcome = sighting.aid(&frame.image, &mut self.navigator, at)?;
        self.tally_stars(&outcome, window);
        Ok(())
    }

    fn tally_stars(&mut self, outcome: &SightingOutcome, window: (f64, f64)) {
        let stars = &mut self.stars;
        stars.frames_attempted = stars.frames_attempted.wrapping_add(1);
        if outcome.lost_in_space {
            stars.lost_in_space_attempted = stars.lost_in_space_attempted.wrapping_add(1);
        }
        if outcome.solved {
            stars.frames_solved = stars.frames_solved.wrapping_add(1);
            stars.stars_in_solved_frames = stars
                .stars_in_solved_frames
                .wrapping_add(outcome.stars as u64);
            stars.stars_per_solved_frame =
                stars.stars_in_solved_frames as f64 / stars.frames_solved as f64;
            if outcome.lost_in_space {
                stars.lost_in_space_solved = stars.lost_in_space_solved.wrapping_add(1);
            }
        }
        let accepted = (0..outcome.accepted).map(|_| Ok(()));
        let verdicts: Vec<Result<(), &RejectionReason>> =
            accepted.chain(outcome.rejected.iter().map(Err)).collect();
        self.ledger
            .record(SensorKind::Stars, window, &Reading::Offered(&verdicts));
    }

    fn gnss_at(&mut self, log: &SensorLog, step: usize, at: SimInstant) -> Result<(), TrialError> {
        let time_s = at.duration_since(SimInstant::START).as_seconds();
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
            let fix = logged.reading;
            let position = ObservationValue::PositionFix {
                position: fix.position,
                covariance: fix.position_cov,
            };
            let velocity = ObservationValue::VelocityFix {
                velocity: fix.velocity,
                covariance: fix.velocity_cov,
            };
            let position = offer(
                &mut self.navigator,
                &mut self.gnss_position_sequence,
                (GNSS_POSITION_SOURCE, SensorClass::Gnss),
                position,
                at,
            )?;
            let velocity = offer(
                &mut self.navigator,
                &mut self.gnss_velocity_sequence,
                (GNSS_VELOCITY_SOURCE, SensorClass::Gnss),
                velocity,
                at,
            )?;
            let verdicts = [position.as_ref().map(|_| ()), velocity.as_ref().map(|_| ())];
            self.ledger
                .record(SensorKind::Gnss, window, &Reading::Offered(&verdicts));
        }
        Ok(())
    }

    fn baro_at(&mut self, log: &SensorLog, step: usize, at: SimInstant) -> Result<(), TrialError> {
        let time_s = at.duration_since(SimInstant::START).as_seconds();
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
            let value = ObservationValue::Altitude {
                altitude_m: logged.reading.altitude_m,
                variance_m2: logged.reading.variance_m2,
            };
            let outcome = offer(
                &mut self.navigator,
                &mut self.baro_sequence,
                (BARO_SOURCE, SensorClass::Barometric),
                value,
                at,
            )?;
            let verdicts = [outcome.as_ref().map(|_| ())];
            self.ledger
                .record(SensorKind::Baro, window, &Reading::Offered(&verdicts));
        }
        Ok(())
    }
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

/// Offers `value` from `source` of `class`, acquired and delivered at `at`,
/// as the next of `sequence`.
fn offer(
    navigator: &mut InertialNavigator,
    sequence: &mut WrappingSequence,
    (source, class): (SourceId, SensorClass),
    value: ObservationValue,
    at: SimInstant,
) -> Result<Result<(), RejectionReason>, TrialError> {
    let stamp = MasterClockBridge::default().stamp(source, SourceEpoch::new(0), *sequence, at)?;
    *sequence = sequence.next();
    let observation = Observation::new(stamp, value, SourceComposition::of(class));
    Ok(match navigator.observe(&observation, at)? {
        IngestOutcome::Accepted => Ok(()),
        IngestOutcome::Rejected(reason) => Err(reason),
    })
}
