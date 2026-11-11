//! Replaying a sensor log through Navigate's inertial filter under one
//! sensor set. Nothing here takes truth: the log and the navigator's own
//! solution are all a replay sees.

use std::collections::BTreeMap;

use navigate_contract::{
    NavigationSolution, SensorClass, SourceComposition, SourceEpoch, SourceId, WrappingSequence,
};
use navigate_fusion::{
    FogmBias, InertialConfig, IngestOutcome, Observation, ObservationValue, RejectionReason,
};
use serde::{Deserialize, Serialize};
use sim_core::SimInstant;

use super::record::SensorLog;
use super::stars::sighting::{SightingOutcome, StarSighting};
use super::{BaroSpec, OutageSpec, TrialError};
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

/// Observations of one sensor offered to the filter and what it did with
/// them.
#[derive(Debug, Clone, PartialEq, Eq, Default, Serialize, Deserialize)]
pub struct ObservationTally {
    /// Offered to the filter.
    pub offered: u64,
    /// Accepted.
    pub accepted: u64,
    /// Refused.
    pub rejected: u64,
    /// Refusals by Navigate's rejection reason.
    pub reasons: BTreeMap<String, u64>,
}

impl ObservationTally {
    pub(super) fn count(&mut self, outcome: Result<(), &RejectionReason>) {
        self.offered = self.offered.wrapping_add(1);
        match outcome {
            Ok(()) => self.accepted = self.accepted.wrapping_add(1),
            Err(reason) => {
                self.rejected = self.rejected.wrapping_add(1);
                let entry = self.reasons.entry(reason_kind(reason)).or_default();
                *entry = entry.wrapping_add(1);
            }
        }
    }
}

/// A rejection reason's variant name, without its values.
fn reason_kind(reason: &RejectionReason) -> String {
    let text = format!("{reason:?}");
    text.split([' ', '{', '('])
        .next()
        .unwrap_or_default()
        .to_owned()
}

/// How the star tracker fared over a replay.
#[derive(Debug, Clone, Copy, PartialEq, Default, Serialize, Deserialize)]
pub struct StarAvailability {
    /// Frames handed to the tracker.
    pub frames_attempted: u64,
    /// Frames identified and solved.
    pub frames_solved: u64,
    /// Frames the tracker tried lost in space.
    pub lost_in_space_attempted: u64,
    /// Of those, solved.
    pub lost_in_space_solved: u64,
    /// Stars over all solved frames.
    pub stars_in_solved_frames: u64,
    /// Stars per solved frame; zero when none solved.
    pub stars_per_solved_frame: f64,
}

/// The sensors a replay may use: its means, and the GNSS outages it honours.
#[derive(Debug, Clone, PartialEq)]
pub struct SetSelection {
    /// The means the filter may take observations from.
    pub means: AblationSet,
    /// GNSS outages; empty when the set ignores them.
    pub outages: Vec<OutageSpec>,
}

/// Navigate's inertial configuration for the trial: the IMU grade's
/// statistics, and the barometric bias state set to the altimeter's.
pub fn trial_inertial_config(spec: &sim_devices::ImuErrorSpec, baro: &BaroSpec) -> InertialConfig {
    InertialConfig {
        baro_bias: FogmBias::new(baro.bias_correlation_time_s, baro.bias_sigma_m),
        ..crate::inertial_config(spec)
    }
}

/// Per-sensor tallies of a replay.
#[derive(Debug, Clone, PartialEq, Eq, Default, Serialize, Deserialize)]
pub struct SensorTallies {
    /// GNSS position fixes.
    pub gnss_position: ObservationTally,
    /// GNSS velocity fixes.
    pub gnss_velocity: ObservationTally,
    /// Barometric altitudes.
    pub baro: ObservationTally,
    /// Star lines of sight.
    pub stars: ObservationTally,
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
    tallies: SensorTallies,
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
            tallies: SensorTallies::default(),
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
        &self.tallies
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

    fn stars_at(&mut self, log: &SensorLog, step: usize, at: SimInstant) -> Result<(), TrialError> {
        let frames = &log.stars;
        let Some(sighting) = self.sighting.as_mut() else {
            return Ok(());
        };
        if log.star_mid_steps.get(self.next_opening) == Some(&step) {
            sighting.hold_prior(&mut self.navigator, at)?;
            self.next_opening = self.next_opening.wrapping_add(1);
        }
        if let Some(frame) = frames.get(self.next_reading)
            && frame.image.end_step == step
        {
            let outcome = sighting.aid(&frame.image, &mut self.navigator, at)?;
            self.next_reading = self.next_reading.wrapping_add(1);
            self.tally_stars(&outcome);
        }
        Ok(())
    }

    fn tally_stars(&mut self, outcome: &SightingOutcome) {
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
        for _ in 0..outcome.accepted {
            self.tallies.stars.count(Ok(()));
        }
        for reason in &outcome.rejected {
            self.tallies.stars.count(Err(reason));
        }
    }

    fn gnss_at(&mut self, log: &SensorLog, step: usize, at: SimInstant) -> Result<(), TrialError> {
        let time_s = at.duration_since(SimInstant::START).as_seconds();
        while let Some(logged) = log.gnss.get(self.next_gnss).filter(|f| f.step <= step) {
            self.next_gnss = self.next_gnss.wrapping_add(1);
            let denied = self.selection.outages.iter().any(|o| o.contains(time_s));
            if !self.selection.means.contains(NavMeans::Gnss) || denied {
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
            let outcome = offer(
                &mut self.navigator,
                &mut self.gnss_position_sequence,
                (GNSS_POSITION_SOURCE, SensorClass::Gnss),
                position,
                at,
            )?;
            self.tallies
                .gnss_position
                .count(outcome.as_ref().map(|_| ()));
            let outcome = offer(
                &mut self.navigator,
                &mut self.gnss_velocity_sequence,
                (GNSS_VELOCITY_SOURCE, SensorClass::Gnss),
                velocity,
                at,
            )?;
            self.tallies
                .gnss_velocity
                .count(outcome.as_ref().map(|_| ()));
        }
        Ok(())
    }

    fn baro_at(&mut self, log: &SensorLog, step: usize, at: SimInstant) -> Result<(), TrialError> {
        while let Some(logged) = log.baro.get(self.next_baro).filter(|b| b.step <= step) {
            self.next_baro = self.next_baro.wrapping_add(1);
            if !self.selection.means.contains(NavMeans::Barometric) {
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
            self.tallies.baro.count(outcome.as_ref().map(|_| ()));
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
