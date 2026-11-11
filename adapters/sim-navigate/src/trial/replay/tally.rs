//! What a replay did with each sensor's readings: counted over the whole
//! run and inside each scheduled interval, and each sensor's latest state.

use std::collections::BTreeMap;

use navigate_fusion::RejectionReason;
use serde::{Deserialize, Serialize};

use crate::trial::availability::{ScheduledInterval, SensorKind, SensorState, SetAvailability};

/// One sensor's readings and observations, and what became of them.
///
/// A reading is what the sensor delivers at once: a GNSS fix, a barometric
/// altitude, a star frame. An observation is what the filter is offered: a
/// fix's position or velocity, an altitude, a star's line of sight.
#[derive(Debug, Clone, PartialEq, Eq, Default, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ObservationTally {
    /// Observations offered to the filter.
    pub offered: u64,
    /// Accepted.
    pub accepted: u64,
    /// Refused.
    pub rejected: u64,
    /// Refusals by Navigate's rejection reason.
    pub reasons: BTreeMap<String, u64>,
    /// Readings a schedule withheld: never offered.
    pub scheduled_unavailable: u64,
    /// Readings that gave the filter nothing: star frames with no solution.
    pub physically_unavailable: u64,
}

impl ObservationTally {
    fn count(&mut self, outcome: Result<(), &RejectionReason>) {
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

    fn withheld(&mut self) {
        self.scheduled_unavailable = self.scheduled_unavailable.wrapping_add(1);
    }

    fn unproductive(&mut self) {
        self.physically_unavailable = self.physically_unavailable.wrapping_add(1);
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

/// Per-sensor tallies of a replay, or of one interval of it.
#[derive(Debug, Clone, PartialEq, Eq, Default, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SensorTallies {
    /// GNSS position fixes.
    pub gnss_position: ObservationTally,
    /// GNSS velocity fixes.
    pub gnss_velocity: ObservationTally,
    /// Barometric altitudes.
    pub baro: ObservationTally,
    /// Star lines of sight; its readings are frames.
    pub stars: ObservationTally,
}

/// What became of one reading.
pub(super) enum Reading<'r> {
    /// A schedule withheld it.
    Withheld,
    /// Offered as observations, with the filter's verdicts.
    Offered(&'r [Result<(), &'r RejectionReason>]),
}

/// A replay's tallies and each sensor's latest state.
#[derive(Debug, Clone, Default)]
pub(super) struct Ledger {
    total: SensorTallies,
    intervals: Vec<(ScheduledInterval, SensorTallies)>,
    last: BTreeMap<SensorKind, SensorState>,
}

impl Ledger {
    /// A ledger with one tally per interval of `availability`.
    pub(super) fn new(availability: &SetAvailability) -> Self {
        Self {
            total: SensorTallies::default(),
            intervals: availability
                .intervals()
                .iter()
                .map(|interval| (interval.clone(), SensorTallies::default()))
                .collect(),
            last: BTreeMap::new(),
        }
    }

    pub(super) const fn total(&self) -> &SensorTallies {
        &self.total
    }

    pub(super) fn intervals(&self) -> &[(ScheduledInterval, SensorTallies)] {
        &self.intervals
    }

    /// The latest state of `sensor`, as its last reading left it.
    pub(super) fn last(&self, sensor: SensorKind) -> SensorState {
        self.last
            .get(&sensor)
            .copied()
            .unwrap_or(SensorState::PhysicallyUnavailable)
    }

    /// Counts a reading of `sensor` acquired at `from_s` and delivered at
    /// `to_s` into the run and every interval it overlaps. GNSS readings
    /// carry two observations, position then velocity.
    pub(super) fn record(&mut self, sensor: SensorKind, window: (f64, f64), reading: &Reading<'_>) {
        let state = match reading {
            Reading::Withheld => SensorState::ScheduledUnavailable,
            Reading::Offered(verdicts) if verdicts.iter().any(Result::is_ok) => SensorState::Aiding,
            Reading::Offered([]) => SensorState::PhysicallyUnavailable,
            Reading::Offered(_) => SensorState::Rejected,
        };
        self.last.insert(sensor, state);
        let (from_s, to_s) = window;
        let overlapping = self
            .intervals
            .iter_mut()
            .filter(|(scheduled, _)| scheduled.interval.overlaps(from_s, to_s))
            .map(|(_, tallies)| tallies);
        for tallies in std::iter::once(&mut self.total).chain(overlapping) {
            count(tallies, sensor, reading);
        }
    }
}

fn count(tallies: &mut SensorTallies, sensor: SensorKind, reading: &Reading<'_>) {
    match (sensor, reading) {
        (SensorKind::Gnss, Reading::Withheld) => {
            tallies.gnss_position.withheld();
            tallies.gnss_velocity.withheld();
        }
        (SensorKind::Gnss, Reading::Offered(verdicts)) => {
            if let Some(verdict) = verdicts.first() {
                tallies.gnss_position.count(*verdict);
            }
            if let Some(verdict) = verdicts.get(1) {
                tallies.gnss_velocity.count(*verdict);
            }
        }
        (SensorKind::Baro, reading) => count_one(&mut tallies.baro, reading),
        (SensorKind::Stars | SensorKind::TerrainCamera, reading) => {
            count_one(&mut tallies.stars, reading);
        }
    }
}

fn count_one(tally: &mut ObservationTally, reading: &Reading<'_>) {
    match reading {
        Reading::Withheld => tally.withheld(),
        Reading::Offered([]) => tally.unproductive(),
        Reading::Offered(verdicts) => {
            for verdict in *verdicts {
                tally.count(*verdict);
            }
        }
    }
}

/// How the star tracker fared over a replay.
#[derive(Debug, Clone, Copy, PartialEq, Default, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct StarAvailability {
    /// Frames handed to the navigator.
    pub frames_attempted: u64,
    /// Frames whose lines of sight the navigator offered to its filter:
    /// identified and solved.
    pub frames_solved: u64,
    /// Frames the navigator identified lost in space, with no prior.
    pub lost_in_space_attempted: u64,
    /// Of those, solved.
    pub lost_in_space_solved: u64,
    /// Stars over all solved frames.
    pub stars_in_solved_frames: u64,
    /// Stars per solved frame; zero when none solved.
    pub stars_per_solved_frame: f64,
}

/// What `availability` and the last readings say `sensor` is doing at
/// `t_s` for a set with `means`.
pub(super) fn sensor_state(
    ledger: &Ledger,
    availability: &SetAvailability,
    has_means: bool,
    sensor: SensorKind,
    t_s: f64,
) -> SensorState {
    if !has_means {
        SensorState::NotInSet
    } else if availability.scheduled(sensor, t_s) {
        SensorState::ScheduledUnavailable
    } else {
        ledger.last(sensor)
    }
}
