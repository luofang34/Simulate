//! Scheduled sensor unavailability: named schedules of route-time
//! intervals in which a sensor delivers nothing, and what each sensor was
//! doing for a set at an instant.
//!
//! A schedule withholds readings and nothing else. The sensors still sense
//! and draw their noise, so a set meets the same realisation at the same
//! route time whatever is withheld, and the filter simply receives no
//! observation. Unavailability the physics causes, such as a star frame the
//! daylight sky saturates, is not scheduled: it comes from the sensor
//! models and is counted apart.

use std::collections::BTreeMap;

use serde::{Deserialize, Serialize};

use super::{SensorSetSpec, TrialError, TrialRequest};
use crate::NavMeans;

/// A sensor a schedule can withhold.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SensorKind {
    /// The GNSS receiver's position and velocity fixes.
    Gnss,
    /// The barometric altimeter.
    Baro,
    /// The star camera's frames.
    Stars,
    /// A terrain-relative camera. Reserved: schedules for it are accepted,
    /// but no set can carry it until the means is simulated.
    TerrainCamera,
}

impl SensorKind {
    /// Every kind, in a fixed order.
    pub const ALL: [Self; 4] = [Self::Gnss, Self::Baro, Self::Stars, Self::TerrainCamera];

    /// The navigation means the sensor serves.
    pub const fn means(self) -> NavMeans {
        match self {
            Self::Gnss => NavMeans::Gnss,
            Self::Baro => NavMeans::Barometric,
            Self::Stars => NavMeans::Celestial,
            Self::TerrainCamera => NavMeans::Visual,
        }
    }

    /// The kind's name in requests and results.
    pub const fn name(self) -> &'static str {
        match self {
            Self::Gnss => "gnss",
            Self::Baro => "baro",
            Self::Stars => "stars",
            Self::TerrainCamera => "terrain_camera",
        }
    }
}

/// An interval of route time in which a sensor delivers nothing, both ends
/// included.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct UnavailableInterval {
    /// First second without the sensor.
    pub start_s: f64,
    /// Last second without it: the error there is the error without it.
    pub end_s: f64,
    /// Why, for the reader: `jamming`, `icing`, `cloud`, `night`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub reason: Option<String>,
}

impl UnavailableInterval {
    /// Whether route time `t_s` falls inside, ends included.
    pub fn contains(&self, t_s: f64) -> bool {
        (self.start_s..=self.end_s).contains(&t_s)
    }

    /// Whether any instant of `[from_s, to_s]` falls inside.
    pub fn overlaps(&self, from_s: f64, to_s: f64) -> bool {
        from_s <= self.end_s && to_s >= self.start_s
    }
}

/// A named schedule of one sensor's unavailability.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct AvailabilitySchedule {
    /// The name sets refer to it by; unique in the request.
    pub name: String,
    /// The sensor it withholds.
    pub sensor: SensorKind,
    /// When the sensor is unavailable.
    pub unavailable: Vec<UnavailableInterval>,
}

/// One scheduled interval as it applies to a set.
#[derive(Debug, Clone, PartialEq)]
pub struct ScheduledInterval {
    /// The sensor withheld.
    pub sensor: SensorKind,
    /// The schedule's name.
    pub schedule: String,
    /// The interval.
    pub interval: UnavailableInterval,
}

/// The schedules one set honours, resolved from the request.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct SetAvailability {
    intervals: Vec<ScheduledInterval>,
}

impl SetAvailability {
    /// No sensor ever withheld.
    pub const NONE: Self = Self {
        intervals: Vec::new(),
    };

    /// The schedules `set` names, from `request`.
    pub fn resolve(request: &TrialRequest, set: &SensorSetSpec) -> Result<Self, TrialError> {
        let mut intervals = Vec::new();
        for (sensor, name) in &set.availability {
            let schedule = request
                .availability
                .iter()
                .find(|s| &s.name == name)
                .ok_or_else(|| TrialError::InvalidSet {
                    set: set.name.clone(),
                    detail: format!("no availability schedule `{name}`"),
                })?;
            if schedule.sensor != *sensor {
                return Err(TrialError::InvalidSet {
                    set: set.name.clone(),
                    detail: format!(
                        "schedule `{name}` is for {}, not {}",
                        schedule.sensor.name(),
                        sensor.name()
                    ),
                });
            }
            intervals.extend(
                schedule
                    .unavailable
                    .iter()
                    .map(|interval| ScheduledInterval {
                        sensor: *sensor,
                        schedule: name.clone(),
                        interval: interval.clone(),
                    }),
            );
        }
        Ok(Self { intervals })
    }

    /// Every interval, by sensor then schedule order.
    pub fn intervals(&self) -> &[ScheduledInterval] {
        &self.intervals
    }

    /// Whether a reading of `sensor` acquired at `from_s` and delivered at
    /// `to_s` is withheld: any instant of it falls in a scheduled interval,
    /// so nothing of it reaches the filter inside the interval.
    pub fn withheld(&self, sensor: SensorKind, from_s: f64, to_s: f64) -> bool {
        self.intervals
            .iter()
            .any(|s| s.sensor == sensor && s.interval.overlaps(from_s, to_s))
    }

    /// Whether `sensor` is scheduled unavailable at `t_s`.
    pub fn scheduled(&self, sensor: SensorKind, t_s: f64) -> bool {
        self.withheld(sensor, t_s, t_s)
    }
}

/// What one sensor was doing for a set at an instant.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SensorState {
    /// Its last reading reached the filter and was accepted.
    Aiding,
    /// A schedule withholds it at this instant.
    ScheduledUnavailable,
    /// Its last reading gave the filter nothing, or none has come yet: a
    /// star frame with no solution (a bright sky, too few stars, or no
    /// identification).
    PhysicallyUnavailable,
    /// Its last reading was offered and the filter refused all of it.
    Rejected,
    /// The set does not have the sensor.
    NotInSet,
}

/// The sensors of each set's schedules, checked against the request.
pub(super) fn check_set(request: &TrialRequest, set: &SensorSetSpec) -> Result<(), TrialError> {
    for sensor in set.availability.keys() {
        if !set.means.contains(&sensor.means()) {
            return Err(TrialError::InvalidSet {
                set: set.name.clone(),
                detail: format!(
                    "a schedule for {} but no {:?} means",
                    sensor.name(),
                    sensor.means()
                ),
            });
        }
    }
    SetAvailability::resolve(request, set).map(|_| ())
}

/// Every schedule's name is unique and every interval is finite, ordered
/// and inside route time.
pub(super) fn check_schedules(schedules: &[AvailabilitySchedule]) -> Result<(), TrialError> {
    let mut names = BTreeMap::new();
    for (row, schedule) in schedules.iter().enumerate() {
        if names.insert(schedule.name.as_str(), row).is_some() {
            return Err(TrialError::InvalidSchedule {
                schedule: schedule.name.clone(),
                detail: "the name is used twice".into(),
            });
        }
        for interval in &schedule.unavailable {
            let ordered = interval.start_s.is_finite()
                && interval.end_s.is_finite()
                && interval.start_s >= 0.0
                && interval.end_s > interval.start_s;
            if !ordered {
                return Err(TrialError::InvalidSchedule {
                    schedule: schedule.name.clone(),
                    detail: format!(
                        "interval {} to {} s is not 0 <= start < end",
                        interval.start_s, interval.end_s
                    ),
                });
            }
        }
    }
    Ok(())
}
