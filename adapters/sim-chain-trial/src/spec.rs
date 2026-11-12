//! What one chain trial is: the navigation trial's request, where and how
//! the aeroplane starts, whose navigation Pilotage flies on, and the
//! devices' outages.

use serde::{Deserialize, Serialize};
use sim_flight_chain::AccelCalibration;
use sim_navigate::trial::{ImuGradeName, SensorKind, TrialRequest};

/// One chain trial, as a link describes it.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ChainTrialSpec {
    /// The route (heights above the ellipsoid), the sets compared, the
    /// seeds, the epoch, the outputs, the terrain camera and the maps.
    pub request: TrialRequest,
    /// Where and how the aeroplane starts. The onboard software is handed
    /// the same values as its declared start.
    pub start: StartSpec,
    /// Whose navigation Pilotage's mission flies on.
    pub steering: Steering,
    /// How far the accelerometers were calibrated.
    #[serde(default)]
    pub calibration: CalibrationName,
    /// Times a device is out of service, for every consumer.
    #[serde(default)]
    pub outages: Vec<DeviceOutage>,
    /// The holding pattern at the last fix the link states; absent for the
    /// standard right-hand hold the mission flies when its plan ends.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub hold: Option<HoldSpec>,
}

/// The holding pattern at the last fix, on the inbound course of the last
/// leg.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct HoldSpec {
    /// The direction of the turns.
    pub turn: HoldTurn,
    /// The inbound leg time, seconds, more than zero.
    pub leg_time_s: f64,
    /// The holding altitude above the WGS84 ellipsoid, metres; absent for
    /// the last fix's.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub altitude_m: Option<f64>,
}

/// The direction of a hold's turns.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum HoldTurn {
    /// Left turns.
    Left,
    /// Right turns, the standard pattern.
    Right,
}

/// The start of the flight.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct StartSpec {
    /// Geodetic latitude, degrees.
    pub lat_deg: f64,
    /// Longitude, degrees east.
    pub lon_deg: f64,
    /// Height above the WGS84 ellipsoid, metres.
    pub height_m: f64,
    /// Heading from true north, degrees.
    pub heading_deg: f64,
    /// Indicated airspeed, m/s; the cruise speed asked of the mission too.
    pub indicated_m_s: f64,
}

/// Whose navigation Pilotage flies on.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Steering {
    /// The flight controller's own estimate. For tests: a link always names
    /// a navigation set.
    FlightController,
    /// The solution of the named Navigate set, handed to Pilotage as the
    /// onboard navigator's.
    Set(String),
}

/// The accelerometers' calibration by name.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum CalibrationName {
    /// [`AccelCalibration::Calibrated`].
    #[default]
    Calibrated,
    /// [`AccelCalibration::Uncalibrated20Mg`].
    Uncalibrated20Mg,
}

impl CalibrationName {
    /// The navigators' name for the grade the calibration gives the unit.
    pub const fn grade_name(self) -> ImuGradeName {
        match self {
            Self::Calibrated => ImuGradeName::ConsumerMemsCalibrated,
            Self::Uncalibrated20Mg => ImuGradeName::ConsumerMems,
        }
    }

    /// The suite's calibration.
    pub const fn calibration(self) -> AccelCalibration {
        match self {
            Self::Calibrated => AccelCalibration::Calibrated,
            Self::Uncalibrated20Mg => AccelCalibration::Uncalibrated20Mg,
        }
    }
}

/// One stretch of time a device delivers nothing to anyone.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct DeviceOutage {
    /// The device.
    pub sensor: SensorKind,
    /// Route time it goes out, seconds.
    pub start_s: f64,
    /// Route time it comes back, seconds; `None` while it stays out.
    pub end_s: Option<f64>,
}

impl DeviceOutage {
    /// Whether the device is out for a cycle starting at route time `t_s`.
    pub fn contains(&self, t_s: f64) -> bool {
        t_s >= self.start_s && self.end_s.is_none_or(|end| t_s < end)
    }

    /// Whether the device was out in the time just before `t_s`: what a
    /// reading or a report at `t_s` reflects.
    pub fn covers_up_to(&self, t_s: f64) -> bool {
        t_s > self.start_s && self.end_s.is_none_or(|end| t_s <= end)
    }
}
