//! What one chain trial is: the navigation trial's request, where and how
//! the aeroplane starts, whose navigation Pilotage flies on, and the
//! devices' outages.

use serde::{Deserialize, Serialize};
use sim_flight_chain::{AccelCalibration, C172Variant};
use sim_navigate::trial::{ImuGradeName, SensorKind, TrialRequest};

/// Cessna 172 sub-variant flown by [`ChainTrialSpec`].
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ChainC172Variant {
    /// Cessna 172S Skyhawk SP (`c172` / `c172s`, default).
    #[default]
    C172s,
    /// Cessna 172P Skyhawk II / Reims F172P (`c172p`).
    C172p,
}

impl ChainC172Variant {
    /// Whether this is [`Self::C172s`], allowing legacy specs to omit `"variant"`.
    #[must_use]
    pub const fn is_default(&self) -> bool {
        matches!(self, Self::C172s)
    }

    /// Maps onto [`C172Variant`] in `sim-flight-chain`.
    #[must_use]
    pub const fn flight_variant(self) -> C172Variant {
        match self {
            Self::C172s => C172Variant::C172s,
            Self::C172p => C172Variant::C172p,
        }
    }
}

/// One chain trial, as a link describes it.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ChainTrialSpec {
    /// Which Cessna 172 sub-variant flies the trial; omitted on default `C172s`
    /// specs so existing trial digests remain unchanged.
    #[serde(default, skip_serializing_if = "ChainC172Variant::is_default")]
    pub variant: ChainC172Variant,
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
    /// A spoofer of the GNSS receiver; absent for a true sky throughout.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub gnss_spoof: Option<GnssSpoofSpec>,
    /// Structured adversarial GNSS spoofing windows supporting step-offset,
    /// ramp, and velocity-pull profiles across one or more `[start_s, end_s)`
    /// intervals. When non-empty, overrides [`Self::gnss_spoof`].
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub spoof_windows: Vec<SpoofWindowSpec>,
}

/// Kind of adversarial GNSS spoofing active on the receiver.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SpoofKind {
    /// No spoofing active (`true` sky).
    #[default]
    None,
    /// Instantaneous position step offset (`offset_enu_m`) with zero Doppler pull.
    Offset,
    /// Coherent position and velocity ramp (`offset_enu_m + rate_enu_mps * dt`).
    Ramp,
    /// Velocity pull with independent position drag and Doppler velocity bias.
    VelocityPull,
}

/// One adversarial GNSS spoofing window `[start_s, end_s)` in local ENU axes.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SpoofWindowSpec {
    /// Route time at which this spoofing window captures the receiver, seconds.
    pub start_s: f64,
    /// Route time at which this spoofing window releases the receiver, seconds;
    /// `None` if spoofing stays active through the end of the run.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub end_s: Option<f64>,
    /// Adversarial spoofing profile kind (`offset`, `ramp`, or `velocity_pull`).
    pub kind: SpoofKind,
    /// Initial step offset `[east, north, up]` applied at `start_s`, metres.
    #[serde(default)]
    pub offset_enu_m: [f64; 3],
    /// Position pull rate `[east, north, up]` while active, m/s.
    #[serde(default)]
    pub rate_enu_mps: [f64; 3],
    /// Optional explicit Doppler velocity bias `[east, north, up]`, m/s, for
    /// [`SpoofKind::VelocityPull`]; defaults to `rate_enu_mps` when omitted.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub velocity_enu_mps: Option<[f64; 3]>,
}

impl SpoofWindowSpec {
    /// Whether this window holds the receiver for a cycle starting at `t_s`.
    #[must_use]
    pub fn contains(&self, t_s: f64) -> bool {
        t_s >= self.start_s && self.end_s.is_none_or(|end| t_s < end)
    }
}

/// A spoofer that drags the GNSS receiver's fixes away from the truth, for
/// every consumer of the receiver. A capture starts from no offset; a span
/// that begins where the one before ends carries its drag on.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct GnssSpoofSpec {
    /// The rate the reported position leaves the true one, m/s, more than
    /// zero; the reported velocity carries the same drag.
    pub rate_m_s: f64,
    /// The direction of the drag, degrees from true north.
    pub bearing_deg: f64,
    /// When the spoofer holds the receiver.
    pub spans: Vec<SpoofSpan>,
}

impl GnssSpoofSpec {
    /// The drag a spoofer switched on by hand applies, m/s: slow enough
    /// that no single fix stands out from the receiver's own errors.
    pub const DEFAULT_RATE_M_S: f64 = 1.0;
    /// The direction a spoofer switched on by hand drags toward, degrees
    /// from true north.
    pub const DEFAULT_BEARING_DEG: f64 = 90.0;

    /// The spoofer a hand switches: the default drag, holding nothing yet.
    pub const fn by_hand() -> Self {
        Self {
            rate_m_s: Self::DEFAULT_RATE_M_S,
            bearing_deg: Self::DEFAULT_BEARING_DEG,
            spans: Vec::new(),
        }
    }
}

/// One stretch of time a spoofer holds a device.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SpoofSpan {
    /// Route time it captures the device, seconds.
    pub start_s: f64,
    /// Route time it lets go, seconds; `None` while it holds on.
    pub end_s: Option<f64>,
}

impl SpoofSpan {
    /// Whether the spoofer holds the device for a cycle starting at route
    /// time `t_s`.
    pub fn contains(&self, t_s: f64) -> bool {
        t_s >= self.start_s && self.end_s.is_none_or(|end| t_s < end)
    }
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
