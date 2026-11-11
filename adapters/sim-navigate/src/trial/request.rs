use serde::{Deserialize, Serialize};
use sim_devices::ImuGrade;

use super::TrialError;
use crate::{AblationSet, NavMeans};

#[cfg(test)]
mod tests;

/// Everything one trial run depends on. The same request gives the same
/// result file, byte for byte.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct TrialRequest {
    /// UTC at the first route instant, ISO 8601 `YYYY-MM-DDTHH:MM:SSZ`. It
    /// sets the sky the star camera sees.
    pub epoch_utc: String,
    /// The route in fly order; the vehicle starts over the first waypoint
    /// heading for the second.
    pub waypoints: Vec<WaypointSpec>,
    /// How the vehicle flies the route.
    pub vehicle: VehicleSpec,
    /// The strapdown IMU's grade.
    pub imu_grade: ImuGradeName,
    /// Declared errors of the in-flight initial alignment.
    pub alignment: AlignmentSpec,
    /// The GNSS receiver model.
    #[serde(default)]
    pub gnss: GnssSpec,
    /// The barometric altimeter model.
    #[serde(default)]
    pub baro: BaroSpec,
    /// The star camera.
    #[serde(default)]
    pub stars: StarSpec,
    /// Route-time intervals in which GNSS is denied to the sets that honour
    /// outages.
    #[serde(default)]
    pub outages: Vec<OutageSpec>,
    /// The sensor sets to compare over the same truth and sensor
    /// realisations. The `nav-trial` binary fills an empty list with
    /// [`super::default_sensor_sets`]: every aiding means alone on the
    /// inertial core, and fused.
    #[serde(default)]
    pub sensor_sets: Vec<SensorSetSpec>,
    /// Seeds of every random draw.
    pub seeds: SeedSpec,
    /// Sampling of the result's time series.
    #[serde(default)]
    pub output: OutputSpec,
    /// When a closed-loop flight ends abnormally; absent takes the defaults.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub termination: Option<TerminationSpec>,
    /// The vehicle camera and the onboard map package, for sets with
    /// `visual`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub visual: Option<super::VisualSpec>,
}

/// The rules that end a closed-loop flight abnormally, each judged on the
/// true state. The first to fire ends the flight and is reported.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct TerminationSpec {
    /// Largest true distance from the leg being flown, metres.
    pub corridor_half_width_m: f64,
    /// Lowest true height above the ellipsoid, metres; the trial has no
    /// terrain.
    pub min_height_m: f64,
    /// Highest true height above the ellipsoid, metres.
    pub max_height_m: f64,
    /// A leg may last this many times its planned duration, plus
    /// `leg_time_margin_s`, before the navigator must sequence it.
    pub leg_time_factor: f64,
    /// Seconds added to every leg's time limit.
    pub leg_time_margin_s: f64,
}

impl Default for TerminationSpec {
    /// A 5 km corridor, 50 m to 20 km of height, and twice each leg's
    /// planned time plus five minutes.
    fn default() -> Self {
        Self {
            corridor_half_width_m: 5_000.0,
            min_height_m: 50.0,
            max_height_m: 20_000.0,
            leg_time_factor: 2.0,
            leg_time_margin_s: 300.0,
        }
    }
}

/// One waypoint of the route.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct WaypointSpec {
    /// Identifier, at most 32 bytes.
    pub ident: String,
    /// Geodetic latitude, degrees.
    pub lat_deg: f64,
    /// Longitude, degrees east.
    pub lon_deg: f64,
    /// Height above the WGS84 ellipsoid to cross at, metres.
    pub altitude_m: f64,
    /// Speed limit on the leg toward this waypoint, m/s.
    #[serde(default)]
    pub speed_m_s: Option<f64>,
    /// Cross the fix before turning instead of anticipating the turn.
    #[serde(default)]
    pub fly_over: bool,
}

/// How the vehicle flies.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct VehicleSpec {
    /// Airspeed outside waypoint speed limits, m/s. The kinematic vehicle
    /// flies 30 to 120 m/s.
    pub cruise_speed_m_s: f64,
    /// Largest bank the route follower commands, degrees. The plan's fly-by
    /// turn anticipation uses the same limit.
    pub bank_limit_deg: f64,
    /// Steepest climb or descent the route follower commands, m/s.
    pub max_climb_m_s: f64,
    /// Distance inside which a waypoint captures regardless of turn
    /// geometry, metres.
    pub capture_radius_m: f64,
    /// Which navigation solution guidance steers on.
    #[serde(default)]
    pub guidance: GuidanceSource,
    /// Lowest solution quality guidance accepts in closed loop; absent is
    /// Navigate's own default, `degraded`. A refused solution holds the
    /// present track and height.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub guidance_min_quality: Option<QualityFloor>,
}

/// A solution quality floor by name.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum QualityFloor {
    /// Only `Good` solutions.
    Good,
    /// `Good` and `Degraded`.
    Degraded,
    /// Every solution, however uncertain.
    Unusable,
}

/// The navigation solution route guidance steers on.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum GuidanceSource {
    /// The simulator's true state: open loop, so the flight does not depend
    /// on the sensor set being scored.
    #[default]
    Truth,
    /// Each set's own navigator: closed loop, so navigation error steers
    /// the vehicle and every set flies its own trajectory.
    Estimate,
}

/// IMU grade by name.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ImuGradeName {
    /// [`ImuGrade::ConsumerMems`].
    ConsumerMems,
    /// [`ImuGrade::Tactical`].
    Tactical,
    /// [`ImuGrade::Navigation`].
    Navigation,
}

impl ImuGradeName {
    /// The device grade.
    pub const fn grade(self) -> ImuGrade {
        match self {
            Self::ConsumerMems => ImuGrade::ConsumerMems,
            Self::Tactical => ImuGrade::Tactical,
            Self::Navigation => ImuGrade::Navigation,
        }
    }
}

/// Initial alignment 1-sigma per axis.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct AlignmentSpec {
    /// Position, metres.
    pub position_sigma_m: f64,
    /// Velocity, m/s.
    pub velocity_sigma_m_s: f64,
    /// Attitude about each body axis, radians.
    pub attitude_sigma_rad: f64,
}

impl Default for AlignmentSpec {
    /// An in-flight alignment from a GNSS position, a velocity and a
    /// transfer-aligned attitude.
    fn default() -> Self {
        Self {
            position_sigma_m: 2.0,
            velocity_sigma_m_s: 0.1,
            attitude_sigma_rad: 1e-3,
        }
    }
}

/// A position-domain GNSS receiver: fixes are truth plus white noise plus a
/// first-order Gauss–Markov error per axis. It models no satellites,
/// pseudoranges or geometry.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct GnssSpec {
    /// Fix rate, Hz; it must divide the 120 Hz simulation rate.
    pub rate_hz: f64,
    /// White position noise per horizontal axis, metres.
    pub horizontal_white_sigma_m: f64,
    /// White position noise, vertical, metres.
    pub vertical_white_sigma_m: f64,
    /// Correlated position error per horizontal axis, metres.
    pub horizontal_correlated_sigma_m: f64,
    /// Correlated position error, vertical, metres.
    pub vertical_correlated_sigma_m: f64,
    /// Correlation time of the correlated error, seconds.
    pub correlation_time_s: f64,
    /// White velocity noise per axis, m/s.
    pub velocity_sigma_m_s: f64,
}

impl Default for GnssSpec {
    /// A single-frequency code receiver in open sky: decimetre-level
    /// receiver noise under metre-level slowly varying ionosphere, orbit and
    /// clock residuals, and centimetre-per-second Doppler velocity.
    fn default() -> Self {
        Self {
            rate_hz: 1.0,
            horizontal_white_sigma_m: 0.6,
            vertical_white_sigma_m: 1.0,
            horizontal_correlated_sigma_m: 1.2,
            vertical_correlated_sigma_m: 2.0,
            correlation_time_s: 300.0,
            velocity_sigma_m_s: 0.05,
        }
    }
}

/// A barometric altimeter: ellipsoidal height plus white noise plus a
/// first-order Gauss–Markov bias, the error the filter's barometric bias
/// state models.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct BaroSpec {
    /// Reading rate, Hz; it must divide the 120 Hz simulation rate.
    pub rate_hz: f64,
    /// White noise, metres.
    pub white_sigma_m: f64,
    /// Bias, metres.
    pub bias_sigma_m: f64,
    /// Correlation time of the bias, seconds.
    pub bias_correlation_time_s: f64,
}

impl Default for BaroSpec {
    /// Decimetre noise and the metres of slowly drifting bias that weather
    /// and the static-port error leave in pressure altitude.
    fn default() -> Self {
        Self {
            rate_hz: 5.0,
            white_sigma_m: 0.3,
            bias_sigma_m: 3.0,
            bias_correlation_time_s: 3_600.0,
        }
    }
}

/// The roof star camera.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct StarSpec {
    /// Time between exposures, seconds; a whole number of simulation steps.
    pub period_s: f64,
    /// Camera-to-body alignment knowledge, radians RMS, added to every
    /// sighting's declared variance.
    pub mount_alignment_sigma_rad: f64,
}

impl Default for StarSpec {
    /// One frame a second and the mount knowledge of a surveyed bracket.
    fn default() -> Self {
        Self {
            period_s: 1.0,
            mount_alignment_sigma_rad: 2.0e-5,
        }
    }
}

/// A GNSS outage in route time.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct OutageSpec {
    /// First second without GNSS.
    pub start_s: f64,
    /// Last second without GNSS: the error there is the unaided error.
    pub end_s: f64,
}

impl OutageSpec {
    /// Whether route time `t_s` falls inside the outage, ends included.
    pub fn contains(&self, t_s: f64) -> bool {
        (self.start_s..=self.end_s).contains(&t_s)
    }
}

/// One sensor set: the means the filter may take observations from.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SensorSetSpec {
    /// Name in the result.
    pub name: String,
    /// The means; `dead_reckoning` (the INS) must be one of them.
    pub means: Vec<NavMeans>,
    /// Whether the request's GNSS outages apply to this set.
    #[serde(default)]
    pub gnss_outages: bool,
}

impl SensorSetSpec {
    /// The means as a set.
    pub fn ablation_set(&self) -> AblationSet {
        self.means
            .iter()
            .fold(AblationSet::EMPTY, |set, means| set.with(*means))
    }
}

/// The seeds of each independent random draw.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SeedSpec {
    /// IMU errors.
    pub imu: u64,
    /// Initial alignment errors.
    pub alignment: u64,
    /// GNSS errors.
    pub gnss: u64,
    /// Barometric errors.
    pub baro: u64,
    /// Star camera noise.
    pub star_camera: u64,
    /// Vehicle camera noise; zero when absent.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub camera: Option<u64>,
}

/// How densely the result samples the run.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct OutputSpec {
    /// Time between scored samples, seconds; a whole number of simulation
    /// steps.
    pub sample_period_s: f64,
    /// Time between points of the truth track, seconds; a whole multiple
    /// of `sample_period_s`.
    pub track_period_s: f64,
}

impl Default for OutputSpec {
    /// Scores at 1 Hz and a track point every ten seconds.
    fn default() -> Self {
        Self {
            sample_period_s: 1.0,
            track_period_s: 10.0,
        }
    }
}

/// The means this trial can simulate.
const SIMULATED_MEANS: [NavMeans; 4] = [
    NavMeans::DeadReckoning,
    NavMeans::Barometric,
    NavMeans::Gnss,
    NavMeans::Celestial,
];

impl TrialRequest {
    /// Parses a request from JSON.
    pub fn from_json(text: &str) -> Result<Self, TrialError> {
        serde_json::from_str(text).map_err(|source| TrialError::Json {
            what: "request",
            source,
        })
    }

    /// The request as pretty JSON.
    pub fn to_json(&self) -> Result<String, TrialError> {
        serde_json::to_string_pretty(self).map_err(|source| TrialError::Json {
            what: "request",
            source,
        })
    }

    /// Refuses a request the trial cannot run as stated.
    pub fn validate(&self) -> Result<(), TrialError> {
        let invalid =
            |field: &'static str, detail: String| TrialError::InvalidRequest { field, detail };
        if self.waypoints.len() < 2 {
            return Err(invalid("waypoints", "at least two".into()));
        }
        if self.sensor_sets.is_empty() {
            return Err(invalid("sensor_sets", "at least one".into()));
        }
        for set in &self.sensor_sets {
            if !set.means.contains(&NavMeans::DeadReckoning) {
                return Err(invalid("sensor_sets", format!("{}: no INS", set.name)));
            }
            let simulated = |m: &&NavMeans| {
                SIMULATED_MEANS.contains(m) || (**m == NavMeans::Visual && self.visual.is_some())
            };
            if let Some(means) = set.means.iter().find(|m| !simulated(m)) {
                return Err(invalid(
                    "sensor_sets",
                    format!("{}: {means:?} is not simulated", set.name),
                ));
            }
        }
        for outage in &self.outages {
            if !(outage.start_s.is_finite() && outage.end_s > outage.start_s) {
                return Err(invalid("outages", format!("{outage:?}")));
            }
        }
        Ok(())
    }

    /// Whether any set takes visual fixes, so the vehicle camera runs.
    pub fn uses_visual(&self) -> bool {
        self.visual.is_some()
            && self
                .sensor_sets
                .iter()
                .any(|set| set.means.contains(&NavMeans::Visual))
    }

    /// Whether any set takes star sightings, so frames need rendering.
    pub fn uses_stars(&self) -> bool {
        self.sensor_sets
            .iter()
            .any(|set| set.means.contains(&NavMeans::Celestial))
    }
}
