use std::collections::BTreeMap;

use serde::{Deserialize, Serialize};
use sim_devices::{GnssErrorSpec, ImuGrade};

use super::TrialError;
use super::availability::{AvailabilitySchedule, SensorKind};
use super::maps::MapsSpec;
use super::terrain_camera::TerrainCameraSpec;
use crate::{AblationSet, NavMeans};

#[cfg(test)]
mod tests;
mod validate;

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
    /// Named schedules of sensor unavailability that sets refer to.
    #[serde(default)]
    pub availability: Vec<AvailabilitySchedule>,
    /// The sensor sets to compare over the same truth and sensor
    /// realisations.
    pub sensor_sets: Vec<SensorSetSpec>,
    /// Seeds of every random draw.
    pub seeds: SeedSpec,
    /// Sampling of the result's time series.
    #[serde(default)]
    pub output: OutputSpec,
    /// When a closed-loop flight ends abnormally; absent takes the defaults.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub termination: Option<TerminationSpec>,
    /// The terrain camera; required when any set takes the `visual` means.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub terrain_camera: Option<TerrainCameraSpec>,
    /// The onboard and world map packages; required with the terrain
    /// camera.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub maps: Option<MapsSpec>,
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

impl GnssSpec {
    /// Horizontal 1-sigma of the simulated position error, metres: the root
    /// sum of both horizontal axes' white and correlated variances.
    pub fn horizontal_sigma_m(&self) -> f64 {
        (2.0 * (self.horizontal_white_sigma_m.powi(2) + self.horizontal_correlated_sigma_m.powi(2)))
            .sqrt()
    }
}

impl Default for GnssSpec {
    /// The device's open-sky receiver ([`GnssErrorSpec::OPEN_SKY`]) fixing
    /// once a second.
    fn default() -> Self {
        let open_sky = GnssErrorSpec::OPEN_SKY;
        Self {
            rate_hz: 1.0,
            horizontal_white_sigma_m: open_sky.horizontal_white_sigma_m,
            vertical_white_sigma_m: open_sky.vertical_white_sigma_m,
            horizontal_correlated_sigma_m: open_sky.horizontal_correlated_sigma_m,
            vertical_correlated_sigma_m: open_sky.vertical_correlated_sigma_m,
            correlation_time_s: open_sky.correlation_time_s,
            velocity_sigma_m_s: open_sky.velocity_sigma_m_s,
        }
    }
}

/// The static port the barometric altimeter reads: a pressure sensor with
/// white noise over a first-order Gauss–Markov offset. The navigator's host
/// reads its pressure as standard-atmosphere pressure altitude.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct BaroSpec {
    /// Sample rate, Hz; it must divide the 120 Hz simulation rate.
    pub rate_hz: f64,
    /// White noise of one sample, pascals.
    pub white_sigma_pa: f64,
    /// Slowly varying offset, pascals.
    pub bias_sigma_pa: f64,
    /// Correlation time of the offset, seconds.
    pub bias_correlation_time_s: f64,
}

impl Default for BaroSpec {
    /// A few pascals of noise over a slowly wandering offset of tens of
    /// pascals, typical of the device class: about 0.3 m and 2 m of
    /// pressure altitude at 1000 m.
    fn default() -> Self {
        Self {
            rate_hz: 5.0,
            white_sigma_pa: 3.0,
            bias_sigma_pa: 20.0,
            bias_correlation_time_s: 600.0,
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

/// One sensor set: the means the filter may take observations from.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SensorSetSpec {
    /// Name in the result.
    pub name: String,
    /// The means; `dead_reckoning` (the INS) must be one of them.
    pub means: Vec<NavMeans>,
    /// The schedule, by name, that withholds each sensor; a sensor not
    /// named is available whenever its physics allows.
    #[serde(default)]
    pub availability: BTreeMap<SensorKind, String>,
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
    /// Terrain camera noise; required with the terrain camera.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub terrain_camera: Option<u64>,
}

impl SeedSpec {
    /// Every sensor's seed from one master seed: the successive outputs of
    /// SplitMix64 (Steele, Lea and Flood, "Fast splittable pseudorandom
    /// number generators", OOPSLA 2014) started at `master`, taken in the
    /// order IMU, alignment, GNSS, barometer, star camera, terrain camera.
    /// The order is part of the contract: the same master gives the same
    /// seeds, and so the same noise, in every host.
    pub fn from_master(master: u64) -> Self {
        let mut state = master;
        let mut next = || {
            state = state.wrapping_add(0x9E37_79B9_7F4A_7C15);
            let mut z = state;
            z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
            z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
            z ^ (z >> 31)
        };
        Self {
            imu: next(),
            alignment: next(),
            gnss: next(),
            baro: next(),
            star_camera: next(),
            terrain_camera: Some(next()),
        }
    }
}

/// How densely the result and the progress stream sample the run.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct OutputSpec {
    /// Time between scored samples, seconds; a whole number of simulation
    /// steps.
    pub sample_period_s: f64,
    /// Time between points of the truth track, seconds; a whole multiple
    /// of `sample_period_s`.
    pub track_period_s: f64,
    /// Time between progress samples, seconds; a whole number of
    /// simulation steps. A scored sample at a whole multiple of it, and the
    /// last, is also a progress sample. It shapes only the progress stream,
    /// though like every request field it enters the result's request
    /// digest.
    #[serde(default = "default_progress_period_s")]
    pub progress_period_s: f64,
    /// The horizontal error the comparison table counts time above,
    /// metres.
    #[serde(default = "default_error_threshold_m")]
    pub horizontal_error_threshold_m: f64,
}

/// One progress sample a second of route time.
const fn default_progress_period_s() -> f64 {
    1.0
}

/// Navigate's guidance refuses a solution whose horizontal 1-sigma passes
/// 50 m, so an error beyond it is where navigation stops being usable for
/// route following.
const fn default_error_threshold_m() -> f64 {
    50.0
}

impl Default for OutputSpec {
    /// Scores at 1 Hz, a track point every ten seconds, progress at 1 Hz,
    /// and a 50 m error threshold.
    fn default() -> Self {
        Self {
            sample_period_s: 1.0,
            track_period_s: 10.0,
            progress_period_s: default_progress_period_s(),
            horizontal_error_threshold_m: default_error_threshold_m(),
        }
    }
}

/// The means this trial can simulate.
const SIMULATED_MEANS: [NavMeans; 5] = [
    NavMeans::DeadReckoning,
    NavMeans::Barometric,
    NavMeans::Gnss,
    NavMeans::Celestial,
    NavMeans::Visual,
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
}
