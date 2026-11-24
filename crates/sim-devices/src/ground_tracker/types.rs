//! Public configuration, measurement, and error types for ground tracking stations.

use core::f64::consts::FRAC_PI_2;

use serde::{Deserialize, Serialize};
use sim_geo::{BodyEllipsoid, Geodetic};
use sim_spacetime::{
    EphemerisError, NaifId, OccultingSphere, OrientationError, SpacetimeTransformError, TdbEpoch,
    TrajectoryQueryError, body_ellipsoid_serde, geodetic_serde,
};

/// Signal propagation mode for a ground tracking observation.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum TrackingLinkMode {
    /// Target transmits at `tx_epoch` and the ground station receives at `rx_epoch`.
    ///
    /// - `light_time_s` equals `rx_epoch - tx_epoch`.
    /// - `range_m` equals `c * light_time_s`.
    /// - `range_rate_m_s` equals `d(range_m) / d(rx_epoch)` (positive when opening).
    /// - `doppler_hz` equals `-carrier_frequency_hz * range_rate_m_s / c`.
    OneWayDownlink,
    /// Ground station transmits at `tx_epoch`, target coherently turns around at
    /// `bounce_epoch`, and the ground station receives at `rx_epoch`.
    ///
    /// - `light_time_s` equals total round-trip light time `rx_epoch - tx_epoch`.
    /// - `range_m` equals the two-way equivalent one-way range
    ///   `0.5 * c * (rx_epoch - tx_epoch) = 0.5 * (R_up + R_down)`.
    /// - `range_rate_m_s` equals `d(range_m) / d(rx_epoch)` by the full two-leg
    ///   light-time chain rule (positive when opening).
    /// - `doppler_hz` equals the unit-turnaround two-way coherent sky-frequency
    ///   shift `-2.0 * carrier_frequency_hz * range_rate_m_s / c`.
    TwoWayCoherent,
}

/// Static configuration of a planetary surface tracking station.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct GroundTrackerStationSpec {
    /// Unique numeric ground station identifier.
    pub station_id: u64,
    /// Host planetary body identifier (`NaifId::EARTH`, `NaifId::MOON`, `NaifId::MARS`, ...).
    pub body: NaifId,
    /// Host body reference ellipsoid used to convert `geodetic` into body-fixed coordinates.
    #[serde(with = "body_ellipsoid_serde")]
    pub ellipsoid: BodyEllipsoid,
    /// Station geodetic latitude, longitude (`rad`), and ellipsoidal height (`m`).
    #[serde(with = "geodetic_serde")]
    pub geodetic: Geodetic,
    /// Minimum local topocentric horizon elevation mask (`rad`, in `[-pi/2, pi/2]`).
    pub min_elevation_rad: f64,
    /// Nominal carrier frequency for Doppler shift conversion (`Hz`, `>= 0`).
    pub carrier_frequency_hz: f64,
    /// Optional spherical planetary bodies checked for line-of-sight signal occultation.
    #[serde(default)]
    pub occulting_bodies: Vec<OccultingSphere>,
}

impl GroundTrackerStationSpec {
    /// Constructs a station specification without extra occulting bodies.
    #[must_use]
    pub fn new(
        station_id: u64,
        body: NaifId,
        ellipsoid: BodyEllipsoid,
        geodetic: Geodetic,
        min_elevation_rad: f64,
        carrier_frequency_hz: f64,
    ) -> Self {
        Self {
            station_id,
            body,
            ellipsoid,
            geodetic,
            min_elevation_rad,
            carrier_frequency_hz,
            occulting_bodies: Vec::new(),
        }
    }

    /// Appends an occultation check against the station's own host body using `radius_m`.
    #[must_use]
    pub fn with_host_occultation(self, radius_m: f64) -> Self {
        let body = self.body;
        self.with_occulting_body(body, radius_m)
    }

    /// Appends a spherical planetary body to check for signal path occultation.
    #[must_use]
    pub fn with_occulting_body(mut self, body: NaifId, radius_m: f64) -> Self {
        self.occulting_bodies
            .push(OccultingSphere { body, radius_m });
        self
    }

    /// Replaces the occulting body list used for signal path checks.
    #[must_use]
    pub fn with_occulting_bodies(mut self, bodies: Vec<OccultingSphere>) -> Self {
        self.occulting_bodies = bodies;
        self
    }

    /// Validates that all geometric, frequency, and occultation parameters are finite and physical.
    pub fn validate(&self) -> Result<(), GroundTrackerError> {
        validate_finite("ellipsoid.semi_major_m", self.ellipsoid.semi_major_m)?;
        if self.ellipsoid.semi_major_m <= 0.0 {
            return Err(GroundTrackerError::InvalidSpec(format!(
                "ellipsoid.semi_major_m must be positive, got {}",
                self.ellipsoid.semi_major_m
            )));
        }
        validate_finite("ellipsoid.flattening", self.ellipsoid.flattening)?;
        if !(0.0..1.0).contains(&self.ellipsoid.flattening) {
            return Err(GroundTrackerError::InvalidSpec(format!(
                "ellipsoid.flattening must lie in [0, 1), got {}",
                self.ellipsoid.flattening
            )));
        }
        validate_finite("geodetic.lat_rad", self.geodetic.lat_rad)?;
        if !(-FRAC_PI_2..=FRAC_PI_2).contains(&self.geodetic.lat_rad) {
            return Err(GroundTrackerError::InvalidSpec(format!(
                "geodetic.lat_rad must lie in [-pi/2, pi/2], got {}",
                self.geodetic.lat_rad
            )));
        }
        validate_finite("geodetic.lon_rad", self.geodetic.lon_rad)?;
        validate_finite("geodetic.height_m", self.geodetic.height_m)?;
        validate_finite("min_elevation_rad", self.min_elevation_rad)?;
        if !(-FRAC_PI_2..=FRAC_PI_2).contains(&self.min_elevation_rad) {
            return Err(GroundTrackerError::InvalidSpec(format!(
                "min_elevation_rad must lie in [-pi/2, pi/2], got {}",
                self.min_elevation_rad
            )));
        }
        validate_non_negative("carrier_frequency_hz", self.carrier_frequency_hz)?;
        for occ in &self.occulting_bodies {
            validate_finite("occulting_bodies.radius_m", occ.radius_m)?;
            if occ.radius_m <= 0.0 {
                return Err(GroundTrackerError::InvalidSpec(format!(
                    "occulting body {} radius_m must be positive, got {}",
                    occ.body, occ.radius_m
                )));
            }
        }
        Ok(())
    }
}

/// Station measurement biases, 1-sigma Gaussian noise levels, and deterministic RNG key.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct TrackingErrorSpec {
    /// Constant range bias, metres.
    pub range_bias_m: f64,
    /// 1-sigma Gaussian range noise standard deviation, metres (`>= 0`).
    pub range_sigma_m: f64,
    /// Constant range-rate bias, m/s.
    pub range_rate_bias_m_s: f64,
    /// 1-sigma Gaussian range-rate noise standard deviation, m/s (`>= 0`).
    pub range_rate_sigma_m_s: f64,
    /// Constant topocentric azimuth bias, radians.
    pub azimuth_bias_rad: f64,
    /// 1-sigma Gaussian azimuth noise standard deviation, radians (`>= 0`).
    pub azimuth_sigma_rad: f64,
    /// Constant topocentric elevation bias, radians.
    pub elevation_bias_rad: f64,
    /// 1-sigma Gaussian elevation noise standard deviation, radians (`>= 0`).
    pub elevation_sigma_rad: f64,
    /// Root seed passed to [`sim_spacetime::TrialStreamKey`].
    pub rng_seed: u64,
    /// Stream tag isolating this tracker's draws within [`sim_spacetime::TrialStreamKey`].
    pub stream_tag: u64,
}

impl TrackingErrorSpec {
    /// Zero-bias, zero-noise ideal measurement configuration.
    pub const IDEAL: Self = Self {
        range_bias_m: 0.0,
        range_sigma_m: 0.0,
        range_rate_bias_m_s: 0.0,
        range_rate_sigma_m_s: 0.0,
        azimuth_bias_rad: 0.0,
        azimuth_sigma_rad: 0.0,
        elevation_bias_rad: 0.0,
        elevation_sigma_rad: 0.0,
        rng_seed: 0,
        stream_tag: 0,
    };

    /// Validates that all biases are finite and all noise standard deviations are finite and non-negative.
    pub fn validate(&self) -> Result<(), GroundTrackerError> {
        validate_finite("range_bias_m", self.range_bias_m)?;
        validate_non_negative("range_sigma_m", self.range_sigma_m)?;
        validate_finite("range_rate_bias_m_s", self.range_rate_bias_m_s)?;
        validate_non_negative("range_rate_sigma_m_s", self.range_rate_sigma_m_s)?;
        validate_finite("azimuth_bias_rad", self.azimuth_bias_rad)?;
        validate_non_negative("azimuth_sigma_rad", self.azimuth_sigma_rad)?;
        validate_finite("elevation_bias_rad", self.elevation_bias_rad)?;
        validate_non_negative("elevation_sigma_rad", self.elevation_sigma_rad)?;
        Ok(())
    }
}

/// Exact causal light-time ground tracking truth at a single reception epoch.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct TrackingTruthMeasurement {
    /// Reception epoch at the ground station (`TDB`).
    pub rx_epoch: TdbEpoch,
    /// Transmission epoch (`TDB`): target emission for [`TrackingLinkMode::OneWayDownlink`],
    /// or ground uplink emission for [`TrackingLinkMode::TwoWayCoherent`].
    pub tx_epoch: TdbEpoch,
    /// Target turnaround epoch (`TDB`) for [`TrackingLinkMode::TwoWayCoherent`]; `None` for one-way.
    pub bounce_epoch: Option<TdbEpoch>,
    /// Total causal light time `rx_epoch - tx_epoch` (`s`): one-way downlink light time
    /// for [`TrackingLinkMode::OneWayDownlink`], or total round-trip light time
    /// `(R_up + R_down) / c` for [`TrackingLinkMode::TwoWayCoherent`].
    pub light_time_s: f64,
    /// Range (`m`): `c * light_time_s` for [`TrackingLinkMode::OneWayDownlink`], or the
    /// two-way equivalent one-way range `0.5 * c * light_time_s = 0.5 * (R_up + R_down)`
    /// for [`TrackingLinkMode::TwoWayCoherent`].
    pub range_m: f64,
    /// Exact reception-time derivative `d(range_m) / d(rx_epoch)` (`m/s`, positive when opening).
    pub range_rate_m_s: f64,
    /// Carrier Doppler shift at the ground receiver (`Hz`): `-f * range_rate_m_s / c`
    /// for one-way downlink, or `-2 * f * range_rate_m_s / c` for unit-turnaround two-way coherent.
    pub doppler_hz: f64,
    /// Topocentric azimuth angle (`rad`, `[0, 2*pi)` clockwise from North):
    /// `East = sin(az) * cos(el)`, `North = cos(az) * cos(el)`, `Up = sin(el)`.
    pub azimuth_rad: f64,
    /// Topocentric elevation angle above the station local horizon (`rad`, `[-pi/2, pi/2]`).
    pub elevation_rad: f64,
}

/// Noisy ground tracking observation paired with its exact underlying [`TrackingTruthMeasurement`].
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct TrackingMeasurement {
    /// Exact underlying geometric and light-time truth for this observation.
    pub truth: TrackingTruthMeasurement,
    /// Biased, noisy range (`m`).
    pub range_m: f64,
    /// Biased, noisy range-rate (`m/s`).
    pub range_rate_m_s: f64,
    /// Carrier Doppler shift derived from the noisy `range_rate_m_s` (`Hz`).
    pub doppler_hz: f64,
    /// Biased, noisy azimuth wrapped into `[0, 2*pi)` (`rad`).
    pub azimuth_rad: f64,
    /// Biased, noisy elevation (`rad`).
    pub elevation_rad: f64,
    /// Whether the observed `elevation_rad` falls below the station's minimum elevation mask.
    pub below_mask: bool,
}

/// Errors returned while constructing or evaluating a [`super::GroundTracker`].
#[derive(Clone, Debug, PartialEq, thiserror::Error)]
pub enum GroundTrackerError {
    /// Target elevation at reception falls below the station's configured minimum mask.
    #[error(
        "target elevation {elevation_rad:.6} rad is below station mask {min_elevation_rad:.6} rad"
    )]
    BelowElevationMask {
        /// True topocentric elevation angle at reception, radians.
        elevation_rad: f64,
        /// Configured minimum elevation mask, radians.
        min_elevation_rad: f64,
    },
    /// Signal path intersects an occulting planetary body.
    #[error("signal path is occulted by body {body}")]
    BodyOcculted {
        /// Occulting body NAIF identifier.
        body: NaifId,
    },
    /// Station or error model specification failed validation.
    #[error("invalid ground tracker specification: {0}")]
    InvalidSpec(String),
    /// Target trajectory evaluation failed.
    #[error("trajectory query failed: {0}")]
    Trajectory(#[from] TrajectoryQueryError),
    /// Ephemeris catalog lookup failed.
    #[error("ephemeris lookup failed: {0}")]
    Ephemeris(#[from] EphemerisError),
    /// Body orientation catalog lookup failed.
    #[error("orientation lookup failed: {0}")]
    Orientation(#[from] OrientationError),
    /// Spatiotemporal frame transformation failed.
    #[error("frame transformation failed: {0}")]
    Transform(#[from] SpacetimeTransformError),
    /// Causal light-time fixed-point iteration failed to converge.
    #[error("causal light-time iteration did not converge")]
    LightTimeDidNotConverge,
}

fn validate_finite(field: &'static str, value: f64) -> Result<(), GroundTrackerError> {
    if !value.is_finite() {
        return Err(GroundTrackerError::InvalidSpec(format!(
            "{field} must be finite, got {value}"
        )));
    }
    Ok(())
}

fn validate_non_negative(field: &'static str, value: f64) -> Result<(), GroundTrackerError> {
    if !value.is_finite() || value < 0.0 {
        return Err(GroundTrackerError::InvalidSpec(format!(
            "{field} must be finite and non-negative, got {value}"
        )));
    }
    Ok(())
}
