//! Path loss, Doppler, and the propagation-loss extension point.

use core::f64::consts::PI;

use glam::DVec3;
use sim_analysis_contract::PropagationModelSpec;
use sim_geo::Geodetic;
use sim_spacetime::{SPEED_OF_LIGHT_M_S, TdbEpoch, sin};
use thiserror::Error;
use world_propagation::{Percentage, Polarisation, ProfilePoint, PropagationError};

use super::units::db_from_ratio;
use crate::geometry::PropagationMediumQuery;

/// Statistical exceedance percentages `(time, location, situation)` strictly in `(0, 100)`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ExceedancePercentages {
    /// Percentage of time the predicted loss is not exceeded (`(0, 100)`).
    pub time: Percentage,
    /// Percentage of locations the predicted loss is not exceeded (`(0, 100)`).
    pub location: Percentage,
    /// Percentage of situations / confidence (`(0, 100)`).
    pub situation: Percentage,
}

impl ExceedancePercentages {
    /// Builds validated exceedance percentages from raw percentage numbers strictly inside `(0, 100)`.
    ///
    /// # Errors
    ///
    /// Returns [`PropagationError::InvalidParameter`] if any value is outside `(0, 100)` or non-finite.
    pub fn try_new(
        time_percent: f64,
        location_percent: f64,
        situation_percent: f64,
    ) -> Result<Self, PropagationError> {
        Ok(Self {
            time: Percentage::new(time_percent)?,
            location: Percentage::new(location_percent)?,
            situation: Percentage::new(situation_percent)?,
        })
    }

    /// Constructs exceedance percentages directly from validated [`Percentage`] values.
    #[must_use]
    pub const fn new(time: Percentage, location: Percentage, situation: Percentage) -> Self {
        Self {
            time,
            location,
            situation,
        }
    }
}

/// Geometry of one signal path, with the transmitter at emission and the receiver at reception.
#[derive(Clone, Debug, PartialEq)]
pub struct PathGeometry {
    /// Reception epoch (`TDB`).
    pub epoch: TdbEpoch,
    /// Carrier frequency at the transmitter (`Hz`).
    pub carrier_frequency_hz: f64,
    /// Transmit antenna position at emission, `SSB ICRF` (`m`).
    pub source_position_icrf_m: DVec3,
    /// Receive antenna position at reception, `SSB ICRF` (`m`).
    pub target_position_icrf_m: DVec3,
    /// Distance between the two positions (`m`).
    pub slant_range_m: f64,
    /// Range rate (`m/s`).
    pub range_rate_m_s: f64,
    /// One-way light time, when the signal-time model supplies it (`s`).
    pub light_time_s: Option<f64>,
    /// Elevation of the receiver above the transmitter's local horizon, when the transmitter stands on a body (`rad`).
    pub source_surface_elevation_rad: Option<f64>,
    /// Elevation of the transmitter above the receiver's local horizon, when the receiver stands on a body (`rad`).
    pub target_surface_elevation_rad: Option<f64>,
    /// Wave polarisation relative to the local ground plane, when specified.
    pub polarisation: Option<Polarisation>,
    /// Time/location/situation exceedance percentages for statistical terrestrial propagation models.
    pub exceedance: Option<ExceedancePercentages>,
    /// Transmitter antenna phase-center site in host-body fixed geodetic coordinates, when resolved.
    pub source_geodetic: Option<Geodetic>,
    /// Receiver antenna phase-center site in host-body fixed geodetic coordinates, when resolved.
    pub target_geodetic: Option<Geodetic>,
    /// Ordered great-circle ground elevation samples from transmitter to receiver (`m`).
    pub terrain_profile: Option<Vec<ProfilePoint>>,
}

impl PathGeometry {
    /// Builds a [`PathGeometry`] with core geometric fields and all optional terrestrial/statistical fields unset.
    #[must_use]
    pub const fn from_path(
        epoch: TdbEpoch,
        carrier_frequency_hz: f64,
        source_position_icrf_m: DVec3,
        target_position_icrf_m: DVec3,
        slant_range_m: f64,
        range_rate_m_s: f64,
    ) -> Self {
        Self {
            epoch,
            carrier_frequency_hz,
            source_position_icrf_m,
            target_position_icrf_m,
            slant_range_m,
            range_rate_m_s,
            light_time_s: None,
            source_surface_elevation_rad: None,
            target_surface_elevation_rad: None,
            polarisation: None,
            exceedance: None,
            source_geodetic: None,
            target_geodetic: None,
            terrain_profile: None,
        }
    }

    /// Sets the wave polarisation.
    #[must_use]
    pub const fn with_polarisation(mut self, polarisation: Polarisation) -> Self {
        self.polarisation = Some(polarisation);
        self
    }

    /// Sets the statistical exceedance percentages.
    #[must_use]
    pub const fn with_exceedance(mut self, exceedance: ExceedancePercentages) -> Self {
        self.exceedance = Some(exceedance);
        self
    }

    /// Sets the body-fixed geodetic terminal sites.
    #[must_use]
    pub const fn with_geodetic_sites(
        mut self,
        source: Option<Geodetic>,
        target: Option<Geodetic>,
    ) -> Self {
        self.source_geodetic = source;
        self.target_geodetic = target;
        self
    }

    /// Sets the ordered terrain profile samples from transmitter to receiver.
    #[must_use]
    pub fn with_terrain_profile(mut self, profile: Vec<ProfilePoint>) -> Self {
        self.terrain_profile = Some(profile);
        self
    }
}

/// Free-space spreading loss $20\log_{10}(4\pi\rho f/c)$ (`dB`).
#[must_use]
pub fn free_space_path_loss_db(range_m: f64, carrier_frequency_hz: f64) -> f64 {
    let x = 4.0 * PI * range_m * carrier_frequency_hz / SPEED_OF_LIGHT_M_S;
    2.0 * db_from_ratio(x)
}

/// First-order Doppler shift $-f\dot\rho/c$ (`Hz`) for a range rate positive when receding.
#[must_use]
pub fn doppler_shift_hz(carrier_frequency_hz: f64, range_rate_m_s: f64) -> f64 {
    -carrier_frequency_hz * range_rate_m_s / SPEED_OF_LIGHT_M_S
}

/// Why a propagation model cannot give a loss for a path.
#[derive(Clone, Debug, PartialEq, Error)]
pub enum PropagationLossError {
    /// The path is below the lowest elevation the model is valid for.
    #[error(
        "{model}: elevation {elevation_rad} rad is below the model limit {min_elevation_rad} rad"
    )]
    ElevationBelowModelLimit {
        /// Model name.
        model: String,
        /// Elevation of the path (`rad`).
        elevation_rad: f64,
        /// Lowest valid elevation (`rad`).
        min_elevation_rad: f64,
    },
    /// An environment model refused the query.
    #[error("{model}: medium query failed: {reason}")]
    MediumQuery {
        /// Model name.
        model: String,
        /// Reason reported by the environment model.
        reason: String,
    },
    /// Required site coordinates, terrain profile, or statistical inputs were absent from [`PathGeometry`].
    #[error("{model}: missing required path geometry field `{field}`")]
    MissingPathData {
        /// Model name.
        model: String,
        /// Name of the missing field or condition.
        field: &'static str,
    },
    /// Underlying `world-propagation` model rejected the frequency, terrain profile, or terminal parameters.
    #[error("{model}: {source}")]
    Model {
        /// Model name.
        model: String,
        /// Underlying propagation error.
        #[source]
        source: PropagationError,
    },
}

/// An extra loss on a signal path beyond free-space spreading.
///
/// An ionosphere model implements this by holding a handle to an environment field (for
/// example slant TEC along `source_position_icrf_m` to `target_position_icrf_m` at `epoch`)
/// and converting it to a frequency-dependent loss; nothing else in the budget changes.
pub trait PropagationLoss: Send + Sync {
    /// Short stable name recorded in the budget.
    fn name(&self) -> String;

    /// Loss on `path` (`dB`, `>= 0` for pure attenuators).
    fn loss_db(&self, path: &PathGeometry) -> Result<f64, PropagationLossError>;
}

impl PropagationLoss for PropagationModelSpec {
    fn name(&self) -> String {
        match self {
            Self::TroposphericCosecant { .. } => "tropospheric-cosecant".to_owned(),
        }
    }

    fn loss_db(&self, path: &PathGeometry) -> Result<f64, PropagationLossError> {
        let Self::TroposphericCosecant {
            zenith_attenuation_db,
            min_elevation_rad,
        } = *self;
        let mut total_db = 0.0;
        for elevation_rad in [
            path.source_surface_elevation_rad,
            path.target_surface_elevation_rad,
        ]
        .into_iter()
        .flatten()
        {
            if elevation_rad < min_elevation_rad {
                return Err(PropagationLossError::ElevationBelowModelLimit {
                    model: self.name(),
                    elevation_rad,
                    min_elevation_rad,
                });
            }
            total_db += zenith_attenuation_db / sin(elevation_rad);
        }
        Ok(total_db)
    }
}

/// Adapts a medium model to a propagation loss through its atmospheric-loss output.
///
/// Its slant TEC is not used here: ionospheric delay is not a power loss, and absorption or
/// scintillation fades need a model of their own.
pub struct MediumQueryLoss<'a>(pub &'a dyn PropagationMediumQuery);

impl PropagationLoss for MediumQueryLoss<'_> {
    fn name(&self) -> String {
        "medium-query-atmospheric-loss".to_owned()
    }

    fn loss_db(&self, path: &PathGeometry) -> Result<f64, PropagationLossError> {
        self.0
            .sample_medium_along_los(
                path.epoch,
                path.source_position_icrf_m,
                path.target_position_icrf_m,
            )
            .map(|m| m.atmospheric_loss_db)
            .map_err(|reason| PropagationLossError::MediumQuery {
                model: self.name(),
                reason,
            })
    }
}
