//! Path loss, Doppler, and the propagation-loss extension point.

use core::f64::consts::PI;

use glam::DVec3;
use sim_analysis_contract::PropagationModelSpec;
use sim_spacetime::{SPEED_OF_LIGHT_M_S, TdbEpoch, sin};
use thiserror::Error;

use super::units::db_from_ratio;
use crate::geometry::PropagationMediumQuery;

/// Geometry of one signal path, with the transmitter at emission and the receiver at reception.
#[derive(Clone, Copy, Debug, PartialEq)]
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
}

/// An extra loss on a signal path beyond free-space spreading.
///
/// An ionosphere model implements this by holding a handle to an environment field (for
/// example slant TEC along `source_position_icrf_m` to `target_position_icrf_m` at `epoch`)
/// and converting it to a frequency-dependent loss; nothing else in the budget changes.
pub trait PropagationLoss: Send + Sync {
    /// Short stable name recorded in the budget.
    fn name(&self) -> String;

    /// Loss on `path` (`dB`, `>= 0`).
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
