//! Position-domain sensor models: the GNSS receiver and the barometric
//! altimeter. Both read truth because they are the sensors' physics; only
//! their readings reach a filter.

use glam::DVec3;
use navigate_contract::{GeodeticPosition, NedVelocity, SymmetricCov3};
use sim_core::VehicleState;
use sim_devices::WhiteNoise;
use sim_geo::{LocalFrame, ecef_to_geodetic};

use super::{BaroSpec, GnssSpec, TrialError};

#[cfg(test)]
mod tests;

/// One GNSS fix with the covariances the receiver declares.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GnssFix {
    /// Measured position of the antenna, which sits at the centre of
    /// gravity.
    pub position: GeodeticPosition,
    /// Declared position covariance, NED m²: white plus correlated
    /// variance per axis, the error's marginal spread.
    pub position_cov: SymmetricCov3,
    /// Measured velocity, NED.
    pub velocity: NedVelocity,
    /// Declared velocity covariance, NED (m/s)².
    pub velocity_cov: SymmetricCov3,
}

/// A GNSS receiver in the position domain: each fix is the true position
/// plus white noise plus a first-order Gauss–Markov error per NED axis,
/// and the true velocity plus white noise. No constellation, pseudorange,
/// geometry or multipath is modelled.
///
/// The correlated error runs at the fix rate from a draw of its stationary
/// spread, so every caller of the same seed and rate sees the same error
/// sequence whatever is later withheld from a filter.
#[derive(Debug, Clone)]
pub struct GnssReceiver {
    spec: GnssSpec,
    noise: WhiteNoise,
    correlated_m: Option<DVec3>,
}

impl GnssReceiver {
    /// A receiver drawing its errors from `seed`.
    pub fn new(spec: GnssSpec, seed: u64) -> Result<Self, TrialError> {
        let sigmas = [
            spec.horizontal_white_sigma_m,
            spec.vertical_white_sigma_m,
            spec.horizontal_correlated_sigma_m,
            spec.vertical_correlated_sigma_m,
            spec.velocity_sigma_m_s,
        ];
        let usable = sigmas.iter().all(|s| s.is_finite() && *s >= 0.0)
            && spec.correlation_time_s > 0.0
            && spec.rate_hz > 0.0
            && spec
                .horizontal_white_sigma_m
                .hypot(spec.horizontal_correlated_sigma_m)
                > 0.0
            && spec
                .vertical_white_sigma_m
                .hypot(spec.vertical_correlated_sigma_m)
                > 0.0
            && spec.velocity_sigma_m_s > 0.0;
        if !usable {
            return Err(TrialError::InvalidRequest {
                field: "gnss",
                detail: format!("{spec:?}"),
            });
        }
        Ok(Self {
            spec,
            noise: WhiteNoise::from_seed(seed),
            correlated_m: None,
        })
    }

    /// Horizontal 1-sigma of the simulated position error, metres: the root
    /// sum of both horizontal axes' white and correlated variances.
    pub fn horizontal_sigma_m(spec: &GnssSpec) -> f64 {
        (2.0 * (spec.horizontal_white_sigma_m.powi(2) + spec.horizontal_correlated_sigma_m.powi(2)))
            .sqrt()
    }

    /// The fix for the vehicle in `truth`, one fix period after the last.
    pub fn measure(&mut self, truth: &VehicleState) -> GnssFix {
        let spec = self.spec;
        let correlated_sigma = DVec3::new(
            spec.horizontal_correlated_sigma_m,
            spec.horizontal_correlated_sigma_m,
            spec.vertical_correlated_sigma_m,
        );
        let white_sigma = DVec3::new(
            spec.horizontal_white_sigma_m,
            spec.horizontal_white_sigma_m,
            spec.vertical_white_sigma_m,
        );
        let draw = self.noise.next_vec3(correlated_sigma);
        let correlated = match self.correlated_m {
            None => draw,
            Some(previous) => {
                let phi = (-1.0 / (spec.rate_hz * spec.correlation_time_s)).exp();
                previous * phi + draw * (1.0 - phi * phi).sqrt()
            }
        };
        self.correlated_m = Some(correlated);
        let error_ned = correlated + self.noise.next_vec3(white_sigma);
        let velocity_error = self.noise.next_vec3(DVec3::splat(spec.velocity_sigma_m_s));

        let frame = LocalFrame::at(&ecef_to_geodetic(truth.position_ecef));
        let measured = ecef_to_geodetic(truth.position_ecef + frame.ned_to_ecef() * error_ned);
        let velocity = frame.ecef_to_ned_vector(truth.velocity_ecef) + velocity_error;
        let variance = white_sigma * white_sigma + correlated_sigma * correlated_sigma;
        let velocity_variance = spec.velocity_sigma_m_s.powi(2);
        GnssFix {
            position: GeodeticPosition::new(measured.lat_rad, measured.lon_rad, measured.height_m),
            position_cov: SymmetricCov3::from_diagonal(variance.x, variance.y, variance.z),
            velocity: NedVelocity::new(velocity.x, velocity.y, velocity.z),
            velocity_cov: SymmetricCov3::from_diagonal(
                velocity_variance,
                velocity_variance,
                velocity_variance,
            ),
        }
    }
}

/// One barometric reading: ellipsoidal height with its white-noise
/// variance. The bias is not in the variance because Navigate's filter
/// carries it as a state, configured from the same [`BaroSpec`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BaroReading {
    /// Measured height above the ellipsoid, metres.
    pub altitude_m: f64,
    /// Declared white-noise variance, m².
    pub variance_m2: f64,
}

/// A barometric altimeter reporting ellipsoidal height plus a first-order
/// Gauss–Markov bias plus white noise.
///
/// It stands in for the BMP390 device model, which converts pressure to
/// counts with no noise or bias: through an ideal standard atmosphere its
/// pressure altitude would be the true height, which no altimeter reads.
#[derive(Debug, Clone)]
pub struct BaroAltimeter {
    spec: BaroSpec,
    noise: WhiteNoise,
    bias_m: Option<f64>,
}

impl BaroAltimeter {
    /// An altimeter drawing its errors from `seed`.
    pub fn new(spec: BaroSpec, seed: u64) -> Result<Self, TrialError> {
        let usable = spec.rate_hz > 0.0
            && spec.white_sigma_m > 0.0
            && spec.bias_sigma_m >= 0.0
            && spec.bias_correlation_time_s > 0.0;
        if !usable {
            return Err(TrialError::InvalidRequest {
                field: "baro",
                detail: format!("{spec:?}"),
            });
        }
        Ok(Self {
            spec,
            noise: WhiteNoise::from_seed(seed),
            bias_m: None,
        })
    }

    /// The reading for the vehicle in `truth`, one period after the last.
    pub fn measure(&mut self, truth: &VehicleState) -> BaroReading {
        let spec = self.spec;
        let draw = self.noise.next_gaussian() * spec.bias_sigma_m;
        let bias = match self.bias_m {
            None => draw,
            Some(previous) => {
                let phi = (-1.0 / (spec.rate_hz * spec.bias_correlation_time_s)).exp();
                previous * phi + draw * (1.0 - phi * phi).sqrt()
            }
        };
        self.bias_m = Some(bias);
        let white = self.noise.next_gaussian() * spec.white_sigma_m;
        BaroReading {
            altitude_m: ecef_to_geodetic(truth.position_ecef).height_m + bias + white,
            variance_m2: spec.white_sigma_m.powi(2),
        }
    }
}
