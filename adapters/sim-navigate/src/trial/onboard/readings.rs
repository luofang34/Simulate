//! What the navigator's host makes of the devices' samples before handing
//! them over: a receiver fix becomes Navigate's fix input under the
//! receiver's declared error budget, and the static pressure becomes ellipsoidal height. The pressure gives pressure
//! altitude above mean sea level through the ICAO standard atmosphere, the
//! reference a barometric altimeter set to standard holds; the EGM96 geoid
//! undulation under the host's own position estimate lifts it to the
//! ellipsoid Navigate works on.

use navigate_contract::{GeodeticPosition, NedVelocity};
use navigate_fusion::FogmBias;
use navigate_onboard::{ErrorBudget, GnssCorrelatedError, SensorData};
use sim_atmosphere::{geometric_to_geopotential_m, isa_at_geopotential};
use sim_devices::{AirDataSample, GnssErrorSpec, GnssReading};
use sim_geo::egm96_undulation_m;

use crate::trial::{BaroSpec, TrialError};

#[cfg(test)]
mod tests;

/// Standard gravity of the ICAO atmosphere's hydrostatic relation, m/s².
const STANDARD_GRAVITY_M_S2: f64 = 9.806_65;
/// Pressure altitude is found to this, metres.
const ALTITUDE_TOLERANCE_M: f64 = 1e-9;
/// Secant steps before an unreadable pressure is refused.
const MAX_ITERATIONS: usize = 60;

/// Navigate's fix input for one receiver fix. It carries no covariance:
/// the receiver's spread is declared once in [`gnss_budget`], whose white
/// part weighs each fix while the filter tracks the correlated part, and a
/// per-fix spread of both would count the correlated error twice.
pub(crate) fn gnss_fix_data(fix: &GnssReading) -> SensorData {
    let p = fix.position;
    let velocity = fix.velocity_ned_m_s;
    SensorData::GnssFix {
        position: GeodeticPosition::new(p.lat_rad, p.lon_rad, p.height_m),
        velocity: Some(NedVelocity::new(velocity.x, velocity.y, velocity.z)),
        position_cov: None,
        velocity_cov: None,
    }
}

/// The receiver's error budget as Navigate takes it: the white position and
/// velocity noise, and the first-order Gauss–Markov position error that
/// successive fixes share, or none when the receiver has no correlated
/// error.
pub(crate) fn gnss_budget(spec: &GnssErrorSpec) -> ErrorBudget {
    let correlated = (spec.horizontal_correlated_sigma_m > 0.0
        || spec.vertical_correlated_sigma_m > 0.0)
        .then_some(GnssCorrelatedError {
            tau_s: spec.correlation_time_s,
            horizontal_sigma_m: spec.horizontal_correlated_sigma_m,
            vertical_sigma_m: spec.vertical_correlated_sigma_m,
        });
    ErrorBudget::Gnss {
        horizontal_sigma_m: spec.horizontal_white_sigma_m,
        vertical_sigma_m: spec.vertical_white_sigma_m,
        velocity_sigma_m_s: spec.velocity_sigma_m_s,
        correlated,
    }
}

/// Navigate's barometric input for one air-data sample taken at latitude
/// `lat_rad` and longitude `lon_rad` as the host estimates them: the
/// pressure altitude of its static pressure, above the ellipsoid.
///
/// The undulation varies by metres over tens of kilometres, so a host
/// position kilometres off moves the height by well under the altimeter's
/// noise.
pub(crate) fn baro_altitude_data(
    sample: &AirDataSample,
    lat_rad: f64,
    lon_rad: f64,
) -> Result<SensorData, TrialError> {
    Ok(SensorData::BaroAltitude {
        altitude_m: pressure_altitude_m(sample.static_pressure_pa)?
            + egm96_undulation_m(lat_rad, lon_rad),
    })
}

/// The height above mean sea level at which the ICAO standard atmosphere
/// has `pressure_pa`, metres: the standard atmosphere's pressure inverted
/// by secant steps on its logarithm, which is nearly linear in height.
pub(crate) fn pressure_altitude_m(pressure_pa: f64) -> Result<f64, TrialError> {
    let unreadable = |detail: String| TrialError::InvalidRequest {
        field: "baro",
        detail: format!("static pressure {pressure_pa} Pa: {detail}"),
    };
    if !(pressure_pa.is_finite() && pressure_pa > 0.0) {
        return Err(unreadable("not a pressure".into()));
    }
    let target = pressure_pa.ln();
    let residual = |height_m: f64| -> Result<f64, TrialError> {
        isa_at_geopotential(geometric_to_geopotential_m(height_m))
            .map(|state| state.pressure_pa.ln() - target)
            .map_err(|source| unreadable(source.to_string()))
    };
    let (mut a, mut b) = (0.0, 1_000.0);
    let (mut ra, mut rb) = (residual(a)?, residual(b)?);
    for _ in 0..MAX_ITERATIONS {
        if (b - a).abs() < ALTITUDE_TOLERANCE_M || rb == 0.0 {
            return Ok(b);
        }
        let next = b - rb * (b - a) / (rb - ra);
        (a, ra) = (b, rb);
        b = next;
        rb = residual(b)?;
    }
    Err(unreadable("pressure altitude did not converge".into()))
}

/// The altimeter's declared error budget in metres: the static port's
/// pressure statistics through the standard atmosphere's pressure gradient
/// at `height_m`.
///
/// Engineering choice: the budget is declared once at the planned height,
/// so far above or below it the declared spread is off by the change of
/// air density (about a fifth per 2000 m).
pub(crate) fn altimeter_budget(baro: &BaroSpec, height_m: f64) -> Result<ErrorBudget, TrialError> {
    let state = isa_at_geopotential(geometric_to_geopotential_m(height_m)).map_err(|source| {
        TrialError::InvalidRequest {
            field: "baro",
            detail: format!("no standard air at {height_m} m: {source}"),
        }
    })?;
    let metres_per_pa = 1.0 / (state.density_kg_m3 * STANDARD_GRAVITY_M_S2);
    Ok(ErrorBudget::Barometer {
        white_sigma_m: baro.white_sigma_pa * metres_per_pa,
        bias: FogmBias::new(
            baro.bias_correlation_time_s,
            baro.bias_sigma_pa * metres_per_pa,
        ),
    })
}
