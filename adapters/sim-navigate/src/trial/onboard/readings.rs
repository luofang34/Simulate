//! What the navigator's host makes of the devices' samples before handing
//! them over: a receiver fix becomes Navigate's fix input as it is, and the
//! static pressure becomes pressure altitude through the ICAO standard
//! atmosphere, the reference a barometric altimeter set to standard holds.

use navigate_contract::{GeodeticPosition, NedVelocity, SymmetricCov3};
use navigate_fusion::FogmBias;
use navigate_onboard::{ErrorBudget, SensorData};
use sim_atmosphere::{geometric_to_geopotential_m, isa_at_geopotential};
use sim_devices::{AirDataSample, GnssReading};

use crate::trial::{BaroSpec, TrialError};

#[cfg(test)]
mod tests;

/// Standard gravity of the ICAO atmosphere's hydrostatic relation, m/s².
const STANDARD_GRAVITY_M_S2: f64 = 9.806_65;
/// Pressure altitude is found to this, metres.
const ALTITUDE_TOLERANCE_M: f64 = 1e-9;
/// Secant steps before an unreadable pressure is refused.
const MAX_ITERATIONS: usize = 60;

/// Navigate's fix input for one receiver fix, with the covariances the
/// receiver declares.
pub(crate) fn gnss_fix_data(fix: &GnssReading) -> SensorData {
    let p = fix.position;
    let sigma = fix.position_sigma_ned_m;
    let velocity = fix.velocity_ned_m_s;
    let velocity_variance = fix.velocity_sigma_m_s * fix.velocity_sigma_m_s;
    SensorData::GnssFix {
        position: GeodeticPosition::new(p.lat_rad, p.lon_rad, p.height_m),
        velocity: Some(NedVelocity::new(velocity.x, velocity.y, velocity.z)),
        position_cov: Some(SymmetricCov3::from_diagonal(
            sigma.x * sigma.x,
            sigma.y * sigma.y,
            sigma.z * sigma.z,
        )),
        velocity_cov: Some(SymmetricCov3::from_diagonal(
            velocity_variance,
            velocity_variance,
            velocity_variance,
        )),
    }
}

/// Navigate's barometric input for one air-data sample: the pressure
/// altitude of its static pressure.
pub(crate) fn baro_altitude_data(sample: &AirDataSample) -> Result<SensorData, TrialError> {
    Ok(SensorData::BaroAltitude {
        altitude_m: pressure_altitude_m(sample.static_pressure_pa)?,
    })
}

/// The height at which the ICAO standard atmosphere has `pressure_pa`,
/// metres: the standard atmosphere's pressure inverted by secant steps on
/// its logarithm, which is nearly linear in height.
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
