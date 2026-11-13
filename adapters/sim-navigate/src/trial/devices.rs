//! The vehicle's position-domain and air-data devices: the shared
//! `sim_devices` GNSS receiver, static port, and forward-axis pitot-static
//! probe behind the air-data boundary, the same device models every consumer
//! aboard a simulated vehicle takes. They read truth because that is their
//! physics; only their samples leave.
//!
//! The air is the standard atmosphere referenced to mean sea level through
//! the EGM96 geoid, still and without weather: the trial models no
//! pressure-altitude error other than the static port's own.

use glam::DVec3;
use sim_atmosphere::{AtmosphereField, GeoidStandardAtmosphere};
use sim_core::VehicleState;
use sim_devices::{
    AirDataSample, AirDataSystem, AirFlow, GaussMarkov, GnssErrorSpec, GnssReceiver, PitotStatic,
    PressureErrorSpec,
};
use sim_geo::{UtcTime, ecef_to_geodetic};

use super::epoch::parse_utc;
use super::{BaroSpec, GnssSpec, TrialError, TrialRequest};

#[cfg(test)]
mod tests;

/// The device statistics of a request's receiver.
pub(super) const fn gnss_error_spec(spec: &GnssSpec) -> GnssErrorSpec {
    GnssErrorSpec {
        horizontal_white_sigma_m: spec.horizontal_white_sigma_m,
        vertical_white_sigma_m: spec.vertical_white_sigma_m,
        horizontal_correlated_sigma_m: spec.horizontal_correlated_sigma_m,
        vertical_correlated_sigma_m: spec.vertical_correlated_sigma_m,
        correlation_time_s: spec.correlation_time_s,
        velocity_sigma_m_s: spec.velocity_sigma_m_s,
    }
}

/// The device statistics of a request's static port.
pub(super) const fn static_port_spec(spec: &BaroSpec) -> PressureErrorSpec {
    PressureErrorSpec {
        white_sigma_pa: spec.white_sigma_pa,
        bias: GaussMarkov {
            tau_s: spec.bias_correlation_time_s,
            sigma: spec.bias_sigma_pa,
        },
    }
}

/// The receiver of `request`, fixing once per period from the GNSS seed.
///
/// A fix the filter could not weigh is refused here although the device
/// would produce it: the filter weighs each fix by the white spread alone,
/// so that and the velocity spread must be positive.
pub(super) fn receiver(request: &TrialRequest) -> Result<GnssReceiver, TrialError> {
    let spec = request.gnss;
    let weighable = spec.rate_hz > 0.0
        && spec.horizontal_white_sigma_m > 0.0
        && spec.vertical_white_sigma_m > 0.0
        && spec.velocity_sigma_m_s > 0.0;
    let invalid = |detail: String| TrialError::InvalidRequest {
        field: "gnss",
        detail,
    };
    if !weighable {
        return Err(invalid(format!("{spec:?}")));
    }
    GnssReceiver::new(gnss_error_spec(&spec), request.seeds.gnss)
        .map_err(|source| invalid(source.to_string()))
}

/// A static port on its own pressure sensor, sampled in the trial's air.
#[derive(Debug, Clone)]
pub(super) struct StaticPort {
    port: PitotStatic,
    start: UtcTime,
}

impl StaticPort {
    /// The static port of `request`, drawing its errors from the baro seed.
    pub(super) fn new(request: &TrialRequest) -> Result<Self, TrialError> {
        let spec = request.baro;
        let invalid = |detail: String| TrialError::InvalidRequest {
            field: "baro",
            detail,
        };
        let usable = spec.rate_hz > 0.0
            && spec.white_sigma_pa > 0.0
            && spec.bias_sigma_pa >= 0.0
            && spec.bias_correlation_time_s > 0.0;
        if !usable {
            return Err(invalid(format!("{spec:?}")));
        }
        let port = PitotStatic::static_only(static_port_spec(&spec), request.seeds.baro)
            .map_err(|source| invalid(source.to_string()))?;
        Ok(Self {
            port,
            start: parse_utc(&request.epoch_utc)?,
        })
    }

    /// The sample for the vehicle in `truth` at route time `time_s`,
    /// `period_s` after the last.
    pub(super) fn sense(
        &mut self,
        truth: &VehicleState,
        time_s: f64,
        period_s: f64,
    ) -> Result<AirDataSample, TrialError> {
        let position = ecef_to_geodetic(truth.position_ecef);
        let air = GeoidStandardAtmosphere::new()
            .sample(&position, self.start.advanced_by(time_s))
            .map_err(|source| TrialError::InvalidRequest {
                field: "baro",
                detail: format!("no air at the vehicle: {source}"),
            })?;
        Ok(self.port.sense(&AirFlow::of(&air, truth), period_s))
    }
}

/// Distinct stream offset for the pitot-static probe so [`StaticPort`]'s
/// barometric PRNG sequence stays bit-identical.
const PITOT_SEED_OFFSET: u64 = 0x5049_544F_5431_5631;

/// Forward-looking pitot-static probe sampled in the trial's air for the
/// closed-loop autopilot's airspeed hold.
#[derive(Debug, Clone)]
pub(super) struct PitotProbe {
    probe: PitotStatic,
    start: UtcTime,
}

impl PitotProbe {
    /// Builds a C172-class forward-axis (`+X` body) pitot-static probe seeded
    /// from `request.seeds.baro` on an independent PRNG stream.
    pub(super) fn new(request: &TrialRequest) -> Result<Self, TrialError> {
        let static_spec = static_port_spec(&request.baro);
        let pitot_spec = PressureErrorSpec {
            white_sigma_pa: request.baro.white_sigma_pa.max(1e-6),
            bias: GaussMarkov {
                tau_s: request.baro.bias_correlation_time_s.max(0.1),
                sigma: (0.5 * request.baro.bias_sigma_pa).max(0.0),
            },
        };
        let seed = request.seeds.baro.wrapping_add(PITOT_SEED_OFFSET);
        let probe =
            PitotStatic::with_pitot(static_spec, pitot_spec, DVec3::X, seed).map_err(|source| {
                TrialError::InvalidRequest {
                    field: "pitot",
                    detail: source.to_string(),
                }
            })?;
        Ok(Self {
            probe,
            start: parse_utc(&request.epoch_utc)?,
        })
    }

    /// Samples static pressure, static temperature, and pitot differential
    /// pressure (`Pa`) at `time_s`, `period_s` after the last sample.
    pub(super) fn sense(
        &mut self,
        truth: &VehicleState,
        time_s: f64,
        period_s: f64,
    ) -> Result<AirDataSample, TrialError> {
        let position = ecef_to_geodetic(truth.position_ecef);
        let air = GeoidStandardAtmosphere::new()
            .sample(&position, self.start.advanced_by(time_s))
            .map_err(|source| TrialError::InvalidRequest {
                field: "pitot",
                detail: format!("no air at the vehicle: {source}"),
            })?;
        Ok(self.probe.sense(&AirFlow::of(&air, truth), period_s))
    }
}
