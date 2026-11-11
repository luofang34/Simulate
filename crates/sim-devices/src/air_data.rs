//! The air-data system: pressure sensors on the static port and the pitot
//! probe. They report pressures, not altitude or airspeed; each consumer
//! converts with the reference it holds, and a wrong reference is its own
//! error.

use glam::DVec3;
use sim_atmosphere::AtmosphereSample;
use sim_core::VehicleState;
use sim_geo::{LocalFrame, ecef_to_geodetic};

use crate::{DeviceError, GaussMarkov, WhiteNoise};

#[cfg(test)]
mod tests;

/// Ratio of specific heats of air.
const GAMMA: f64 = 1.4;
/// Specific gas constant of dry air, J/(kg·K).
const GAS_CONSTANT_J_KG_K: f64 = 287.052_87;

/// The undisturbed air at the vehicle and the vehicle's motion through it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AirFlow {
    /// Static pressure of the air, pascals.
    pub static_pressure_pa: f64,
    /// Static temperature of the air, kelvin.
    pub static_temperature_k: f64,
    /// Velocity of the vehicle relative to the air, body forward-right-down
    /// axes, m/s: its velocity over the ground less the wind.
    pub air_velocity_body_m_s: DVec3,
}

impl AirFlow {
    /// The flow a vehicle in `state` meets in `air`.
    pub fn of(air: &AtmosphereSample, state: &VehicleState) -> Self {
        let frame = LocalFrame::at(&ecef_to_geodetic(state.position_ecef));
        let wind_ecef = frame.ned_to_ecef() * air.wind_ned_m_s;
        Self {
            static_pressure_pa: air.pressure_pa,
            static_temperature_k: air.temperature_k,
            air_velocity_body_m_s: state.attitude.inverse() * (state.velocity_ecef - wind_ecef),
        }
    }
}

/// Pressure a pitot probe reads above static in subsonic flow at
/// `speed_m_s`, pascals: the isentropic stagnation rise.
pub fn impact_pressure_pa(
    static_pressure_pa: f64,
    static_temperature_k: f64,
    speed_m_s: f64,
) -> f64 {
    let sound_squared = GAMMA * GAS_CONSTANT_J_KG_K * static_temperature_k;
    let mach_squared = speed_m_s * speed_m_s / sound_squared;
    let ratio = (1.0 + 0.5 * (GAMMA - 1.0) * mach_squared).powf(GAMMA / (GAMMA - 1.0));
    static_pressure_pa * (ratio - 1.0)
}

/// The true airspeed at which a pitot probe reads `impact_pa` above static,
/// m/s: the inverse of [`impact_pressure_pa`]. A reading below static gives
/// zero.
pub fn speed_from_impact_pressure_m_s(
    static_pressure_pa: f64,
    static_temperature_k: f64,
    impact_pa: f64,
) -> f64 {
    let ratio = (impact_pa.max(0.0) / static_pressure_pa + 1.0).powf((GAMMA - 1.0) / GAMMA);
    let mach_squared = 2.0 * (ratio - 1.0) / (GAMMA - 1.0);
    (mach_squared * GAMMA * GAS_CONSTANT_J_KG_K * static_temperature_k).sqrt()
}

/// Error statistics of one pressure sensor.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PressureErrorSpec {
    /// White noise of one sample, pascals.
    pub white_sigma_pa: f64,
    /// Slowly varying offset, pascals.
    pub bias: GaussMarkov,
}

impl PressureErrorSpec {
    /// A sensor that reports the pressure exactly.
    pub const IDEAL: Self = Self {
        white_sigma_pa: 0.0,
        bias: GaussMarkov::NONE,
    };

    fn validate(&self, device: &'static str) -> Result<(), DeviceError> {
        let fields = [
            ("white_sigma_pa", self.white_sigma_pa),
            ("bias.sigma", self.bias.sigma),
        ];
        match fields.iter().find(|(_, v)| !(v.is_finite() && *v >= 0.0)) {
            Some(&(field, value)) => Err(DeviceError::InvalidStatistic {
                device,
                field,
                value,
            }),
            None => Ok(()),
        }
    }
}

/// One pressure sensor's realised offset.
#[derive(Debug, Clone, Copy, PartialEq)]
struct PressureSensor {
    spec: PressureErrorSpec,
    bias_pa: Option<f64>,
}

impl PressureSensor {
    /// The reading of `true_pa`, `period_s` after the last: the offset moves
    /// on by the exact discrete form of its model, from a first draw of its
    /// stationary spread.
    fn sense(&mut self, true_pa: f64, period_s: f64, noise: &mut WhiteNoise) -> f64 {
        let draw = noise.next_gaussian() * self.spec.bias.sigma;
        let tau_s = self.spec.bias.tau_s;
        let bias = match self.bias_pa {
            Some(previous) if tau_s.is_finite() && tau_s > 0.0 => {
                let decay = (-period_s / tau_s).exp();
                previous * decay + draw * (1.0 - decay * decay).sqrt()
            }
            Some(previous) => previous,
            None => draw,
        };
        self.bias_pa = Some(bias);
        true_pa + bias + noise.next_gaussian() * self.spec.white_sigma_pa
    }
}

/// What an air-data system reports at one sample.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AirDataSample {
    /// Pressure at the static port, pascals.
    pub static_pressure_pa: f64,
    /// Temperature of the static air, kelvin.
    pub static_temperature_k: f64,
    /// Pitot pressure above static, pascals; `None` on a system without a
    /// pitot probe.
    pub differential_pressure_pa: Option<f64>,
}

/// A vehicle's air-data system: true air and relative flow in, pressure
/// samples out. Whatever designs the system (a probe model here, a system
/// model elsewhere) stays behind this boundary.
pub trait AirDataSystem {
    /// The sample for `flow`, `period_s` after the last.
    fn sense(&mut self, flow: &AirFlow, period_s: f64) -> AirDataSample;
}

/// A static port and, on a vehicle that has one, a pitot probe, each on its
/// own pressure sensor. No position error with incidence or sideslip and no
/// line lag are modelled: the static port reads the undisturbed static
/// pressure and the probe the stagnation rise of the flow along its axis.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PitotStatic {
    static_port: PressureSensor,
    pitot: Option<(DVec3, PressureSensor)>,
    noise: WhiteNoise,
}

impl PitotStatic {
    /// A static port alone: a barometer.
    pub fn static_only(spec: PressureErrorSpec, seed: u64) -> Result<Self, DeviceError> {
        spec.validate("static port")?;
        Ok(Self {
            static_port: PressureSensor {
                spec,
                bias_pa: None,
            },
            pitot: None,
            noise: WhiteNoise::from_seed(seed),
        })
    }

    /// A static port and a pitot probe looking along `axis_body`.
    pub fn with_pitot(
        static_port: PressureErrorSpec,
        pitot: PressureErrorSpec,
        axis_body: DVec3,
        seed: u64,
    ) -> Result<Self, DeviceError> {
        pitot.validate("pitot")?;
        let Some(axis) = axis_body.try_normalize() else {
            return Err(DeviceError::InvalidStatistic {
                device: "pitot",
                field: "axis_body",
                value: axis_body.length(),
            });
        };
        let sensor = PressureSensor {
            spec: pitot,
            bias_pa: None,
        };
        Ok(Self {
            pitot: Some((axis, sensor)),
            ..Self::static_only(static_port, seed)?
        })
    }
}

impl AirDataSystem for PitotStatic {
    fn sense(&mut self, flow: &AirFlow, period_s: f64) -> AirDataSample {
        let noise = &mut self.noise;
        let static_pressure_pa = self
            .static_port
            .sense(flow.static_pressure_pa, period_s, noise);
        let differential_pressure_pa = self.pitot.as_mut().map(|(axis, sensor)| {
            let along_m_s = flow.air_velocity_body_m_s.dot(*axis).max(0.0);
            let impact = impact_pressure_pa(
                flow.static_pressure_pa,
                flow.static_temperature_k,
                along_m_s,
            );
            sensor.sense(impact, period_s, noise)
        });
        AirDataSample {
            static_pressure_pa,
            static_temperature_k: flow.static_temperature_k,
            differential_pressure_pa,
        }
    }
}
