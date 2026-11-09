use glam::{DMat3, DVec3};

use crate::{DeviceError, WhiteNoise};

mod grade;
#[cfg(test)]
mod tests;

pub use grade::ImuGrade;

/// First-order Gauss–Markov bias: `ḃ = −b/τ + w`, with stationary 1-sigma
/// `sigma`. A `tau_s` that is not finite and positive holds the bias constant
/// at its turn-on draw, a random constant.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GaussMarkov {
    /// Correlation time, seconds.
    pub tau_s: f64,
    /// Stationary 1-sigma, in the unit of the sensed quantity.
    pub sigma: f64,
}

impl GaussMarkov {
    /// No bias at all.
    pub const NONE: Self = Self {
        tau_s: 0.0,
        sigma: 0.0,
    };

    fn correlates(&self) -> bool {
        self.tau_s.is_finite() && self.tau_s > 0.0
    }
}

/// Error statistics of one sensor triad: three orthogonal accelerometers in
/// m/s², or three gyroscopes in rad/s, every axis independent with the same
/// statistics.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TriadErrorSpec {
    /// White noise density, unit/√Hz: velocity or angle random walk. One
    /// increment over `dt` carries `noise_density·√dt` of it, the variance a
    /// filter with this continuous density assumes.
    pub noise_density: f64,
    /// In-run bias; its turn-on value is a draw from the stationary
    /// distribution.
    pub bias: GaussMarkov,
    /// Density of a further random walk on the bias, unit/s/√Hz.
    pub bias_random_walk_density: f64,
    /// 1-sigma scale-factor error of each axis, as a fraction.
    pub scale_factor_sigma: f64,
    /// 1-sigma of each of the six cross-axis sensitivities, radians.
    pub misalignment_sigma_rad: f64,
}

impl TriadErrorSpec {
    /// A perfect triad.
    pub const IDEAL: Self = Self {
        noise_density: 0.0,
        bias: GaussMarkov::NONE,
        bias_random_walk_density: 0.0,
        scale_factor_sigma: 0.0,
        misalignment_sigma_rad: 0.0,
    };

    fn validate(&self, triad: &'static str) -> Result<(), DeviceError> {
        let fields = [
            ("noise_density", self.noise_density),
            ("bias.sigma", self.bias.sigma),
            ("bias_random_walk_density", self.bias_random_walk_density),
            ("scale_factor_sigma", self.scale_factor_sigma),
            ("misalignment_sigma_rad", self.misalignment_sigma_rad),
        ];
        for (field, value) in fields {
            if !(value.is_finite() && value >= 0.0) {
                return Err(DeviceError::InvalidErrorModel {
                    triad,
                    field,
                    value,
                });
            }
        }
        if self.bias.tau_s.is_nan() {
            return Err(DeviceError::InvalidErrorModel {
                triad,
                field: "bias.tau_s",
                value: self.bias.tau_s,
            });
        }
        Ok(())
    }
}

/// Error statistics of a six-axis inertial measurement unit.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ImuErrorSpec {
    /// The accelerometers, m/s².
    pub accel: TriadErrorSpec,
    /// The gyroscopes, rad/s.
    pub gyro: TriadErrorSpec,
}

impl ImuErrorSpec {
    /// A unit that reports the motion exactly.
    pub const IDEAL: Self = Self {
        accel: TriadErrorSpec::IDEAL,
        gyro: TriadErrorSpec::IDEAL,
    };
}

/// What an integrating IMU reports over one interval, in its sensor axes.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct InertialIncrement {
    /// Length of the interval, seconds.
    pub dt_s: f64,
    /// Rotation relative to inertial space, radians.
    pub delta_theta_rad: DVec3,
    /// Integral of the specific force, m/s.
    pub delta_velocity_m_s: DVec3,
}

/// The realised errors of one triad: fixed at turn-on except the biases,
/// which wander.
#[derive(Debug, Clone, Copy, PartialEq)]
struct TriadState {
    sensitivity: DMat3,
    markov: DVec3,
    walk: DVec3,
}

impl TriadState {
    fn turn_on(spec: &TriadErrorSpec, noise: &mut WhiteNoise) -> Self {
        let scale = noise.next_vec3(DVec3::splat(spec.scale_factor_sigma));
        let upper = noise.next_vec3(DVec3::splat(spec.misalignment_sigma_rad));
        let lower = noise.next_vec3(DVec3::splat(spec.misalignment_sigma_rad));
        let sensitivity = DMat3::from_cols(
            DVec3::new(1.0 + scale.x, lower.x, lower.y),
            DVec3::new(upper.x, 1.0 + scale.y, lower.z),
            DVec3::new(upper.y, upper.z, 1.0 + scale.z),
        );
        Self {
            sensitivity,
            markov: noise.next_vec3(DVec3::splat(spec.bias.sigma)),
            walk: DVec3::ZERO,
        }
    }

    fn bias(&self) -> DVec3 {
        self.markov + self.walk
    }

    /// One interval's reading of `ideal`, then the bias moves on to the next
    /// interval by the exact discrete form of its model.
    fn sense(
        &mut self,
        spec: &TriadErrorSpec,
        ideal: DVec3,
        dt_s: f64,
        noise: &mut WhiteNoise,
    ) -> DVec3 {
        let white = noise.next_vec3(DVec3::splat(spec.noise_density * dt_s.sqrt()));
        let sensed = self.sensitivity * ideal + self.bias() * dt_s + white;
        let markov_step = noise.next_vec3(DVec3::ONE);
        let walk_step = noise.next_vec3(DVec3::ONE);
        if spec.bias.correlates() {
            let decay = (-dt_s / spec.bias.tau_s).exp();
            let drive = spec.bias.sigma * (1.0 - decay * decay).sqrt();
            self.markov = self.markov * decay + markov_step * drive;
        }
        self.walk += walk_step * (spec.bias_random_walk_density * dt_s.sqrt());
        sensed
    }
}

/// Seeded errors of a six-axis IMU applied to ideal increments: scale factor
/// and misalignment, Gauss–Markov and random-walk biases, and white noise.
///
/// Every random draw comes from one [`WhiteNoise`] stream in a fixed order,
/// and the whole model is `Copy`, so a copy is a snapshot that resumes the
/// stream bit-identically.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ImuErrorModel {
    spec: ImuErrorSpec,
    accel: TriadState,
    gyro: TriadState,
    noise: WhiteNoise,
}

impl ImuErrorModel {
    /// A unit at turn-on, its fixed errors and first biases drawn from
    /// `seed`. A negative or non-finite statistic is an error.
    pub fn new(spec: ImuErrorSpec, seed: u64) -> Result<Self, DeviceError> {
        spec.accel.validate("accel")?;
        spec.gyro.validate("gyro")?;
        let mut noise = WhiteNoise::from_seed(seed);
        let accel = TriadState::turn_on(&spec.accel, &mut noise);
        let gyro = TriadState::turn_on(&spec.gyro, &mut noise);
        Ok(Self {
            spec,
            accel,
            gyro,
            noise,
        })
    }

    /// The statistics this unit draws from.
    pub const fn spec(&self) -> &ImuErrorSpec {
        &self.spec
    }

    /// The accelerometer bias the next interval carries, m/s².
    pub fn accel_bias_m_s2(&self) -> DVec3 {
        self.accel.bias()
    }

    /// The gyroscope bias the next interval carries, rad/s.
    pub fn gyro_bias_rad_s(&self) -> DVec3 {
        self.gyro.bias()
    }

    /// What the unit reports for the true motion `ideal`. A non-finite
    /// increment or an interval that is not finite and positive is an error.
    pub fn sense(&mut self, ideal: &InertialIncrement) -> Result<InertialIncrement, DeviceError> {
        let dt_s = ideal.dt_s;
        if !(dt_s.is_finite()
            && dt_s > 0.0
            && ideal.delta_theta_rad.is_finite()
            && ideal.delta_velocity_m_s.is_finite())
        {
            return Err(DeviceError::InvalidIncrement { dt_s });
        }
        let noise = &mut self.noise;
        let delta_velocity_m_s =
            self.accel
                .sense(&self.spec.accel, ideal.delta_velocity_m_s, dt_s, noise);
        let delta_theta_rad = self
            .gyro
            .sense(&self.spec.gyro, ideal.delta_theta_rad, dt_s, noise);
        Ok(InertialIncrement {
            dt_s,
            delta_theta_rad,
            delta_velocity_m_s,
        })
    }
}
