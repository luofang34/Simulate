use std::f64::consts::PI;

use super::{GaussMarkov, ImuErrorSpec, TriadErrorSpec};
use crate::icm42688::{ACCEL_NOISE_DENSITY, GYRO_NOISE_DENSITY, STANDARD_GRAVITY_M_S2};

const RAD_PER_DEG: f64 = PI / 180.0;
const SECONDS_PER_HOUR: f64 = 3_600.0;
/// `1 unit/√h` in `unit/√s`.
const PER_SQRT_HOUR: f64 = 1.0 / 60.0;
/// Correlation time every preset declares for its in-run bias. None of the
/// sources gives one; an hour keeps the bias a near constant over a flight
/// segment, which is how a turn-on bias behaves.
const BIAS_TAU_S: f64 = SECONDS_PER_HOUR;

/// Broad performance class of an inertial measurement unit.
///
/// The presets carry noise and bias only. Scale-factor and misalignment
/// errors stay zero because Navigate's inertial filter has no states for
/// them; set them on the [`ImuErrorSpec`] to study that mismodelling.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum ImuGrade {
    /// TDK ICM-42688-P as shipped: datasheet noise densities of 70 µg/√Hz
    /// and 0.0028 °/s/√Hz, and its typical initial offset tolerances of
    /// 20 mg and 0.5 °/s taken as the 1-sigma turn-on bias.
    ConsumerMems,
    /// A typical tactical unit: 1 mg and 1 °/h bias, 0.05 m/s/√h velocity
    /// random walk and 0.1 °/√h angle random walk.
    Tactical,
    /// Navigate's own default inertial model: 25 µg and 0.01 °/h bias,
    /// 1e-4 (m/s²)/√Hz and 5.8e-7 (rad/s)/√Hz noise.
    Navigation,
}

impl ImuGrade {
    /// Every grade, best last.
    pub const ALL: [Self; 3] = [Self::ConsumerMems, Self::Tactical, Self::Navigation];

    /// The error statistics of this grade.
    pub const fn spec(self) -> ImuErrorSpec {
        let (accel_density, accel_bias, gyro_density, gyro_bias) = match self {
            Self::ConsumerMems => (
                ACCEL_NOISE_DENSITY,
                20e-3 * STANDARD_GRAVITY_M_S2,
                GYRO_NOISE_DENSITY,
                0.5 * RAD_PER_DEG,
            ),
            Self::Tactical => (
                0.05 * PER_SQRT_HOUR,
                1e-3 * STANDARD_GRAVITY_M_S2,
                0.1 * RAD_PER_DEG * PER_SQRT_HOUR,
                RAD_PER_DEG / SECONDS_PER_HOUR,
            ),
            Self::Navigation => (1e-4, 2.5e-4, 5.8e-7, 4.85e-8),
        };
        ImuErrorSpec {
            accel: triad(accel_density, accel_bias),
            gyro: triad(gyro_density, gyro_bias),
        }
    }
}

const fn triad(noise_density: f64, bias_sigma: f64) -> TriadErrorSpec {
    TriadErrorSpec {
        noise_density,
        bias: GaussMarkov {
            tau_s: BIAS_TAU_S,
            sigma: bias_sigma,
        },
        ..TriadErrorSpec::IDEAL
    }
}
