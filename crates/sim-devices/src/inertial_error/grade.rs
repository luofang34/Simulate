use std::f64::consts::PI;

use super::{GaussMarkov, ImuErrorSpec, TriadErrorSpec};
use crate::icm42688::{ACCEL_NOISE_DENSITY, GYRO_NOISE_DENSITY, STANDARD_GRAVITY_M_S2};

const RAD_PER_DEG: f64 = PI / 180.0;
const SECONDS_PER_HOUR: f64 = 3_600.0;
/// `1 unit/√h` in `unit/√s`.
const PER_SQRT_HOUR: f64 = 1.0 / 60.0;
/// Correlation time the tactical and navigation presets declare for their
/// bias. Neither source gives one; an hour keeps the bias a near constant
/// over a flight segment.
const BIAS_TAU_S: f64 = SECONDS_PER_HOUR;
/// The ICM-42688-P's gyroscope zero-rate output initial tolerance, ±0.5 °/s
/// in TDK's datasheet (DS-000347), taken as the 1-sigma turn-on bias.
const ICM42688_GYRO_TURN_ON_DEG_S: f64 = 0.5;
/// Its accelerometer zero-g initial tolerance, ±20 mg in the same
/// datasheet, taken as the 1-sigma turn-on bias.
const ICM42688_ACCEL_TURN_ON_G: f64 = 20e-3;
/// In-run gyroscope bias instability, °/h, 1-sigma: assumed. The datasheet
/// states none; a few °/h is typical of the class.
const CONSUMER_GYRO_INSTABILITY_DEG_H: f64 = 5.0;
/// In-run accelerometer bias instability, g, 1-sigma: assumed, as above.
const CONSUMER_ACCEL_INSTABILITY_G: f64 = 0.1e-3;
/// Correlation time of the consumer part's in-run bias, seconds: assumed,
/// the averaging time where such parts reach their bias instability.
const CONSUMER_INSTABILITY_TAU_S: f64 = 300.0;
/// Accelerometer turn-on bias left by a level calibration, g, 1-sigma: an
/// assumed value, not a measured one.
const CALIBRATED_ACCEL_BIAS_G: f64 = 5e-3;

/// Broad performance class of an inertial measurement unit.
///
/// The presets carry noise and bias only. Scale-factor and misalignment
/// errors stay zero because Navigate's inertial filter has no states for
/// them; set them on the [`ImuErrorSpec`] to study that mismodelling.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum ImuGrade {
    /// TDK ICM-42688-P as shipped: datasheet noise densities of 70 µg/√Hz
    /// and 0.0028 °/s/√Hz, its initial offset tolerances of 20 mg and
    /// 0.5 °/s taken as the 1-sigma turn-on bias, constant for a power-up,
    /// and an assumed in-run instability of 0.1 mg and 5 °/h on top.
    ConsumerMems,
    /// The same part after a level calibration on the ground: its
    /// accelerometer turn-on bias reduced to an assumed residual of 5 mg
    /// 1-sigma; noise and gyroscope bias as shipped, since nothing on the
    /// ground calibrates the gyroscopes of an aeroplane started in the air.
    ConsumerMemsCalibrated,
    /// A typical tactical unit: 1 mg and 1 °/h bias, 0.05 m/s/√h velocity
    /// random walk and 0.1 °/√h angle random walk.
    Tactical,
    /// Navigate's own default inertial model: 25 µg and 0.01 °/h bias,
    /// 1e-4 (m/s²)/√Hz and 5.8e-7 (rad/s)/√Hz noise.
    Navigation,
}

impl ImuGrade {
    /// Every device class, best last; [`Self::ConsumerMemsCalibrated`] is a
    /// calibration state of the first.
    pub const ALL: [Self; 3] = [Self::ConsumerMems, Self::Tactical, Self::Navigation];

    /// The error statistics of this grade.
    pub const fn spec(self) -> ImuErrorSpec {
        match self {
            Self::ConsumerMems => consumer(ICM42688_ACCEL_TURN_ON_G),
            Self::ConsumerMemsCalibrated => consumer(CALIBRATED_ACCEL_BIAS_G),
            Self::Tactical => ImuErrorSpec {
                accel: triad(0.05 * PER_SQRT_HOUR, 1e-3 * STANDARD_GRAVITY_M_S2),
                gyro: triad(
                    0.1 * RAD_PER_DEG * PER_SQRT_HOUR,
                    RAD_PER_DEG / SECONDS_PER_HOUR,
                ),
            },
            Self::Navigation => ImuErrorSpec {
                accel: triad(1e-4, 2.5e-4),
                gyro: triad(5.8e-7, 4.85e-8),
            },
        }
    }
}

/// The consumer part's in-run bias of `sigma`.
const fn in_run(sigma: f64) -> GaussMarkov {
    GaussMarkov {
        tau_s: CONSUMER_INSTABILITY_TAU_S,
        sigma,
    }
}

/// The ICM-42688-P with an accelerometer turn-on bias of
/// `accel_turn_on_g` 1-sigma: datasheet noise densities and gyroscope
/// turn-on bias, each turn-on bias a constant for the power-up, and the
/// assumed in-run instabilities wandering on top.
const fn consumer(accel_turn_on_g: f64) -> ImuErrorSpec {
    ImuErrorSpec {
        accel: TriadErrorSpec {
            noise_density: ACCEL_NOISE_DENSITY,
            turn_on_bias_sigma: accel_turn_on_g * STANDARD_GRAVITY_M_S2,
            bias: in_run(CONSUMER_ACCEL_INSTABILITY_G * STANDARD_GRAVITY_M_S2),
            ..TriadErrorSpec::IDEAL
        },
        gyro: TriadErrorSpec {
            noise_density: GYRO_NOISE_DENSITY,
            turn_on_bias_sigma: ICM42688_GYRO_TURN_ON_DEG_S * RAD_PER_DEG,
            bias: in_run(CONSUMER_GYRO_INSTABILITY_DEG_H * RAD_PER_DEG / SECONDS_PER_HOUR),
            ..TriadErrorSpec::IDEAL
        },
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
