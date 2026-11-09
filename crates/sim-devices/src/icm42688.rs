use glam::{DVec3, I16Vec3};
use mbd_core::{DigitalBusFrame, EngineeringValue, SiteEnvironment};
use sim_core::{Block, BlockError, TICKS_PER_SECOND, TickSpan};

use crate::WhiteNoise;
use crate::supply::rail_up;

#[cfg(test)]
mod tests;

const BLOCK: &str = "icm42688";
pub(crate) const STANDARD_GRAVITY_M_S2: f64 = 9.806_65;
/// Accelerometer full scale, ±16 g.
const ACCEL_FULL_SCALE_M_S2: f64 = 16.0 * STANDARD_GRAVITY_M_S2;
/// Gyroscope full scale, ±2000 °/s.
const GYRO_FULL_SCALE_RAD_S: f64 = 2_000.0 * (std::f64::consts::PI / 180.0);
/// Counts in the positive half of a 16-bit converter's range.
const HALF_RANGE_COUNTS: f64 = 32_768.0;
const ACCEL_LSB_M_S2: f64 = ACCEL_FULL_SCALE_M_S2 / HALF_RANGE_COUNTS;
const GYRO_LSB_RAD_S: f64 = GYRO_FULL_SCALE_RAD_S / HALF_RANGE_COUNTS;
/// Die temperature transfer: `°C = counts / 132.48 + 25`.
const TEMP_COUNTS_PER_K: f64 = 132.48;
const TEMP_OFFSET_K: f64 = 298.15;
/// Temperature the bias coefficients are referred to, kelvin.
const BIAS_REFERENCE_K: f64 = 298.15;
/// Lowest supply the part runs on, volts.
const VDD_MIN_V: f64 = 1.71;
/// First byte of the temperature, accelerometer burst.
const TEMP_DATA1: u8 = 0x1D;
/// First byte of the gyroscope burst.
const GYRO_DATA_X1: u8 = 0x25;
const BURST_BYTES: u8 = 8;
const GYRO_BYTES: u8 = 6;
const DEFAULT_RATE_HZ: u64 = 1_000;
const _: () = assert!(TICKS_PER_SECOND.is_multiple_of(DEFAULT_RATE_HZ));
/// Accelerometer noise density, 70 µg/√Hz.
pub(crate) const ACCEL_NOISE_DENSITY: f64 = 70e-6 * STANDARD_GRAVITY_M_S2;
/// Gyroscope noise density, 0.0028 °/s/√Hz.
pub(crate) const GYRO_NOISE_DENSITY: f64 = 0.0028 * (std::f64::consts::PI / 180.0);

/// Setup of one [`Icm42688Block`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Icm42688Config {
    /// Output data period.
    pub period: TickSpan,
    /// Accelerometer bias at 298.15 K, m/s².
    pub accel_bias_m_s2: DVec3,
    /// Gyroscope bias at 298.15 K, rad/s.
    pub gyro_bias_rad_s: DVec3,
    /// Accelerometer bias drift, m/s² per kelvin from 298.15 K.
    pub accel_temp_coeff: DVec3,
    /// Gyroscope bias drift, rad/s per kelvin from 298.15 K.
    pub gyro_temp_coeff: DVec3,
    /// White noise per accelerometer sample, m/s² RMS.
    pub accel_noise_rms_m_s2: DVec3,
    /// White noise per gyroscope sample, rad/s RMS.
    pub gyro_noise_rms_rad_s: DVec3,
    /// Seed of the noise stream; give each part on a vehicle its own.
    pub noise_seed: u64,
    /// SPI bus instance.
    pub bus_id: u8,
    /// SPI chip-select line.
    pub chip_select: u8,
}

impl Icm42688Config {
    /// Datasheet noise at 1 kHz output, no bias or drift.
    pub fn nominal(noise_seed: u64) -> Self {
        let noise_bandwidth_hz = (DEFAULT_RATE_HZ as f64 / 2.0).sqrt();
        Self {
            period: TickSpan::from_ticks(TICKS_PER_SECOND / DEFAULT_RATE_HZ),
            accel_bias_m_s2: DVec3::ZERO,
            gyro_bias_rad_s: DVec3::ZERO,
            accel_temp_coeff: DVec3::ZERO,
            gyro_temp_coeff: DVec3::ZERO,
            accel_noise_rms_m_s2: DVec3::splat(ACCEL_NOISE_DENSITY * noise_bandwidth_hz),
            gyro_noise_rms_rad_s: DVec3::splat(GYRO_NOISE_DENSITY * noise_bandwidth_hz),
            noise_seed,
            bus_id: 0,
            chip_select: 0,
        }
    }
}

/// One conversion, at both port tiers.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Icm42688Sample {
    /// Tier 1: specific force in sensor axes, m/s².
    pub accel_m_s2: EngineeringValue,
    /// Tier 1: angular rate in sensor axes, rad/s.
    pub gyro_rad_s: EngineeringValue,
    /// Tier 1: die temperature, K.
    pub temperature_k: EngineeringValue,
    /// Tier 2: the data registers as two big-endian SPI bursts, temperature
    /// and accelerometer from `TEMP_DATA1`, then gyroscope from
    /// `GYRO_DATA_X1`.
    pub spi_bursts: [DigitalBusFrame; 2],
}

/// Complete state of an [`Icm42688Block`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Icm42688Snapshot {
    noise: WhiteNoise,
    latest: Option<Icm42688Sample>,
}

/// TDK ICM-42688-P six-axis MEMS IMU at a mounting site.
///
/// Reads the ideal motion in [`SiteEnvironment::kinematics`], adds a
/// temperature-dependent bias and white noise, and then saturates and
/// quantises to 16 bits, so both port tiers report the same converted value.
/// The vibration and stray-field inputs are not modelled.
///
/// An unpowered or browned-out part outputs nothing: `None`, with no bus
/// frames, since a dead chip drives neither its registers nor MISO.
#[derive(Debug, Clone, PartialEq)]
pub struct Icm42688Block {
    config: Icm42688Config,
    snapshot: Icm42688Snapshot,
}

impl Icm42688Block {
    /// A part with no conversion yet. A zero period is an error.
    pub fn new(config: Icm42688Config) -> Result<Self, BlockError> {
        if config.period == TickSpan::ZERO {
            return Err(BlockError::InputOutOfRange {
                block: BLOCK,
                input: "period",
                detail: "a zero period never advances".to_owned(),
            });
        }
        Ok(Self {
            snapshot: Icm42688Snapshot {
                noise: WhiteNoise::from_seed(config.noise_seed),
                latest: None,
            },
            config,
        })
    }

    fn convert(&mut self, site: &SiteEnvironment) -> Icm42688Sample {
        let config = &self.config;
        let heating_k = site.temperature_k - BIAS_REFERENCE_K;
        let accel_bias = config.accel_bias_m_s2 + config.accel_temp_coeff * heating_k;
        let gyro_bias = config.gyro_bias_rad_s + config.gyro_temp_coeff * heating_k;
        let noise = &mut self.snapshot.noise;
        let accel = site.kinematics.specific_force_site_m_s2
            + accel_bias
            + noise.next_vec3(config.accel_noise_rms_m_s2);
        let gyro = site.kinematics.angular_rate_site_rad_s
            + gyro_bias
            + noise.next_vec3(config.gyro_noise_rms_rad_s);

        let accel_counts = quantise_vec3(accel, ACCEL_LSB_M_S2);
        let gyro_counts = quantise_vec3(gyro, GYRO_LSB_RAD_S);
        let temp_counts = quantise((site.temperature_k - TEMP_OFFSET_K) * TEMP_COUNTS_PER_K);
        Icm42688Sample {
            accel_m_s2: EngineeringValue::Vec3(accel_counts.as_dvec3() * ACCEL_LSB_M_S2),
            gyro_rad_s: EngineeringValue::Vec3(gyro_counts.as_dvec3() * GYRO_LSB_RAD_S),
            temperature_k: EngineeringValue::Scalar(
                f64::from(temp_counts) / TEMP_COUNTS_PER_K + TEMP_OFFSET_K,
            ),
            spi_bursts: self.bursts(temp_counts, accel_counts, gyro_counts),
        }
    }

    fn bursts(&self, temp: i16, accel: I16Vec3, gyro: I16Vec3) -> [DigitalBusFrame; 2] {
        let [t1, t0] = temp.to_be_bytes();
        let [ax1, ax0] = accel.x.to_be_bytes();
        let [ay1, ay0] = accel.y.to_be_bytes();
        let [az1, az0] = accel.z.to_be_bytes();
        let [gx1, gx0] = gyro.x.to_be_bytes();
        let [gy1, gy0] = gyro.y.to_be_bytes();
        let [gz1, gz0] = gyro.z.to_be_bytes();
        let frame = |register, payload, len| DigitalBusFrame::Spi {
            bus_id: self.config.bus_id,
            cs: self.config.chip_select,
            register,
            payload,
            len,
        };
        [
            frame(
                TEMP_DATA1,
                [t1, t0, ax1, ax0, ay1, ay0, az1, az0],
                BURST_BYTES,
            ),
            frame(
                GYRO_DATA_X1,
                [gx1, gx0, gy1, gy0, gz1, gz0, 0, 0],
                GYRO_BYTES,
            ),
        ]
    }
}

/// Rounds to whole counts, pinned to the converter's range.
fn quantise(counts: f64) -> i16 {
    counts.round().clamp(i16::MIN.into(), i16::MAX.into()) as i16
}

fn quantise_vec3(value: DVec3, lsb: f64) -> I16Vec3 {
    let counts = value / lsb;
    I16Vec3::new(quantise(counts.x), quantise(counts.y), quantise(counts.z))
}

fn validate(site: &SiteEnvironment) -> Result<(), BlockError> {
    let kinematics = &site.kinematics;
    if kinematics.specific_force_site_m_s2.is_finite()
        && kinematics.angular_rate_site_rad_s.is_finite()
        && site.temperature_k.is_finite()
    {
        return Ok(());
    }
    Err(BlockError::InputOutOfRange {
        block: BLOCK,
        input: "site",
        detail: format!("{site:?} is not finite"),
    })
}

impl Block for Icm42688Block {
    type Input = SiteEnvironment;
    type Output = Option<Icm42688Sample>;
    type Snapshot = Icm42688Snapshot;

    fn period(&self) -> TickSpan {
        self.config.period
    }

    fn step(&mut self, site: &SiteEnvironment) -> Result<(), BlockError> {
        validate(site)?;
        self.snapshot.latest = rail_up(&site.power_rail, VDD_MIN_V).then(|| self.convert(site));
        Ok(())
    }

    fn output(&self) -> Option<Icm42688Sample> {
        self.snapshot.latest
    }

    fn snapshot(&self) -> Icm42688Snapshot {
        self.snapshot
    }

    fn restore(&mut self, snapshot: &Icm42688Snapshot) {
        self.snapshot = *snapshot;
    }
}
