use mbd_core::{DigitalBusFrame, EngineeringValue, SiteEnvironment};
use sim_core::{Block, BlockError, TICKS_PER_SECOND, TickSpan};

use crate::supply::rail_up;

#[cfg(test)]
mod tests;

const BLOCK: &str = "bmp390";
/// Default 7-bit I2C address (`SDO` low).
pub const BMP390_ADDRESS: u8 = 0x76;
/// `DATA_0`, the first of the pressure then temperature data registers.
const DATA_0: u8 = 0x04;
const BURST_BYTES: u8 = 6;
/// Finest pressure resolution, Pa.
const PRESSURE_LSB_PA: f64 = 0.016;
/// Finest temperature resolution, K.
const TEMPERATURE_LSB_K: f64 = 0.005;
/// Operating pressure range, Pa.
const PRESSURE_RANGE_PA: (f64, f64) = (30_000.0, 125_000.0);
/// Operating temperature range, K.
const TEMPERATURE_RANGE_K: (f64, f64) = (233.15, 358.15);
/// Lowest supply the part runs on, volts.
const VDD_MIN_V: f64 = 1.65;
const DEFAULT_RATE_HZ: u64 = 50;
const _: () = assert!(TICKS_PER_SECOND.is_multiple_of(DEFAULT_RATE_HZ));

/// Setup of one [`Bmp390Block`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Bmp390Config {
    /// Output data period.
    pub period: TickSpan,
    /// I2C bus instance.
    pub bus_id: u8,
    /// 7-bit I2C address.
    pub address: u8,
}

impl Default for Bmp390Config {
    /// 50 Hz output at the default address.
    fn default() -> Self {
        Self {
            period: TickSpan::from_ticks(TICKS_PER_SECOND / DEFAULT_RATE_HZ),
            bus_id: 0,
            address: BMP390_ADDRESS,
        }
    }
}

/// What a [`Bmp390Block`] reads each step.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Bmp390Input {
    /// Static pressure at the port, Pa.
    pub static_pressure_pa: f64,
    /// The mounting site: board temperature and supply rail.
    pub site: SiteEnvironment,
}

/// One conversion, at both port tiers.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Bmp390Sample {
    /// Tier 1: pressure, Pa.
    pub pressure_pa: EngineeringValue,
    /// Tier 1: temperature, K.
    pub temperature_k: EngineeringValue,
    /// Tier 2: the data registers from `DATA_0` over I2C, 24-bit
    /// little-endian pressure counts then temperature counts.
    pub i2c_burst: DigitalBusFrame,
}

/// Bosch BMP390 barometric pressure sensor.
///
/// Reports linear counts of the part's finest resolution rather than raw
/// converter codes, so a reader needs no factory calibration table. Inputs
/// outside the operating range pin to its limits. An unpowered or browned-out
/// part outputs `None`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Bmp390Block {
    config: Bmp390Config,
    latest: Option<Bmp390Sample>,
}

impl Bmp390Block {
    /// A part with no conversion yet. A zero period or an address outside
    /// 7 bits is an error.
    pub fn new(config: Bmp390Config) -> Result<Self, BlockError> {
        let usable = config.period != TickSpan::ZERO && config.address < 0x80;
        if !usable {
            return Err(BlockError::InputOutOfRange {
                block: BLOCK,
                input: "config",
                detail: format!("{config:?}"),
            });
        }
        Ok(Self {
            config,
            latest: None,
        })
    }

    fn convert(&self, input: &Bmp390Input) -> Bmp390Sample {
        let (low_pa, high_pa) = PRESSURE_RANGE_PA;
        let (low_k, high_k) = TEMPERATURE_RANGE_K;
        let pressure = counts(
            input.static_pressure_pa.clamp(low_pa, high_pa),
            PRESSURE_LSB_PA,
        );
        let temperature = counts(
            input.site.temperature_k.clamp(low_k, high_k),
            TEMPERATURE_LSB_K,
        );
        let [p0, p1, p2, _] = pressure.to_le_bytes();
        let [t0, t1, t2, _] = temperature.to_le_bytes();
        Bmp390Sample {
            pressure_pa: EngineeringValue::Scalar(f64::from(pressure) * PRESSURE_LSB_PA),
            temperature_k: EngineeringValue::Scalar(f64::from(temperature) * TEMPERATURE_LSB_K),
            i2c_burst: DigitalBusFrame::I2c {
                bus_id: self.config.bus_id,
                addr_7bit: self.config.address,
                register: DATA_0,
                payload: [p0, p1, p2, t0, t1, t2, 0, 0],
                len: BURST_BYTES,
            },
        }
    }
}

/// Whole counts of `lsb` in a nonnegative value already within range.
fn counts(value: f64, lsb: f64) -> u32 {
    (value / lsb).round() as u32
}

impl Block for Bmp390Block {
    type Input = Bmp390Input;
    type Output = Option<Bmp390Sample>;
    type Snapshot = Option<Bmp390Sample>;

    fn period(&self) -> TickSpan {
        self.config.period
    }

    fn step(&mut self, input: &Bmp390Input) -> Result<(), BlockError> {
        if !input.static_pressure_pa.is_finite() || !input.site.temperature_k.is_finite() {
            return Err(BlockError::InputOutOfRange {
                block: BLOCK,
                input: "pressure or temperature",
                detail: format!(
                    "{} Pa, {} K",
                    input.static_pressure_pa, input.site.temperature_k
                ),
            });
        }
        self.latest = rail_up(&input.site.power_rail, VDD_MIN_V).then(|| self.convert(input));
        Ok(())
    }

    fn output(&self) -> Option<Bmp390Sample> {
        self.latest
    }

    fn snapshot(&self) -> Option<Bmp390Sample> {
        self.latest
    }

    fn restore(&mut self, snapshot: &Option<Bmp390Sample>) {
        self.latest = *snapshot;
    }
}
