//! Shared off-the-shelf devices: sensors driven by the physics at their
//! mounting site, each publishing both engineering values and the frames that
//! would cross its digital bus, plus bus channels timed on the master clock.
//!
//! Noise comes from a seeded generator kept in each block's snapshot, so a
//! device replays bit-identically.

mod air_data;
mod arinc429_bus;
mod bmp390;
mod error;
mod geomagnetic;
mod gnss_receiver;
mod icm42688;
mod inertial_error;
mod magnetometer;
mod oscillator;
mod sensor_suite;
mod solar_array;
mod supply;
mod white_noise;

pub use air_data::{
    AirDataSample, AirDataSystem, AirFlow, PitotStatic, PressureErrorSpec, impact_pressure_pa,
    indicated_airspeed_m_s, sensed_true_airspeed_m_s, speed_from_impact_pressure_m_s,
};
pub use arinc429_bus::{ARINC429_WORD_BITS, Arinc429Bus, Arinc429Delivery, Arinc429Speed};
pub use bmp390::{BMP390_ADDRESS, Bmp390Block, Bmp390Config, Bmp390Input, Bmp390Sample};
pub use error::DeviceError;
pub use geomagnetic::{MagneticEnvironment, earth_field_ned_ut};
pub use gnss_receiver::{GnssErrorSpec, GnssReading, GnssReceiver, GnssSpoof};
pub use icm42688::{Icm42688Block, Icm42688Config, Icm42688Sample, Icm42688Snapshot};
pub use inertial_error::{
    GaussMarkov, ImuErrorModel, ImuErrorSpec, ImuGrade, InertialIncrement, TriadErrorSpec,
};
pub use magnetometer::{Magnetometer, MagnetometerSpec};
pub use oscillator::{OscillatorBlock, OscillatorConfig, OscillatorSnapshot};
pub use sensor_suite::{DeviceSamples, SensedTruth, SensorSuite, SensorSuiteSpec};
pub use solar_array::{SolarArrayBlock, SolarArrayConfig, SolarArrayInput};
pub use white_noise::WhiteNoise;
