//! Shared off-the-shelf devices: sensors driven by the physics at their
//! mounting site, each publishing both engineering values and the frames that
//! would cross its digital bus, plus bus channels timed on the master clock.
//!
//! Noise comes from a seeded generator kept in each block's snapshot, so a
//! device replays bit-identically.

mod arinc429_bus;
mod bmp390;
mod error;
mod icm42688;
mod supply;
mod white_noise;

pub use arinc429_bus::{ARINC429_WORD_BITS, Arinc429Bus, Arinc429Delivery, Arinc429Speed};
pub use bmp390::{BMP390_ADDRESS, Bmp390Block, Bmp390Config, Bmp390Input, Bmp390Sample};
pub use error::DeviceError;
pub use icm42688::{Icm42688Block, Icm42688Config, Icm42688Sample, Icm42688Snapshot};
pub use white_noise::WhiteNoise;
