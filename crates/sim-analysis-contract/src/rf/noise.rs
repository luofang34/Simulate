//! Receiver noise description and the Boltzmann constant.

use serde::{Deserialize, Serialize};

/// Boltzmann constant (`J/K`), exact by the 2019 SI definition.
pub const BOLTZMANN_CONSTANT_J_K: f64 = 1.380_649e-23;

/// How a receiver's noise floor is stated.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub enum ReceiverNoise {
    /// Total system noise temperature referred to the receiver reference plane, beyond the receive line loss (`K`).
    SystemTemperature {
        /// $T_{sys}$ (`K`, `> 0`).
        system_noise_temperature_k: f64,
    },
    /// Boresight figure of merit at the same reference plane; with the receive antenna's peak
    /// gain and line loss it fixes $T_{sys}$.
    GainOverTemperature {
        /// $G/T$ on boresight (`dB/K`).
        g_over_t_db_per_k: f64,
    },
}
