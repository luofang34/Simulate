//! Headless route navigation trial in two phases. `prepare` fetches the
//! route corridor's imagery and elevation before departure and writes the
//! onboard map package and the simulated world's imagery
//! ([`sim_navigate::package::prepare_package_blocking`]); `fly` reads a
//! [`sim_navigate::trial::TrialRequest`], runs it on the CPU with no network
//! ([`sim_navigate::trial::run_trial`]), and writes the
//! [`sim_navigate::trial::TrialReport`].

pub mod cli;
mod error;
#[cfg(not(target_arch = "wasm32"))]
pub mod prepare;

pub use error::NavTrialError;
