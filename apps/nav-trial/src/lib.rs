//! Headless route navigation trial: reads a [`sim_navigate::trial::TrialRequest`]
//! as JSON, runs it on the CPU with no window, GPU or network, and writes the
//! [`sim_navigate::trial::TrialReport`] as JSON. The work is
//! [`sim_navigate::trial::run_trial`]; this crate only moves files.

pub mod cli;
mod error;

pub use error::NavTrialError;
