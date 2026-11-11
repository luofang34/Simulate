//! Headless route navigation trial: reads a [`sim_navigate::trial::TrialRequest`]
//! as JSON, or completes one from a route of waypoints, runs it on the CPU
//! with no window, GPU or network, optionally streams progress as NDJSON,
//! and writes the [`sim_navigate::trial::TrialReport`] as JSON. The work is
//! [`sim_navigate::trial::run_trial_with_progress`]; this crate only moves
//! files.

pub mod cli;
mod error;
pub mod run;

pub use error::NavTrialError;
