//! Why a chain trial could not be built or stepped.

use sim_flight_chain::ChainError;
use sim_navigate::trial::TrialError;

/// A refused build or step.
#[derive(Debug, thiserror::Error)]
pub enum ChainTrialError {
    /// The vehicle's control chain refused its setup or a cycle.
    #[error("the control chain failed")]
    Chain(#[from] ChainError),
    /// The navigation trial refused its request or a step.
    #[error("the navigation trial failed")]
    Trial(#[from] TrialError),
    /// The spec does not describe a trial this entry can run.
    #[error("invalid chain trial: {detail}")]
    Invalid {
        /// What is wrong.
        detail: String,
    },
    /// A frame was delivered that no exposure is waiting for.
    #[error("no exposure {index} is waiting for its frame")]
    UnexpectedFrame {
        /// The index delivered.
        index: u64,
    },
}
