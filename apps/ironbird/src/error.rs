use mbd_core::MbdError;
use sim_atmosphere::AtmosphereError;
use sim_aviate::AviateBridgeError;
use sim_core::{BlockError, BridgeError, SessionError};
use sim_pack::PackError;
use sim_xil::XilError;

/// Why an ironbird closed-loop rig run failed.
#[derive(Debug, thiserror::Error)]
pub enum IronbirdError {
    /// A simulation dynamics or sensor block failed.
    #[error("simulation block failed: {0}")]
    Block(#[from] BlockError),
    /// A system package failed verification or loading.
    #[error("vehicle package failed: {0}")]
    Pack(#[from] PackError),
    /// Component tree evaluation failed.
    #[error("component tree failed: {0}")]
    Mbd(#[from] MbdError),
    /// Atmosphere sampling failed.
    #[error("atmosphere sampling failed: {0}")]
    Atmosphere(#[from] AtmosphereError),
    /// Tier-0 or Tier-1 Aviate bridge failed.
    #[error("Aviate XIL bridge failed: {0}")]
    Aviate(#[from] AviateBridgeError),
    /// The simulation session refused a sensor registration or failed a step.
    #[error("simulation session failed: {0}")]
    Session(#[source] Box<SessionError>),
    /// External node bridge failed.
    #[error("external XIL bridge failed: {0}")]
    Bridge(#[from] BridgeError),
    /// Tier-2 wire transport or codec failed.
    #[error("external XIL wire failed: {0}")]
    Xil(#[from] XilError),
    /// X500 multirotor kernel builder failed validation.
    #[error("Aviate X500 kernel builder rejected configuration")]
    X500KernelBuild,
    /// Fixed-wing kernel builder failed validation.
    #[error("Aviate fixed-wing kernel builder rejected configuration")]
    FixedWingKernelBuild,
    /// Invalid CLI flag or argument value.
    #[error("invalid CLI arguments: {detail}")]
    CliUsage {
        /// Diagnostic message and usage hint.
        detail: String,
    },
}

impl From<SessionError> for IronbirdError {
    fn from(error: SessionError) -> Self {
        Self::Session(Box::new(error))
    }
}
