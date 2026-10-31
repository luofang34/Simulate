use crate::TickSpan;

/// A simulated component with a fixed native period: the flight dynamics
/// model, the guidance computer, an IMU, a radar, an engine.
///
/// A block advances only by whole periods, so its behaviour depends on its
/// inputs and state alone, never on wall-clock or frame timing. Blocks are
/// wired by the vehicle that owns them: each step reads the outputs of other
/// blocks as its typed input.
pub trait Block {
    /// Signals this block reads each step.
    type Input;
    /// Signals this block publishes after each step.
    type Output;
    /// Everything needed to resume this block exactly where it was.
    type Snapshot: Clone;

    /// Native period; [`crate::RateAdapter`] turns elapsed time into steps.
    fn period(&self) -> TickSpan;

    /// Advances one period.
    fn step(&mut self, input: &Self::Input) -> Result<(), BlockError>;

    /// The current outputs. Valid before the first step (initial outputs).
    fn output(&self) -> Self::Output;

    /// Captures complete state, so save/restore, replay, and handing a
    /// vehicle between propagators lose nothing.
    fn snapshot(&self) -> Self::Snapshot;

    /// Restores state captured by [`Block::snapshot`].
    fn restore(&mut self, snapshot: &Self::Snapshot);
}

/// Why a block could not advance.
#[derive(Debug, Clone, PartialEq, thiserror::Error)]
pub enum BlockError {
    /// An input was outside what the block can model.
    #[error("block {block}: input {input} out of range: {detail}")]
    InputOutOfRange {
        /// Block name.
        block: &'static str,
        /// Offending input.
        input: &'static str,
        /// What was wrong.
        detail: String,
    },
    /// The environment the block depends on was unavailable.
    #[error("block {block}: environment unavailable")]
    Environment {
        /// Block name.
        block: &'static str,
        /// The underlying atmosphere failure.
        #[source]
        source: sim_atmosphere::AtmosphereError,
    },
    /// The vehicle's component tree or aerodynamics could not be evaluated.
    #[error("block {block}: vehicle model failed")]
    Mbd {
        /// Block name.
        block: &'static str,
        /// The underlying model failure.
        #[source]
        source: mbd_core::MbdError,
    },
    /// The JSBSim executive refused a load, a read, or a step.
    #[error("block {block}: JSBSim failed")]
    Fdm {
        /// Block name.
        block: &'static str,
        /// The underlying JSBSim failure.
        #[source]
        source: jsbsim::Error,
    },
    /// The JSBSim executive ended the simulation.
    #[error("block {block}: JSBSim terminated the simulation")]
    FdmTerminated {
        /// Block name.
        block: &'static str,
    },
    /// Integration left the range of finite numbers.
    #[error("block {block}: state is not finite: {detail}")]
    NonFiniteState {
        /// Block name.
        block: &'static str,
        /// The offending state.
        detail: String,
    },
}
