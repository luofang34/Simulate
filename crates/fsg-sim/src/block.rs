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
        source: fsg_atmosphere::AtmosphereError,
    },
}
