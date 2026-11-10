use sim_core::BlockError;
use sim_orbital::{NaifId, OrientationError, SchedulerError, VesselId};
use sim_spacetime::TimeError;

/// Why a vessel could not be added, advanced, or handed between tiers.
#[derive(Debug, thiserror::Error)]
pub enum UniverseError {
    /// A vessel with this id already exists.
    #[error("vessel {0:?} already exists")]
    DuplicateVessel(VesselId),
    /// No vessel has this id.
    #[error("no vessel {0:?}")]
    UnknownVessel(VesselId),
    /// The vessel's central body has no orbital ephemeris or orientation.
    #[error("the vessel's central body is not one the orbital tier models")]
    UnmodelledBody,
    /// An orbital state centred on one body was offered to another's frame.
    #[error("orbital state is centred on {found:?}, not {expected:?}")]
    WrongPrimary {
        /// The body the conversion was for.
        expected: NaifId,
        /// The body the state is centred on.
        found: NaifId,
    },
    /// The vessel is already in the requested tier.
    #[error("vessel {0:?} is already in that tier")]
    AlreadyInTier(VesselId),
    /// A handoff was asked for while the vessel's local tier lags the
    /// universe's instant.
    #[error("vessel {vessel:?} is between steps; hand it off at one of its step instants")]
    BetweenSteps {
        /// The vessel.
        vessel: VesselId,
    },
    /// The universe was asked to go back in time.
    #[error("cannot advance backwards")]
    Backwards,
    /// A local flight-dynamics block failed.
    #[error("flight dynamics failed")]
    Block(#[from] BlockError),
    /// The body orientation was unavailable at the handoff instant.
    #[error("body orientation unavailable")]
    Orientation(#[from] OrientationError),
    /// The orbital scheduler refused or failed a step, an insertion or a
    /// removal.
    #[error("the orbital tier failed")]
    Scheduler(#[source] Box<SchedulerError>),
    /// The run's clock could not name the instant.
    #[error("the run clock failed")]
    Time(#[from] TimeError),
}
