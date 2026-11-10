use sim_core::BlockError;
use sim_orbital::{LagrangeError, NaifId, OrientationError, SchedulerError, VesselId};
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
    /// The vessel's central body has no orbital ephemeris or orientation,
    /// or its orbit is centred on an origin with no body-fixed frame.
    #[error("the vessel's central body has no body-fixed frame the tiers share")]
    UnmodelledBody,
    /// An orbital state centred on one body was offered to another's frame.
    #[error("orbital state is centred on {found:?}, not {expected:?}")]
    WrongPrimary {
        /// The body the conversion was for.
        expected: NaifId,
        /// The body the state is centred on.
        found: NaifId,
    },
    /// The vessel's plant can only be flown by flight dynamics.
    #[error("vessel {0:?} cannot leave the local tier")]
    LocalOnly(VesselId),
    /// A local plant of a kind this crate does not know failed.
    #[error("a local plant failed")]
    LocalPlant(#[source] Box<dyn std::error::Error + Send + Sync>),
    /// The request needs the vessel in the orbital tier.
    #[error("vessel {0:?} is not in the orbital tier")]
    NotOrbital(VesselId),
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
    /// A checkpoint was offered to a universe whose vessels are no longer
    /// in the tiers they were in when it was taken.
    #[error("a vessel was added, removed or changed tier since the checkpoint")]
    TiersChangedSinceCheckpoint,
    /// A local plant was not put back before its universe was.
    #[error("vessel {vessel:?}'s local plant is later than the checkpoint; restore it first")]
    LocalAheadOfCheckpoint {
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
    /// A station-keeping burn could not be solved: the vessel is beyond holding, or
    /// its prediction failed.
    #[error("station keeping failed")]
    Holding(#[source] Box<LagrangeError>),
}
