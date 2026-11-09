//! Error variants returned by [`crate::OrbitalUniverse`] transactional operations.

use sim_spacetime::SimInstant;

use crate::{
    dop853::Dop853Error, gravity::GravityError, ids::VesselId, manifest::ManifestMismatchField,
};

/// Transactional scheduling, rollback, or manifest verification failure.
#[derive(Clone, Debug, PartialEq, thiserror::Error)]
pub enum SchedulerError {
    /// Referenced `VesselId` is not registered in the universe (or is `VesselId::NONE`).
    #[error("unknown vessel {0}")]
    UnknownVessel(VesselId),
    /// Attempted to insert a `VesselId` that already exists.
    #[error("duplicate vessel {0}")]
    DuplicateVessel(VesselId),
    /// Requested commit or event instant precedes authoritative `committed_instant`.
    #[error(
        "target instant ({} ticks) precedes committed_instant ({} ticks)",
        requested.ticks_u128(),
        committed.ticks_u128()
    )]
    BackwardCommitTarget {
        /// Requested target timestamp.
        requested: SimInstant,
        /// Current authoritative committed timestamp.
        committed: SimInstant,
    },
    /// Player command was stamped prior to `committed_instant`.
    #[error(
        "command instant ({} ticks) precedes committed_instant ({} ticks)",
        command_instant.ticks_u128(),
        committed.ticks_u128()
    )]
    CommandInPast {
        /// Command timestamp.
        command_instant: SimInstant,
        /// Current authoritative committed timestamp.
        committed: SimInstant,
    },
    /// Same-tick discrete transitions on one vessel exceeded `MAX_SAME_TICK_TRANSITIONS`.
    #[error(
        "zero-time event loop on {vessel_id} at {} ticks (exceeded {limit} transitions)",
        instant.ticks_u128()
    )]
    ZeroTimeEventLoop {
        /// Offending vessel.
        vessel_id: VesselId,
        /// Tick at which loop occurred.
        instant: SimInstant,
        /// Configured transition cap.
        limit: u32,
    },
    /// Continuous propagation failed on `vessel_id`.
    #[error("propagation failed for {vessel_id}: {source}")]
    Propagation {
        /// Affected vessel.
        vessel_id: VesselId,
        /// Underlying integrator / gravity failure.
        source: Dop853Error<GravityError>,
    },
    /// Coordinate-origin switch failed on `vessel_id`.
    #[error("primary-body switch failed for {vessel_id}: {source}")]
    FrameSwitch {
        /// Affected vessel.
        vessel_id: VesselId,
        /// Underlying gravity / ephemeris failure.
        source: GravityError,
    },
    /// Surface-fixed rest orientation evaluation failed on `vessel_id`.
    #[error("pinned surface rest evaluation failed for {vessel_id}: {source}")]
    SurfaceRest {
        /// Affected vessel.
        vessel_id: VesselId,
        /// Underlying body orientation / gravity failure.
        source: GravityError,
    },
    /// Parallel worker thread panicked or failed to return its expected proposal batch.
    #[error(
        "parallel worker batch failed on worker {worker_index} (expected {expected_proposals} proposals, received {received_proposals})"
    )]
    WorkerBatchFailed {
        /// Index of the worker thread or batch that failed.
        worker_index: usize,
        /// Number of proposals requested in the batch.
        expected_proposals: usize,
        /// Number of proposals actually collected before failure.
        received_proposals: usize,
    },
    /// A vessel's speculative frontier failed to advance toward `step_goal`, preventing an infinite retry loop.
    #[error(
        "speculative frontier stalled for {vessel_id} at {} ticks (goal {} ticks)",
        frontier.ticks_u128(),
        step_goal.ticks_u128()
    )]
    SpeculationStalled {
        /// Stalled vessel.
        vessel_id: VesselId,
        /// Current frontier timestamp of the vessel.
        frontier: SimInstant,
        /// Target step timestamp that required coverage.
        step_goal: SimInstant,
    },
    /// Snapshot manifest disagreed with the active universe's model manifest.
    #[error("snapshot manifest mismatch on field {0}")]
    ManifestMismatch(#[from] ManifestMismatchField),
}
