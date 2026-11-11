//! Why a link that decodes still cannot be hosted.

use sim_navigate::trial::TrialError;
use thiserror::Error;

/// A link the trial request or the ephemeris cannot take as stated.
#[derive(Debug, Error)]
pub enum HostError {
    /// The trial library refuses the route or the request built from it.
    #[error("the route of the link is not flyable as stated: {source}")]
    Trial {
        /// The trial library's refusal.
        #[from]
        source: TrialError,
    },
    /// A waypoint or the start gives no height and the link has no default.
    /// A decoded link never does; a link value built in code can.
    #[error("{what} has no height and the link has no default height")]
    MissingHeight {
        /// The waypoint, or the start.
        what: String,
    },
    /// The ephemeris gives no Sun for the link's start time.
    #[error("no Sun position for {time_utc} at waypoint {waypoint}: {detail}")]
    Sun {
        /// The instant asked for, ISO 8601.
        time_utc: String,
        /// The waypoint the Sun was asked at.
        waypoint: String,
        /// The ephemeris's refusal.
        detail: String,
    },
    /// A line of a recorded progress stream is not a progress record.
    #[error("line {line} of the recorded progress stream: {source}")]
    Replay {
        /// The line, counted from one.
        line: usize,
        /// The trial library's refusal of the line.
        #[source]
        source: TrialError,
    },
}
