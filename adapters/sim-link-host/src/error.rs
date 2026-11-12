//! Why a link that decodes still cannot be hosted.

use sim_chain_trial::ChainTrialError;
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
    /// A fix or the start gives no height and the link has no default.
    #[error("{what} has no height and the link has no default height `h`")]
    MissingHeight {
        /// The fix, or the start.
        what: String,
    },
    /// A leg, or an attribute of it, that the vehicle's software does not
    /// fly yet. The link is valid; this host cannot fly it as stated.
    #[error(
        "leg {leg} ({code}{}): {reason}; the link is valid, but the vehicle's software \
         does not fly it as stated yet",
        fix.as_deref().map(|name| format!(" {name}")).unwrap_or_default()
    )]
    Unflyable {
        /// The leg, counted from one.
        leg: usize,
        /// Its path terminator code.
        code: &'static str,
        /// The name of its fix.
        fix: Option<String>,
        /// What is not flown.
        reason: String,
    },
    /// A link the page's vehicle, the C172 under its own software, cannot
    /// fly.
    #[error("the page flies the C172 through its own software, and cannot fly {detail}")]
    ChainVehicle {
        /// What the link asks that the C172 cannot fly.
        detail: String,
    },
    /// A route with fewer than two fixes to fly to.
    #[error("{count} fix(es) to fly to; a route needs at least two")]
    TooFewFixes {
        /// The fixes the route flies to.
        count: usize,
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
    /// The terrain camera's renderer refused a pose or its world.
    #[error("the terrain camera cannot render this frame: {detail}")]
    Render {
        /// The renderer's refusal.
        detail: String,
    },
    /// The vehicle's control chain refused the link or failed in flight.
    #[error("the flight on the vehicle's own software stopped: {source}")]
    Chain {
        /// The chain trial's error.
        #[from]
        source: ChainTrialError,
    },
}
