use navigate_contract::IdentError;
use navigate_guidance::GuidanceRefusal;
use navigate_route::PlanActivationError;
use navigate_startrack::StarTrackError;
use sim_core::BlockError;

use crate::NavigateBridgeError;

/// Why a trial could not run or could not be written.
#[derive(Debug, thiserror::Error)]
#[non_exhaustive]
pub enum TrialError {
    /// A request field is unusable as stated.
    #[error("request field `{field}`: {detail}")]
    InvalidRequest {
        /// The field.
        field: &'static str,
        /// What is wrong with it.
        detail: String,
    },
    /// A request or result did not parse or serialise.
    #[error("{what} JSON")]
    Json {
        /// Request or result.
        what: &'static str,
        /// The parser's refusal.
        #[source]
        source: serde_json::Error,
    },
    /// A waypoint identifier is not a Navigate identifier.
    #[error("waypoint identifier `{ident}`")]
    Ident {
        /// The identifier as given.
        ident: String,
        /// Navigate's refusal.
        #[source]
        source: IdentError,
    },
    /// Navigate's route executor refused the plan.
    #[error("flight plan refused")]
    Plan(#[source] PlanActivationError),
    /// Navigate's guidance refused the truth solution.
    #[error("guidance refused at {time_s} s")]
    Guidance {
        /// Route time, seconds.
        time_s: f64,
        /// Navigate's refusal.
        #[source]
        source: GuidanceRefusal,
    },
    /// The vehicle did not finish the route in the time allowed.
    #[error("route not complete after {time_s} s at waypoint {active_index}")]
    RouteIncomplete {
        /// Route time flown, seconds.
        time_s: f64,
        /// The waypoint still being flown to.
        active_index: usize,
    },
    /// The vehicle model refused a step.
    #[error("vehicle step")]
    Vehicle(#[source] BlockError),
    /// The star camera or its pipeline refused a frame.
    #[error("star camera: {what}")]
    StarCamera {
        /// The stage.
        what: String,
    },
    /// Navigate's star tracker refused its configuration or a frame.
    #[error("star tracker")]
    StarTrack(#[source] StarTrackError),
    /// A result number is not finite, which JSON cannot carry.
    #[error("{what} holds a non-finite number")]
    NonFinite {
        /// What was being written.
        what: &'static str,
    },
    /// The bridge into Navigate refused a value.
    #[error(transparent)]
    Bridge(#[from] NavigateBridgeError),
}
