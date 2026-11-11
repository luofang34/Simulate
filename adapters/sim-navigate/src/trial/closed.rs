//! Closed loop: each sensor set's own navigator flies the vehicle. The pilot
//! reads only the navigator's solution and the IMU's sensed rates; truth
//! moves the vehicle, drives the sensors, and is scored, judged against the
//! termination rules, and reported.

mod autopilot;
mod flight;
mod report;
mod run;
mod sensors;

use navigate_contract::NavigationSolution;
use sim_core::SimInstant;

pub use autopilot::{Autopilot, PilotInput, PilotOutput};
pub use report::{ClosedLoopReport, CrossTrackSample, TerminationReport, WaypointOutcome};
pub(super) use run::ClosedSet;

use super::availability::ScheduledInterval;
use super::replay::{SensorTallies, StarAvailability};
use super::report::SeriesSample;
use crate::Estimate;

/// One closed-loop flight's results, with the pilot's inputs and outputs
/// when they were traced.
#[derive(Debug, Clone)]
pub struct ClosedLoopOutcome {
    /// The scored samples.
    pub series: Vec<SeriesSample>,
    /// The navigator's estimates at the samples.
    pub estimates: Vec<Estimate<(SimInstant, NavigationSolution)>>,
    /// The true flight's outcome.
    pub report: ClosedLoopReport,
    /// Observations offered, accepted and refused, per sensor.
    pub tallies: SensorTallies,
    /// Observations per sensor inside each scheduled interval.
    pub interval_tallies: Vec<(ScheduledInterval, SensorTallies)>,
    /// Star tracker availability.
    pub stars: StarAvailability,
    /// Every step's pilot input and output, when traced.
    pub trace: Vec<(PilotInput, PilotOutput)>,
}
