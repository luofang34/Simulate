//! Closed loop: each sensor set's own navigator flies the vehicle. The pilot
//! reads only the navigator's solution and the IMU's sensed rates; truth
//! moves the vehicle, drives the sensors, and is scored, judged against the
//! termination rules, and reported.

mod pilot;
mod report;
mod run;
mod sensors;

use navigate_contract::NavigationSolution;
use sim_core::SimInstant;

pub use pilot::{Pilot, PilotEvent, PilotInput, PilotOutput};
pub use report::{ClosedLoopReport, CrossTrackSample, TerminationReport, WaypointOutcome};
pub(super) use run::fly_closed_loop;

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
    /// Star tracker availability.
    pub stars: StarAvailability,
    /// Every step's pilot input and output, when traced.
    pub trace: Vec<(PilotInput, PilotOutput)>,
    /// The set's visual navigation, when it has it.
    pub visual: Option<super::visual::VisualReport>,
    /// Per-frame wall times of the visual chain; not part of the report.
    pub visual_timings: Vec<super::visual::FrameTiming>,
    /// Mean distance from the camera to the ground, metres.
    pub mean_ground_distance_m: Option<f64>,
}
